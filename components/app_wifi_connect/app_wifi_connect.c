#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "app_wifi_connect.h"
#include "app_rtos_task.h"
#include "app_nvs_rw.h"


// Publish following events on successful connection or disconnection
#define WIFI_CONNECTED_BIT                  BIT0
#define WIFI_DISCONNECTED_BIT               BIT1
// Publish following events when forcibly interrupting connection
#define WIFI_CONNECT_STOP_BIT               BIT2
// If entered APP_WIFI_CONNECT_STOP due to reconnect_interval=0, following event can resume connection task
#define WIFI_CONNECT_START_BIT              BIT3

// Default reconnect interval, in seconds, 1 second
#define WIFI_RECONNECT_INTERVAL_DEFAULT     (1)
// Back-off between failed connect attempts (AP gone, wrong password), doubling up to the max
#define WIFI_CONNECT_RETRY_MIN_MS           (1000)
#define WIFI_CONNECT_RETRY_MAX_MS           (30000)
// Connect task exit wait on stop. Must exceed the esp-mqtt network timeout (10 s):
// the disconnect callback may be inside app_mqtt_stop(), and a force-delete there
// leaves the MQTT locks held and deadlocks the next start.
#define WIFI_CONNECT_TASK_EXIT_TIMEOUT_MS   (12000)

// Define standard max length, consistent with array size
#define WIFI_SSID_MAX_LEN       sizeof(((wifi_config_t *)0)->sta.ssid)
#define WIFI_PASSWORD_MAX_LEN   sizeof(((wifi_config_t *)0)->sta.password)
#define WIFI_BSSID_LEN          sizeof(((wifi_config_t *)0)->sta.bssid)

// NVS persistence-related namespace and key definitions (length limited by NVS_KEY_NAME_MAX_SIZE)
#define WIFI_RECONNECT_INTERVAL_NVS_NP      "wifi_connect"
#define WIFI_RECONNECT_INTERVAL_NVS_KEY     "reconn_intvl"

// Log tag
#define TAG                                 "wifi_connect"

// WiFi connection context structure
typedef struct {
    // Event group
    EventGroupHandle_t wifi_event_group;
    // WiFi configuration (statically allocated)
    wifi_config_t wifi_config;
    // Network interface
    esp_netif_t *sta_netif;
    // Reconnect interval (seconds)
    uint16_t reconnect_interval;
    // FreeRTOS task handle
    TaskHandle_t connect_task_handle;
    // Task running flag
    volatile bool connect_task_running;
    // WiFi connection state
    volatile enum {
        APP_WIFI_DISCONNECTED,      // Indicates disconnected
        APP_WIFI_CONNECTING,        // Indicates connecting
        APP_WIFI_CONNECTED,         // Indicates connected
        APP_WIFI_RECONNECT_WAIT,    // Indicates waiting for reconnection delay
        APP_WIFI_CONNECT_STOP       // Indicates connection task exited
    } wifi_connect_state;
    // WiFi error reason code
    wifi_err_reason_t wifi_err_reason_value;
    // Whether WiFi obtained IP
    volatile bool wifi_got_ip;
    // Callback function pointer
    app_wifi_connect_callback_gotip_t callback_gotip;
    app_wifi_disconnect_callback_t callback_disconnect;
} app_wifi_connect_context_t;
// Module context: context dynamically allocated, internal fields statically resident in same memory block
static app_wifi_connect_context_t *s_ctx = NULL;


/**
 * @brief WiFi event callback
 * 
 * @param arg 
 * @param event_base 
 * @param event_id 
 * @param event_data 
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    (void)arg;
    if (s_ctx == NULL) {
        return;
    }
    // Focus on handling WiFi events
    if (event_base != WIFI_EVENT) {
        return;
    }
    switch (event_id) {
        case WIFI_EVENT_STA_CONNECTED: {
            // WiFi reconnection requires re-acquiring IP, so when connected set IP acquisition status to false for app_wifi_connect_get_status to judge
            s_ctx->wifi_got_ip = false;
            // Once connected, reset connection error reason; per type description, 0 means no error
            s_ctx->wifi_err_reason_value = 0x00;
            xEventGroupSetBits(s_ctx->wifi_event_group, WIFI_CONNECTED_BIT);
            ESP_LOGI(TAG,"wifi_event_handler: WIFI AP connected");
            break;
        }

        case WIFI_EVENT_STA_DISCONNECTED: {
            // Description of this event can be found in documentation:
            // https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32/api-guides/wifi-driver/overview.html#wifi-event-sta-disconnected
            // Roughly means manually calling disconnect/stop triggers this event, connect anomalies also trigger it; must distinguish and handle specially
            wifi_event_sta_disconnected_t* disconnected = (wifi_event_sta_disconnected_t*) event_data;
            s_ctx->wifi_err_reason_value = disconnected->reason; // Cache WiFi connection error reason code
            ESP_LOGI(TAG, "wifi_event_handler: WIFI AP Disconnected, reason: %d", disconnected->reason); // wifi_err_reason_t
            // Publish event to connection task to selectively perform reconnection
            xEventGroupSetBits(s_ctx->wifi_event_group, WIFI_DISCONNECTED_BIT);
            break;
        }
        
        default:
            break;
    }
}

/**
 * @brief IP event callback
 * 
 * @param arg 
 * @param event_base 
 * @param event_id 
 * @param event_data 
 */
static void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    (void)arg;
    if (s_ctx == NULL) {
        return;
    }
    // Focus on handling IP events
    if (event_base != IP_EVENT) {
        return;
    }
    switch (event_id) {
        case IP_EVENT_STA_GOT_IP: {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
            ESP_LOGI(TAG, "Free Heap(after got ip): %ld", esp_get_free_heap_size());
            // Mark as successfully obtained IP
            s_ctx->wifi_got_ip = true;

            // Can now callback/publish event to initialize other application layer protocol wrappers
            if (s_ctx->callback_gotip) {
                s_ctx->callback_gotip(event->ip_changed);
            }
            break;
        }
        
        default:
            break;
    }
}

/**
 * @brief Wrapped function to call disconnection callback on demand
 * 
 */
static void notify_disconnect(bool *firstRun) {
    // If disconnection callback registered, notify WiFi disconnection event to release WiFi connection-related resources
    if (s_ctx->callback_disconnect && *firstRun) {
        *firstRun = false;
        s_ctx->callback_disconnect();
    }
}

/**
 * @brief WiFi connection task, performs WiFi connection in this task
 * 
 * @param pvParameters 
 */
static void wifi_connect_task(void *pvParameters) {
    (void)pvParameters;
    if (s_ctx == NULL) {
        vTaskDelete(NULL);
        return;
    }
    esp_err_t err;
    uint32_t retry_ms = WIFI_CONNECT_RETRY_MIN_MS;

    // Before starting WiFi connection task, reset connection state to disconnected
    s_ctx->wifi_connect_state = APP_WIFI_DISCONNECTED;

    ESP_LOGI(TAG, "WIFI connect task started");
    while (s_ctx->connect_task_running) {
        switch (s_ctx->wifi_connect_state) {
            // WiFi is disconnected state, can initiate connection
            case APP_WIFI_DISCONNECTED: {
                // Before entering connected state, should clear possible event BITs to avoid state machine interference after entering APP_WIFI_CONNECTING
                xEventGroupClearBits(s_ctx->wifi_event_group, WIFI_DISCONNECTED_BIT);
                // Call lower layer connection interface to initiate WiFi connection
                err = esp_wifi_connect();
                if (err == ESP_OK) {
                    // State machine transitions to connecting
                    s_ctx->wifi_connect_state = APP_WIFI_CONNECTING;
                    ESP_LOGI(TAG, "APP_WIFI_DISCONNECTED -> APP_WIFI_CONNECTING");
                } else {
                    // esp_wifi_connect call has some strange error codes, may need to cache locally or callback report?
                    // If interface is confirmed stable and won't have this issue, then no need to handle it
                    ESP_LOGE(TAG, "esp_wifi_connect() return error: %s", esp_err_to_name(err));
                    // Best to reduce esp_wifi_connect call frequency when errors occur to avoid bigger problems or flushing useful debug info
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
                break;
            }
            // WiFi is connecting state, need to wait for connection result
            case APP_WIFI_CONNECTING: {
                // Blocking wait for WIFI_CONNECTED_BIT and WIFI_DISCONNECTED_BIT and WIFI_CONNECT_STOP_BIT events; disconnect event may be triggered by connection failure
                EventBits_t bits = xEventGroupWaitBits(
                    s_ctx->wifi_event_group,
                    WIFI_CONNECTED_BIT | WIFI_DISCONNECTED_BIT | WIFI_CONNECT_STOP_BIT, 
                    pdTRUE, 
                    pdFALSE, 
                    portMAX_DELAY
                );
                if (bits & WIFI_CONNECTED_BIT) {
                    // State machine transitions to connected state
                    s_ctx->wifi_connect_state = APP_WIFI_CONNECTED;
                    ESP_LOGI(TAG, "APP_WIFI_CONNECTING -> APP_WIFI_CONNECTED");
                    retry_ms = WIFI_CONNECT_RETRY_MIN_MS;
                }
                if (bits & WIFI_DISCONNECTED_BIT) {
                    // In any case, state machine must transition back to APP_WIFI_DISCONNECTED to re-initiate connection
                    // Why not APP_WIFI_RECONNECT_WAIT? Obviously we're trying to connect, not reconnecting after being connected and disconnected
                    // reconnect_interval controls reconnection interval after being normally connected and then disconnected
                    s_ctx->wifi_connect_state = APP_WIFI_DISCONNECTED;
                    ESP_LOGI(TAG, "APP_WIFI_CONNECTING -> APP_WIFI_DISCONNECTED");
                    // Failed attempt: back off, retrying at once keeps the radio scanning nonstop
                    if ((bits & (WIFI_CONNECTED_BIT | WIFI_CONNECT_STOP_BIT)) == 0) {
                        ESP_LOGI(TAG, "Connect failed, retrying in %lu ms", (unsigned long)retry_ms);
                        EventBits_t sbits = xEventGroupWaitBits(
                            s_ctx->wifi_event_group,
                            WIFI_CONNECT_STOP_BIT,
                            pdTRUE,
                            pdFALSE,
                            pdMS_TO_TICKS(retry_ms)
                        );
                        if (sbits & WIFI_CONNECT_STOP_BIT) {
                            bits |= WIFI_CONNECT_STOP_BIT; // handled just below
                        }
                        retry_ms = (retry_ms * 2 > WIFI_CONNECT_RETRY_MAX_MS) ? WIFI_CONNECT_RETRY_MAX_MS : retry_ms * 2;
                    }
                }
                if (bits & WIFI_CONNECT_STOP_BIT) {
                    s_ctx->wifi_connect_state = APP_WIFI_CONNECT_STOP; // Received stop connection event, need to exit connection task
                    ESP_LOGI(TAG, "APP_WIFI_CONNECTED -> APP_WIFI_CONNECT_STOP");
                }
                break;
            }
            // WiFi is connected state, need to wait for WiFi disconnect
            case APP_WIFI_CONNECTED: {
                // Below waiting for WiFi disconnect or stop event can trigger disconnect callback only once
                bool cbk_first_run = true;
                // When connected, this task just needs to blocking wait for disconnect or stop connection events, nothing else to do
                EventBits_t bits = xEventGroupWaitBits(
                    s_ctx->wifi_event_group,
                    WIFI_DISCONNECTED_BIT | WIFI_CONNECT_STOP_BIT, 
                    pdTRUE, 
                    pdFALSE, 
                    portMAX_DELAY
                );
                if (bits & WIFI_DISCONNECTED_BIT) {
                    // State machine returns to waiting for reconnection state, this task will re-initiate connection
                    s_ctx->wifi_connect_state = APP_WIFI_RECONNECT_WAIT;
                    ESP_LOGI(TAG, "APP_WIFI_CONNECTED -> APP_WIFI_RECONNECT_WAIT");
                    notify_disconnect(&cbk_first_run);
                }
                if (bits & WIFI_CONNECT_STOP_BIT) {
                    s_ctx->wifi_connect_state = APP_WIFI_CONNECT_STOP; // Received stop connection event, need to exit connection task
                    ESP_LOGI(TAG, "APP_WIFI_CONNECTED -> APP_WIFI_CONNECT_STOP");
                    notify_disconnect(&cbk_first_run);
                }
                break;
            }
            // WiFi didn't connect or disconnected due to issues, need delay before re-initiating connection
            case APP_WIFI_RECONNECT_WAIT: {
                // If reconnect_interval=0, directly exit connection task, only trigger one normal connection, no auto reconnect on later disconnects
                if (s_ctx->reconnect_interval > 0) {
                    ESP_LOGI(TAG, "Start waiting for wifi reconnect, interval = %d", s_ctx->reconnect_interval);
                    // While waiting for reconnection delay, also watch WIFI_CONNECT_STOP_BIT event; if it occurs no need to reconnect
                    EventBits_t bits = xEventGroupWaitBits(
                        s_ctx->wifi_event_group,
                        WIFI_CONNECT_STOP_BIT, 
                        pdTRUE, 
                        pdFALSE, 
                        // pdMS_TO_TICKS(1000)=100, 65535*100=6553500, no overflow concern
                        pdMS_TO_TICKS(1000) * s_ctx->reconnect_interval // Reconnect interval is in seconds
                    );
                    if (bits & WIFI_CONNECT_STOP_BIT) {
                        s_ctx->wifi_connect_state = APP_WIFI_CONNECT_STOP; // Received stop connection event, need to exit connection task
                        ESP_LOGI(TAG, "APP_WIFI_RECONNECT_WAIT -> APP_WIFI_CONNECT_STOP");
                    } else {
                        s_ctx->wifi_connect_state = APP_WIFI_DISCONNECTED; // If event not triggered, means waiting delay timed out, can go back to reconnect
                        ESP_LOGI(TAG, "APP_WIFI_RECONNECT_WAIT -> APP_WIFI_DISCONNECTED");
                    }
                } else {
                    s_ctx->wifi_connect_state = APP_WIFI_CONNECT_STOP; // Received stop connection event, need to exit connection task
                    ESP_LOGI(TAG, "APP_WIFI_RECONNECT_WAIT -> APP_WIFI_CONNECT_STOP");
                }
                break;
            }
            case APP_WIFI_CONNECT_STOP: {
                // If connect_task_running still true, means not manually stopped but stopped by reconnect_interval etc.
                if (s_ctx->connect_task_running) {
                    // Should now wait for WIFI_CONNECT_STOP_BIT or WIFI_CONNECT_START_BIT event
                    //  These events mean user called app_wifi_connect_start or app_wifi_connect_stop interface
                    EventBits_t bits = xEventGroupWaitBits(
                        s_ctx->wifi_event_group,
                        WIFI_CONNECT_START_BIT | WIFI_CONNECT_STOP_BIT, 
                        pdTRUE, 
                        pdFALSE, 
                        portMAX_DELAY
                    );
                    // User called app_wifi_connect_start interface, need to resume scan task work
                    if (bits & WIFI_CONNECT_START_BIT) {
                        s_ctx->wifi_connect_state = APP_WIFI_DISCONNECTED;
                        ESP_LOGI(TAG, "APP_WIFI_CONNECT_STOP -> APP_WIFI_DISCONNECTED");
                    }
                }
                break;
            }
            default:
                // Should never reach here!
                break;
        }
    }

    ESP_LOGI(TAG, "WIFI connect task exit.");
    s_ctx->connect_task_handle = NULL;
    s_ctx->connect_task_running = false;
    s_ctx->wifi_connect_state = APP_WIFI_CONNECT_STOP;
    vTaskDelete(NULL);
}

/**
 * @brief Internal unified cleanup function (used for rollback on init failure)
 */
static void cleanup_on_init_failure(void) {
    if (s_ctx == NULL) {
        return;
    }
    esp_wifi_deinit();
    if (s_ctx->sta_netif) {
        esp_netif_destroy_default_wifi(s_ctx->sta_netif);
        s_ctx->sta_netif = NULL;
    }
    if (s_ctx->wifi_event_group) {
        vEventGroupDelete(s_ctx->wifi_event_group);
        s_ctx->wifi_event_group = NULL;
    }
    esp_event_loop_delete_default();
    esp_netif_deinit();
    free(s_ctx);
    s_ctx = NULL;
}

/**
 * @brief Read reconnection interval from NVS
 * 
 * @param out Read result
 * @return esp_err_t 
 */
static esp_err_t get_reconnect_interval_nvs(uint16_t *out) {
    return app_nvs_rw_read(
        WIFI_RECONNECT_INTERVAL_NVS_NP, 
        (app_nvs_rw_read_item_t[]) {
            {
                .key = WIFI_RECONNECT_INTERVAL_NVS_KEY,
                .type = APP_NVS_RW_TYPE_U16,
                .data = out,
                .default_value = WIFI_RECONNECT_INTERVAL_DEFAULT,
            }
        },
        1
    );
}

/**
 * @brief Set reconnection interval to NVS
 * 
 * @param value Reconnect interval value
 * @return esp_err_t 
 */
static esp_err_t set_reconnect_interval_nvs(uint16_t value) {
    return app_nvs_rw_write(
        WIFI_RECONNECT_INTERVAL_NVS_NP, 
        (app_nvs_rw_write_item_t[]) {
            {
                .key = WIFI_RECONNECT_INTERVAL_NVS_KEY,
                .type = APP_NVS_RW_TYPE_U16,
                .data = &value,
                .length = sizeof(value),
            }
        },
        1
    );
}

/**
 * @brief Initialize WiFi connection wrapper, allocate necessary resources
 * 
 * @return esp_err_t 
 */
esp_err_t app_wifi_connect_init(void) {
    esp_err_t err;

    if (s_ctx != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Allocate memory and initialize context (critical)
    s_ctx = (app_wifi_connect_context_t *)calloc(1, sizeof(app_wifi_connect_context_t));
    if (s_ctx == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // Default power-on state: no connection attempt started
    s_ctx->wifi_connect_state = APP_WIFI_CONNECT_STOP;

    err = esp_netif_init();
    if (err != ESP_OK) {
        cleanup_on_init_failure();
        return err;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK) {
        cleanup_on_init_failure();
        return err;
    }

    s_ctx->sta_netif = esp_netif_create_default_wifi_sta();
    assert(s_ctx->sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        cleanup_on_init_failure();
        return err;
    }
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        cleanup_on_init_failure();
        return err;
    }
    s_ctx->wifi_event_group = xEventGroupCreate();
    if (s_ctx->wifi_event_group == NULL) {
        cleanup_on_init_failure();
        return ESP_ERR_NO_MEM;
    }
    err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    if (err != ESP_OK) {
        cleanup_on_init_failure();
        return err;
    }
    err = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL, NULL);
    if (err != ESP_OK) {
        cleanup_on_init_failure();
        return err;
    }

    // Read reconnect interval value from NVS, if not set default to WIFI_RECONNECT_INTERVAL_DEFAULT
    err = get_reconnect_interval_nvs(&s_ctx->reconnect_interval);
    if (err != ESP_OK) {
        cleanup_on_init_failure();
        return err;
    }

    err = esp_wifi_get_config(WIFI_IF_STA, &s_ctx->wifi_config);
    if (err != ESP_OK) {
        // If unable to get previous config due to some reason, clear as default config
        memset(&s_ctx->wifi_config, 0x00, sizeof(wifi_config_t));
    }

    return ESP_OK;
}

/**
 * @brief Start WiFi connection task; it keeps retrying in background until connected
 *  or until app_wifi_connect_stop is called
 * 
 * @return esp_err_t 
 */
esp_err_t app_wifi_connect_start(void) {
    // Check if WiFi config initialized also indirectly judges if this wrapper is inited
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    // If connection task already started, don't start again
    if (s_ctx->connect_task_running) {
        // If already started but entered stop state for some reason, this logic can resume task
        if (s_ctx->wifi_connect_state == APP_WIFI_CONNECT_STOP) {
            xEventGroupSetBits(s_ctx->wifi_event_group, WIFI_CONNECT_START_BIT);
            ESP_LOGI(TAG,"WIFI connect task restart");
        }
        return ESP_OK;
    }
    // Start WiFi, then can create connection task
    s_ctx->connect_task_running = true;
    // Should clear possible previous event bits to avoid connection task being misled by previous bits after startup
    xEventGroupClearBits(s_ctx->wifi_event_group, 
        WIFI_CONNECTED_BIT | 
        WIFI_DISCONNECTED_BIT | 
        WIFI_CONNECT_STOP_BIT | 
        WIFI_CONNECT_START_BIT
    );
    esp_err_t err = esp_wifi_start();
    if (err != ESP_OK) {
        s_ctx->connect_task_running = false;
        return err; // WiFi startup failed means mode may be wrong or some parameters misconfigured, developer needs to check
    }
    // After WiFi startup, need to start connection task, in connection task call esp_wifi_connect as needed to connect
    xTaskCreate(wifi_connect_task, "wifiConnTask", 4096, s_ctx, 8, &s_ctx->connect_task_handle);
    if (s_ctx->connect_task_handle == NULL) {
        s_ctx->connect_task_running = false;
        esp_wifi_stop(); // Failed to create connection task, we need to close wifi
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Free Heap(after app_wifi_connect_start): %ld", esp_get_free_heap_size());
    return ESP_OK;
}

/**
 * @brief Stop wifi connection task
 * 
 * @return esp_err_t 
 */
esp_err_t app_wifi_connect_stop(void) {
    // Ensure initialized
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    // Only stop when connection task is actually running
    if (s_ctx->connect_task_running) {
        // Stop wifi connection task
        s_ctx->connect_task_running = false;
        // Publish an exit connection task event, let task exit from blocked waiting state
        xEventGroupSetBits(s_ctx->wifi_event_group, WIFI_CONNECT_STOP_BIT);
        // Before stopping WiFi, ensure any existing connection is disconnected
        esp_wifi_disconnect();
        // Stop WiFi protocol stack
        esp_wifi_stop();
        // Wait for connection task to completely exit
        wait_for_rtos_task_exit(WIFI_CONNECT_TASK_EXIT_TIMEOUT_MS, &s_ctx->connect_task_handle);
        ESP_LOGI(TAG, "WIFI connect task stoped");
    }
    return ESP_OK;
}

/**
 * @brief Deinitialize wifi connection wrapper library
 * 
 * @return esp_err_t 
 */
esp_err_t app_wifi_connect_deinit(void) {
    // Ensure initialized
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Deinitializing WIFI connect...");

    // Stop wifi connection task
    app_wifi_connect_stop();
    // Deinitialize WiFi
    esp_wifi_deinit();
    // Clean up Netif
    if (s_ctx->sta_netif) {
        esp_netif_destroy_default_wifi(s_ctx->sta_netif);
        s_ctx->sta_netif = NULL;
    }
    // Clean up Event Group
    if (s_ctx->wifi_event_group) {
        vEventGroupDelete(s_ctx->wifi_event_group);
        s_ctx->wifi_event_group = NULL;
    }
    // Delete event loop
    esp_event_loop_delete_default();
    // Deinitialize Netif driver
    esp_netif_deinit();

    // Finally clean up context
    free(s_ctx);
    s_ctx = NULL;

    ESP_LOGI(TAG, "WIFI connect deinit success");
    return ESP_OK;
}

/**
 * @brief Internal helper for persistent save
 * 
 * @return esp_err_t Return value from esp_wifi_set_config
 */
static inline esp_err_t app_wifi_connect_set_save(void) {
    return esp_wifi_set_config(WIFI_IF_STA, &s_ctx->wifi_config); // Set to underlying config, this interface will call NVS for persistence
}

/**
 * @brief Set wifi ssid configuration
 * 
 * @param ssid SSID buffer; content is copied to NVS so caller lifetime is not required
 * @param length Effective SSID length (<= 32 bytes, WiFi standard max)
 * @return esp_err_t 
 */
esp_err_t app_wifi_connect_set_ssid(const char* ssid, uint8_t length) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ssid == NULL && length > 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (length > WIFI_SSID_MAX_LEN) {
        return ESP_ERR_INVALID_ARG; 
    }

    // Clear the whole buffer first (32 bytes)
    memset(s_ctx->wifi_config.sta.ssid, 0, WIFI_SSID_MAX_LEN);

    if (length > 0) {
        memcpy(s_ctx->wifi_config.sta.ssid, ssid, length);
    }
    
    return app_wifi_connect_set_save();
}

/**
 * @brief Set wifi password configuration
 * 
 * @param password Password buffer; content is copied to NVS so caller lifetime is not required
 * @param length Effective password length (<= 64 bytes, WiFi standard max)
 * @return esp_err_t 
 */
esp_err_t app_wifi_connect_set_password(const char* password, uint8_t length) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (password == NULL && length > 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (length > WIFI_PASSWORD_MAX_LEN) {
        return ESP_ERR_INVALID_ARG; 
    }

    // Clear the whole buffer first (64 bytes)
    memset(s_ctx->wifi_config.sta.password, 0, WIFI_PASSWORD_MAX_LEN);

    if (length > 0) {
        memcpy(s_ctx->wifi_config.sta.password, password, length);
    }

    return app_wifi_connect_set_save();
}

/**
 * @brief Set wifi bssid configuration
 * 
 * @param bssid BSSID buffer; content is copied to NVS so caller lifetime is not required
 * @param length Effective BSSID length (typically 6, set 0 to clear BSSID)
 * @return esp_err_t 
 */
esp_err_t app_wifi_connect_set_bssid(const uint8_t* bssid, uint8_t length) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (length != 0 && length != WIFI_BSSID_LEN) {
        return ESP_ERR_INVALID_ARG; 
    }
    if (bssid == NULL && length > 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (length > 0) {
        memcpy(s_ctx->wifi_config.sta.bssid, bssid, WIFI_BSSID_LEN);
        s_ctx->wifi_config.sta.bssid_set = 1;
    } else {
        memset(s_ctx->wifi_config.sta.bssid, 0, WIFI_BSSID_LEN);
        s_ctx->wifi_config.sta.bssid_set = 0;
    }
    
    return app_wifi_connect_set_save();
}

/**
 * @brief Set wifi authmode configuration
 * 
 * @param authmode Auth mode, see wifi_auth_mode_t for details
 * @return esp_err_t 
 */
esp_err_t app_wifi_connect_set_authmode(uint8_t authmode) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (authmode >= WIFI_AUTH_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    s_ctx->wifi_config.sta.threshold.authmode = (wifi_auth_mode_t)authmode;
    return app_wifi_connect_set_save();
}

/**
 * @brief Set wifi listen_interval configuration
 * 
 * @param listen_interval AP beacon listening interval, unit: beacon interval, default 3, range [1,100]
 * @return esp_err_t 
 */
esp_err_t app_wifi_connect_set_listen_interval(uint16_t listen_interval) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    s_ctx->wifi_config.sta.listen_interval = listen_interval;
    return app_wifi_connect_set_save();
}

/**
 * @brief Set wifi scan_mode configuration
 * 
 * @param scan_mode Scan mode, see wifi_scan_method_t enum description
 * @return esp_err_t 
 */
esp_err_t app_wifi_connect_set_scan_mode(uint8_t scan_mode) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    switch (scan_mode) {
        case WIFI_FAST_SCAN:
        case WIFI_ALL_CHANNEL_SCAN:
            s_ctx->wifi_config.sta.scan_method = (wifi_scan_method_t)scan_mode;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    return app_wifi_connect_set_save();
}

/**
 * @brief Set WiFi PMF (Protected Management Frame) configuration
 * 
 * @param pmf_mode Bitmask mode:
 *                 0       : disable PMF
 *                 Bit 0   : PMF capable
 *                 Bit 1   : PMF required
 * @return esp_err_t 
 */
esp_err_t app_wifi_connect_set_pmf_mode(uint8_t pmf_mode) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (pmf_mode > 0x03) {
        return ESP_ERR_INVALID_ARG;
    }
    bool capable = false;
    bool required = false;
    if (pmf_mode != 0) {
        if (pmf_mode & 0x01) {
            capable = true;
        }
        if (pmf_mode & 0x02) {
            required = true;
            capable = true; 
        }
    }
    s_ctx->wifi_config.sta.pmf_cfg.capable = capable;
    s_ctx->wifi_config.sta.pmf_cfg.required = required;
    return app_wifi_connect_set_save();
}

/**
 * @brief Set WiFi reconnect interval
 * 
 * @param interval Reconnect interval in seconds
 * @return esp_err_t 
 */
esp_err_t app_wifi_connect_set_reconnect_interval(uint16_t interval) {
    // This variable is only to check if wifi connection library is initialized
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    // If unchanged, ignore and return success directly
    if (s_ctx->reconnect_interval == interval) {
        return ESP_OK;
    }
    // Save to context first
    s_ctx->reconnect_interval = interval;
    // Persist to NVS on each call
    return set_reconnect_interval_nvs(interval);
}

/**
 * @brief Get wifi ssid configuration
 * 
 * Note: internal buffer size is 32; when SSID length is 32 it may not end with '\0'.
 * This function will calculate actual length, copy data to user buffer, and force add '\0'.
 * 
 * @param ssid Output buffer, **must** be at least 33 bytes (32 + '\0')
 * @param length Output parameter, return valid length of ssid (0 ~ 32)
 * @return esp_err_t 
 *         - ESP_OK: success
 *         - ESP_ERR_INVALID_STATE: Wi-Fi connection context not initialized
 *         - ESP_ERR_INVALID_ARG: input pointer is NULL
 */
esp_err_t app_wifi_connect_get_ssid(char* ssid, uint8_t* length) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ssid == NULL || length == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t* src = s_ctx->wifi_config.sta.ssid;
    size_t len = 0;

    // Iterate until 0 terminator or max length 32
    while (len < WIFI_SSID_MAX_LEN && src[len] != '\0') {
        len++; // Manually calculate length, because may not have '\0' terminator
    }
    
    *length = (uint8_t)len;
    
    // Copy to user buffer
    memcpy(ssid, src, len);
    ssid[len] = '\0'; // Force terminator so caller always gets a valid C string

    return ESP_OK;
}

/**
 * @brief Get wifi password configuration
 * 
 * Note: internal buffer size is 64; when password length is 64 it may not end with '\0'.
 * This function will calculate actual length, copy data to user buffer, and force add '\0'.
 * 
 * @param password Output buffer, **must** be at least 65 bytes (64 + '\0')
 * @param length Output parameter, return valid length of password (0 ~ 64)
 * @return esp_err_t 
 *         - ESP_OK: success
 *         - ESP_ERR_INVALID_STATE: Wi-Fi connection context not initialized
 *         - ESP_ERR_INVALID_ARG: input pointer is NULL
 */
esp_err_t app_wifi_connect_get_password(char* password, uint8_t* length) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (password == NULL || length == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t* src = s_ctx->wifi_config.sta.password;
    size_t len = 0;

    // Iterate until 0 terminator or max length 64
    while (len < WIFI_PASSWORD_MAX_LEN && src[len] != '\0') {
        len++; // Manually calculate length, because may not have '\0' terminator
    }

    *length = (uint8_t)len;
    
    memcpy(password, src, len);
    password[len] = '\0'; // Force terminator

    return ESP_OK;
}

/**
 * @brief Get wifi bssid configuration
 * 
 * @param bssid Output buffer, size at least 6 bytes
 * @param length Output: returns 6 when bssid_set is 1, otherwise 0
 * @param bssid_set Output: whether BSSID filtering is enabled (1 enabled, 0 disabled)
 * @return esp_err_t 
 *         - ESP_OK: success
 *         - ESP_ERR_INVALID_STATE: Wi-Fi connection context not initialized
 *         - ESP_ERR_INVALID_ARG: input pointer is NULL
 */
esp_err_t app_wifi_connect_get_bssid(uint8_t* bssid, uint8_t* length, uint8_t* bssid_set) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (bssid == NULL || length == NULL || bssid_set == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *bssid_set = s_ctx->wifi_config.sta.bssid_set;

    if (*bssid_set) {
        *length = WIFI_BSSID_LEN;
        memcpy(bssid, s_ctx->wifi_config.sta.bssid, WIFI_BSSID_LEN);
    } else {
        *length = 0;
        memset(bssid, 0, WIFI_BSSID_LEN); // Clear output buffer to avoid stale bytes
    }

    return ESP_OK;
}

/**
 * @brief Get WiFi authmode configuration
 * 
 * @param authmode Output current auth mode
 * @return esp_err_t 
 *         - ESP_OK: success
 *         - ESP_ERR_INVALID_STATE: Wi-Fi connection context not initialized
 *         - ESP_ERR_INVALID_ARG: input pointer is NULL
 */
esp_err_t app_wifi_connect_get_authmode(uint8_t* authmode) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (authmode == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *authmode = (uint8_t)s_ctx->wifi_config.sta.threshold.authmode;
    return ESP_OK;
}

/**
 * @brief Get WiFi listen_interval configuration
 * 
 * @param listen_interval Output pointer to current listen interval
 * @return esp_err_t
 *         - ESP_OK: success
 *         - ESP_ERR_INVALID_STATE: Wi-Fi connection context not initialized
 *         - ESP_ERR_INVALID_ARG: input pointer is NULL
 */
esp_err_t app_wifi_connect_get_listen_interval(uint16_t* listen_interval) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (listen_interval == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *listen_interval = s_ctx->wifi_config.sta.listen_interval;
    return ESP_OK;
}

/**
 * @brief Get WiFi scan mode configuration
 * 
 * @param scan_mode Output pointer to current scan mode
 * @return esp_err_t
 *         - ESP_OK: success
 *         - ESP_ERR_INVALID_STATE: Wi-Fi connection context not initialized
 *         - ESP_ERR_INVALID_ARG: input pointer is NULL
 */
esp_err_t app_wifi_connect_get_scan_mode(uint8_t* scan_mode) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (scan_mode == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *scan_mode = (uint8_t)s_ctx->wifi_config.sta.scan_method;
    return ESP_OK;
}

/**
 * @brief Get WiFi PMF (Protected Management Frame) configuration
 * 
 * @param pmf_mode Output bitmask:
 *                 0       : PMF disabled
 *                 Bit 0   : PMF capable
 *                 Bit 1   : PMF required
 * @return esp_err_t
 *         - ESP_OK: success
 *         - ESP_ERR_INVALID_STATE: Wi-Fi connection context not initialized
 *         - ESP_ERR_INVALID_ARG: input pointer is NULL
 */
esp_err_t app_wifi_connect_get_pmf_mode(uint8_t* pmf_mode) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (pmf_mode == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t mode = 0;
    
    if (s_ctx->wifi_config.sta.pmf_cfg.capable) {
        mode |= 0x01; // Set Bit 0
    }
    if (s_ctx->wifi_config.sta.pmf_cfg.required) {
        mode |= 0x02; // Set Bit 1
    }

    *pmf_mode = mode;
    return ESP_OK;
}

/**
 * @brief Get WiFi reconnect interval
 * 
 * @param interval Output pointer to reconnect interval (seconds)
 * @return esp_err_t
 */
esp_err_t app_wifi_connect_get_reconnect_interval(uint16_t* interval) {
    // This API does not access driver fields directly, but still requires initialized context
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (interval == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *interval = s_ctx->reconnect_interval;
    return ESP_OK;
}

/**
 * @brief Restore reconnect interval to default value
 * 
 * @return esp_err_t
 */
esp_err_t app_wifi_connect_restore_reconnect_interval(void) {
    // Ensure wrapper is initialized
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    // Persist default value to NVS
    esp_err_t err = set_reconnect_interval_nvs(WIFI_RECONNECT_INTERVAL_DEFAULT);
    if (err == ESP_OK) {
        // Update in-memory value after successful persistence
        s_ctx->reconnect_interval = WIFI_RECONNECT_INTERVAL_DEFAULT;
        return ESP_OK;
    }
    // Return persistence error
    return err;
}

/**
 * @brief Wait for WiFi connect result within the specified timeout
 * 
 * @param timeout Timeout in seconds (max meaningful value: 255)
 * @param err_reason Output WiFi disconnect reason (valid when result == 1)
 * @param result Output result code:
 *        0: connected successfully
 *        1: connection failed (see err_reason)
 *        2: timeout
 * @return esp_err_t
 */
esp_err_t app_wifi_connect_wait_finish(uint8_t timeout, uint8_t *result, uint8_t *err_reason) {
    // Ensure wrapper/context is initialized
    if (s_ctx == NULL || s_ctx->wifi_event_group == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    // Fast path: if already connected, return immediately.
    // In this case WIFI_CONNECTED_BIT might not be generated again.
    if (s_ctx->wifi_connect_state == APP_WIFI_CONNECTED) {
        *result = 0;
        return ESP_OK;
    }
    // Wait for either connected or disconnected event within timeout
    EventBits_t bits = xEventGroupWaitBits(s_ctx->wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_DISCONNECTED_BIT, // Set by wifi_event_handler
            pdFALSE, // Do not clear bits here
            pdFALSE,
            pdMS_TO_TICKS(1000) * timeout); // pdMS_TO_TICKS(1000)=100; 255*100=25500, no overflow concern

    // If no event bit is set, treat as timeout
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap");
        ESP_LOGI(TAG, "Free Heap(after wifi connected): %ld\n", esp_get_free_heap_size());
        *result = 0;
    } else if (bits & WIFI_DISCONNECTED_BIT) {
        *err_reason = s_ctx->wifi_err_reason_value; // Disconnect reason from WiFi event
        ESP_LOGI(TAG, "Failed to connect to ap");
        *result = 1;
    } else {
        // Timeout case
        *result = 2;
    }
    return ESP_OK;
}

/**
 * @brief Get current WiFi connection status
 * 
 * @param status Output status value:
 *  0: station not connected and task stopped
 *  1: station connected to AP but IPv4 not acquired yet
 *  2: station connected to AP and IPv4 acquired
 *  3: station is connecting/reconnecting
 *  4: station is disconnected (task running)
 * @return esp_err_t
 */
esp_err_t app_wifi_connect_get_status(uint8_t *status) {
    // Ensure wrapper is initialized
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    // Map internal state to exported status code
    switch (s_ctx->wifi_connect_state) {
        case APP_WIFI_CONNECT_STOP:
            // Includes initial stop state and stop due to reconnect_interval == 0
            *status = 0;
            break;
        case APP_WIFI_CONNECTED:
            // Connected to AP; differentiate by IP acquisition
            if (s_ctx->wifi_got_ip) {
                *status = 2;
            } else {
                *status = 1;
            }
            break;
        case APP_WIFI_CONNECTING:
        case APP_WIFI_RECONNECT_WAIT: // Connecting/reconnecting state
            *status = 3;
            break;
        case APP_WIFI_DISCONNECTED:
            *status = 4;
            break;
        default:
            break;
    }
    return ESP_OK;
}

/**
 * @brief Set WiFi connect module callbacks
 * 
 * @param type Callback type
 * @param callback Callback function pointer, or NULL to clear
 * @return esp_err_t
 */
esp_err_t app_wifi_connect_set_callback(app_wifi_connect_callback_type_t type, void* callback) {
    // Ensure wrapper is initialized
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    // Set callback by type
    switch (type) {
        // Notify when IP is obtained / changed
        case APP_WIFI_CONNECT_CALLBACK_GOTIP:
            s_ctx->callback_gotip = callback;
            break;
        // Notify when WiFi disconnects
        case APP_WIFI_CONNECT_CALLBACK_DISCONN:
            s_ctx->callback_disconnect = callback;
            break;
        default:
            break;
    }

    return ESP_OK;
}
