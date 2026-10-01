#include <sys/time.h>
#include <time.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_app_desc.h"
#include "nvs_flash.h"
#include "app_cmd_uart.h"
#include "app_log_uart.h"
#include "app_com_defs.h"
#include "app_ble_spp.h"
#include "app_wifi_sntp.h"
#include "app_wifi_scanner.h"
#include "app_wifi_connect.h"
#include "app_wifi_netif_cfg.h"
#include "app_wifi_mdns.h"
#include "app_tcp_server.h"
#include "app_tcp_client.h"
#include "app_udp_server.h"
#include "app_udp_client.h"
#include "app_mqtt_client.h"
#include "app_ota_ops.h"
#include "main_settings.h"


// Casts pointer p to type t and dereferences it to return the value.
// NOTE: Only use this when the memory pointed to by p actually stores a value of type t;
// otherwise the behavior is undefined.
#define P2V(t, p) (*(t *)(p))
// Disables struct padding so that fields are arranged compactly with no padding bytes.
// This ensures data structures returned to the host have no padding bytes, preventing
// parse errors on the host side caused by alignment padding.
//  NOTE: The host must also define data structures with the same packed attribute to
//  ensure identical memory layout on both sides.
#define PACKED __attribute__((packed))
// Macro to simplify error checking: if the expression does not return ESP_OK,
// immediately propagate the error to the caller.
#define RETURN_ON_FAILURE(expr) do { \
    esp_err_t __err = (expr); \
    if (__err != ESP_OK) { \
        return __err; \
    } \
} while (0)

// Current WiFi functional mode of the module. Disabled by default unless a command to
// enable WiFi is received.
typedef enum {
    WIFI_FUNCTION_MODE_WIFI_DISABLE = 0U, // WiFi fully disabled
    WIFI_FUNCTION_MODE_WIFI_SCANNER,
    WIFI_FUNCTION_MODE_WIFI_FORWARD,
} wifi_function_mode_t;

// Current WiFi forwarding protocol type; each type has its own configuration options.
typedef enum {
    WIFI_FORWARD_TCP_SERVER = 0U,
    WIFI_FORWARD_TCP_CLIENT,
    WIFI_FORWARD_UDP_SERVER,
    WIFI_FORWARD_UDP_CLIENT,
    WIFI_FORWARD_MQTT_CLIENT,
    WIFI_FORWARD_TYPE_MAX, // Sentinel value for the total number of supported forwarding protocol types
} wifi_forward_type_t;

// Payload struct for the WiFi country code configuration API
typedef struct {
    uint8_t country_policy;        // byte0
    uint8_t country_code[3];       // byte1-3
    uint8_t start_channel;         // byte4
    uint8_t total_channel_count;   // byte5
} PACKED wifi_country_payload_t;

// Log tag for this module
static const char *TAG = "main";
// Indicates whether power-on initialization has completed
static bool system_ready = false;
// WiFi functional mode; defaults to disabled. May switch to WIFI_FORWARD mode after
// loading config on boot (SCAN mode can only be started at runtime).
static wifi_function_mode_t g_wifi_function_mode = WIFI_FUNCTION_MODE_WIFI_DISABLE;
// Buffer for the WiFi STA mode MAC address
static uint8_t wifi_sta_mac[6];
// Default forwarding type is TCP_SERVER
static wifi_forward_type_t wifi_forward_type = WIFI_FORWARD_TCP_SERVER;
// esp_wifi_set_country requires a const pointer, so keep this alive as a global
static wifi_country_t wifi_country = { 0x00 };
// WiFi TX power in dBm. Default is the compile-time maximum.
// Note: this value uses 0.25 dBm steps, so divide by 0.25 to obtain the correct step count.
static int8_t wifi_tx_power = CONFIG_ESP_PHY_MAX_WIFI_TX_POWER / 0.25;
// WiFi inactive time in seconds. Default is 6 s. If no data is exchanged within
// this period after connection, WiFi is considered inactive and may enter power-saving mode.
static uint16_t wifi_inactive_time = 6;
// Whether DHCP is enabled. Enabled by default.
static uint8_t wifi_dhcp_enable = 1;
// WiFi IP address info in little-endian IPv4 format
static uint32_t wifi_ip_info[3] = { 0 }; // [0]=IP address, [1]=gateway, [2]=netmask
// WiFi hostname (ESP hostname max is 32 bytes + null terminator = 33 bytes)
static char wifi_host_name[33] = CONFIG_DEVICE_IDENTIFIER "\0"; // ESP hostname max is 32 bytes plus null terminator = 33 bytes


/**
 * @brief Callback for UART baud rate switch. When this callback fires, the command UART
 * is still operating at the old baud rate; the new rate takes effect after this returns.
 * 
 */
static void on_uart_cmd_baudrate_change(void) {
    // Notify the host that the new baud rate is valid; subsequent communication will use it.
    // NOTE: This is the normal response to APP_CMD_SET_SYS_UART_CMD_BAUD_RATE, not a broadcast.
    app_uart_send_response(APP_CMD_SET_SYS_UART_CMD_BAUD_RATE, NULL, 0); // Return value can be ignored
}

/**
 * @brief Callback for sending log data over UART. Broadcasts log data via the CMD UART.
 * 
 * @param data Pointer to the log data; do not modify or free
 * @param len Length of the log data
 */
static void on_log_printf_uart_report(uint8_t *data, uint16_t length) {
    // Broadcast the log data over the CMD UART
    app_uart_send_broadcast(APP_BROADCAST_SYS_LOG_MESSAGE, (const uint8_t *)data, length); // Return value can be ignored
}

/**
 * @brief WiFi scan result callback. Broadcasts scan results;
 * app_uart_send_broadcast is thread-safe so no explicit locking is needed.
 * 
 * @param result Pointer to scan results
 * @param count Number of scan results
 */

static void on_wifi_scan_result_report(wifi_scan_result_t *result, uint8_t count) {
    // Return value can be ignored
    app_uart_send_broadcast(APP_BROADCAST_WIFI_SCAN_RESULT, (const uint8_t *)result, count * sizeof(wifi_scan_result_t));
}

/**
 * @brief Receive callback for forwarded data. Sends the received data back to the host
 * via a UART broadcast packet.
 * 
 * @param data Pointer to data buffer; do not free
 * @param length Data length
 */
static void on_forward_data_received(uint8_t *data, uint16_t length) {
    // Forward received data to UART regardless of whether the source is BLE or WiFi
    app_uart_send_broadcast(APP_BROADCAST_DATA_FORWARD, (const uint8_t *)data, length); // Return value can be ignored
}

/**
 * @brief Callback for WiFi connected and IP obtained (or IP changed). The forwarding
 * application layer protocol can be started here. NOTE: this callback runs in the
 * sys_evt task — do not block!
 * 
 * @param ip_changed Whether the event was triggered by an IP address change; usually
 *                   not important to the caller
 */
static void on_wifi_connect_gotip(bool ip_changed) {
    // Start SNTP service
    app_wifi_sntp_start();
    // mDNS: <hostname>.local
    {
        uint16_t mdns_port = 0;
        if (wifi_forward_type == WIFI_FORWARD_TCP_SERVER) {
            app_tcp_server_get_port(&mdns_port);
        }
        app_wifi_mdns_start(wifi_host_name, mdns_port);
    }
    // Start the forwarding application for the current WiFi forward type
    switch (wifi_forward_type) {
        case WIFI_FORWARD_TCP_SERVER:
            app_tcp_server_start();
            break;
        case WIFI_FORWARD_TCP_CLIENT:
            app_tcp_client_start();
            break;
        case WIFI_FORWARD_UDP_SERVER:
            app_udp_server_start();
            break;
        case WIFI_FORWARD_UDP_CLIENT:
            app_udp_client_start();
            break;
        case WIFI_FORWARD_MQTT_CLIENT:
            app_mqtt_start();
            break;
        default:
            break;
    }
}

/**
 * @brief WiFi disconnect callback. Runs in wifiConnTask; may block briefly but
 * avoid long blocking to not delay WiFi reconnection.
 * 
 */
static void on_wifi_disconnect(void) {
    // Stop SNTP service
    app_wifi_sntp_stop();
    app_wifi_mdns_stop();
    // Stop the forwarding service for the current WiFi forward type
    switch (wifi_forward_type) {
        case WIFI_FORWARD_TCP_SERVER:
            app_tcp_server_stop();
            break;
        case WIFI_FORWARD_TCP_CLIENT:
            app_tcp_client_stop();
            break;
        case WIFI_FORWARD_UDP_SERVER:
            app_udp_server_stop();
            break;
        case WIFI_FORWARD_UDP_CLIENT:
            app_udp_client_stop();
            break;
        case WIFI_FORWARD_MQTT_CLIENT:
            app_mqtt_stop();
            break;
        default:
            break;
    }
}

/**
 * @brief Initializes supplemental resources (application-layer protocol stack) after
 * the WiFi connection library has been initialized.
 * 
 */
static esp_err_t wifi_connect_init_additional(void) {

    // Initialize SNTP resources
    RETURN_ON_FAILURE(app_wifi_sntp_init());

    // Initialize resources for the current forwarding protocol type
    switch (wifi_forward_type) {
        case WIFI_FORWARD_TCP_SERVER:
            RETURN_ON_FAILURE(app_tcp_server_init()); // Init TCP server; do not start yet — wait for WiFi IP
            RETURN_ON_FAILURE(app_tcp_server_set_rx_callback(on_forward_data_received)); // Register rx callback to forward data to UART
            break;
        case WIFI_FORWARD_TCP_CLIENT:
            RETURN_ON_FAILURE(app_tcp_client_init());
            RETURN_ON_FAILURE(app_tcp_client_set_rx_callback(on_forward_data_received));
            break;
        case WIFI_FORWARD_UDP_SERVER:
            RETURN_ON_FAILURE(app_udp_server_init());
            RETURN_ON_FAILURE(app_udp_server_set_rx_callback(on_forward_data_received));
            break;
        case WIFI_FORWARD_UDP_CLIENT:
            RETURN_ON_FAILURE(app_udp_client_init());
            RETURN_ON_FAILURE(app_udp_client_set_rx_callback(on_forward_data_received));
            break;
        case WIFI_FORWARD_MQTT_CLIENT:
            RETURN_ON_FAILURE(app_mqtt_init());
            RETURN_ON_FAILURE(app_mqtt_set_rx_callback(on_forward_data_received));
            break;
        default:
            break;
    }

    return ESP_OK;
}

/**
 * @brief Deinitializes supplemental resources (application-layer protocol stack) before
 * the WiFi connection library is deinitialized.
 * 
 */
static esp_err_t wifi_connect_deinit_additional(void) {

    // Release SNTP resources
    RETURN_ON_FAILURE(app_wifi_sntp_deinit());

    // Release resources for the current forwarding protocol type
    switch (wifi_forward_type) {
        case WIFI_FORWARD_TCP_SERVER:
            RETURN_ON_FAILURE(app_tcp_server_deinit());
            RETURN_ON_FAILURE(app_tcp_server_set_rx_callback(NULL));
            break;
        case WIFI_FORWARD_TCP_CLIENT:
            RETURN_ON_FAILURE(app_tcp_client_deinit());
            RETURN_ON_FAILURE(app_tcp_client_set_rx_callback(NULL));
            break;
        case WIFI_FORWARD_UDP_SERVER:
            RETURN_ON_FAILURE(app_udp_server_deinit());
            RETURN_ON_FAILURE(app_udp_server_set_rx_callback(NULL));
            break;
        case WIFI_FORWARD_UDP_CLIENT:
            RETURN_ON_FAILURE(app_udp_client_deinit());
            RETURN_ON_FAILURE(app_udp_client_set_rx_callback(NULL));
            break;
        case WIFI_FORWARD_MQTT_CLIENT:
            RETURN_ON_FAILURE(app_mqtt_deinit());
            RETURN_ON_FAILURE(app_mqtt_set_rx_callback(NULL));
            break;
        default:
            break;
    }

    return ESP_OK;
}

/**
 * @brief Forwards data received from UART to all currently active pass-through endpoints.
 * 
 * @param data Pointer to data buffer; must not be freed
 * @param length Data length
 */
static esp_err_t tx_data_forward_from_uart(uint8_t *data, uint16_t length) {

    // Always forward to the BLE pass-through endpoint; BLE is enabled by default
    // unless the module has not yet completed initialization
    esp_err_t err_ble_send = app_ble_send(data, length);

    // If WiFi forwarding mode is active, also forward to the WiFi endpoint
    esp_err_t err_net_send = ESP_OK;
    if (g_wifi_function_mode == WIFI_FUNCTION_MODE_WIFI_FORWARD) {
        // Call the send function for the appropriate application-layer protocol stack
        switch (wifi_forward_type) {
            case WIFI_FORWARD_TCP_SERVER:
                err_net_send = app_tcp_server_send(data, length);
                break;
            case WIFI_FORWARD_TCP_CLIENT:
                err_net_send = app_tcp_client_send(data, length);
                break;
            case WIFI_FORWARD_UDP_SERVER:
                err_net_send = app_udp_server_send(data, length);
                break;
            case WIFI_FORWARD_UDP_CLIENT:
                err_net_send = app_udp_client_send(data, length);
                break;
            case WIFI_FORWARD_MQTT_CLIENT:
                err_net_send = app_mqtt_send(data, length);
                break;
            default:
                break;
        }
    }

    // If at least one endpoint succeeds, consider the forward successful
    if (err_ble_send == ESP_OK || err_net_send == ESP_OK) {
        return ESP_OK;
    } else {
        // Both endpoints failed; return an error to indicate forwarding failed.
        // NOTE: We do not distinguish between BLE or network send failures since
        // the caller only needs to know that the data was not successfully forwarded.
        return ESP_FAIL;
    }
}

/**
 * @brief Returns string length, or 0 if the string is NULL or exceeds max_len.
 * 
 * @param str String pointer
 * @param max_len Maximum allowed length
 * @return size_t String length, or 0 if NULL or exceeds max_len
 */
static size_t strlen_or0(const char *str, size_t max_len) {
    if (str == NULL) return 0;
    size_t len = strlen(str);
    return (len > max_len) ? 0 : len;
}

/**
 * @brief Calls the setter with the string pointer, or NULL if length is 0.
 * If length is 1 but the string is empty, the setter is still called with the
 * empty string pointer (not NULL).
 * @param set Function pointer taking a const char* and returning esp_err_t
 * @param str String pointer
 * @param length String length
 * @return esp_err_t ESP_OK on success, other values indicate failure
 */
static esp_err_t set_str_or_null(esp_err_t (*set)(const char*), void *str, size_t length) {
    if (length == 0) {
        return set(NULL);
    } else {
        return set((const char *)str);
    }
}

/**
 * @brief Applies a timezone string to the system to ensure correct local time.
 * 
 * @param tz Timezone string in GNU libc format
 *             e.g. "CST-8" for China Standard Time (UTC+8), "UTC0" for UTC
 */
static void settings_time_zone_apply(const char *tz) {
    // Apply the timezone: set the TZ environment variable then call tzset to update the C library
    setenv("TZ", tz, 1); // Set the TZ environment variable, overwriting any existing value
    tzset(); // Update the C library timezone settings to take effect
}

/**
 * @brief Shared initialization logic for WiFi forwarding mode. Called on power-on or
 * when switching to WiFi forwarding mode via command.
 * 
 */
static esp_err_t wifi_forward_common_init(void) {
    // Initialize the WiFi connection library
    RETURN_ON_FAILURE(app_wifi_connect_init());
    // Register WiFi event callbacks
    RETURN_ON_FAILURE(app_wifi_connect_set_callback(APP_WIFI_CONNECT_CALLBACK_GOTIP, on_wifi_connect_gotip));
    RETURN_ON_FAILURE(app_wifi_connect_set_callback(APP_WIFI_CONNECT_CALLBACK_DISCONN, on_wifi_disconnect));
    // Initialize supplemental WiFi resources
    RETURN_ON_FAILURE(wifi_connect_init_additional());
    return ESP_OK;
}

/**
 * @brief Reports a UART command processing error by broadcasting the error information
 * to the host. The host can use this to detect that command processing failed, identify
 * which command failed, and determine the error code.
 * 
 * @param cmd The command code that failed
 * @param err The error code for the failure
 */
static void uart_cmd_error_report(uint16_t cmd, esp_err_t err) {
    struct Payload {
        uint16_t cmd;
        int32_t err;
    } PACKED payload = {
        .cmd = cmd,
        .err = (int32_t)err,
    };
    app_uart_send_broadcast(APP_BROADCAST_CMD_ERROR, (const uint8_t *)&payload, sizeof(payload));
}

// UART command packet handler callback. Runs in the uart_rx_task.
// Be mindful of multi-task resource access issues.
static void on_uart_cmd_complete(PacketType_t type, uint16_t cmd, uint8_t *p_data, uint16_t length) {
    if (type != TYPE_HOST_CMD) return; // Only handle commands from the host; ignore loopback data
    switch (cmd) {
        case APP_CMD_GET_VERSION_INFO: {
            const esp_app_desc_t *app_desc = esp_app_get_description();
            const char *version = (app_desc == NULL) ? "" : app_desc->version;
            size_t version_len = strlen_or0(version, MAX_PAYLOAD_LEN);
            app_uart_send_response(cmd, (uint8_t *)version, version_len);
            break;
        }

        case APP_CMD_GET_DEVICE_MODEL: {
            // Currently hardcoded; could be extended per chip variant in the future.
            // Additional model IDs for custom series with the same chip can be added as needed.
            // 0xDA10: ESP32-C2 (ESP8684) — WiFi + BLE only; the first PM5 WiFi+BLE expansion module
            const uint8_t model[2] = { 0xDA, 0x10 };
            app_uart_send_response(cmd, model, sizeof(model));
            break;
        }

        case APP_CMD_GET_SYS_FREE_HEAP: {
            uint32_t free_heap = (uint32_t)esp_get_free_heap_size();
            app_uart_send_response(cmd, (uint8_t *)&free_heap, sizeof(free_heap));
            break;
        }

        case APP_CMD_GET_SYS_TIMESTAMP: {
            struct timeval tv;
            gettimeofday(&tv, NULL); // Get current UTC time
            app_uart_send_response(cmd, (uint8_t *)&tv.tv_sec, sizeof(tv.tv_sec));
            break;
        }

        case APP_CMD_GET_APP_COMPILE_DATETIME: {
            // NOTE: some fields in app_desc (date/time) may reflect an older build.
            // See this issue for details: https://esp32.com/viewtopic.php?t=27623
            // To get the true build time, use the __TIME__ and __DATE__ macros; however
            // those only reflect the compile time of the file that uses those macros,
            // not the final link time of the whole project.
            const esp_app_desc_t *app_desc = esp_app_get_description();
            char build_datetime[sizeof(app_desc->date) + sizeof(app_desc->time) + 1] = { 0 };
            if (app_desc != NULL) {
                snprintf(build_datetime, sizeof(build_datetime), "%s %s", app_desc->date, app_desc->time);
            }
            size_t build_datetime_len = strlen_or0(build_datetime, MAX_PAYLOAD_LEN);
            app_uart_send_response(cmd, (uint8_t *)build_datetime, build_datetime_len);
            break;
        }

        case APP_CMD_SET_SYS_TIMESTAMP: {
            if (length != 8) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            struct timeval tv = { 0x00 };
            tv.tv_sec = P2V(time_t, p_data);
            settimeofday(&tv, NULL); // Set UTC time
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_SYS_TIME_ZONE: {
            char* tzstr = getenv("TZ");
            // Calculate string length only if tzstr is non-NULL
            size_t tzstr_len = tzstr ? strlen_or0(tzstr, MAX_PAYLOAD_LEN) : 0;
            app_uart_send_response(cmd, (uint8_t *)tzstr, tzstr_len);
            break;
        }

        case APP_CMD_SET_SYS_TIME_ZONE: {
            if (length == 0) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            if (length > 50) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_SIZE);
                break;
            }

            // Apply the new timezone to the system
            settings_time_zone_apply((const char *)p_data);

            // Persist to NVS to survive reboots
            esp_err_t err = settings_time_zone_save((const char *)p_data);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to save time zone to flash: %s", esp_err_to_name(err));
                uart_cmd_error_report(cmd, err);
                break;
            }

            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_SYS_BASE_MAC_ADDR: {
            // Buffer size depends on chip MAC support:
            // IEEE 802.15.4 uses 8-byte MAC addresses; otherwise 6 bytes
#if CONFIG_SOC_IEEE802154_SUPPORTED
            uint8_t base_mac[8] = { 0 };
#else
            uint8_t base_mac[6] = { 0 };
#endif

            esp_err_t err = esp_efuse_mac_get_default(base_mac); // Read the factory-programmed base MAC address from eFuse
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to get default MAC address: %s", esp_err_to_name(err));
                uart_cmd_error_report(cmd, err);
                break;
            }

            app_uart_send_response(cmd, base_mac, sizeof(base_mac));
            break;
        }

        case APP_CMD_GET_SYS_UART_CMD_BAUD_RATE: {
            uint32_t baud_rate = app_uart_get_baud_rate();
            app_uart_send_response(cmd, (uint8_t *)&baud_rate, sizeof(baud_rate));
            break;
        }

        case APP_CMD_GET_SYS_UART_CMD_MAX_BAUD_RATE: {
            uint32_t max_baud_rate = CONFIG_SOC_UART_BITRATE_MAX;
            app_uart_send_response(cmd, (uint8_t *)&max_baud_rate, sizeof(max_baud_rate));
            break;
        }

        case APP_CMD_SET_SYS_UART_CMD_BAUD_RATE: {
            if (length != sizeof(uint32_t)) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_SIZE);
                break;
            }

            // Convert received bytes to a uint32_t baud rate value
            uint32_t baud_rate = P2V(uint32_t, p_data);
            // Validate the baud rate
            if (!app_uart_is_baud_rate_supported(baud_rate)) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }

            // Set the baud rate; if successful, triggers the on_uart_cmd_baudrate_change callback
            esp_err_t err = app_uart_set_baud_rate(baud_rate);
            // Only handle the error if setting the baud rate failed
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set UART baud rate: %s", esp_err_to_name(err));
                uart_cmd_error_report(cmd, err);
                break;
            }

            // NOTE: Do not send a response here; it is handled inside on_uart_cmd_baudrate_change.
            break;
        }

        case APP_CMD_GET_SYS_NVS_STATS: {
            // Retrieve NVS partition statistics
            nvs_stats_t stats = {0};
            esp_err_t err = nvs_get_stats(NULL, &stats);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to get NVS stats: %s", esp_err_to_name(err));
                uart_cmd_error_report(cmd, err);
                break;
            }
            // NOTE: Manually pack a struct since nvs_stats_t is not PACKED and may differ across SDK versions
            struct NvsStatsPayload {
                uint32_t used_entries;
                uint32_t free_entries;
                uint32_t available_entries;
                uint32_t total_entries;
                uint32_t namespace_count;
            } PACKED payload = {
                .used_entries = (uint32_t)stats.used_entries,
                .free_entries = (uint32_t)stats.free_entries,
                .available_entries = (uint32_t)stats.available_entries,
                .total_entries = (uint32_t)stats.total_entries,
                .namespace_count = (uint32_t)stats.namespace_count,
            };
            // Send response to host
            app_uart_send_response(cmd, (uint8_t *)&payload, sizeof(payload));
            break;
        }

        case APP_CMD_RESTORE_TO_FACTORY_SETTINGS: {
            ESP_LOGW(TAG, "Host requested factory settings restore, must to ensure that the module is keep power on.");
            // Erase NVS partition — key step for restoring factory settings
            esp_err_t err = nvs_flash_erase();
            if (err == ESP_OK) {
                // After erasing, reinitialize NVS
                err = nvs_flash_init();
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "Factory settings restored successfully.");
                    app_uart_send_response(cmd, NULL, 0);
                    break; // Skip error handling below
                }
            }
            ESP_LOGE(TAG, "Failed to erase NVS flash: %s", esp_err_to_name(err));
            uart_cmd_error_report(cmd, err);
            break;
        }

        case APP_CMD_GET_SYS_READY_STATUS: {
            uint8_t ready_status = system_ready ? 1 : 0;
            app_uart_send_response(cmd, &ready_status, sizeof(ready_status));
            break;
        }

        case APP_CMD_REBOOT: {
            ESP_LOGW(TAG, "Host requested reboot.");
            // Acknowledge before rebooting so the host knows a reboot is coming, not a timeout
            app_uart_send_response(cmd, NULL, 0);
            // Wait for UART transmission to complete; it may still be in-progress or queued
            app_uart_wait_for_tx_done(portMAX_DELAY);
            // After TX completes, perform the reboot; nothing after this should execute
            esp_restart();
            break;
        }

        case APP_CMD_OTA_BEGIN: {
            if (length != sizeof(uint32_t)) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_SIZE);
                break;
            }
            uint32_t firmware_size = P2V(uint32_t, p_data);
            esp_err_t err = app_ota_begin(firmware_size);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to begin OTA: %s", esp_err_to_name(err));
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_OTA_WRITE: {
            if (length == 0) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_SIZE);
                break;
            }
            esp_err_t err = app_ota_write(p_data, length);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to write OTA chunk: %s", esp_err_to_name(err));
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_OTA_END: {
            esp_err_t err = app_ota_end();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to finish OTA: %s", esp_err_to_name(err));
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_SET_TO_WIFI_DISABLE_MODE: {
            if (g_wifi_function_mode == WIFI_FUNCTION_MODE_WIFI_FORWARD) {
                wifi_connect_deinit_additional();
                app_wifi_connect_deinit();
            } else if (g_wifi_function_mode == WIFI_FUNCTION_MODE_WIFI_SCANNER) {
                app_wifi_scanner_deinit();
            }

            g_wifi_function_mode = WIFI_FUNCTION_MODE_WIFI_DISABLE;
            ESP_LOGI(TAG, "Free Heap(after WIFI_FUNCTION_MODE_WIFI_DISABLE): %ld", esp_get_free_heap_size());

            // Persist to NVS to survive reboots (disable mode)
            esp_err_t err = settings_wifi_mode_save(WIFI_FUNCTION_MODE_WIFI_DISABLE);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to save wifi mode to NVS: %s", esp_err_to_name(err));
                uart_cmd_error_report(cmd, err);
                break;
            }

            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_SET_TO_WIFI_FORWARD_MODE: {
            // Single byte indicates the WiFi forwarding type;
            // multiple app-layer protocols are supported so this must be specified
            wifi_forward_type_t new_wifi_forward_type = (wifi_forward_type_t)p_data[0];
            if (length != 1 || new_wifi_forward_type >= WIFI_FORWARD_TYPE_MAX) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }

            // Persist mode and forward type to NVS to survive reboots
            esp_err_t err = settings_wifi_mode_save(WIFI_FUNCTION_MODE_WIFI_FORWARD);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to save wifi mode to NVS: %s", esp_err_to_name(err));
                uart_cmd_error_report(cmd, err);
                break;
            }
            err = settings_wifi_forward_type_save(new_wifi_forward_type);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to save wifi forward type to NVS: %s", esp_err_to_name(err));
                uart_cmd_error_report(cmd, err);
                break;
            }

            if (g_wifi_function_mode == WIFI_FUNCTION_MODE_WIFI_SCANNER) {
                app_wifi_scanner_deinit();
            }

            // If already in WiFi forward mode but with a different forward type,
            // reinitialize for the new type
            if (g_wifi_function_mode == WIFI_FUNCTION_MODE_WIFI_FORWARD) {
                if (wifi_forward_type != new_wifi_forward_type) {
                    // Deinitialize resources for the current forwarding type
                    wifi_connect_deinit_additional();
                    // Update the forwarding type
                    wifi_forward_type = new_wifi_forward_type;
                    // Initialize resources for the new forwarding type
                    wifi_connect_init_additional();
                } else {
                    ESP_LOGI(TAG, "Already in WIFI_FUNCTION_MODE_WIFI_FORWARD with the same forward type, no need to reinitialize");
                }
            } else {
                // Not currently in WiFi forward mode; initialize it now
                g_wifi_function_mode = WIFI_FUNCTION_MODE_WIFI_FORWARD;
                wifi_forward_type = new_wifi_forward_type;
                // Initialize WiFi forward mode via the shared init helper
                wifi_forward_common_init();
            }

            ESP_LOGI(TAG, "Free Heap(after WIFI_FUNCTION_MODE_WIFI_FORWARD): %ld", esp_get_free_heap_size());
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_SET_TO_WIFI_SCAN_MODE: {
            if (g_wifi_function_mode == WIFI_FUNCTION_MODE_WIFI_FORWARD) {
                wifi_connect_deinit_additional();
                app_wifi_connect_deinit();
            }

            // Only initialize scanner if not already in scan mode
            if (g_wifi_function_mode != WIFI_FUNCTION_MODE_WIFI_SCANNER) {
                esp_err_t err = app_wifi_scanner_init(); // Initialize the WiFi scanner
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to initialize wifi scanner: %s", esp_err_to_name(err));
                    uart_cmd_error_report(cmd, err);
                    break;
                }
                err = esp_wifi_set_max_tx_power(wifi_tx_power); // Set WiFi TX power
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to set wifi max tx power: %s", esp_err_to_name(err));
                    uart_cmd_error_report(cmd, err);
                    break;
                }
                g_wifi_function_mode = WIFI_FUNCTION_MODE_WIFI_SCANNER;
                ESP_LOGI(TAG, "Free Heap(after APP_CMD_SET_TO_WIFI_SCAN_MODE): %ld", esp_get_free_heap_size());
            }

            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_SET_LOG_UART_FORWARD_ENABLE: {
            if (length != 1) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            // First byte controls log forwarding: 0 = stop, non-zero = start
            if (p_data[0] == 0) {
                app_log_uart_stop();
            } else {
                app_log_uart_start();
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_LOG_UART_FORWARD_ENABLE: {
            uint8_t status;
            app_log_uart_get_status(&status);
            app_uart_send_response(cmd, &status, sizeof(status));
            break;
        }

        case APP_CMD_SET_LOG_LEVEL: {
            if (length < 1) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            // First byte is the log level (required); remaining bytes are an optional tag
            // filter string. If absent or empty, sets the global log level.
            uint8_t level = p_data[0];
            // Tag filter string starts at byte 2; check whether it is present and non-empty
            const char *tag_filter = NULL;
            if (length == 1 || strlen((const char *)(p_data + 1)) == 0) {
                // No tag filter; set global log level
                tag_filter = "*";
            } else {
                // Tag filter present; use it
                tag_filter = (const char *)(p_data + 1);
            }
            // Set the log level; NULL/empty tag filter sets the global level
            esp_log_level_set(tag_filter, level);
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_LOG_LEVEL: {
            uint8_t level;
            // No tag filter: get global log level; otherwise get level for the specified tag
            if (length == 0 || strlen((const char*)p_data) == 0) {
                level = esp_log_level_get("*"); // Get global log level
            } else {
                level = esp_log_level_get((const char *)(p_data)); // Get log level for the specified tag
            }
            app_uart_send_response(cmd, &level, sizeof(level));
            break;
        }

        case APP_CMD_START_WIFI_SCAN_TASK: {
            app_wifi_scanner_set_callback(on_wifi_scan_result_report);
            app_wifi_scanner_start();
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_STOP_WIFI_SCAN_TASK: {
            app_wifi_scanner_set_callback(NULL);
            app_wifi_scanner_stop();
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_SET_WIFI_SCAN_CONFIG: {
            struct Payload {
                uint8_t scan_type;              // byte0
                uint8_t show_hidden;            // byte1
                uint16_t scan_time_min;         // byte2-3
                uint16_t scan_time_max;         // byte4-5
                uint16_t channel_bitmap_2ghz;   // byte6-7
            } PACKED *payload = (struct Payload*)p_data;
            if (length != sizeof(struct Payload)) { 
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG); 
                break; 
            }
            // Debug logging
            ESP_LOGI(TAG, "Set wifi scanner config<scan_type> = %d", payload->scan_type);
            ESP_LOGI(TAG, "Set wifi scanner config<show_hidden> = %d", payload->show_hidden);
            ESP_LOGI(TAG, "Set wifi scanner config<scan_time_min> = %d", payload->scan_time_min);
            ESP_LOGI(TAG, "Set wifi scanner config<scan_time_max> = %d", payload->scan_time_max);
            ESP_LOGI(TAG, "Set wifi scanner config<channel_bitmap_2ghz> = %d", payload->channel_bitmap_2ghz);
            // Get scan config reference; update relevant fields if successful
            wifi_scan_config_t *p_wsc;
            if (app_wifi_scanner_get_scan_config(&p_wsc) == ESP_OK) {
                p_wsc->scan_type = payload->scan_type;
                p_wsc->show_hidden = payload->show_hidden;
                // scan_time_min is irrelevant for passive scan
                if (p_wsc->scan_type == WIFI_SCAN_TYPE_ACTIVE) {
                    // NOTE: When Bluetooth is enabled, this should be the default value
                    // to avoid BLE advertising/connection issues.
                    p_wsc->scan_time.active.min = payload->scan_time_min;
                    p_wsc->scan_time.active.max = payload->scan_time_max;
                } else {
                    p_wsc->scan_time.passive = payload->scan_time_max;
                }
                p_wsc->channel_bitmap.ghz_2_channels = payload->channel_bitmap_2ghz;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_WIFI_SCAN_STATUS: {
            uint8_t status = app_wifi_scanner_get_status();
            app_uart_send_response(cmd, &status, sizeof(status));
            break;
        }

        case APP_CMD_SET_WIFI_CFG_COUNTRY: {
            wifi_country_payload_t *payload = (wifi_country_payload_t*)p_data;
            // Update WiFi country configuration
            wifi_country.policy = payload->country_policy;
            memcpy(wifi_country.cc, payload->country_code, sizeof(wifi_country.cc));
            wifi_country.schan = payload->start_channel;
            wifi_country.nchan = payload->total_channel_count;
            // Apply via the underlying API
            esp_err_t err = esp_wifi_set_country((const wifi_country_t*)&wifi_country); 
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_WIFI_CFG_COUNTRY: {
            // Default according to ESP32 official docs:
            // {.cc="01", .schan=1, .nchan=11, .policy=WIFI_COUNTRY_POLICY_AUTO}
            // WIFI_COUNTRY_POLICY_AUTO   = 0
            // WIFI_COUNTRY_POLICY_MANUAL = 1
            esp_err_t err = esp_wifi_get_country(&wifi_country);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            wifi_country_payload_t payload;
            // Copy data into payload
            payload.country_policy = wifi_country.policy;
            memcpy(payload.country_code, wifi_country.cc, sizeof(payload.country_code));
            payload.start_channel       = wifi_country.schan;
            payload.total_channel_count = wifi_country.nchan;
            // Send response to host
            app_uart_send_response(cmd, (uint8_t *)&payload, sizeof(wifi_country_payload_t));
            break;
        }

        case APP_CMD_SET_WIFI_CFG_TX_PWR: {
            if (length != 1) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            
            wifi_tx_power = (int8_t)p_data[0];
            if (wifi_tx_power <= 7) wifi_tx_power = 8;   // Clamp minimum to 8
            if (wifi_tx_power >= 81) wifi_tx_power = 80; // Clamp maximum to 80

            // Persist to NVS to survive reboots (TX power)
            esp_err_t err = settings_wifi_tx_pwr_save(wifi_tx_power);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to save wifi tx power to NVS: %s", esp_err_to_name(err));
                uart_cmd_error_report(cmd, err);
                break;
            }

            // Apply via the underlying API
            err = esp_wifi_set_max_tx_power(wifi_tx_power);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }

            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_WIFI_CFG_TX_PWR: {
            int8_t max_pwr;
            esp_err_t err = esp_wifi_get_max_tx_power(&max_pwr);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, (uint8_t *)&max_pwr, sizeof(int8_t));
            break;
        }

        case APP_CMD_SET_WIFI_CFG_INACTIVE_TIME: {
            if (length != 2) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }

            // Convert received bytes to a uint16_t inactive time value (little-endian)
            wifi_inactive_time = P2V(uint16_t, p_data);
            
            // Persist to NVS to survive reboots (inactive time)
            esp_err_t err = settings_wifi_inactive_time_save(wifi_inactive_time);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to save wifi inactive_time to NVS: %s", esp_err_to_name(err));
                uart_cmd_error_report(cmd, err);
                break;
            }

            // Apply via the underlying API
            err = esp_wifi_set_inactive_time(WIFI_IF_STA, wifi_inactive_time);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }

            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_WIFI_CFG_INACTIVE_TIME: {
            uint16_t sec;
            esp_err_t err = esp_wifi_get_inactive_time(WIFI_IF_STA, &sec);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, (uint8_t*)&sec, sizeof(uint16_t));
            break;
        }

        case APP_CMD_SET_WIFI_CFG_DHCP: {
            if (length != 1) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            
            // Enable/disable DHCP: 0 = disable, non-zero = enable
            wifi_dhcp_enable = p_data[0];
            
            // Persist to NVS to survive reboots (DHCP enable)
            esp_err_t err = settings_wifi_dhcp_enable_save(wifi_dhcp_enable);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to save wifi dhcp_enable to NVS: %s", esp_err_to_name(err));
                uart_cmd_error_report(cmd, err);
                break;
            }

            app_wifi_cfg_set_dhcp_enable(wifi_dhcp_enable);
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_WIFI_CFG_DHCP: {
            esp_netif_dhcp_status_t dhcp_status;
            esp_err_t err = app_wifi_cfg_get_dhcp_status(&dhcp_status);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, (uint8_t*)&dhcp_status, 1);
            break;
        }

        case APP_CMD_SET_WIFI_CFG_PROTOCOL: {
            if (length != 2) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }

            wifi_protocols_t protocols;
            // Fetch the existing protocol config first to avoid unintentionally overwriting fields
            esp_err_t err = esp_wifi_get_protocols(WIFI_IF_STA, &protocols);
            if (err == ESP_OK) {
                protocols.ghz_2g = P2V(uint16_t, p_data);
                err = esp_wifi_set_protocols(WIFI_IF_STA, &protocols); // Persisted internally; no extra NVS save needed
                if (err == ESP_OK) {
                    app_uart_send_response(cmd, NULL, 0);
                }
            }

            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set wifi protocols: %s", esp_err_to_name(err));
                uart_cmd_error_report(cmd, err);
            }
            break;
        }

        case APP_CMD_GET_WIFI_CFG_PROTOCOL: {
            wifi_protocols_t protocols;
            esp_err_t err = esp_wifi_get_protocols(WIFI_IF_STA, &protocols); 
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, (uint8_t*)&protocols.ghz_2g, sizeof(protocols.ghz_2g));
            break;
        }

        case APP_CMD_SET_WIFI_CFG_MAC_ADDR: {
            if (length != sizeof(wifi_sta_mac)) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            memcpy(wifi_sta_mac, p_data, sizeof(wifi_sta_mac));

            // Persist to NVS to survive reboots (MAC address)
            esp_err_t err = settings_wifi_mac_addr_save(wifi_sta_mac);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "fail to save wifi mac: %d", err);
                uart_cmd_error_report(cmd, err);
                break;
            }

            err = esp_wifi_set_mac(WIFI_IF_STA, (const uint8_t *)wifi_sta_mac);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "fail to esp_wifi_set_mac: %d", err);
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_WIFI_CFG_MAC_ADDR: {
            esp_err_t err = esp_wifi_get_mac(WIFI_IF_STA, wifi_sta_mac);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, wifi_sta_mac, sizeof(wifi_sta_mac));
            break;
        }

        case APP_CMD_SET_WIFI_CFG_IP_ADDR: {
            if (length != sizeof(wifi_ip_info)) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }

            // NOTE: this API expects big-endian byte order, i.e.
            // p_data[0]=192, p_data[1]=168, p_data[2]=1, p_data[3]=100
            // (not little-endian)

            wifi_ip_info[0] = P2V(uint32_t, &p_data[0]);
            wifi_ip_info[1] = P2V(uint32_t, &p_data[4]);
            wifi_ip_info[2] = P2V(uint32_t, &p_data[8]); // After conversion this is stored as little-endian U32 internally

            // ESP_LOGI(TAG, "APP_CMD_SET_WIFI_CFG_IP_ADDR: ip=%d, gateway=%d, netmask=%d", 
            //     wifi_ip_info[0], wifi_ip_info[1], wifi_ip_info[2]);

            // Persist to NVS first; stored as little-endian content
            esp_err_t err = settings_wifi_ip_addr_save(wifi_ip_info[0], wifi_ip_info[1], wifi_ip_info[2]);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to save wifi ip addr to NVS: %d", err);
                uart_cmd_error_report(cmd, err);
                break;
            }

            // Apply via the underlying wrapper function
            err = app_wifi_cfg_set_ipv4(wifi_ip_info[0], wifi_ip_info[1], wifi_ip_info[2]); 
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err); 
                break;
            } 

            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_WIFI_CFG_IP_ADDR: {
            uint32_t resp[3];
            esp_err_t err = app_wifi_cfg_get_ipv4(&resp[0], &resp[1], &resp[2]); 
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, (uint8_t *)&resp, sizeof(resp));
            break;
        }

        case APP_CMD_SET_WIFI_CFG_HOST_NAME: {
            if (length > (sizeof(wifi_host_name) - 1)) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }

            // Copy hostname data into buffer; ensure the null terminator is set correctly
            memcpy(wifi_host_name, p_data, length);
            wifi_host_name[length] = '\0'; // Ensure null terminator

            // Persist to NVS to survive reboots (hostname)
            esp_err_t err = settings_wifi_host_name_save((const char *)wifi_host_name);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to save wifi host name to NVS: %d", err);
                uart_cmd_error_report(cmd, err);
                break;
            }

            err = app_wifi_cfg_set_host_name((const char*)wifi_host_name);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_wifi_mdns_set_hostname((const char*)wifi_host_name);

            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_WIFI_CFG_HOST_NAME: {
            const char *hostname = NULL;

            esp_err_t err = app_wifi_cfg_get_host_name(&hostname);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }

            size_t hostname_len = hostname ? strlen(hostname) : 0;
            app_uart_send_response(cmd, (uint8_t *)hostname, hostname_len);
            break;
        }

        case APP_CMD_SET_WIFI_CONNECT_CFG_SSID: {
            esp_err_t err;
            if (length == 0) {
                // Pass NULL to reset SSID (clears the SSID filter)
                err = app_wifi_connect_set_ssid(NULL, 0); // Persisted via esp_wifi_set_config
            } else {
                err = app_wifi_connect_set_ssid((char*)p_data, length);
            }
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_WIFI_CONNECT_CFG_SSID: {
            uint8_t ssid[33]; // 32 + 1: SSID max length is 32, plus one null terminator
            uint8_t ssid_length;

            esp_err_t err = app_wifi_connect_get_ssid((char*)ssid, &ssid_length); 
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err); 
                break;
            }

            app_uart_send_response(cmd, ssid, ssid_length);
            break;
        }

        case APP_CMD_SET_WIFI_CONNECT_CFG_PASSWORD: {
            esp_err_t err;
            
            if (length == 0) {
                // Pass NULL to reset password (clears the password)
                err = app_wifi_connect_set_password(NULL, 0); // Persisted via esp_wifi_set_config
            } else {
                err = app_wifi_connect_set_password((char*)p_data, length); 
            }

            if (err != ESP_OK) { 
                uart_cmd_error_report(cmd, err);
                break; 
            }

            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_WIFI_CONNECT_CFG_PASSWORD: {
            uint8_t password[65]; // 64 + 1: password max length is 64, plus one null terminator
            uint8_t pwd_length;
            esp_err_t err = app_wifi_connect_get_password((char*)password, &pwd_length);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, password, pwd_length);
            break;
        }

        case APP_CMD_SET_WIFI_CONNECT_CFG_BSSID: {
            esp_err_t err;
            if (length == 0) {
                // Reset BSSID (clear MAC filter)
                err = app_wifi_connect_set_bssid(NULL, 0); // Persisted via esp_wifi_set_config
            } else {
                err = app_wifi_connect_set_bssid((uint8_t*)p_data, length);
            }
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_WIFI_CONNECT_CFG_BSSID: {
            uint8_t bssid[6]; // BSSID is always 6 bytes
            uint8_t bssid_len;
            uint8_t bssid_set_flag;
            esp_err_t err = app_wifi_connect_get_bssid(bssid, &bssid_len, &bssid_set_flag);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            // Return the actual valid length (0 or 6)
            app_uart_send_response(cmd, bssid, bssid_len);
            break;
        }

        case APP_CMD_SET_WIFI_CONNECT_CFG_AUTHMODE: {
            if (length != sizeof(uint8_t)) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            uint8_t authmode = p_data[0];
            esp_err_t err = app_wifi_connect_set_authmode(authmode); // Persisted via esp_wifi_set_config
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_WIFI_CONNECT_CFG_AUTHMODE: {
            uint8_t authmode;
            esp_err_t err = app_wifi_connect_get_authmode(&authmode);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &authmode, sizeof(authmode));
            break;
        }

        case APP_CMD_SET_WIFI_CONNECT_CFG_LISTEN_INTERVAL: {
            if (length != sizeof(uint16_t)) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            uint16_t interval = P2V(uint16_t, p_data);
            esp_err_t err = app_wifi_connect_set_listen_interval(interval); // Persisted via esp_wifi_set_config
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_WIFI_CONNECT_CFG_LISTEN_INTERVAL: {
            uint16_t interval;
            esp_err_t err = app_wifi_connect_get_listen_interval(&interval);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, (uint8_t*)&interval, sizeof(interval));
            break;
        }

        case APP_CMD_SET_WIFI_CONNECT_CFG_SCAN_MODE: {
            if (length != sizeof(uint8_t)) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            uint8_t mode = p_data[0];
            esp_err_t err = app_wifi_connect_set_scan_mode(mode); // Persisted via esp_wifi_set_config
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_WIFI_CONNECT_CFG_SCAN_MODE: {
            uint8_t mode;
            esp_err_t err = app_wifi_connect_get_scan_mode(&mode);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &mode, sizeof(mode));
            break;
        }

        case APP_CMD_SET_WIFI_CONNECT_CFG_PMF: {
            if (length != sizeof(uint8_t)) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            uint8_t pmf_mode = p_data[0];
            esp_err_t err = app_wifi_connect_set_pmf_mode(pmf_mode); // Persisted via esp_wifi_set_config
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_WIFI_CONNECT_CFG_PMF: {
            uint8_t pmf_mode;
            esp_err_t err = app_wifi_connect_get_pmf_mode(&pmf_mode);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &pmf_mode, sizeof(pmf_mode));
            break;
        }

        case APP_CMD_SET_WIFI_CONNECT_CFG_RECONNECT_INTERVAL: {
            if (length != sizeof(uint16_t)) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            uint16_t interval = P2V(uint16_t, p_data);
            esp_err_t err = app_wifi_connect_set_reconnect_interval(interval); // Persisted internally by the module
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_WIFI_CONNECT_CFG_RECONNECT_INTERVAL: {
            uint16_t interval;
            esp_err_t err = app_wifi_connect_get_reconnect_interval(&interval);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, (uint8_t*)&interval, sizeof(interval));
            break;
        }

        case APP_CMD_SET_WIFI_SNTP_ENABLE: {
            if (length != 1) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            app_wifi_sntp_set_enable(p_data[0]); // Persisted internally by the module
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_WIFI_SNTP_ENABLE: {
            uint8_t enable;
            app_wifi_sntp_get_enable(&enable);
            app_uart_send_response(cmd, &enable, sizeof(enable));
            break;
        }

        case APP_CMD_SET_WIFI_SNTP_SERVER: {
            if (length == 0) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            app_wifi_sntp_set_server((const char *)p_data); // Persisted internally by the module
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_WIFI_SNTP_SERVER: {
            const char *server = NULL;
            app_wifi_sntp_get_server(&server);
            size_t server_len = server ? strlen_or0(server, MAX_PAYLOAD_LEN) : 0;
            app_uart_send_response(cmd, (uint8_t *)server, server_len);
            break;
        }

        case APP_CMD_SET_WIFI_SNTP_INTERVAL: {
            if (length != sizeof(uint32_t)) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            uint32_t interval = P2V(uint32_t, p_data); // Persisted internally by the module
            app_wifi_sntp_set_interval(interval);
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_WIFI_SNTP_INTERVAL: {
            uint32_t interval = 0;
            app_wifi_sntp_get_interval(&interval);
            app_uart_send_response(cmd, (uint8_t*)&interval, sizeof(interval));
            break;
        }

        case APP_CMD_START_WIFI_SNTP: {
            esp_err_t err = app_wifi_sntp_start();
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_STOP_WIFI_SNTP: {
            esp_err_t err = app_wifi_sntp_stop();
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_WIFI_SNTP_SYNC_STATUS: {
            uint8_t status = 0;
            app_wifi_sntp_get_sync_status(&status);
            app_uart_send_response(cmd, &status, sizeof(status));
            break;
        }

        case APP_CMD_START_WIFI_CONNECT_TASK: {
            esp_err_t err = app_wifi_connect_start();
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_STOP_WIFI_CONNECT_TASK: {
            esp_err_t err = app_wifi_connect_stop();
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_WIFI_CONNECT_STATUS: {
            uint8_t status;
            esp_err_t err = app_wifi_connect_get_status(&status);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &status, sizeof(uint8_t));
            break;
        }

        case APP_CMD_WAIT_FOR_WIFI_CONNECT_TASK: {
            if (length != 1 || p_data[0] == 0) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            struct Payload {
                uint8_t connect_result;
                uint8_t wifi_err_reason;
            } PACKED payload = {0x00};
            esp_err_t err = app_wifi_connect_wait_finish(p_data[0], &payload.connect_result, &payload.wifi_err_reason);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, (uint8_t*)&payload, sizeof(struct Payload));
            break;
        }

        case APP_CMD_SEND_FORWARD_DATA: {
            // No need to validate data length; a zero-length packet may be valid
            esp_err_t err = tx_data_forward_from_uart(p_data, length);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_SET_TCP_SERVER_IP_PROTOCOL: {
            if (length != 1) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = app_tcp_server_set_ip_mode(p_data[0]);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_TCP_SERVER_IP_PROTOCOL: {
            uint8_t ip_mode;
            app_tcp_server_get_ip_mode(&ip_mode);
            app_uart_send_response(cmd, &ip_mode, sizeof(ip_mode));
            break;
        }

        case APP_CMD_SET_TCP_SERVER_PORT: {
            if (length != 2) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            uint16_t port = P2V(uint16_t, p_data);
            esp_err_t err = app_tcp_server_set_port(port);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_TCP_SERVER_PORT: {
            uint16_t port = 0;
            esp_err_t err = app_tcp_server_get_port(&port);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, (uint8_t*)&port, sizeof(uint16_t));
            break;
        }

        case APP_CMD_SET_TCP_SERVER_SO_LINGER: {
            if (length != 4) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            int so_linger = P2V(int, p_data);
            esp_err_t err = app_tcp_server_set_so_linger(so_linger);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_TCP_SERVER_SO_LINGER: {
            int so_linger = 0;
            esp_err_t err = app_tcp_server_get_so_linger(&so_linger);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, (uint8_t *)&so_linger, sizeof(so_linger));
            break;
        }

        case APP_CMD_SET_TCP_SERVER_NODELAY: {
            if (length != 1) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            uint8_t enable = p_data[0];
            esp_err_t err = app_tcp_server_set_tcp_nodelay(enable);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_TCP_SERVER_NODELAY: {
            uint8_t enable = 0;
            esp_err_t err = app_tcp_server_get_tcp_nodelay(&enable);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &enable, 1);
            break;
        }

        case APP_CMD_SET_TCP_SERVER_SO_SNDTIMEO: {
            if (length != 4) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            int sndtimeo = P2V(int, p_data);
            esp_err_t err = app_tcp_server_set_so_sndtimeo(sndtimeo);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_TCP_SERVER_SO_SNDTIMEO: {
            int sndtimeo = 0;
            esp_err_t err = app_tcp_server_get_so_sndtimeo(&sndtimeo);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, (uint8_t *)&sndtimeo, sizeof(sndtimeo));
            break;
        }

        case APP_CMD_SET_TCP_SERVER_KEEP_ALIVE: {
            // Payload structure: enable(1 byte) + keep_idle(4 bytes) + keep_interval(4 bytes) + keep_count(4 bytes) = 13 bytes
            if (length != 13) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            
            struct KeepalivePayload {
                uint8_t enable;
                int32_t keep_idle;
                int32_t keep_interval;
                int32_t keep_count;
            } PACKED *payload = (struct KeepalivePayload *)p_data;

            esp_err_t err = app_tcp_server_set_keep_alive(payload->enable, (int)payload->keep_idle, (int)payload->keep_interval, (int)payload->keep_count);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_TCP_SERVER_KEEP_ALIVE: {
            uint8_t enable = 0;
            int keep_idle = 0, keep_interval = 0, keep_count = 0;
            
            esp_err_t err = app_tcp_server_get_keep_alive(&enable, &keep_idle, &keep_interval, &keep_count);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            
            // Build response: enable(1) + idle(4) + interval(4) + count(4) = 13 bytes
            struct KeepalivePayload {
                uint8_t enable;
                int32_t keep_idle;
                int32_t keep_interval;
                int32_t keep_count;
            } PACKED resp;
            resp.enable = enable;
            resp.keep_idle = (int32_t)keep_idle;
            resp.keep_interval = (int32_t)keep_interval;
            resp.keep_count = (int32_t)keep_count;
            app_uart_send_response(cmd, (uint8_t*)&resp, sizeof(resp));
            break;
        }

        case APP_CMD_GET_TCP_SERVER_STATUS: {
            uint8_t state;
            esp_err_t err = app_tcp_server_get_state(&state);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &state, sizeof(uint8_t));
            break;
        }

        case APP_CMD_START_TCP_SERVER: {
            esp_err_t err = app_tcp_server_start();
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_STOP_TCP_SERVER: {
            esp_err_t err = app_tcp_server_stop();
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_SET_TCP_CLIENT_IP_ADDR: {
            // IP address is a string, max 48 bytes
            if (length == 0 || length > 48) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            // Call the IP address setter
            esp_err_t err = app_tcp_client_set_ip((const char*)p_data);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_TCP_CLIENT_IP_ADDR: {
            // Allow one extra byte for null terminator; usable content is 48 bytes
            char ip_str[49] = {0};
            // Retrieve IP string; assume the API fills a valid null-terminated string
            esp_err_t err = app_tcp_client_get_ip(ip_str, sizeof(ip_str));
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            // Calculate actual send length (excluding null terminator)
            size_t send_len = strlen_or0(ip_str, 48);
            app_uart_send_response(cmd, (uint8_t*)ip_str, send_len);
            break;
        }

        case APP_CMD_SET_TCP_CLIENT_PORT: {
            if (length != 2) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            uint16_t port = P2V(uint16_t, p_data);
            // NOTE: app_tcp_client_set_port requires the IP setter to be called first
            // to determine IPv4 vs IPv6; otherwise the port may not be set correctly
            esp_err_t err = app_tcp_client_set_port(port);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_TCP_CLIENT_PORT: {
            uint16_t port = 0;
            esp_err_t err = app_tcp_client_get_port(&port);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, (uint8_t*)&port, sizeof(uint16_t));
            break;
        }

        case APP_CMD_SET_TCP_CLIENT_SO_LINGER: {
            if (length != 4) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            int so_linger = P2V(int, p_data);
            esp_err_t err = app_tcp_client_set_so_linger(so_linger);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_TCP_CLIENT_SO_LINGER: {
            int so_linger = 0;
            esp_err_t err = app_tcp_client_get_so_linger(&so_linger);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, (uint8_t *)&so_linger, sizeof(so_linger));
            break;
        }

        case APP_CMD_SET_TCP_CLIENT_NODELAY: {
            if (length != 1) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            uint8_t enable = p_data[0];
            esp_err_t err = app_tcp_client_set_tcp_nodelay(enable);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_TCP_CLIENT_NODELAY: {
            uint8_t enable = 0;
            esp_err_t err = app_tcp_client_get_tcp_nodelay(&enable);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &enable, 1);
            break;
        }

        case APP_CMD_SET_TCP_CLIENT_SO_SNDTIMEO: {
            if (length != 4) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            int sndtimeo = P2V(int, p_data);
            esp_err_t err = app_tcp_client_set_so_sndtimeo(sndtimeo);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_TCP_CLIENT_SO_SNDTIMEO: {
            int sndtimeo = 0;
            esp_err_t err = app_tcp_client_get_so_sndtimeo(&sndtimeo);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, (uint8_t *)&sndtimeo, sizeof(sndtimeo));
            break;
        }

        case APP_CMD_SET_TCP_CLIENT_KEEP_ALIVE: {
            // Payload structure: enable(1 byte) + keep_idle(4 bytes) + keep_interval(4 bytes) + keep_count(4 bytes) = 13 bytes
            if (length != 13) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            
            struct KeepalivePayload {
                uint8_t enable;
                int32_t keep_idle;
                int32_t keep_interval;
                int32_t keep_count;
            } PACKED *payload = (struct KeepalivePayload *)p_data;

            esp_err_t err = app_tcp_client_set_keep_alive(payload->enable, (int)payload->keep_idle, (int)payload->keep_interval, (int)payload->keep_count);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_TCP_CLIENT_KEEP_ALIVE: {
            uint8_t enable = 0;
            int keep_idle = 0, keep_interval = 0, keep_count = 0;
            
            esp_err_t err = app_tcp_client_get_keep_alive(&enable, &keep_idle, &keep_interval, &keep_count);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            
            // Build response: enable(1) + idle(4) + interval(4) + count(4) = 13 bytes
            struct KeepalivePayload {
                uint8_t enable;
                int32_t keep_idle;
                int32_t keep_interval;
                int32_t keep_count;
            } PACKED resp;
            resp.enable = enable;
            resp.keep_idle = (int32_t)keep_idle;
            resp.keep_interval = (int32_t)keep_interval;
            resp.keep_count = (int32_t)keep_count;
            app_uart_send_response(cmd, (uint8_t*)&resp, sizeof(resp));
            break;
        }

        case APP_CMD_GET_TCP_CLIENT_STATUS: {
            uint8_t state;
            esp_err_t err = app_tcp_client_get_state(&state);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &state, sizeof(uint8_t));
            break;
        }

        case APP_CMD_START_TCP_CLIENT: {
            esp_err_t err = app_tcp_client_start();
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_STOP_TCP_CLIENT: {
            esp_err_t err = app_tcp_client_stop();
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_SET_UDP_SERVER_IP_PROTOCOL: {
            if (length != 1) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = app_udp_server_set_ip_mode(p_data[0]);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_UDP_SERVER_IP_PROTOCOL: {
            uint8_t ip_mode;
            esp_err_t err = app_udp_server_get_ip_mode(&ip_mode);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &ip_mode, sizeof(ip_mode));
            break;
        }

        case APP_CMD_SET_UDP_SERVER_PORT: {
            if (length != 2) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            uint16_t port = P2V(uint16_t, p_data);
            esp_err_t err = app_udp_server_set_port(port);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_UDP_SERVER_PORT: {
            uint16_t port = 0;
            esp_err_t err = app_udp_server_get_port(&port);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, (uint8_t*)&port, sizeof(uint16_t));
            break;
        }

        case APP_CMD_SET_UDP_SERVER_SO_SNDTIMEO: {
            if (length != 4) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            int sndtimeo = P2V(int, p_data);
            esp_err_t err = app_udp_server_set_so_sndtimeo(sndtimeo);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_UDP_SERVER_SO_SNDTIMEO: {
            int sndtimeo = 0;
            esp_err_t err = app_udp_server_get_so_sndtimeo(&sndtimeo);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, (uint8_t*)&sndtimeo, sizeof(sndtimeo));
            break;
        }
        
        case APP_CMD_GET_UDP_SERVER_STATUS: {
            uint8_t state;
            esp_err_t err = app_udp_server_get_state(&state);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &state, sizeof(uint8_t));
            break;
        }

        case APP_CMD_SET_UDP_SERVER_CLIENT_IP_ADDR: {
            // IP address is a string, max 48 bytes
            if (length == 0 || length > 48) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            // Call the IP address setter
            esp_err_t err = app_udp_server_set_client_ip((const char*)p_data);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_UDP_SERVER_CLIENT_IP_ADDR: {
            // Allow one extra byte for null terminator; usable content is 48 bytes
            char ip_str[49] = {0};
            // Retrieve IP string; assume the API fills a valid null-terminated string
            esp_err_t err = app_udp_server_get_client_ip(ip_str, sizeof(ip_str));
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            // Calculate actual send length (excluding null terminator)
            size_t send_len = strlen_or0(ip_str, 48);
            app_uart_send_response(cmd, (uint8_t*)ip_str, send_len);
            break;
        }

        case APP_CMD_SET_UDP_SERVER_CLIENT_PORT: {
            if (length != 2) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            uint16_t port = P2V(uint16_t, p_data);
            esp_err_t err = app_udp_server_set_client_port(port);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_UDP_SERVER_CLIENT_PORT: {
            uint16_t port = 0;
            esp_err_t err = app_udp_server_get_client_port(&port);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, (uint8_t*)&port, sizeof(uint16_t));
            break;
        }

        case APP_CMD_START_UDP_SERVER: {
            esp_err_t err = app_udp_server_start();
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_STOP_UDP_SERVER: {
            esp_err_t err = app_udp_server_stop();
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_SET_UDP_CLIENT_IP_PROTOCOL: {
            if (length != 1) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = app_udp_client_set_ip_mode(p_data[0]);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_UDP_CLIENT_IP_PROTOCOL: {
            uint8_t ip_mode;
            esp_err_t err = app_udp_client_get_ip_mode(&ip_mode);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &ip_mode, sizeof(ip_mode));
            break;
        }

        case APP_CMD_SET_UDP_CLIENT_LOCAL_PORT: {
            if (length != 2) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            uint16_t port = P2V(uint16_t, p_data);
            esp_err_t err = app_udp_client_set_local_port(port);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_UDP_CLIENT_LOCAL_PORT: {
            uint16_t port = 0;
            esp_err_t err = app_udp_client_get_local_port(&port);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, (uint8_t*)&port, sizeof(uint16_t));
            break;
        }

        case APP_CMD_SET_UDP_CLIENT_SO_SNDTIMEO: {
            if (length != 4) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            int sndtimeo = P2V(int, p_data);
            esp_err_t err = app_udp_client_set_so_sndtimeo(sndtimeo);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_UDP_CLIENT_SO_SNDTIMEO: {
            int sndtimeo = 0;
            esp_err_t err = app_udp_client_get_so_sndtimeo(&sndtimeo);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, (uint8_t *)&sndtimeo, sizeof(sndtimeo));
            break;
        }

        case APP_CMD_GET_UDP_CLIENT_STATUS: {
            uint8_t state;
            esp_err_t err = app_udp_client_get_state(&state);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &state, sizeof(uint8_t));
            break;
        }

        case APP_CMD_SET_UDP_CLIENT_SERVER_IP_ADDR: {
            if (length == 0 || length > 48) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = app_udp_client_set_server_ip((const char*)p_data);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_UDP_CLIENT_SERVER_IP_ADDR: {
            char ip_str[49] = {0};
            esp_err_t err = app_udp_client_get_server_ip(ip_str, sizeof(ip_str));
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            size_t send_len = strlen_or0(ip_str, 48);
            app_uart_send_response(cmd, (uint8_t*)ip_str, send_len);
            break;
        }

        case APP_CMD_SET_UDP_CLIENT_SERVER_PORT: {
            if (length != 2) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            uint16_t port = P2V(uint16_t, p_data);
            esp_err_t err = app_udp_client_set_server_port(port);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_UDP_CLIENT_SERVER_PORT: {
            uint16_t port = 0;
            esp_err_t err = app_udp_client_get_server_port(&port);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, (uint8_t*)&port, sizeof(uint16_t));
            break;
        }

        case APP_CMD_START_UDP_CLIENT: {
            esp_err_t err = app_udp_client_start();
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_STOP_UDP_CLIENT: {
            esp_err_t err = app_udp_client_stop();
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_SET_MQTT_CLIENT_HOST: {
            if (length > MQTT_HOST_MAX_LEN) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = set_str_or_null(app_mqtt_set_host, p_data, length);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_HOST: {
            char *host;
            esp_err_t err = app_mqtt_get_host(&host);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            size_t send_len = strlen_or0(host, MQTT_HOST_MAX_LEN);
            app_uart_send_response(cmd, (uint8_t*)host, send_len);
            break;
        }

        case APP_CMD_SET_MQTT_CLIENT_PORT: {
            if (length != 2) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            uint16_t port = P2V(uint16_t, p_data);
            esp_err_t err = app_mqtt_set_port(port);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_PORT: {
            uint16_t port = 0;
            esp_err_t err = app_mqtt_get_port(&port);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, (uint8_t*)&port, sizeof(uint16_t));
            break;
        }

        case APP_CMD_SET_MQTT_CLIENT_PATH: {
            if (length > MQTT_PATH_MAX_LEN) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = set_str_or_null(app_mqtt_set_path, p_data, length);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_PATH: {
            char *path;
            esp_err_t err = app_mqtt_get_path(&path);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            size_t send_len = strlen_or0(path, MQTT_PATH_MAX_LEN);
            app_uart_send_response(cmd, (uint8_t*)path, send_len);
            break;
        }

        case APP_CMD_SET_MQTT_CLIENT_SCHEME: {
            if (length != 1) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            uint8_t scheme = (uint8_t)p_data[0];
            esp_err_t err = app_mqtt_set_scheme(scheme);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_SCHEME: {
            uint8_t scheme;
            esp_err_t err = app_mqtt_get_scheme(&scheme);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &scheme, sizeof(scheme));
            break;
        }

        case APP_CMD_SET_MQTT_CLIENT_SUBSCRIBE_TOPIC: {
            if (length > MQTT_SUBSCRIBE_TOPIC_MAX_LEN) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = set_str_or_null(app_mqtt_set_subscribe_topic, p_data, length);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_SUBSCRIBE_TOPIC: {
            char *subscribe_topic;
            esp_err_t err = app_mqtt_get_subscribe_topic(&subscribe_topic);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            size_t send_len = strlen_or0(subscribe_topic, MQTT_SUBSCRIBE_TOPIC_MAX_LEN);
            app_uart_send_response(cmd, (uint8_t*)subscribe_topic, send_len);
            break;
        }

        case APP_CMD_SET_MQTT_CLIENT_SUBSCRIBE_QOS: {
            if (length != 1) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            uint8_t qos = p_data[0];
            esp_err_t err = app_mqtt_set_subscribe_qos(qos);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_SUBSCRIBE_QOS: {
            uint8_t qos;
            esp_err_t err = app_mqtt_get_subscribe_qos(&qos);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &qos, sizeof(qos));
            break;
        }

        case APP_CMD_SET_MQTT_CLIENT_PUBLISH_TOPIC: {
            if (length > MQTT_PUBLISH_TOPIC_MAX_LEN) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = set_str_or_null(app_mqtt_set_publish_topic, p_data, length);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_PUBLISH_TOPIC: {
            char *publish_topic;
            esp_err_t err = app_mqtt_get_publish_topic(&publish_topic);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            size_t send_len = strlen_or0(publish_topic, MQTT_PUBLISH_TOPIC_MAX_LEN);
            app_uart_send_response(cmd, (uint8_t*)publish_topic, send_len);
            break;
        }

        case APP_CMD_SET_MQTT_CLIENT_PUBLISH_QOS: {
            if (length != 1) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            uint8_t qos = p_data[0];
            esp_err_t err = app_mqtt_set_publish_qos(qos);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_PUBLISH_QOS: {
            uint8_t qos;
            esp_err_t err = app_mqtt_get_publish_qos(&qos);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &qos, sizeof(qos));
            break;
        }

        case APP_CMD_SET_MQTT_CLIENT_PUBLISH_RETAIN: {
            if (length != 1) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            uint8_t retain = p_data[0];
            esp_err_t err = app_mqtt_set_publish_retain(retain);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_PUBLISH_RETAIN: {
            uint8_t retain;
            esp_err_t err = app_mqtt_get_publish_retain(&retain);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &retain, sizeof(retain));
            break;
        }

        case APP_CMD_SET_MQTT_CLIENT_CLIENT_ID: {
            if (length > MQTT_CLIENT_ID_MAX_LEN) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = set_str_or_null(app_mqtt_set_client_id, p_data, length);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_CLIENT_ID: {
            char *client_id;
            esp_err_t err = app_mqtt_get_client_id(&client_id);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            size_t send_len = strlen_or0(client_id, MQTT_CLIENT_ID_MAX_LEN);
            app_uart_send_response(cmd, (uint8_t*)client_id, send_len);
            break;
        }

        case APP_CMD_SET_MQTT_CLIENT_USERNAME: {
            if (length > MQTT_USERNAME_MAX_LEN) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = set_str_or_null(app_mqtt_set_username, p_data, length);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_USERNAME: {
            char *username;
            esp_err_t err = app_mqtt_get_username(&username);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            size_t send_len = strlen_or0(username, MQTT_USERNAME_MAX_LEN);
            app_uart_send_response(cmd, (uint8_t*)username, send_len);
            break;
        }

        case APP_CMD_SET_MQTT_CLIENT_PASSWORD: {
            if (length > MQTT_PASSWORD_MAX_LEN) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = set_str_or_null(app_mqtt_set_password, p_data, length);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_PASSWORD: {
            char *password;
            esp_err_t err = app_mqtt_get_password(&password);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            size_t send_len = strlen_or0(password, MQTT_PASSWORD_MAX_LEN);
            app_uart_send_response(cmd, (uint8_t*)password, send_len);
            break;
        }

        case APP_CMD_SET_MQTT_CLIENT_KEEP_ALIVE: {
            if (length != 4) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            int keep_alive = P2V(int, p_data);
            esp_err_t err = app_mqtt_set_keepalive(keep_alive);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_KEEP_ALIVE: {
            int keep_alive = 0;
            esp_err_t err = app_mqtt_get_keepalive(&keep_alive);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, (uint8_t*)&keep_alive, sizeof(keep_alive));
            break;
        }

        case APP_CMD_SET_MQTT_CLIENT_DISABLE_CLEAN_SESSION: {
            if (length != 1) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = app_mqtt_set_disable_clean_session(p_data[0]);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_DISABLE_CLEAN_SESSION: {
            uint8_t disable_clean_session;
            esp_err_t err = app_mqtt_get_disable_clean_session(&disable_clean_session);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &disable_clean_session, sizeof(disable_clean_session));
            break;
        }

        case APP_CMD_SET_MQTT_CLIENT_LWT_TOPIC: {
            if (length > MQTT_LWT_TOPIC_MAX_LEN) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = set_str_or_null(app_mqtt_set_lwt_topic, p_data, length);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_LWT_TOPIC: {
            char *lwt_topic;
            esp_err_t err = app_mqtt_get_lwt_topic(&lwt_topic);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            size_t send_len = strlen_or0(lwt_topic, MQTT_LWT_TOPIC_MAX_LEN);
            app_uart_send_response(cmd, (uint8_t*)lwt_topic, send_len);
            break;
        }

        case APP_CMD_SET_MQTT_CLIENT_LWT_MESSAGE: {
            if (length > MQTT_LWT_MESSAGE_MAX_LEN) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = set_str_or_null(app_mqtt_set_lwt_message, p_data, length);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_LWT_MESSAGE: {
            char *lwt_message;
            esp_err_t err = app_mqtt_get_lwt_message(&lwt_message);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            size_t send_len = strlen_or0(lwt_message, MQTT_LWT_MESSAGE_MAX_LEN);
            app_uart_send_response(cmd, (uint8_t*)lwt_message, send_len);
            break;
        }

        case APP_CMD_SET_MQTT_CLIENT_LWT_QOS: {
            if (length != 1) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            uint8_t lwt_qos = p_data[0];
            esp_err_t err = app_mqtt_set_lwt_qos(lwt_qos);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_LWT_QOS: {
            uint8_t lwt_qos;
            esp_err_t err = app_mqtt_get_lwt_qos(&lwt_qos);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &lwt_qos, sizeof(lwt_qos));
            break;
        }

        case APP_CMD_SET_MQTT_CLIENT_LWT_RETAIN: {
            if (length != 1) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            uint8_t lwt_retain = p_data[0];
            esp_err_t err = app_mqtt_set_lwt_retain(lwt_retain);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_LWT_RETAIN: {
            uint8_t lwt_retain;
            esp_err_t err = app_mqtt_get_lwt_retain(&lwt_retain);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &lwt_retain, sizeof(lwt_retain));
            break;
        }

        case APP_CMD_ADD_MQTT_CLIENT_ALPN: {
            if (length == 0 || length > MQTT_ALPN_STR_MAX_LEN) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = app_mqtt_alpn_add((const char*)p_data);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_DEL_MQTT_CLIENT_ALPN: {
            if (length != 1) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            // A single byte encodes the index of the ALPN to remove (0-based)
            esp_err_t err = app_mqtt_alpn_remove(p_data[0]);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_ALPN: {
            // Get ALPN list and return as a '\0'-separated string;
            // the current payload buffer should be sufficient
            char **alpn_list;
            uint8_t alpn_count;
            esp_err_t err = app_mqtt_alpn_get_list(&alpn_list, &alpn_count);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            // If alpn_count is 0, no ALPN has been added; return an empty response
            if (alpn_count == 0) {
                app_uart_send_response(cmd, NULL, 0);
                break;
            }
            // Iterate alpn_list, compute total length including '\0' separators,
            // then build a contiguous string
            int total_len = 0;
            for (uint8_t i = 0; i < alpn_count; i++) {
                total_len += strlen(alpn_list[i]) + 1; // length of each ALPN string plus one '\0' separator
            }
            // If total length exceeds the maximum sendable length, return an empty response
            if (total_len > MAX_PAYLOAD_LEN) {
                app_uart_send_response(cmd, NULL, 0);
                break;
            }
            // Build a contiguous string in the format "alpn1\0alpn2\0alpn3\0"
            char *response_buf = (char*)malloc(total_len);
            if (response_buf == NULL) { // memory allocation failed; return empty response
                app_uart_send_response(cmd, NULL, 0);
                break;
            }
            // Use response_buf as the build buffer; copy each ALPN string and append '\0' separator
            char *ptr = response_buf;
            for (uint8_t i = 0; i < alpn_count; i++) {
                size_t len = strlen(alpn_list[i]);
                memcpy(ptr, alpn_list[i], len);
                ptr += len;  // advance pointer past the string
                *ptr = '\0'; // append separator
                ptr++;       // advance to next position for the next string
            }
            app_uart_send_response(cmd, (uint8_t*)response_buf, total_len);
            free(response_buf);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_ALPN_COUNT: {
            uint8_t alpn_count;
            esp_err_t err = app_mqtt_alpn_get_count(&alpn_count);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &alpn_count, sizeof(alpn_count));
            break;
        }

        case APP_CMD_CLEAR_MQTT_CLIENT_ALPN: {
            esp_err_t err = app_mqtt_alpn_clear();
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_SET_MQTT_CLIENT_SNI_HOST: {
            if (length > MQTT_SNI_HOST_MAX_LEN) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = set_str_or_null(app_mqtt_set_sni_host, p_data, length);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_SNI_HOST: {
            char *sni_host;
            esp_err_t err = app_mqtt_get_sni_host(&sni_host);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            size_t send_len = strlen_or0(sni_host, MQTT_SNI_HOST_MAX_LEN);
            app_uart_send_response(cmd, (uint8_t*)sni_host, send_len);
            break;
        }

        case APP_CMD_SET_MQTT_CLIENT_CACERT: {
            if (length > MQTT_CACERT_MAX_LEN) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = set_str_or_null(app_mqtt_set_cacert, p_data, length);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_CACERT: {
            char *cacert;
            esp_err_t err = app_mqtt_get_cacert(&cacert);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            size_t send_len = strlen_or0(cacert, MQTT_CACERT_MAX_LEN);
            app_uart_send_response(cmd, (uint8_t*)cacert, send_len);
            break;
        }

        case APP_CMD_SET_MQTT_CLIENT_CCERT: {
            if (length > MQTT_CCERT_MAX_LEN) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = set_str_or_null(app_mqtt_set_ccert, p_data, length);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_CCERT: {
            char *ccert;
            esp_err_t err = app_mqtt_get_ccert(&ccert);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            size_t send_len = strlen_or0(ccert, MQTT_CCERT_MAX_LEN);
            app_uart_send_response(cmd, (uint8_t*)ccert, send_len);
            break;
        }

        case APP_CMD_SET_MQTT_CLIENT_CCKEY: {
            if (length > MQTT_CCKEY_MAX_LEN) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = set_str_or_null(app_mqtt_set_cckey, p_data, length);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_CCKEY: {
            char *cckey;
            esp_err_t err = app_mqtt_get_cckey(&cckey);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            size_t send_len = strlen_or0(cckey, MQTT_CCKEY_MAX_LEN);
            app_uart_send_response(cmd, (uint8_t*)cckey, send_len);
            break;
        }

        case APP_CMD_SET_MQTT_CLIENT_PSK_DATA: {
            if (length > MQTT_PSK_KEY_MAX_LEN) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            // app_mqtt_set_psk_key takes binary data (not a string) and the setter
            // signature has additional parameters, so set_str_or_null cannot be used here
            esp_err_t err;
            if (length == 0) {
                err = app_mqtt_set_psk_key(NULL, 0);
            } else {
                err = app_mqtt_set_psk_key(p_data, length);
            }
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_PSK_DATA: {
            uint8_t *psk_data;
            size_t psk_len = 0;
            esp_err_t err = app_mqtt_get_psk_key(&psk_data, &psk_len);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            if (psk_len > MQTT_PSK_KEY_MAX_LEN) {
                psk_len = 0; // If actual length exceeds the maximum, treat it as a retrieval failure and return empty
            }
            app_uart_send_response(cmd, psk_data, psk_len);
            break;
        }

        case APP_CMD_SET_MQTT_CLIENT_PSK_HINT: {
            if (length > MQTT_PSK_HINT_MAX_LEN) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = set_str_or_null(app_mqtt_set_psk_hint, p_data, length);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_PSK_HINT: {
            char *psk_hint;
            esp_err_t err = app_mqtt_get_psk_hint(&psk_hint);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            size_t send_len = strlen_or0(psk_hint, MQTT_PSK_HINT_MAX_LEN);
            app_uart_send_response(cmd, (uint8_t*)psk_hint, send_len);
            break;
        }

        case APP_CMD_GET_MQTT_CLIENT_STATUS: {
            uint8_t state;
            esp_err_t err = app_mqtt_get_state(&state);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &state, sizeof(state));
            break;
        }

        case APP_CMD_START_MQTT_CLIENT: {
            esp_err_t err = app_mqtt_start();
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_STOP_MQTT_CLIENT: {
            esp_err_t err = app_mqtt_stop();
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_SET_BLE_ADV_MFG_DATA: {
            if (length > APP_BLE_ADV_MFG_DATA_MAX_LEN) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = app_ble_set_adv_mfg_data(p_data, length);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_BLE_ADV_MFG_DATA: {
            uint8_t mfg_len = 0;
            uint8_t mfg_buf[APP_BLE_ADV_MFG_DATA_MAX_LEN] = {0};
            esp_err_t err = app_ble_get_adv_mfg_data(mfg_buf, sizeof(mfg_buf), &mfg_len);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, mfg_buf, mfg_len);
            break;
        }

        case APP_CMD_SET_BLE_DEVICE_NAME: {
            if (length == 0 || length > APP_BLE_DEVICE_NAME_MAX_LEN) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = app_ble_set_device_name((const char*)p_data);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_BLE_DEVICE_NAME: {
            char dev_name[APP_BLE_DEVICE_NAME_MAX_LEN] = {0};
            esp_err_t err = app_ble_get_device_name(dev_name, sizeof(dev_name));
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            size_t send_len = strlen_or0(dev_name, APP_BLE_DEVICE_NAME_MAX_LEN);
            app_uart_send_response(cmd, (uint8_t*)dev_name, send_len);
            break;
        }

        case APP_CMD_SET_BLE_NOTIFY_RETRY_MAX: {
            if (length != 4) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            struct NotifyRetryPayload {
                uint16_t nomem_retry_max;
                uint16_t fail_retry_max;
            } PACKED *payload = (struct NotifyRetryPayload *)p_data;
            esp_err_t err = app_ble_set_notify_retry_max(payload->nomem_retry_max, payload->fail_retry_max);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_BLE_NOTIFY_RETRY_MAX: {
            uint16_t nomem_retry_max = 0;
            uint16_t fail_retry_max = 0;
            esp_err_t err = app_ble_get_notify_retry_max(&nomem_retry_max, &fail_retry_max);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            
            struct NotifyRetryPayload {
                uint16_t nomem_retry_max;
                uint16_t fail_retry_max;
            } PACKED resp;
            resp.nomem_retry_max = nomem_retry_max;
            resp.fail_retry_max = fail_retry_max;

            app_uart_send_response(cmd, (uint8_t*)&resp, sizeof(resp));
            break;
        }

        case APP_CMD_SET_BLE_DEVICE_ADDR: {
            if (length != BLE_DEV_ADDR_LEN) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = app_ble_set_device_addr(p_data);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_BLE_DEVICE_ADDR: {
            uint8_t addr[BLE_DEV_ADDR_LEN] = {0};
            esp_err_t err = app_ble_get_device_addr(addr);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, addr, sizeof(addr));
            break;
        }

        case APP_CMD_SET_BLE_BONDING_ENABLE: {
            if (length != 1) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = app_ble_set_bonding_enable(p_data[0]);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_BLE_BONDING_ENABLE: {
            uint8_t enable = 0;
            esp_err_t err = app_ble_get_bonding_enable(&enable);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &enable, sizeof(enable));
            break;
        }

        case APP_CMD_SET_BLE_BONDING_KEY: {
            if (length != 6 && length != 7) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            char passkey[7] = {0};
            memcpy(passkey, p_data, 6); // Copy content only; no null terminator needed
            esp_err_t err = app_ble_set_bonding_key(passkey);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_BLE_BONDING_KEY: {
            char passkey[7] = {0};
            esp_err_t err = app_ble_get_bonding_key(passkey, sizeof(passkey));
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, (uint8_t*)passkey, 6); // Return 6 bytes only; no null terminator
            break;
        }

        case APP_CMD_GET_BLE_BONDED_DEVICE_NUMS: {
            uint8_t count = 0;
            esp_err_t err = app_ble_get_bonded_count(&count);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &count, sizeof(count));
            break;
        }

        case APP_CMD_GET_BLE_BONDED_DEVICE_ADDR: {
            if (length != 1) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            uint8_t bonded_addr[BLE_DEV_ADDR_LEN + 1] = {0}; // Extra byte holds address type, so upper layer can distinguish public vs random
            esp_err_t err = app_ble_get_bonded_device_address(p_data[0], bonded_addr, &bonded_addr[BLE_DEV_ADDR_LEN]);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, bonded_addr, sizeof(bonded_addr));
            break;
        }

        case APP_CMD_DEL_BLE_BONDED_DEVICE: {
            if (length != 1) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = app_ble_del_bonded_device(p_data[0]);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_CLEAR_BLE_BONDED: {
            esp_err_t err = app_ble_clear_bonded();
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_SET_BLE_BATTERY_LEVEL: {
            if (length != 1) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            esp_err_t err = app_ble_set_battery_level(p_data[0]);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_BLE_BATTERY_LEVEL: {
            uint8_t level = 0;
            esp_err_t err = app_ble_get_battery_level(&level);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &level, sizeof(level));
            break;
        }

        case APP_CMD_SET_BLE_TX_POWER: {
            if (length != 2) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            struct TxPowerPayload {
                uint8_t type; // 0 = advertising power, 1 = connection power
                uint8_t power_level; // Power level; actual dBm mapping is defined by esp_power_level_t
            } PACKED *payload = (struct TxPowerPayload *)p_data;
            esp_err_t err = app_ble_set_tx_power(payload->type, payload->power_level);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_GET_BLE_TX_POWER: {
            if (length != 1) {
                uart_cmd_error_report(cmd, ESP_ERR_INVALID_ARG);
                break;
            }
            uint8_t tx_power = 0;
            esp_err_t err = app_ble_get_tx_power(p_data[0], &tx_power);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, (uint8_t*)&tx_power, sizeof(tx_power));
            break;
        }

        case APP_CMD_GET_BLE_SPP_STATUS: {
            uint8_t state = 0;
            esp_err_t err = app_ble_get_state(&state);
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, &state, sizeof(state));
            break;
        }

        case APP_CMD_START_BLE_SPP: {
            esp_err_t err = app_ble_start();
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        case APP_CMD_STOP_BLE_SPP: {
            esp_err_t err = app_ble_stop();
            if (err != ESP_OK) {
                uart_cmd_error_report(cmd, err);
                break;
            }
            app_uart_send_response(cmd, NULL, 0);
            break;
        }

        default:
            break;
    }
}

/**
 * @brief Initializes the NVS partition and prints current NVS usage statistics.
 * 
 */
static void app_nvs_flash_init(void) {
    // Initialize the NVS library; many persisted and calibration parameters depend on it
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Retrieve NVS partition statistics and log current usage to aid debugging and monitoring
    nvs_stats_t nvs_stats;
    esp_err_t err = nvs_get_stats(NULL, &nvs_stats);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "NVS Used(entries): %d, Free: %d, Total: %d",
            nvs_stats.used_entries,
            nvs_stats.free_entries,
            nvs_stats.total_entries);
        // Approximate free capacity in bytes (each entry is 32 bytes)
        ESP_LOGI(TAG, "NVS Free capacity(Approximate): %dbytes", nvs_stats.free_entries * 32);
    }
}

/**
 * @brief Loads certain configurations from NVS — primarily parameters set directly by this module,
 * or parameters that must be applied in app_main before the system fully starts,
 * such as the WiFi forwarding mode enable/disable state.
 */
static void app_nvs_flash_load(void) {
    esp_err_t err;

    // ----------------------------- Load TIMEZONE configuration -----------------------------
    char *timezone;
    err = settings_time_zone_load(&timezone);
    if (err == ESP_OK && timezone != NULL) {
        ESP_LOGI(TAG, "Loaded timezone from NVS: %s", timezone);
        settings_time_zone_apply(timezone);
        free(timezone); // The load function allocates memory for the string; free it after use to avoid leaks
    }

    // ----------------------------- Load WiFi function mode configuration -----------------------------
    int wifi_mode = 0;
    err = settings_wifi_mode_load(&wifi_mode, g_wifi_function_mode);
    if (err == ESP_OK) {
        g_wifi_function_mode = (wifi_function_mode_t)wifi_mode;
        ESP_LOGI(TAG, "Loaded wifi mode from NVS: %d", wifi_mode);
    }

    // ----------------------------- Load WiFi forward type configuration -----------------------------
    int fwd_type = WIFI_FORWARD_TCP_SERVER;
    err = settings_wifi_forward_type_load(&fwd_type, wifi_forward_type);
    if (err == ESP_OK) {
        wifi_forward_type = (wifi_forward_type_t)fwd_type;
        ESP_LOGI(TAG, "Loaded wifi forward type from NVS: %d", fwd_type);
    }

    // ----------------------------- Load WiFi TX power configuration -----------------------------
    int8_t tx_pwr;
    err = settings_wifi_tx_pwr_load(&tx_pwr, wifi_tx_power);
    if (err == ESP_OK) {
        wifi_tx_power = tx_pwr;
        ESP_LOGI(TAG, "Loaded wifi tx power from NVS: %d", tx_pwr);
    }

    // ----------------------------- Load WiFi inactive_time configuration -----------------------------
    uint16_t inactive_time; // default 6 seconds
    err = settings_wifi_inactive_time_load(&inactive_time, wifi_inactive_time);
    if (err == ESP_OK) {
        wifi_inactive_time = inactive_time;
        ESP_LOGI(TAG, "Loaded wifi inactive_time from NVS: %u", inactive_time);
    }

    // ----------------------------- Load WiFi DHCP enable configuration -----------------------------
    uint8_t dhcp_enable = 1; // enabled by default
    err = settings_wifi_dhcp_enable_load(&dhcp_enable, wifi_dhcp_enable);
    if (err == ESP_OK) {
        wifi_dhcp_enable = dhcp_enable;
        ESP_LOGI(TAG, "Loaded wifi dhcp_enable from NVS: %u", dhcp_enable);
    }

    // ----------------------------- Load WiFi MAC address configuration -----------------------------
    uint8_t *sta_mac;
    err = settings_wifi_mac_addr_load(&sta_mac);
    if (err == ESP_OK && sta_mac != NULL) {
        memcpy(wifi_sta_mac, sta_mac, sizeof(wifi_sta_mac)); // Copy into global variable
        free(sta_mac); // Free allocated memory to avoid leaks
    } else {
        esp_read_mac(wifi_sta_mac, ESP_MAC_WIFI_STA); // No MAC stored in NVS; read default (usually burned in eFuse)
    }
    ESP_LOGI(TAG, "Loaded wifi mac from NVS: %02x:%02x:%02x:%02x:%02x:%02x", 
            wifi_sta_mac[0], wifi_sta_mac[1], wifi_sta_mac[2], wifi_sta_mac[3], wifi_sta_mac[4], wifi_sta_mac[5]);

    // ----------------------------- Load WiFi IP address configuration -----------------------------
    uint32_t *ipinfo;
    err = settings_wifi_ip_addr_load(&ipinfo);
    if (err == ESP_OK && ipinfo != NULL) {
        // Copy into global variable for later use.
        // The ipinfo array holds IP, gateway, and netmask in order, each as a uint32_t IPv4 address.
        memcpy(wifi_ip_info, ipinfo, sizeof(wifi_ip_info));
        
        // Stored in little-endian order, so the highest byte is at the highest address;
        // shift right to extract each octet in the correct order.
        ESP_LOGI(TAG, "Loaded wifi ip info from NVS: ip=%u.%u.%u.%u, gateway=%u.%u.%u.%u, netmask=%u.%u.%u.%u", 
            ipinfo[0] & 0xFF, (ipinfo[0] >> 8) & 0xFF, (ipinfo[0] >> 16) & 0xFF, (ipinfo[0] >> 24) & 0xFF,
            ipinfo[1] & 0xFF, (ipinfo[1] >> 8) & 0xFF, (ipinfo[1] >> 16) & 0xFF, (ipinfo[1] >> 24) & 0xFF,
            ipinfo[2] & 0xFF, (ipinfo[2] >> 8) & 0xFF, (ipinfo[2] >> 16) & 0xFF, (ipinfo[2] >> 24) & 0xFF);
            
        free(ipinfo); // Free allocated memory to avoid leaks
    }

    // ----------------------------- Load WiFi hostname configuration -----------------------------
    char *host_name = NULL;
    err = settings_wifi_host_name_load(&host_name);
    if (err == ESP_OK && host_name) {
        uint8_t len = MIN(strlen(host_name), sizeof(wifi_host_name) - 1); // Clamp to global buffer size, reserving one byte for null terminator
        memcpy(wifi_host_name, host_name, len); // Copy into global variable for later use
        wifi_host_name[len] = '\0'; // Ensure null termination
        ESP_LOGI(TAG, "Loaded wifi host name from NVS: %s", wifi_host_name);
        free(host_name);
    }
}

/**
 * @brief Application entry point.
 * Initializes all modules, optionally applies stored configuration, sets system state
 * to ready, then waits for incoming commands.
 */
void app_main(void) {
    // Use esp_random() output as seed to initialize the stdlib PRNG,
    // ensuring a different random sequence each boot
    srand(esp_random());

    // Initialize the NVS partition; must be called before any NVS-dependent functionality
    app_nvs_flash_init();

    // Load persisted module configuration before fully starting the system
    app_nvs_flash_load();

    // Initialize the UART port used for command TX/RX
    ESP_ERROR_CHECK(app_uart_init());
    ESP_ERROR_CHECK(app_uart_set_command_callback(on_uart_cmd_complete));
    ESP_ERROR_CHECK(app_uart_set_baudrate_change_callback(on_uart_cmd_baudrate_change));

    // DO NOT use ESP_ERROR_CHECK for the following section.
    // If any module fails to initialize, ESP_ERROR_CHECK would trigger an infinite reboot,
    // preventing the device from entering normal operation to receive commands.
    // We must ensure the device never truly bricks.

    // After UART is ready, initialize the log forwarding module
    ESP_ERROR_CHECK_WITHOUT_ABORT(app_log_uart_init());
    ESP_ERROR_CHECK_WITHOUT_ABORT(app_log_uart_set_tx_callback(on_log_printf_uart_report));

    // Always initialize the BLE forwarding module and keep it running;
    // it stays available until a command switches the forwarding mode or sends data
    ESP_ERROR_CHECK_WITHOUT_ABORT(app_ble_init());
    ESP_ERROR_CHECK_WITHOUT_ABORT(app_ble_set_rx_callback(on_forward_data_received));
    ESP_ERROR_CHECK_WITHOUT_ABORT(app_ble_start()); // Final BLE module start

    // Initialize the WiFi forwarding module based on the stored configuration
    if (g_wifi_function_mode == WIFI_FUNCTION_MODE_WIFI_FORWARD) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(wifi_forward_common_init());
        // On power-up DHCP is enabled by default; only disable it when explicitly required
        // (must be done before starting the connection)
        if (false == wifi_dhcp_enable) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(app_wifi_cfg_set_dhcp_enable(false));
            // Static IP is only effective when DHCP is disabled; apply only in that case
            ESP_ERROR_CHECK_WITHOUT_ABORT(app_wifi_cfg_set_ipv4(wifi_ip_info[0], wifi_ip_info[1], wifi_ip_info[2]));
        }
        ESP_ERROR_CHECK_WITHOUT_ABORT(app_wifi_cfg_set_host_name(wifi_host_name));
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_mac(WIFI_IF_STA, wifi_sta_mac));
        ESP_ERROR_CHECK_WITHOUT_ABORT(app_wifi_connect_start());
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_max_tx_power(wifi_tx_power));
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_inactive_time(WIFI_IF_STA, wifi_inactive_time));
    }

    // Mark system as ready; safe to receive and process commands from this point
    system_ready = true;
}
