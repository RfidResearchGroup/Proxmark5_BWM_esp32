#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_rtos_task.h" 
#include "app_udp_client.h"
#include "app_netutils.h" 
#include "app_nvs_rw.h"


#define NAMESPACE_UDP_CLIENT           "app_udp_client"
#define KEY_UDP_LOCAL_PORT             "udp_local_port"
#define KEY_UDP_IP_MODE                "udp_ip_mode"
#define KEY_UDP_SNDTIMEO               "udp_sndtimeo"
#define KEY_UDP_SERVER_IP              "udp_svr_ip"
#define KEY_UDP_SERVER_PORT            "udp_svr_port"

#define TAG                             "udp_client"
#define DEFAULT_SERVER_PORT             7892
#define DEFAULT_LOCAL_PORT              0       // 0 means the system auto-assigns a local port
#define DEFAULT_IP_MODE                 0       // 0: IPv4, 1: IPv6
#define DEFAULT_SO_SNDTIMEO_VALUE       -1      // -1 means wait indefinitely


typedef struct {
    TaskHandle_t task_handle;
    SemaphoreHandle_t mutex_socket;
    SemaphoreHandle_t mutex_state;
    volatile int socket_client_handle;
    volatile bool task_running;
    volatile bool client_stop;
    app_udp_client_rx_callback_t rx_callback;
    uint16_t local_port;
    uint8_t rx_buffer[CONFIG_CMD_PAYLOAD_SIZE];
    uint8_t client_ip_mode;
    int client_sndtimeo;
    struct sockaddr_storage server_addr_target;
    bool is_target_set;
} app_udp_client_ctx_t;

static app_udp_client_ctx_t *s_ctx = NULL;

/**
 * @brief Helper: convert milliseconds to timeval structure
 * Used to set Socket timeout options
 */
static void ms_to_timeval(int timeout_ms, struct timeval *tv) {
    if (timeout_ms < 0) {
        tv->tv_sec = 0;
        tv->tv_usec = 0;
        return;
    }
    tv->tv_sec = timeout_ms / 1000;
    tv->tv_usec = (timeout_ms % 1000) * 1000;
}

static app_udp_client_ctx_t *app_udp_client_ctx_create(void) {
    app_udp_client_ctx_t *ctx = calloc(1, sizeof(app_udp_client_ctx_t));
    if (ctx == NULL) {
        return NULL;
    }

    ctx->socket_client_handle = -1;
    ctx->client_stop = true;
    ctx->local_port = DEFAULT_LOCAL_PORT;
    ctx->client_ip_mode = DEFAULT_IP_MODE;
    ctx->client_sndtimeo = DEFAULT_SO_SNDTIMEO_VALUE;

    ctx->mutex_socket = xSemaphoreCreateMutex();
    if (ctx->mutex_socket == NULL) {
        free(ctx);
        return NULL;
    }

    ctx->mutex_state = xSemaphoreCreateMutex();
    if (ctx->mutex_state == NULL) {
        vSemaphoreDelete(ctx->mutex_socket);
        free(ctx);
        return NULL;
    }

    memset(&ctx->server_addr_target, 0, sizeof(ctx->server_addr_target));
    return ctx;
}

static void app_udp_client_ctx_destroy(app_udp_client_ctx_t *ctx) {
    if (ctx == NULL) {
        return;
    }

    if (ctx->mutex_state != NULL) {
        vSemaphoreDelete(ctx->mutex_state);
    }

    if (ctx->mutex_socket != NULL) {
        vSemaphoreDelete(ctx->mutex_socket);
    }

    free(ctx);
}

/**
 * @brief Thread-safely close Socket
 * Prevent repeated closing or accessing invalid handles during task execution
 */
static void close_client_socket(app_udp_client_ctx_t *ctx) {
    if (ctx == NULL || ctx->mutex_socket == NULL) {
        return;
    }

    if (xSemaphoreTake(ctx->mutex_socket, portMAX_DELAY) == pdTRUE) {
        if (ctx->socket_client_handle != -1) {
            close(ctx->socket_client_handle);
            ctx->socket_client_handle = -1;
            ESP_LOGI(TAG, "UDP Client Socket closed");
        }
        xSemaphoreGive(ctx->mutex_socket);
    }
}

/**
 * @brief Internal function: create and configure UDP Socket
 * Initialize socket according to current IP mode (IPv4/IPv6) and port configuration
 * Set send/receive timeouts so the task can respond to stop signals
 */
static bool udp_client_create_socket(app_udp_client_ctx_t *ctx) {
    if (ctx == NULL) {
        return false;
    }

    struct sockaddr_storage local_addr;
    int ip_protocol = 0;
    int s_addr_family = AF_INET; 

#if CONFIG_LWIP_IPV6
    if (ctx->client_ip_mode == 1) {
        s_addr_family = AF_INET6;
    }
#endif

    // Initialize local address structure
    if (s_addr_family == AF_INET) {
        struct sockaddr_in *local_addr_ip4 = (struct sockaddr_in *)&local_addr;
        local_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        local_addr_ip4->sin_family = AF_INET;
        local_addr_ip4->sin_port = htons(ctx->local_port);
        ip_protocol = IPPROTO_IP;
    } 
#if CONFIG_LWIP_IPV6
    else if (s_addr_family == AF_INET6) {
        struct sockaddr_in6 *local_addr_ip6 = (struct sockaddr_in6 *)&local_addr;
        bzero(&local_addr_ip6->sin6_addr.un, sizeof(local_addr_ip6->sin6_addr.un));
        local_addr_ip6->sin6_family = AF_INET6;
        local_addr_ip6->sin6_port = htons(ctx->local_port);
        ip_protocol = IPPROTO_IPV6;
    }
#endif

    // Create socket
    ctx->socket_client_handle = socket(s_addr_family, SOCK_DGRAM, ip_protocol);
    if (ctx->socket_client_handle < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return false;
    }

    // Enable address reuse to allow fast rebind after restart
    int opt = 1;
    setsockopt(ctx->socket_client_handle, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

#if CONFIG_LWIP_IPV6
    if (s_addr_family == AF_INET6) {
        setsockopt(ctx->socket_client_handle, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
    }
#endif

    // Set send timeout
    if (ctx->client_sndtimeo >= 0) {
        struct timeval tv = {};
        ms_to_timeval(ctx->client_sndtimeo, &tv);
        setsockopt(ctx->socket_client_handle, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    }

    // Bind local port (if configured non-0 port)
    if (ctx->local_port > 0) {
        int err = bind(ctx->socket_client_handle, (struct sockaddr *)&local_addr, sizeof(local_addr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            close_client_socket(ctx);
            return false;
        }
        ESP_LOGI(TAG, "UDP Client bound to local port %d", ctx->local_port);
    } else {
        ESP_LOGI(TAG, "UDP Client created (Local port auto-assigned)");
    }

    return true;
}

/**
 * @brief UDP client main task
 * Creates socket and enters receive loop
 * Notifies upper layer via callback when data is received
 */
static void udp_client_task(void *pvParameters) {
    app_udp_client_ctx_t *ctx = (app_udp_client_ctx_t *)pvParameters;

    if (ctx == NULL) {
        vTaskDelete(NULL);
        return;
    }

    while (ctx->task_running) {
        // If currently stopped, wait for start signal
        if (ctx->client_stop) {
            if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != 1) {
                continue;
            }
        }

        // Try to create socket
        if (!udp_client_create_socket(ctx)) {
            vTaskDelay(pdMS_TO_TICKS(1000)); // Wait before retry after failure
            continue;
        }

        ESP_LOGI(TAG, "UDP Client listening for responses...");

        // Receive loop
        while (1) {
            if (ctx->client_stop || !ctx->task_running) break; // Check stop flag

            struct sockaddr_storage source_addr;
            socklen_t addr_len = sizeof(source_addr);
            
            // Receive data
            int len = recvfrom(ctx->socket_client_handle, ctx->rx_buffer, CONFIG_CMD_PAYLOAD_SIZE, 0, 
                               (struct sockaddr *)&source_addr, &addr_len);

            if (len < 0) {
                ESP_LOGE(TAG, "Recv error: errno %d", errno);
                break; // Critical error occurred, exit loop and rebuild Socket
            } else if (len > 0) {
                // Valid data received, trigger callback
                if (ctx->rx_callback) {
                    ctx->rx_callback(ctx->rx_buffer, len);
                }
            }
        }

        // Close Socket when loop ends or error occurs
        close_client_socket(ctx);
    }

    // Task exit cleanup
    ctx->task_running = false;
    ctx->client_stop = true;
    // Last ctx access: deinit frees ctx once it sees the handle cleared
    ctx->task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t app_udp_client_init(void) {
    if (s_ctx != NULL && s_ctx->task_running) return ESP_OK;

    if (s_ctx != NULL) {
        app_udp_client_ctx_destroy(s_ctx);
        s_ctx = NULL;
    }

    s_ctx = app_udp_client_ctx_create();
    if (s_ctx == NULL) {
        return ESP_ERR_NO_MEM;
    }

    char *server_ip = NULL;
    uint16_t server_port;
    // Load parameters from NVS
    esp_err_t err = app_nvs_rw_read(NAMESPACE_UDP_CLIENT, (app_nvs_rw_read_item_t[]) {
        { .key = KEY_UDP_LOCAL_PORT, .type = APP_NVS_RW_TYPE_U16, .data = &s_ctx->local_port, .default_value = DEFAULT_LOCAL_PORT },
        { .key = KEY_UDP_IP_MODE, .type = APP_NVS_RW_TYPE_U8, .data = &s_ctx->client_ip_mode, .default_value = DEFAULT_IP_MODE },
        { .key = KEY_UDP_SNDTIMEO, .type = APP_NVS_RW_TYPE_I32, .data = &s_ctx->client_sndtimeo, .default_value = DEFAULT_SO_SNDTIMEO_VALUE },
        { .key = KEY_UDP_SERVER_IP, .type = APP_NVS_RW_TYPE_STR, .data = &server_ip, .default_value = 0 },
        { .key = KEY_UDP_SERVER_PORT, .type = APP_NVS_RW_TYPE_U16, .data = &server_port, .default_value = DEFAULT_SERVER_PORT },
    }, 5);
    if (err != ESP_OK) {
        free(server_ip); // Free any partially loaded server IP string
        app_udp_client_ctx_destroy(s_ctx);
        s_ctx = NULL;
        return err;
    }

    // Print loaded parameter values except IP address (may be NULL)
    ESP_LOGI(TAG, "UDP Client config - Local Port: %d, IP Mode: %s, Send Timeout: %d ms, Server Port: %d",
             s_ctx->local_port,
             s_ctx->client_ip_mode == 0 ? "IPv4" : "IPv6",
             s_ctx->client_sndtimeo,
             server_port);

    // IP address is a string; default for STR/BLOB is NULL
    if (server_ip != NULL) {
        ESP_LOGI(TAG, "Loaded UDP Client target server IP from NVS: %s", server_ip);
        err = get_sockaddr_storage_from_string(server_ip, server_port, &s_ctx->server_addr_target, NULL);
        free(server_ip); // Release string memory allocated by NVS library
        // Persisted target is as valid as one set at runtime
        s_ctx->is_target_set = (err == ESP_OK);
    }
    if (err != ESP_OK) {
        app_udp_client_ctx_destroy(s_ctx);
        s_ctx = NULL;
        return err;
    }

    s_ctx->task_running = true;
    if (xTaskCreate(udp_client_task, "udp_client", 2048, s_ctx, 5, &s_ctx->task_handle) != pdPASS) {
        s_ctx->task_running = false;
        app_udp_client_ctx_destroy(s_ctx);
        s_ctx = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t app_udp_client_deinit(void) {
    if (s_ctx != NULL && s_ctx->task_running) {
        s_ctx->task_running = false;
        s_ctx->client_stop = true;
        if (s_ctx->task_handle != NULL) {
            xTaskNotifyGive(s_ctx->task_handle);
        }
        close_client_socket(s_ctx);
        
        if (s_ctx->task_handle != NULL) {
            wait_for_rtos_task_exit(2000, &s_ctx->task_handle);
        }
    }

    if (s_ctx != NULL) {
        app_udp_client_ctx_destroy(s_ctx);
        s_ctx = NULL;
    }
    return ESP_OK;
}

esp_err_t app_udp_client_start(void) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL || s_ctx->mutex_state == NULL) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_ctx->mutex_state, portMAX_DELAY) != pdTRUE) return ESP_FAIL;
    if (s_ctx->client_stop) {
        s_ctx->client_stop = false;
        xTaskNotifyGive(s_ctx->task_handle);
    }
    xSemaphoreGive(s_ctx->mutex_state);
    return ESP_OK;
}

esp_err_t app_udp_client_stop(void) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL || s_ctx->mutex_state == NULL) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_ctx->mutex_state, portMAX_DELAY) != pdTRUE) return ESP_FAIL;
    if (!s_ctx->client_stop) {
        s_ctx->client_stop = true;
        if (s_ctx->socket_client_handle != -1) {
            close_client_socket(s_ctx);
        }
        xSemaphoreGive(s_ctx->mutex_state);
        return ESP_OK;
    }
    xSemaphoreGive(s_ctx->mutex_state);
    return ESP_OK;
}

esp_err_t app_udp_client_send(uint8_t *data, size_t length) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL || s_ctx->socket_client_handle < 0) {
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    bool target_valid = s_ctx->is_target_set;
    struct sockaddr_storage target_addr;
    memcpy(&target_addr, &s_ctx->server_addr_target, sizeof(struct sockaddr_storage));

    if (!target_valid) {
        ESP_LOGW(TAG, "Send failed: Target server address not set.");
        return ESP_ERR_NOT_FOUND;
    }

    // The logic below is for debugging output of destination address and port
    // char ip_str[INET6_ADDRSTRLEN] = {0};
    // get_ipstr_from_sockaddr_storage(&target_addr, ip_str, sizeof(ip_str));
    // ESP_LOGI(TAG, "Target server ip address for sending: %s", ip_str);
    // uint16_t target_port = 0;
    // get_port_from_sockaddr_storage(&target_port, &target_addr);
    // ESP_LOGI(TAG, "Target server port for sending: %d", target_valid ? target_port : 0);

    // Execute send
    ssize_t written = sendto(s_ctx->socket_client_handle, data, length, 0, (struct sockaddr *)&target_addr, target_addr.s2_len);
    if (written < 0) {
        ESP_LOGE(TAG, "Sendto error: errno %d", errno);
        return ESP_FAIL;
    }

    if ((size_t)written != length) {
        ESP_LOGE(TAG, "Sendto partial write: expect %u, actual %d", (unsigned int)length, (int)written);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sent %u bytes to server.", (unsigned int)length);
    return ESP_OK;
}

esp_err_t app_udp_client_set_rx_callback(app_udp_client_rx_callback_t callback) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE;
    s_ctx->rx_callback = callback;
    return ESP_OK;
}

esp_err_t app_udp_client_get_state(uint8_t *state) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE;
    *state = s_ctx->client_stop ? 0 : 1;
    return ESP_OK;
}

esp_err_t app_udp_client_set_ip_mode(uint8_t mode) { 
    if(s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE; 
    if(mode > 1) return ESP_ERR_INVALID_ARG; 
    s_ctx->client_ip_mode = mode; 
    return app_nvs_rw_write(NAMESPACE_UDP_CLIENT, (app_nvs_rw_write_item_t[]){
        { .key = KEY_UDP_IP_MODE, .type = APP_NVS_RW_TYPE_U8, .data = &mode, .length = sizeof(mode) }
    }, 1);
}

esp_err_t app_udp_client_set_local_port(uint16_t port) { 
    if(s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE; 
    s_ctx->local_port = port; 
    return app_nvs_rw_write(NAMESPACE_UDP_CLIENT, (app_nvs_rw_write_item_t[]){
        { .key = KEY_UDP_LOCAL_PORT, .type = APP_NVS_RW_TYPE_U16, .data = &port, .length = sizeof(port) }
    }, 1);
}

esp_err_t app_udp_client_set_so_sndtimeo(int t) { 
    if(s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE; 
    s_ctx->client_sndtimeo = t; 
    return app_nvs_rw_write(NAMESPACE_UDP_CLIENT, (app_nvs_rw_write_item_t[]){
        { .key = KEY_UDP_SNDTIMEO, .type = APP_NVS_RW_TYPE_I32, .data = &t, .length = sizeof(t) }
    }, 1);
}

esp_err_t app_udp_client_set_server_ip(const char *ip_str) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL || !ip_str) return ESP_ERR_INVALID_STATE;
    if (strlen(ip_str) >= UDP_CLIENT_IP_STR_LEN) return ESP_ERR_INVALID_ARG;
    if (ip_str[0] == '\0') return ESP_ERR_INVALID_ARG;

    uint16_t current_port = 0;
    if (s_ctx->is_target_set) {
         get_port_from_sockaddr_storage(&current_port, &s_ctx->server_addr_target);
    } else {
        current_port = DEFAULT_SERVER_PORT;
    }

    esp_err_t err = get_sockaddr_storage_from_string(ip_str, current_port, &s_ctx->server_addr_target, NULL);
    if (err == ESP_OK) {
        s_ctx->is_target_set = true;
        ESP_LOGI(TAG, "Target server IP set to: %s", ip_str);
        return app_nvs_rw_write(NAMESPACE_UDP_CLIENT, (app_nvs_rw_write_item_t[]){
            { .key = KEY_UDP_SERVER_IP, .type = APP_NVS_RW_TYPE_STR, .data = ip_str, .length = strlen(ip_str) + 1 }
        }, 1);
    } else {
        ESP_LOGE(TAG, "Failed to parse target IP address: %d", err);
    }
    return err;
}

esp_err_t app_udp_client_set_server_port(uint16_t port) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE;
    if (port == 0) return ESP_ERR_INVALID_ARG;

    esp_err_t err = set_port_in_sockaddr_storage(port, &s_ctx->server_addr_target);
    if (err == ESP_OK && s_ctx->is_target_set) {
        ESP_LOGD(TAG, "Target server port updated to: %d", port);
        return app_nvs_rw_write(NAMESPACE_UDP_CLIENT, (app_nvs_rw_write_item_t[]){
            { .key = KEY_UDP_SERVER_PORT, .type = APP_NVS_RW_TYPE_U16, .data = &port, .length = sizeof(port) }
        }, 1);
    }
    return err;
}

esp_err_t app_udp_client_get_ip_mode(uint8_t *mode) { 
    if(s_ctx == NULL || s_ctx->task_handle == NULL || !mode) return ESP_ERR_INVALID_STATE; 
    *mode = s_ctx->client_ip_mode; 
    return ESP_OK; 
}

esp_err_t app_udp_client_get_local_port(uint16_t *port) { 
    if(s_ctx == NULL || s_ctx->task_handle == NULL || !port) return ESP_ERR_INVALID_STATE; 
    *port = s_ctx->local_port; 
    return ESP_OK; 
}

esp_err_t app_udp_client_get_so_sndtimeo(int *t) { 
    if(s_ctx == NULL || s_ctx->task_handle == NULL || !t) return ESP_ERR_INVALID_STATE; 
    *t = s_ctx->client_sndtimeo; 
    return ESP_OK; 
}

esp_err_t app_udp_client_get_server_ip(char *ip_str, size_t max_len) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL || !ip_str || max_len == 0) return ESP_ERR_INVALID_STATE;

    if (!s_ctx->is_target_set) {
        return ESP_ERR_NOT_FOUND;
    }

    //  IP 
    return get_ipstr_from_sockaddr_storage(&s_ctx->server_addr_target, ip_str, max_len);
}

esp_err_t app_udp_client_get_server_port(uint16_t *port) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL || !port) return ESP_ERR_INVALID_STATE;

    if (!s_ctx->is_target_set) {
        return ESP_ERR_NOT_FOUND;
    }

    // 
    return get_port_from_sockaddr_storage(port, &s_ctx->server_addr_target);
}
