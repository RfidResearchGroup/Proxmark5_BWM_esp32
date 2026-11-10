#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "app_tcp_client.h"
#include "app_netutils.h"
#include "app_nvs_rw.h"
#include "app_rtos_task.h"


#define NAMESPACE_TCP_CLIENT           "app_tcp_client"
#define KEY_TCP_IP                     "tcp_ip"
#define KEY_TCP_PORT                   "tcp_port"
#define KEY_TCP_KEEPALIVE_ENABLE       "tcp_ka_en"
#define KEY_TCP_KEEPALIVE_IDLE         "tcp_ka_idle"
#define KEY_TCP_KEEPALIVE_INTERVAL     "tcp_ka_intv"
#define KEY_TCP_KEEPALIVE_COUNT        "tcp_ka_cnt"
#define KEY_TCP_SO_LINGER              "tcp_so_linger"
#define KEY_TCP_NODELAY                "tcp_nodelay"
#define KEY_TCP_SNDTIMEO               "tcp_sndtimeo"

#define TAG                             "tcp_client"
#define DEFAULT_DEST_PORT               7891
#define DEFAULT_IP_MODE                 0 // 0: IPv4, 1: IPv6
#define DEFAULT_IPV4_STR                "192.168.2.174"
#define DEFAULT_IPV6_STR                "::1"
#define DEFAULT_KEEPALIVE_ENABLE        1
#define DEFAULT_KEEPALIVE_IDLE          60
#define DEFAULT_KEEPALIVE_INTERVAL      1
#define DEFAULT_KEEPALIVE_COUNT         3
#define DEFAULT_SO_LINGER_VALUE         -1
#define DEFAULT_TCP_NO_DELAY_ENABLE     0
#define DEFAULT_SO_SNDTIMEO_VALUE       -1
#define RECONNECT_DELAY_MS              2000

typedef struct {
    TaskHandle_t task_handle;
    SemaphoreHandle_t mutex_socket;
    SemaphoreHandle_t mutex_state;
    volatile int socket_handle;
    volatile bool task_running;
    volatile bool client_stop;
    volatile bool is_connected;
    app_tcp_client_rx_callback_t rx_callback;
    uint8_t rx_buffer[CONFIG_CMD_PAYLOAD_SIZE];
    struct sockaddr_storage dest_addr;
    uint8_t keep_alive_enable;
    int keep_idle;
    int keep_interval;
    int keep_count;
    int so_linger;
    uint8_t tcp_nodelay;
    int sndtimeo;
} app_tcp_client_ctx_t;

static app_tcp_client_ctx_t *s_ctx = NULL;

// Helper: convert milliseconds to timeval
static inline void ms_to_timeval(int timeout_ms, struct timeval *tv) {
    tv->tv_sec = timeout_ms / 1000;
    tv->tv_usec = (timeout_ms % 1000) * 1000;
}

/**
 * @brief Creates a TCP client context.
 * 
 * @return app_tcp_client_ctx_t* 
 */
static app_tcp_client_ctx_t *app_tcp_client_ctx_create(void) {
    app_tcp_client_ctx_t *ctx = calloc(1, sizeof(app_tcp_client_ctx_t));
    if (ctx == NULL) {
        return NULL;
    }

    ctx->socket_handle = -1;
    ctx->client_stop = true;
    ctx->keep_alive_enable = DEFAULT_KEEPALIVE_ENABLE;
    ctx->keep_idle = DEFAULT_KEEPALIVE_IDLE;
    ctx->keep_interval = DEFAULT_KEEPALIVE_INTERVAL;
    ctx->keep_count = DEFAULT_KEEPALIVE_COUNT;
    ctx->so_linger = DEFAULT_SO_LINGER_VALUE;
    ctx->tcp_nodelay = DEFAULT_TCP_NO_DELAY_ENABLE;
    ctx->sndtimeo = DEFAULT_SO_SNDTIMEO_VALUE;

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

    memset(&ctx->dest_addr, 0, sizeof(ctx->dest_addr));
    return ctx;
}

/**
 * @brief Destroys a TCP client context.
 * 
 * @param ctx TCP client context
 */
static void app_tcp_client_ctx_destroy(app_tcp_client_ctx_t *ctx) {
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
 * @brief Applies socket options to the current socket.
 * 
 * Configures KeepAlive, Linger, Nodelay, and SndTimeo settings.
 */
static void tcp_client_set_opt(app_tcp_client_ctx_t *ctx) {
    if (ctx == NULL || ctx->socket_handle < 0) return;

    int optval;
    
    // 1. Keep-Alive
    optval = ctx->keep_alive_enable;
    if (setsockopt(ctx->socket_handle, SOL_SOCKET, SO_KEEPALIVE, &optval, sizeof(int)) < 0) {
        ESP_LOGE(TAG, "setsockopt(SO_KEEPALIVE) failed: errno %d", errno);
    } else if (ctx->keep_alive_enable) {
        ESP_LOGI(TAG, "SO_KEEPALIVE enabled");
        setsockopt(ctx->socket_handle, IPPROTO_TCP, TCP_KEEPIDLE, &ctx->keep_idle, sizeof(int));
        setsockopt(ctx->socket_handle, IPPROTO_TCP, TCP_KEEPINTVL, &ctx->keep_interval, sizeof(int));
        setsockopt(ctx->socket_handle, IPPROTO_TCP, TCP_KEEPCNT, &ctx->keep_count, sizeof(int));
    }

    // 2. SO_LINGER
    struct linger so_linger_inst;
    if (ctx->so_linger == -1) {
        so_linger_inst.l_onoff = 0;
        so_linger_inst.l_linger = 0;
    } else {
        so_linger_inst.l_onoff = 1;
        so_linger_inst.l_linger = ctx->so_linger;
    }
    if (setsockopt(ctx->socket_handle, SOL_SOCKET, SO_LINGER, &so_linger_inst, sizeof(so_linger_inst)) < 0) {
        ESP_LOGE(TAG, "setsockopt(SO_LINGER) failed: errno %d", errno);
    } else {
        ESP_LOGI(TAG, "SO_LINGER set: onoff=%d, linger=%d", so_linger_inst.l_onoff, so_linger_inst.l_linger);
    }

    // 3. TCP_NODELAY
    optval = ctx->tcp_nodelay;
    if (setsockopt(ctx->socket_handle, IPPROTO_TCP, TCP_NODELAY, &optval, sizeof(optval)) < 0) {
        ESP_LOGE(TAG, "setsockopt(TCP_NODELAY) failed: errno %d", errno);
    } else {
        ESP_LOGI(TAG, "TCP_NODELAY set: %d", optval);
    }

    // 4. SO_SNDTIMEO
    if (ctx->sndtimeo != -1) {
        struct timeval tv = {};
        ms_to_timeval(ctx->sndtimeo, &tv);
        if (setsockopt(ctx->socket_handle, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0) {
            ESP_LOGE(TAG, "setsockopt(SO_SNDTIMEO) failed: errno %d", errno);
        } else {
            ESP_LOGI(TAG, "SO_SNDTIMEO set: %d ms", ctx->sndtimeo);
        }
    }
}

/**
 * @brief Safely closes the socket.
 */
static void close_client_socket(app_tcp_client_ctx_t *ctx) {
    if (ctx == NULL || ctx->mutex_socket == NULL) {
        return;
    }

    if (xSemaphoreTake(ctx->mutex_socket, portMAX_DELAY) == pdTRUE) {
        if (ctx->socket_handle != -1) {
            shutdown(ctx->socket_handle, 0);
            close(ctx->socket_handle);
            ctx->socket_handle = -1;
            ctx->is_connected = false;
            ESP_LOGI(TAG, "Socket closed");
        }
        xSemaphoreGive(ctx->mutex_socket);
    }
}

/**
 * @brief Performs a single connection attempt.
 * 
 * Connects using the address previously resolved and stored in ctx->dest_addr.
 * Automatically selects IPv4 or IPv6 based on ctx->dest_addr.ss_family.
 */
static bool tcp_client_connect(app_tcp_client_ctx_t *ctx) {
    int ip_protocol = 0;
    int err = 0;
    socklen_t addr_size = 0;

    if (ctx == NULL) {
        return false;
    }

    if (IS_IPV4(&ctx->dest_addr)) {
        ip_protocol = IPPROTO_IP;
        addr_size = sizeof(struct sockaddr_in);
        ESP_LOGI(TAG, "Connecting using IPv4, port %d", ntohs(SOCKIN4(&ctx->dest_addr)->sin_port));
    }
#if CONFIG_LWIP_IPV6
    else if (IS_IPV6(&ctx->dest_addr)) {
        ip_protocol = IPPROTO_IPV6;
        addr_size = sizeof(struct sockaddr_in6);
        ESP_LOGI(TAG, "Connecting using IPv6, port %d", ntohs(SOCKIN6(&ctx->dest_addr)->sin6_port));
    }
#endif
    else {
        ESP_LOGE(TAG, "Invalid address family stored: %d", ctx->dest_addr.ss_family);
        return false;
    }

    ESP_LOGI(TAG, "Attempting to connect to server...");

    ctx->socket_handle = socket(ctx->dest_addr.ss_family, SOCK_STREAM, ip_protocol);
    if (ctx->socket_handle < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return false;
    }

    err = connect(ctx->socket_handle, (struct sockaddr *)&ctx->dest_addr, addr_size);
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
        close_client_socket(ctx);
        return false;
    }

    ESP_LOGI(TAG, "Successfully connected");
    ctx->is_connected = true;
    
    tcp_client_set_opt(ctx);
    
    return true;
}

/**
 * @brief Data receive loop.
 */
static void tcp_client_data_rx_loop(app_tcp_client_ctx_t *ctx) {
    int len;
    if (ctx == NULL) {
        return;
    }

    while (ctx->is_connected && ctx->socket_handle >= 0) {
        len = recv(ctx->socket_handle, ctx->rx_buffer, CONFIG_CMD_PAYLOAD_SIZE, 0);
        
        if (len < 0) {
            ESP_LOGE(TAG, "Receive error: errno %d", errno);
            break; 
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed by server");
            break; 
        } else {
            if (ctx->rx_callback) {
                ctx->rx_callback(ctx->rx_buffer, len);
            }
            ESP_LOGD(TAG, "Received %d bytes", len);
        }
    }
}

/**
 * @brief TCP client main task.
 */
static void tcp_client_task(void *pvParameters) {
    app_tcp_client_ctx_t *ctx = (app_tcp_client_ctx_t *)pvParameters;

    if (ctx == NULL) {
        vTaskDelete(NULL);
        return;
    }

    while (ctx->task_running) {
        if (ctx->client_stop) {
            // Re-check after waking: start() also notifies when we are not waiting,
            // so a stale notification can be pending while stopped again
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }

        if (tcp_client_connect(ctx)) {
            // stop() during connect() may have missed the not yet published socket
            if (!ctx->client_stop) {
                tcp_client_data_rx_loop(ctx);
            }
            close_client_socket(ctx);
            
            if (ctx->task_running && !ctx->client_stop) {
                ESP_LOGW(TAG, "Connection lost. Reconnecting in %d ms...", RECONNECT_DELAY_MS);
                // Interruptible delay: deinit notifies, so it needn't wait it out
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(RECONNECT_DELAY_MS));
            }
        } else {
            if (ctx->task_running && !ctx->client_stop) {
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(RECONNECT_DELAY_MS));
            }
        }
    }

    close_client_socket(ctx);
    ctx->task_running = false;
    // Last ctx access: deinit frees ctx once it sees the handle cleared
    ctx->task_handle = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief Applies a new IP address string to the sockaddr_storage structure.
 * 
 * @param ip_str New IP address string
 * @return esp_err_t 
 */
static esp_err_t apply_ip_update_to_sockaddr_storage(char *ip_str) {
    // Preserve the existing port so it is not lost when the IP is updated
    uint16_t port;
    esp_err_t err = get_port_from_sockaddr_storage(&port, &s_ctx->dest_addr);
    if (err != ESP_OK) {
        port = DEFAULT_DEST_PORT;
    }
    // Parse new IP and update s_ctx->dest_addr
    err = get_sockaddr_storage_from_string(ip_str, port, &s_ctx->dest_addr, NULL);
    if (err != ESP_OK) {
        return err;
    }
    return ESP_OK;
}

/**
 * @brief Initialises the TCP client task.
 * 
 * Defaults to IPv4 mode.
 */
esp_err_t app_tcp_client_init(void) {
    if (s_ctx != NULL && s_ctx->task_running) {
        return ESP_OK;
    }

    if (s_ctx != NULL) {
        app_tcp_client_ctx_destroy(s_ctx);
        s_ctx = NULL;
    }

    s_ctx = app_tcp_client_ctx_create();
    if (s_ctx == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // Load parameters from NVS
    char* ip_buf = NULL;
    uint16_t port = 0;
    esp_err_t err = app_nvs_rw_read(NAMESPACE_TCP_CLIENT, (app_nvs_rw_read_item_t[]) {
        { .key = KEY_TCP_IP, .type = APP_NVS_RW_TYPE_STR, .data = &ip_buf, .default_value = 0 },
        { .key = KEY_TCP_PORT, .type = APP_NVS_RW_TYPE_U16, .data = &port, .default_value = DEFAULT_DEST_PORT },
        { .key = KEY_TCP_KEEPALIVE_ENABLE, .type = APP_NVS_RW_TYPE_U8, .data = &s_ctx->keep_alive_enable, .default_value = DEFAULT_KEEPALIVE_ENABLE },
        { .key = KEY_TCP_KEEPALIVE_IDLE, .type = APP_NVS_RW_TYPE_I32, .data = &s_ctx->keep_idle, .default_value = DEFAULT_KEEPALIVE_IDLE },
        { .key = KEY_TCP_KEEPALIVE_INTERVAL, .type = APP_NVS_RW_TYPE_I32, .data = &s_ctx->keep_interval, .default_value = DEFAULT_KEEPALIVE_INTERVAL },
        { .key = KEY_TCP_KEEPALIVE_COUNT, .type = APP_NVS_RW_TYPE_I32, .data = &s_ctx->keep_count, .default_value = DEFAULT_KEEPALIVE_COUNT },
        { .key = KEY_TCP_SO_LINGER, .type = APP_NVS_RW_TYPE_I32, .data = &s_ctx->so_linger, .default_value = DEFAULT_SO_LINGER_VALUE },
        { .key = KEY_TCP_NODELAY, .type = APP_NVS_RW_TYPE_U8, .data = &s_ctx->tcp_nodelay, .default_value = DEFAULT_TCP_NO_DELAY_ENABLE },
        { .key = KEY_TCP_SNDTIMEO, .type = APP_NVS_RW_TYPE_I32, .data = &s_ctx->sndtimeo, .default_value = DEFAULT_SO_SNDTIMEO_VALUE },
    }, 9);
    if (err != ESP_OK) { // Failed to load config; cannot create TCP client task
        free(ip_buf); // Free any partially loaded IP string
        app_tcp_client_ctx_destroy(s_ctx);
        s_ctx = NULL;
        return err;
    }

    // Print loaded parameter values, except IP address (may be NULL or invalid string)
    ESP_LOGI(TAG, "Loaded TCP client config: port=%d, keep_alive_enable=%d, keep_idle=%d, keep_interval=%d, keep_count=%d, so_linger=%d, tcp_nodelay=%d, sndtimeo=%d",
             port, s_ctx->keep_alive_enable, s_ctx->keep_idle, s_ctx->keep_interval, s_ctx->keep_count,
             s_ctx->so_linger, s_ctx->tcp_nodelay, s_ctx->sndtimeo);

    // IP address is a string; for STR/BLOB the default_value is NULL,
    // so fall back to the compile-time default if nothing was saved
    if (ip_buf == NULL) {
        ip_buf = DEFAULT_IP_MODE == 0 ? DEFAULT_IPV4_STR : DEFAULT_IPV6_STR;
        ESP_LOGI(TAG, "Using default IP address: %s", ip_buf);
        err = apply_ip_update_to_sockaddr_storage(ip_buf);
    } else {
        ESP_LOGI(TAG, "Loaded IP address from NVS: %s", ip_buf);
        err = apply_ip_update_to_sockaddr_storage(ip_buf);
        free(ip_buf); // app_nvs_rw_read allocates memory for the string; free it here
    }
    if (err != ESP_OK) {
        app_tcp_client_ctx_destroy(s_ctx);
        s_ctx = NULL;
        return err;
    }
    // Set port number
    err = set_port_in_sockaddr_storage(port, &s_ctx->dest_addr);
    if (err != ESP_OK) {
        app_tcp_client_ctx_destroy(s_ctx);
        s_ctx = NULL;
        return err;
    }

    s_ctx->task_running = true;
    s_ctx->client_stop = true;

    if (xTaskCreate(tcp_client_task, "tcp_client", 2048, s_ctx, 5, &s_ctx->task_handle) != pdPASS) {
        s_ctx->task_running = false;
        app_tcp_client_ctx_destroy(s_ctx);
        s_ctx = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

/**
 * @brief Deinitialises the TCP client task.
 */
esp_err_t app_tcp_client_deinit(void) {
    if (s_ctx != NULL) {
        if (s_ctx->task_running) {
            s_ctx->task_running = false;
            s_ctx->client_stop = true;

            if (s_ctx->task_handle != NULL) {
                xTaskNotifyGive(s_ctx->task_handle);
            }

            close_client_socket(s_ctx);

            // Never free ctx under a live task: force-deletes it on timeout
            if (s_ctx->task_handle != NULL) {
                wait_for_rtos_task_exit(2000, &s_ctx->task_handle);
            }
        }

        app_tcp_client_ctx_destroy(s_ctx);
        s_ctx = NULL;
    }
    return ESP_OK;
}

/**
 * @brief Starts the TCP client connection.
 */
esp_err_t app_tcp_client_start(void) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_ctx->mutex_state == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_ctx->mutex_state, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (s_ctx->client_stop) {
        s_ctx->client_stop = false;
        xTaskNotifyGive(s_ctx->task_handle);
    }

    xSemaphoreGive(s_ctx->mutex_state);
    return ESP_OK;
}

/**
 * @brief Stops the TCP client connection.
 */
esp_err_t app_tcp_client_stop(void) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_ctx->mutex_state == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_ctx->mutex_state, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (!s_ctx->client_stop) {
        s_ctx->client_stop = true;
        close_client_socket(s_ctx);
        xSemaphoreGive(s_ctx->mutex_state);
        return ESP_OK;
    }

    xSemaphoreGive(s_ctx->mutex_state);
    return ESP_OK;
}

/**
 * @brief Sends data.
 */
esp_err_t app_tcp_client_send(uint8_t *data, size_t length) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL || s_ctx->socket_handle < 0) {
        return ESP_ERR_INVALID_STATE;
    }

    int to_write = length;
    while (to_write > 0) {
        int written = send(s_ctx->socket_handle, data + (length - to_write), to_write, 0);
        if (written < 0) {
            ESP_LOGE(TAG, "Send error: errno %d", errno);
            // Drop the link: after a partial write the stream would continue out of frame;
            // the task reconnects
            close_client_socket(s_ctx);
            return ESP_FAIL;
        }
        to_write -= written;
    }
    return ESP_OK;
}

/**
 * @brief Registers a receive callback.
 */
esp_err_t app_tcp_client_set_rx_callback(app_tcp_client_rx_callback_t callback) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE;
    s_ctx->rx_callback = callback;
    return ESP_OK;
}

/**
 * @brief Gets the current connection state.
 */
esp_err_t app_tcp_client_get_state(uint8_t *state) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE;
    
    if (s_ctx->client_stop) {
        *state = 0; 
    } else {
        *state = s_ctx->is_connected ? 2 : 1;
    }
    return ESP_OK;
}

/**
 * @brief Sets the target IP address (string form).
 * 
 * On success the parsed binary address is stored in ctx->dest_addr
 * and used by get_ip.
 * 
 * @param ip_str IP address string (e.g. "192.168.1.5" or "2001:db8::1")
 * @return esp_err_t 
 */
esp_err_t app_tcp_client_set_ip(const char *ip_str) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL || ip_str == NULL) return ESP_ERR_INVALID_STATE;

    // Parse and apply the new IP address into s_ctx->dest_addr
    esp_err_t err = apply_ip_update_to_sockaddr_storage((char *)ip_str);
    if (err != ESP_OK) {
        return err;
    }

    // Persist configuration to NVS
    return app_nvs_rw_write(NAMESPACE_TCP_CLIENT, (app_nvs_rw_write_item_t[]) {
        { .key = KEY_TCP_IP, .type = APP_NVS_RW_TYPE_STR, .data = ip_str, .length = strlen(ip_str) + 1 }
    }, 1);
}

/**
 * @brief Sets the target port.
 */
esp_err_t app_tcp_client_set_port(uint16_t port) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE;
    esp_err_t err = set_port_in_sockaddr_storage(port, &s_ctx->dest_addr);
    if (err == ESP_OK) {
        return app_nvs_rw_write(NAMESPACE_TCP_CLIENT, (app_nvs_rw_write_item_t[]) {
            { .key = KEY_TCP_PORT, .type = APP_NVS_RW_TYPE_U16, .data = &port, .length = sizeof(port) }
        }, 1);
    }
    return err;
}

/**
 * @brief Sets SO_LINGER.
 */
esp_err_t app_tcp_client_set_so_linger(int so_linger) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE;
    s_ctx->so_linger = so_linger;
    return app_nvs_rw_write(NAMESPACE_TCP_CLIENT, (app_nvs_rw_write_item_t[]) {
        { .key = KEY_TCP_SO_LINGER, .type = APP_NVS_RW_TYPE_I32, .data = &so_linger, .length = sizeof(so_linger) }
    }, 1);
}

/**
 * @brief Sets TCP_NODELAY.
 */
esp_err_t app_tcp_client_set_tcp_nodelay(uint8_t enable) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE;
    s_ctx->tcp_nodelay = enable;
    return app_nvs_rw_write(NAMESPACE_TCP_CLIENT, (app_nvs_rw_write_item_t[]) {
        { .key = KEY_TCP_NODELAY, .type = APP_NVS_RW_TYPE_U8, .data = &enable, .length = sizeof(enable) }
    }, 1);
}

/**
 * @brief Sets SO_SNDTIMEO.
 */
esp_err_t app_tcp_client_set_so_sndtimeo(int sndtimeo) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE;
    s_ctx->sndtimeo = sndtimeo;
    return app_nvs_rw_write(NAMESPACE_TCP_CLIENT, (app_nvs_rw_write_item_t[]) {
        { .key = KEY_TCP_SNDTIMEO, .type = APP_NVS_RW_TYPE_I32, .data = &sndtimeo, .length = sizeof(sndtimeo) }
    }, 1);
}

/**
 * @brief Sets keep-alive parameters.
 */
esp_err_t app_tcp_client_set_keep_alive(uint8_t enable, int keep_idle, int keep_interval, int keep_count) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE;
    s_ctx->keep_alive_enable = enable;
    s_ctx->keep_idle = keep_idle;
    s_ctx->keep_interval = keep_interval;
    s_ctx->keep_count = keep_count;
    return app_nvs_rw_write(NAMESPACE_TCP_CLIENT, (app_nvs_rw_write_item_t[]) {
        { .key = KEY_TCP_KEEPALIVE_ENABLE, .type = APP_NVS_RW_TYPE_U8, .data = &enable, .length = sizeof(enable) },
        { .key = KEY_TCP_KEEPALIVE_IDLE, .type = APP_NVS_RW_TYPE_I32, .data = &keep_idle, .length = sizeof(keep_idle) },
        { .key = KEY_TCP_KEEPALIVE_INTERVAL, .type = APP_NVS_RW_TYPE_I32, .data = &keep_interval, .length = sizeof(keep_interval) },
        { .key = KEY_TCP_KEEPALIVE_COUNT, .type = APP_NVS_RW_TYPE_I32, .data = &keep_count, .length = sizeof(keep_count) },
    }, 4);
}

/**
 * @brief Gets the configured IP address string.
 * 
 * @param ip_buf Output buffer
 * @param buf_len Buffer length
 * @return esp_err_t 
 */
esp_err_t app_tcp_client_get_ip(char *ip_buf, size_t buf_len) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL || ip_buf == NULL) return ESP_ERR_INVALID_STATE;
    return get_ipstr_from_sockaddr_storage(&s_ctx->dest_addr, ip_buf, buf_len);
}

/**
 * @brief Gets the configured target port.
 */
esp_err_t app_tcp_client_get_port(uint16_t *port) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE;
    return get_port_from_sockaddr_storage(port, &s_ctx->dest_addr);
}

/**
 * @brief Gets SO_LINGER.
 */
esp_err_t app_tcp_client_get_so_linger(int *so_linger) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE;
    *so_linger = s_ctx->so_linger;
    return ESP_OK;
}

/**
 * @brief Gets TCP_NODELAY.
 */
esp_err_t app_tcp_client_get_tcp_nodelay(uint8_t *enable) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE;
    *enable = s_ctx->tcp_nodelay;
    return ESP_OK;
}

/**
 * @brief Gets SO_SNDTIMEO.
 */
esp_err_t app_tcp_client_get_so_sndtimeo(int *sndtimeo) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE;
    *sndtimeo = s_ctx->sndtimeo;
    return ESP_OK;
}

/**
 * @brief Gets keep-alive configuration.
 */
esp_err_t app_tcp_client_get_keep_alive(uint8_t *enable, int *keep_idle, int *keep_interval, int *keep_count) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE;
    *enable = s_ctx->keep_alive_enable;
    *keep_idle = s_ctx->keep_idle;
    *keep_interval = s_ctx->keep_interval;
    *keep_count = s_ctx->keep_count;
    return ESP_OK;
}
