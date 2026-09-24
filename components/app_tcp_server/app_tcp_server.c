#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "app_rtos_task.h"
#include "app_tcp_server.h"
#include "app_nvs_rw.h"


// NVS namespace and key definitions
#define NAMESPACE_TCP_SERVER           "app_tcp_server"
#define KEY_TCP_PORT                   "tcp_port"
#define KEY_TCP_IP_MODE                "tcp_ip_mode"
#define KEY_TCP_KEEPALIVE_ENABLE       "tcp_ka_en"
#define KEY_TCP_KEEPALIVE_IDLE         "tcp_ka_idle"
#define KEY_TCP_KEEPALIVE_INTERVAL     "tcp_ka_intv"
#define KEY_TCP_KEEPALIVE_COUNT        "tcp_ka_cnt"
#define KEY_TCP_SO_LINGER              "tcp_so_linger"
#define KEY_TCP_NODELAY                "tcp_nodelay"
#define KEY_TCP_SNDTIMEO               "tcp_sndtimeo"


/*
 * Our design requirements for TCP service
 * 1. Some configurations can be saved to NVS and loaded at init
 * 2. When switching to WiFi mode and detecting TCP server mode, can call init to initialize wrapper
 * 3. After connection and getting IP, create TCP server listening; start/stop functions must be non-blocking
 * 3. When WiFi disconnects, must be able to stop TCP server listening and close handle to reclaim resources
 * 4. Export getter/setter for key configurations
 * 5. Export data reception callback/send interface
 */


// LOG tag
#define TAG                             "tcp_server"
// Default port for TCP server socket binding
#define DEFAULT_SERVER_PORT             7891
// Keep-alive idle enable flag
#define DEFAULT_KEEPALIVE_ENABLE        1  // TCP keep-alive enable for Server is true required
// Keep-alive idle time
#define DEFAULT_KEEPALIVE_IDLE          60 // TCP keep-alive idle time(s)
// Keep-alive probe packet interval time
#define DEFAULT_KEEPALIVE_INTERVAL      1  // TCP keep-alive interval time(s)
// Keep-alive probe packet retry count
#define DEFAULT_KEEPALIVE_COUNT         3  // TCP keep-alive packet retry send counts
// Default so_linger configuration
#define DEFAULT_SO_LINGER_VALUE         -1 // Unit: seconds, default: -1, means don't configure this parameter
// Default tcp_nodelay configuration
#define DEFAULT_TCP_NO_DELAY_ENABLE     0  // Default disable TCP_NODELAY, reduce real-time performance for higher throughput
// Default sndtimeo configuration
#define DEFAULT_SO_SNDTIMEO_VALUE       -1 // Unit: milliseconds, default: -1, means don't configure this parameter
// Default IPv4 configuration
#define DEFAULT_IP_MODE                 0 // 0: IPv4, 1: IPv6

typedef struct {
    TaskHandle_t task_handle;
    SemaphoreHandle_t mutex_client_close;
    SemaphoreHandle_t mutex_server_close;
    SemaphoreHandle_t mutex_state;
    volatile int socket_server_handle;
    volatile int socket_client_handle;
    volatile bool task_running;
    volatile bool server_stop;
    app_tcp_server_rx_callback_t rx_callback;
    app_tcp_server_link_callback_t link_callback;
    uint16_t server_port;
    uint8_t rx_buffer[CONFIG_CMD_PAYLOAD_SIZE];
    uint8_t server_ip_mode;
    uint8_t keep_alive_enable;
    int keep_idle;
    int keep_interval;
    int keep_count;
    int so_linger;
    uint8_t tcp_nodelay;
    int sndtimeo;
} app_tcp_server_ctx_t;

static app_tcp_server_ctx_t *s_ctx = NULL;


// Convert milliseconds to timeval structure
static void ms_to_timeval(int timeout_ms, struct timeval *tv) {
    tv->tv_sec = timeout_ms / 1000;
    tv->tv_usec = (timeout_ms % 1000) * 1000;
}

static app_tcp_server_ctx_t *app_tcp_server_ctx_create(void) {
    app_tcp_server_ctx_t *ctx = calloc(1, sizeof(app_tcp_server_ctx_t));
    if (ctx == NULL) {
        return NULL;
    }

    ctx->socket_server_handle = -1;
    ctx->socket_client_handle = -1;
    ctx->server_stop = true;
    ctx->server_port = DEFAULT_SERVER_PORT;
    ctx->server_ip_mode = DEFAULT_IP_MODE;
    ctx->keep_alive_enable = DEFAULT_KEEPALIVE_ENABLE;
    ctx->keep_idle = DEFAULT_KEEPALIVE_IDLE;
    ctx->keep_interval = DEFAULT_KEEPALIVE_INTERVAL;
    ctx->keep_count = DEFAULT_KEEPALIVE_COUNT;
    ctx->so_linger = DEFAULT_SO_LINGER_VALUE;
    ctx->tcp_nodelay = DEFAULT_TCP_NO_DELAY_ENABLE;
    ctx->sndtimeo = DEFAULT_SO_SNDTIMEO_VALUE;

    ctx->mutex_client_close = xSemaphoreCreateMutex();
    if (ctx->mutex_client_close == NULL) {
        free(ctx);
        return NULL;
    }

    ctx->mutex_server_close = xSemaphoreCreateMutex();
    if (ctx->mutex_server_close == NULL) {
        vSemaphoreDelete(ctx->mutex_client_close);
        free(ctx);
        return NULL;
    }

    ctx->mutex_state = xSemaphoreCreateMutex();
    if (ctx->mutex_state == NULL) {
        vSemaphoreDelete(ctx->mutex_client_close);
        vSemaphoreDelete(ctx->mutex_server_close);
        free(ctx);
        return NULL;
    }

    return ctx;
}

static void app_tcp_server_ctx_destroy(app_tcp_server_ctx_t *ctx) {
    if (ctx == NULL) {
        return;
    }

    if (ctx->mutex_state != NULL) {
        vSemaphoreDelete(ctx->mutex_state);
    }

    if (ctx->mutex_client_close != NULL) {
        vSemaphoreDelete(ctx->mutex_client_close);
    }

    if (ctx->mutex_server_close != NULL) {
        vSemaphoreDelete(ctx->mutex_server_close);
    }

    free(ctx);
}

/**
 * @brief Set socket client parameters
 * 
 */
static void tcp_client_set_opt(app_tcp_server_ctx_t *ctx) {
    if (ctx == NULL || ctx->socket_client_handle < 0) {
        return;
    }

    int optval;
    // Set TCP keep-alive option
    optval = ctx->keep_alive_enable;
    if (setsockopt(ctx->socket_client_handle, SOL_SOCKET, SO_KEEPALIVE, &optval, sizeof(int)) < 0) {
        ESP_LOGE(TAG, "setsockopt(SO_KEEPALIVE) failed: errno %d", errno);
    } else {
        ESP_LOGI(TAG, "SO_KEEPALIVE set: %d", ctx->keep_alive_enable);
        if (ctx->keep_alive_enable) {
            if (setsockopt(ctx->socket_client_handle, IPPROTO_TCP, TCP_KEEPIDLE, &ctx->keep_idle, sizeof(int)) < 0) {
                ESP_LOGE(TAG, "setsockopt(TCP_KEEPIDLE) failed: errno %d", errno);
            } else {
                ESP_LOGI(TAG, "TCP_KEEPIDLE set: %d", ctx->keep_idle);
            }
            if (setsockopt(ctx->socket_client_handle, IPPROTO_TCP, TCP_KEEPINTVL, &ctx->keep_interval, sizeof(int)) < 0) {
                ESP_LOGE(TAG, "setsockopt(TCP_KEEPINTVL) failed: errno %d", errno);
            } else {
                ESP_LOGI(TAG, "TCP_KEEPINTVL set: %d", ctx->keep_interval);
            }
            if (setsockopt(ctx->socket_client_handle, IPPROTO_TCP, TCP_KEEPCNT, &ctx->keep_count, sizeof(int)) < 0) {
                ESP_LOGE(TAG, "setsockopt(TCP_KEEPCNT) failed: errno %d", errno);
            } else {
                ESP_LOGI(TAG, "TCP_KEEPCNT set: %d", ctx->keep_count);
            }
        }
    }
    // Set TCP SO_LINGER option
    struct linger so_linger_inst;
    if (ctx->so_linger == -1) {
        so_linger_inst.l_onoff = 0;
        so_linger_inst.l_linger = 0;
    } else {
        so_linger_inst.l_onoff = 1;
        so_linger_inst.l_linger = ctx->so_linger; // Linger time in seconds
    }
    if (setsockopt(ctx->socket_client_handle, SOL_SOCKET, SO_LINGER, &so_linger_inst, sizeof(so_linger_inst)) < 0) {
        ESP_LOGE(TAG, "setsockopt(SO_LINGER) failed: errno %d", errno);
    } else {
        ESP_LOGI(TAG, "SO_LINGER set: l_onoff=%d, l_linger=%d", so_linger_inst.l_onoff, so_linger_inst.l_linger);
    }
    // Set TCP NODELAY option
    optval = ctx->tcp_nodelay;
    if (setsockopt(ctx->socket_client_handle, IPPROTO_TCP, TCP_NODELAY, &optval, sizeof(optval)) < 0) {
        ESP_LOGE(TAG, "setsockopt(TCP_NODELAY) failed: errno %d", errno);
    } else {
        ESP_LOGI(TAG, "TCP_NODELAY set: %d", optval);
    }
    // Set TX timeout option
    if (ctx->sndtimeo != -1) {
        struct timeval tv = {};
        ms_to_timeval(ctx->sndtimeo, &tv); // Update tv structure
        if (setsockopt(ctx->socket_client_handle, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0) {
            ESP_LOGE(TAG, "setsockopt(SO_SNDTIMEO) failed: errno %d", errno);
        } else {
            ESP_LOGI(TAG, "SO_SNDTIMEO set: %d", ctx->sndtimeo);
        }
    }
}

/**
 * @brief Print socket connection address
 * 
 * @param source_addr 
 */
static void tcp_printf_address(struct sockaddr_storage *source_addr) {
    // Convert ip address to string
    char addr_str[128];
    if (source_addr->ss_family == PF_INET) {
        inet_ntoa_r(((struct sockaddr_in *)source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
    }
    if (source_addr->ss_family == PF_INET6) {
        inet6_ntoa_r(((struct sockaddr_in6 *)source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
    }
    ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);
}

/**
 * @brief Blocking receive socket data, process externally via callback after reception
 * 
 */
static void tcp_client_data_rx(app_tcp_server_ctx_t *ctx) {
    if (ctx == NULL) {
        return;
    }

    int len;
    do {
        len = recv(ctx->socket_client_handle, ctx->rx_buffer, CONFIG_CMD_PAYLOAD_SIZE, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
        } else {
            // If receive callback registered, notify of received data via callback
            if (ctx->rx_callback) {
                ctx->rx_callback(ctx->rx_buffer, len);
            }
            ESP_LOGI(TAG, "Recv from tcp client length = %d", len);
        }
    } while (len > 0);
}

/**
 * @brief Safely close old server handle in critical section
 * 
 */
static void close_server_socket(app_tcp_server_ctx_t *ctx) {
    if (ctx == NULL || ctx->mutex_server_close == NULL) {
        return;
    }

    if (xSemaphoreTake(ctx->mutex_server_close, portMAX_DELAY) == pdTRUE) {
        /* --- CRITICAL SECTION START --- */
        // After entering critical section, confirm again if service is closed, don't repeat close operation
        if (ctx->socket_server_handle != -1) {
            close(ctx->socket_server_handle);
            ctx->socket_server_handle = -1;
        }
        /* --- CRITICAL SECTION END --- */
        xSemaphoreGive(ctx->mutex_server_close);
    }
}

/**
 * @brief Safely close old client handle in critical section
 * 
 */
static void close_client_socket(app_tcp_server_ctx_t *ctx) {
    if (ctx == NULL || ctx->mutex_client_close == NULL) {
        return;
    }

    if (xSemaphoreTake(ctx->mutex_client_close, portMAX_DELAY) == pdTRUE) {
        /* --- CRITICAL SECTION START --- */
        // After entering critical section, confirm again if client is closed, don't repeat close operation
        if (ctx->socket_client_handle != -1) {
            shutdown(ctx->socket_client_handle, 0);
            close(ctx->socket_client_handle);
            ctx->socket_client_handle = -1;
        }
        /* --- CRITICAL SECTION END --- */
        xSemaphoreGive(ctx->mutex_client_close);
    }
}

/**
 * @brief Create a TCP server listening
 * 
 * @return true Creation succeeded, can continue waiting for connections
 * @return false Creation failed, s_socket_server_handle is -1
 */
static bool tcp_server_create(app_tcp_server_ctx_t *ctx) {
    if (ctx == NULL) {
        return false;
    }

    struct sockaddr_storage dest_addr;
    int ip_protocol = 0;
    int s_addr_family = AF_INET; // Default IPv4 mode

#if CONFIG_LWIP_IPV6 // If IPv6 enabled and current protocol configured as IPv6, force socket to AF_INET6 mode
    if (ctx->server_ip_mode == 1) {
        s_addr_family = AF_INET6;
    }
#endif

    // Determine the bind address and protocol family
    if (s_addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(ctx->server_port);
        ip_protocol = IPPROTO_IP;
    }
    
#if CONFIG_LWIP_IPV6
    if (s_addr_family == AF_INET6) {
        struct sockaddr_in6 *dest_addr_ip6 = (struct sockaddr_in6 *)&dest_addr;
        bzero(&dest_addr_ip6->sin6_addr.un, sizeof(dest_addr_ip6->sin6_addr.un));
        dest_addr_ip6->sin6_family = AF_INET6;
        dest_addr_ip6->sin6_port = htons(ctx->server_port);
        ip_protocol = IPPROTO_IPV6;
    }
#endif

    ESP_LOGI(TAG, "Socket server start");

    ctx->socket_server_handle = socket(s_addr_family, SOCK_STREAM, ip_protocol);
    if (ctx->socket_server_handle < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return false;
    }
    int opt = 1;
    // Set SO_REUSEADDR to avoid "Address already in use" errors
    setsockopt(ctx->socket_server_handle, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

#if CONFIG_LWIP_IPV6
    if (s_addr_family == AF_INET6) {
        // Single stack at runtime, don't consider simultaneous IPv4 and IPv6 binding
        setsockopt(ctx->socket_server_handle, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
    }
#endif

    ESP_LOGI(TAG, "Socket server created");

    int err = bind(ctx->socket_server_handle, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", s_addr_family);
        close_server_socket(ctx);
        return false;
    }

    ESP_LOGI(TAG, "Socket server bound, port %d", ctx->server_port);

    err = listen(ctx->socket_server_handle, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        close_server_socket(ctx);
        return false;
    }

    return true;
}

/**
 * @brief Task providing TCP server capability
 * 
 * @param pvParameters 
 */
static void tcp_server_task(void *pvParameters) {
    app_tcp_server_ctx_t *ctx = (app_tcp_server_ctx_t *)pvParameters;

    if (ctx == NULL) {
        vTaskDelete(NULL);
        return;
    }

    // Outer loop creates TCP server listening to ensure always provide TCP service
    while (ctx->task_running) {

        // If TCP service needs to pause, don't create service, quietly wait for start notification
        if (ctx->server_stop) {
            // Wait for notification indefinitely
            if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != 1) {
                continue; // If timeout, skip and continue next loop iteration
            }
        }

        // Create TCP service first, then can wait for connections
        bool create_ok = tcp_server_create(ctx);
        if (!create_ok) {
            // May have issues causing creation failure, best to delay before retrying
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Inner loop creates TCP client connection listening, only handle one client at a time
        while (1) {
            if (ctx->server_stop || !ctx->task_running) {
                break;
            }

            ESP_LOGI(TAG, "Socket client listening");

            struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            socklen_t addr_len = sizeof(source_addr);
            ctx->socket_client_handle = accept(ctx->socket_server_handle, (struct sockaddr *)&source_addr, &addr_len);
            if (ctx->socket_client_handle < 0) {
                ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
                break; // Accept failed: TCP service may be closed or network may be abnormal
            }

            ESP_LOGI(TAG, "Socket client connected");
            if (ctx->link_callback) {
                ctx->link_callback(true);
            }

            tcp_client_set_opt(ctx);
            tcp_printf_address(&source_addr);
            tcp_client_data_rx(ctx); // After entering this function, start blocking and listening for client data, exit when client disconnects
            close_client_socket(ctx);

            ESP_LOGI(TAG, "Socket client disconnected");
            if (ctx->link_callback) {
                ctx->link_callback(false);
            }
        }
        
        close_server_socket(ctx);
    }

    // If outer loop exits too, means user called deinit
    ctx->task_handle = NULL;
    ctx->task_running = false;
    ctx->server_stop = true;
    vTaskDelete(NULL);
}

/**
 * @brief Initialize TCP server task
 * 
 * @return esp_err_t 
 */
esp_err_t app_tcp_server_init(void) {
    if (s_ctx != NULL && s_ctx->task_running) {
        return ESP_OK;
    }

    if (s_ctx != NULL) {
        app_tcp_server_ctx_destroy(s_ctx);
        s_ctx = NULL;
    }

    s_ctx = app_tcp_server_ctx_create();
    if (s_ctx == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // Load parameters from NVS
    esp_err_t err = app_nvs_rw_read(NAMESPACE_TCP_SERVER, (app_nvs_rw_read_item_t[]) {
        { .key = KEY_TCP_PORT, .type = APP_NVS_RW_TYPE_U16, .data = &s_ctx->server_port, .default_value = DEFAULT_SERVER_PORT },
        { .key = KEY_TCP_IP_MODE, .type = APP_NVS_RW_TYPE_U8, .data = &s_ctx->server_ip_mode, .default_value = DEFAULT_IP_MODE },
        { .key = KEY_TCP_KEEPALIVE_ENABLE, .type = APP_NVS_RW_TYPE_U8, .data = &s_ctx->keep_alive_enable, .default_value = DEFAULT_KEEPALIVE_ENABLE },
        { .key = KEY_TCP_KEEPALIVE_IDLE, .type = APP_NVS_RW_TYPE_I32, .data = &s_ctx->keep_idle, .default_value = DEFAULT_KEEPALIVE_IDLE },
        { .key = KEY_TCP_KEEPALIVE_INTERVAL, .type = APP_NVS_RW_TYPE_I32, .data = &s_ctx->keep_interval, .default_value = DEFAULT_KEEPALIVE_INTERVAL },
        { .key = KEY_TCP_KEEPALIVE_COUNT, .type = APP_NVS_RW_TYPE_I32, .data = &s_ctx->keep_count, .default_value = DEFAULT_KEEPALIVE_COUNT },
        { .key = KEY_TCP_SO_LINGER, .type = APP_NVS_RW_TYPE_I32, .data = &s_ctx->so_linger, .default_value = DEFAULT_SO_LINGER_VALUE },
        { .key = KEY_TCP_NODELAY, .type = APP_NVS_RW_TYPE_U8, .data = &s_ctx->tcp_nodelay, .default_value = DEFAULT_TCP_NO_DELAY_ENABLE },
        { .key = KEY_TCP_SNDTIMEO, .type = APP_NVS_RW_TYPE_I32, .data = &s_ctx->sndtimeo, .default_value = DEFAULT_SO_SNDTIMEO_VALUE },
    }, 9);
    if (err != ESP_OK) { // Failed to load parameters, cannot create TCP service task
        app_tcp_server_ctx_destroy(s_ctx);
        s_ctx = NULL;
        return err;
    }

    // Print loaded parameter values
    ESP_LOGI(TAG, "TCP Server Config - Port: %d, IP Mode: %s, KeepAlive: %s, KeepIdle: %d, KeepInterval: %d, KeepCount: %d, SoLinger: %d, TCPNoDelay: %d, SndTimeo: %d",
        s_ctx->server_port,
        s_ctx->server_ip_mode == 0 ? "IPv4" : "IPv6",
        s_ctx->keep_alive_enable ? "Enabled" : "Disabled",
        s_ctx->keep_idle,
        s_ctx->keep_interval,
        s_ctx->keep_count,
        s_ctx->so_linger,
        s_ctx->tcp_nodelay,
        s_ctx->sndtimeo
    );

    s_ctx->task_running = true;
    if (xTaskCreate(tcp_server_task, "tcp_server", 2048, s_ctx, 5, &s_ctx->task_handle) != pdPASS) {
        s_ctx->task_running = false;
        app_tcp_server_ctx_destroy(s_ctx);
        s_ctx = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

/**
 * @brief Deinitialize TCP server task and reclaim resources
 * 
 * @return esp_err_t 
 */
esp_err_t app_tcp_server_deinit(void) {
    // 
    if (s_ctx != NULL && s_ctx->task_running) {
        // Set running flag to false, at this point TCP service task should know it needs to exit
        s_ctx->task_running = false;
        s_ctx->server_stop = true;
        // Task may be waiting for startup event, we need to let it exit waiting
        if (s_ctx->task_handle != NULL) {
            xTaskNotifyGive(s_ctx->task_handle);
        }
        close_client_socket(s_ctx);
        close_server_socket(s_ctx);
        // Wait for service task to exit and reclaim some of its own resources
        if (s_ctx->task_handle != NULL) {
            wait_for_rtos_task_exit(2000, &s_ctx->task_handle);
        }
    }

    if (s_ctx != NULL) {
        app_tcp_server_ctx_destroy(s_ctx);
        s_ctx = NULL;
    }

    return ESP_OK;
}

/**
 * @brief Start TCP service task, after starting will automatically begin creating SOCKET listening
 * Note: avoid blocking in start interface whenever possible
 * @return esp_err_t 
 */
esp_err_t app_tcp_server_start(void) {
    // Ensure TCP service task is initialized, if not initialized means init was never called
    if (s_ctx == NULL || s_ctx->task_handle == NULL || s_ctx->mutex_state == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_ctx->mutex_state, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    // Execute start transition only once
    if (s_ctx->server_stop == true) {
        // Set stop TCP service flag to false, otherwise service task won't start service
        s_ctx->server_stop = false;
        // Need to notify task to wake up, otherwise task may wait silently until timeout occurs much later
        xTaskNotifyGive(s_ctx->task_handle);
    }

    xSemaphoreGive(s_ctx->mutex_state);
    return ESP_OK;
}

/**
 * @brief Stop TCP service task, and close already opened handles to reclaim related resources
 * Note: blocking in stop affects WiFi reconnection speed, but typically less than blocking in start
 * @return esp_err_t 
 */
esp_err_t app_tcp_server_stop(void) {
    // Check if initialized
    if (s_ctx == NULL || s_ctx->task_handle == NULL || s_ctx->mutex_state == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_ctx->mutex_state, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    // Execute stop transition only once
    if (s_ctx->server_stop == false) {
        // Set TCP service stop flag so service task won't try to recreate service
        s_ctx->server_stop = true;
        // Close client handle first; blocking recv should exit and return to listening state
        if (s_ctx->socket_client_handle != -1) {
            close_client_socket(s_ctx);
        }
        // Close server handle again, theoretically this will completely end TCP service task's inner loop, then the stop flag set above will pause creating new TCP service listening
        if (s_ctx->socket_server_handle != -1) {
            close_server_socket(s_ctx);
        }
        xSemaphoreGive(s_ctx->mutex_state);
        return ESP_OK;
    }

    xSemaphoreGive(s_ctx->mutex_state);
    return ESP_OK;
}

/**
 * @brief Send data to client through SOCKET
 * 
 * @param data Data buffer
 * @param length Data length
 * @return esp_err_t 
 *  - ESP_ERR_INVALID_STATE: returned when state/connection is invalid
 *  - ESP_OK: returned on successful send
 *  - ESP_FAIL: returned on send failure; inspect errno for details
 */
esp_err_t app_tcp_server_send(uint8_t *data, size_t length) {
    // Check if initialized, if client connection is normal, if not normal don't proceed with send
    if (s_ctx == NULL || s_ctx->task_handle == NULL || s_ctx->socket_client_handle < 0) {
        return ESP_ERR_INVALID_STATE;
    }
    // send() can return less bytes than supplied length.
    // Walk-around for robust implementation.
    int to_write = length;
    while (to_write > 0) {
        int written = send(s_ctx->socket_client_handle, data + (length - to_write), to_write, 0);
        if (written < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            // Failed to retransmit, giving up
            return ESP_FAIL;
        }
        to_write -= written;
        ESP_LOGI(TAG, "Send to tcp client length = %d, remain = %d", written, to_write);
    }
    return ESP_OK;
}

/**
 * @brief Register data receive callback
 * 
 * @param callback Callback function reference
 * @return esp_err_t 
 */
esp_err_t app_tcp_server_set_rx_callback(app_tcp_server_rx_callback_t callback) {
    // Check if initialized
    if (s_ctx == NULL || s_ctx->task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    s_ctx->rx_callback = callback;
    return ESP_OK;
}

esp_err_t app_tcp_server_set_link_callback(app_tcp_server_link_callback_t callback) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    s_ctx->link_callback = callback;
    return ESP_OK;
}

/**
 * @brief Get current TCP server state
 * 
 * @param state State value
 *                      0: TCP server stopped; server/client parameters can be preconfigured
 *                      1: TCP server started with no client connected; configuration still allowed
 *                      2: TCP server started with client connected; config changes apply after reconnect
 * @return esp_err_t 
 */
esp_err_t app_tcp_server_get_state(uint8_t *state) {
    // Check if initialized
    if (s_ctx == NULL || s_ctx->task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_ctx->server_stop) {
         *state = 0;
    } else {
        if (s_ctx->socket_client_handle < 0) {
            *state = 1;
        } else {
            *state = 2;
        }
    }
    return ESP_OK;
}

/**
 * @brief  IP  (0: IPv4, 1: IPv6)
 * 
 * @param mode 0  IPv4, 1  IPv6
 * @return esp_err_t 
 */
esp_err_t app_tcp_server_set_ip_mode(uint8_t mode) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE;
    if (mode > 1) return ESP_ERR_INVALID_ARG;

    s_ctx->server_ip_mode = mode;
    ESP_LOGI(TAG, "IP Mode set to: %s", (mode == 0) ? "IPv4" : "IPv6");

    // NVS
    return app_nvs_rw_write(NAMESPACE_TCP_SERVER, (app_nvs_rw_write_item_t[]) {
        { .key = KEY_TCP_IP_MODE, .type = APP_NVS_RW_TYPE_U8, .data = &mode, .length = sizeof(mode) }
    }, 1);
}

/**
 * @brief  IP 
 * 
 * @param mode  (0: IPv4, 1: IPv6)
 * @return esp_err_t 
 */
esp_err_t app_tcp_server_get_ip_mode(uint8_t *mode) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL || mode == NULL) return ESP_ERR_INVALID_STATE;
    *mode = s_ctx->server_ip_mode;
    return ESP_OK;
}

/**
 * @brief Set TCP server listening port
 *
 * The new port is used by subsequent server starts/restarts and is persisted to NVS.
 *
 * @param port TCP port (0-65535)
 * @return esp_err_t
 */
esp_err_t app_tcp_server_set_port(uint16_t port) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_ctx->server_port = port;

    // NVS
    return app_nvs_rw_write(NAMESPACE_TCP_SERVER, (app_nvs_rw_write_item_t[]) {
        { .key = KEY_TCP_PORT, .type = APP_NVS_RW_TYPE_U16, .data = &port, .length = sizeof(port) }
    }, 1);
}

/**
 * @brief Get TCP server listening port
 *
 * @param port Output pointer for current TCP port (0-65535)
 * @return esp_err_t
 */
esp_err_t app_tcp_server_get_port(uint16_t *port) {
    // Check if initialized
    if (s_ctx == NULL || s_ctx->task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    *port = s_ctx->server_port;
    return ESP_OK;
}

/**
 * @brief Set SO_LINGER behavior
 *
 * @param so_linger = -1: do not apply SO_LINGER (system default)
 *                  = 0: force close with linger time 0
 *                  > 0: enable linger with timeout value in seconds
 * @return esp_err_t
 */
esp_err_t app_tcp_server_set_so_linger(int so_linger) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_ctx->so_linger = so_linger;

    // NVS
    return app_nvs_rw_write(NAMESPACE_TCP_SERVER, (app_nvs_rw_write_item_t[]) {
        { .key = KEY_TCP_SO_LINGER, .type = APP_NVS_RW_TYPE_I32, .data = &so_linger, .length = sizeof(so_linger) }
    }, 1);
}

/**
 * @brief Get SO_LINGER configuration value
 *
 * @param so_linger Output pointer for SO_LINGER value
 * @return esp_err_t
 */
esp_err_t app_tcp_server_get_so_linger(int *so_linger) {
    // Check if initialized
    if (s_ctx == NULL || s_ctx->task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    *so_linger = s_ctx->so_linger;
    return ESP_OK;
}

/**
 * @brief Set TCP_NODELAY option
 *
 * Enabling TCP_NODELAY disables Nagle's algorithm and sends packets without aggregation delay.
 *
 * @param enable 0: disable TCP_NODELAY (Nagle enabled)
 *               1: enable TCP_NODELAY (Nagle disabled)
 * @return esp_err_t
 */
esp_err_t app_tcp_server_set_tcp_nodelay(uint8_t enable) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_ctx->tcp_nodelay = enable;

    // NVS
    return app_nvs_rw_write(NAMESPACE_TCP_SERVER, (app_nvs_rw_write_item_t[]) {
        { .key = KEY_TCP_NODELAY, .type = APP_NVS_RW_TYPE_U8, .data = &enable, .length = sizeof(enable) }
    }, 1);
}

/**
 * @brief Get TCP_NODELAY option
 *
 * @param enable Output pointer for TCP_NODELAY setting
 * @return esp_err_t
 */
esp_err_t app_tcp_server_get_tcp_nodelay(uint8_t *enable) {
    // Check if initialized
    if (s_ctx == NULL || s_ctx->task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    *enable = s_ctx->tcp_nodelay;
    return ESP_OK;
}

/**
 * @brief Set SO_SNDTIMEO option
 *
 * @param sndtimeo = -1: do not apply SO_SNDTIMEO (system default)
 *                 = 0: set SO_SNDTIMEO to zero
 *                 > 0: set SO_SNDTIMEO to specified timeout value
 * @return esp_err_t
 */
esp_err_t app_tcp_server_set_so_sndtimeo(int sndtimeo) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_ctx->sndtimeo = sndtimeo;

    // NVS
    return app_nvs_rw_write(NAMESPACE_TCP_SERVER, (app_nvs_rw_write_item_t[]) {
        { .key = KEY_TCP_SNDTIMEO, .type = APP_NVS_RW_TYPE_I32, .data = &sndtimeo, .length = sizeof(sndtimeo) }
    }, 1);
}

/**
 * @brief Get SO_SNDTIMEO configuration value
 *
 * @param sndtimeo Output pointer for SO_SNDTIMEO value
 * @return esp_err_t
 */
esp_err_t app_tcp_server_get_so_sndtimeo(int *sndtimeo) {
    // Check if initialized
    if (s_ctx == NULL || s_ctx->task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    *sndtimeo = s_ctx->sndtimeo;
    return ESP_OK;
}

/**
 * @brief Set SO_KEEPALIVE related options
 *
 * @param enable 0: disable keep-alive, 1: enable keep-alive
 * @param keep_idle Idle time before first keep-alive probe (seconds)
 * @param keep_interval Interval between keep-alive probes (seconds)
 * @param keep_count Number of keep-alive probes before disconnect is considered
 * @return esp_err_t
 */
esp_err_t app_tcp_server_set_keep_alive(uint8_t enable, int keep_idle, int keep_interval, int keep_count) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    s_ctx->keep_alive_enable = enable;
    s_ctx->keep_idle = keep_idle;
    s_ctx->keep_interval = keep_interval;
    s_ctx->keep_count = keep_count;
    // NVS
    return app_nvs_rw_write(NAMESPACE_TCP_SERVER, (app_nvs_rw_write_item_t[]) {
        { .key = KEY_TCP_KEEPALIVE_ENABLE, .type = APP_NVS_RW_TYPE_U8, .data = &enable, .length = sizeof(enable) },
        { .key = KEY_TCP_KEEPALIVE_IDLE, .type = APP_NVS_RW_TYPE_I32, .data = &keep_idle, .length = sizeof(keep_idle) },
        { .key = KEY_TCP_KEEPALIVE_INTERVAL, .type = APP_NVS_RW_TYPE_I32, .data = &keep_interval, .length = sizeof(keep_interval) },
        { .key = KEY_TCP_KEEPALIVE_COUNT, .type = APP_NVS_RW_TYPE_I32, .data = &keep_count, .length = sizeof(keep_count) },
    }, 4);
}

/**
 * @brief Get SO_KEEPALIVE related options
 *
 * @param enable Output pointer for keep-alive enable state
 * @param keep_idle Output pointer for keep-idle value (seconds)
 * @param keep_interval Output pointer for keep-interval value (seconds)
 * @param keep_count Output pointer for keep-count value
 * @return esp_err_t
 */
esp_err_t app_tcp_server_get_keep_alive(uint8_t *enable, int *keep_idle, int *keep_interval, int *keep_count) {
    // Check if initialized
    if (s_ctx == NULL || s_ctx->task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    *enable = s_ctx->keep_alive_enable;
    *keep_idle = s_ctx->keep_idle;
    *keep_interval = s_ctx->keep_interval;
    *keep_count = s_ctx->keep_count;
    return ESP_OK;
}
