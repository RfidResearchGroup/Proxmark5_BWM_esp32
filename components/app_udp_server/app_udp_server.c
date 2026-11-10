#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "app_rtos_task.h"
#include "app_udp_server.h"
#include "app_netutils.h"
#include "app_nvs_rw.h"


#define NAMESPACE_UDP_SERVER           "app_udp_server"
#define KEY_UDP_SERVER_PORT            "udp_svr_port"
#define KEY_UDP_IP_MODE                "udp_ip_mode"
#define KEY_UDP_SNDTIMEO               "udp_sndtimeo"
#define KEY_UDP_CLIENT_IP              "udp_cli_ip"
#define KEY_UDP_CLIENT_PORT            "udp_cli_port"

#define TAG                             "udp_server"
#define DEFAULT_SERVER_PORT             7892
#define DEFAULT_IP_MODE                 0
#define DEFAULT_SO_SNDTIMEO_VALUE       -1


typedef struct {
    TaskHandle_t task_handle;
    SemaphoreHandle_t mutex_socket;
    SemaphoreHandle_t mutex_client_addr;
    SemaphoreHandle_t mutex_state;
    volatile int socket_server_handle;
    volatile bool task_running;
    volatile bool server_stop;
    app_udp_server_rx_callback_t rx_callback;
    uint16_t server_port;
    uint8_t rx_buffer[CONFIG_CMD_PAYLOAD_SIZE];
    uint8_t server_ip_mode;
    int sndtimeo;
    struct sockaddr_storage client_addr_inst;
} app_udp_server_ctx_t;

static app_udp_server_ctx_t *s_ctx = NULL;

/**
 * @brief Helper: convert milliseconds to timeval structure
 * @param timeout_ms Timeout in milliseconds
 * @param tv Output timeval structure pointer
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

static app_udp_server_ctx_t *app_udp_server_ctx_create(void) {
    app_udp_server_ctx_t *ctx = calloc(1, sizeof(app_udp_server_ctx_t));
    if (ctx == NULL) {
        return NULL;
    }

    ctx->socket_server_handle = -1;
    ctx->server_stop = true;
    ctx->server_port = DEFAULT_SERVER_PORT;
    ctx->server_ip_mode = DEFAULT_IP_MODE;
    ctx->sndtimeo = DEFAULT_SO_SNDTIMEO_VALUE;

    ctx->mutex_socket = xSemaphoreCreateMutex();
    if (ctx->mutex_socket == NULL) {
        free(ctx);
        return NULL;
    }

    ctx->mutex_client_addr = xSemaphoreCreateMutex();
    if (ctx->mutex_client_addr == NULL) {
        vSemaphoreDelete(ctx->mutex_socket);
        free(ctx);
        return NULL;
    }

    ctx->mutex_state = xSemaphoreCreateMutex();
    if (ctx->mutex_state == NULL) {
        vSemaphoreDelete(ctx->mutex_socket);
        vSemaphoreDelete(ctx->mutex_client_addr);
        free(ctx);
        return NULL;
    }

    memset(&ctx->client_addr_inst, 0, sizeof(ctx->client_addr_inst));
    return ctx;
}

static void app_udp_server_ctx_destroy(app_udp_server_ctx_t *ctx) {
    if (ctx == NULL) {
        return;
    }

    if (ctx->mutex_state != NULL) {
        vSemaphoreDelete(ctx->mutex_state);
    }

    if (ctx->mutex_client_addr != NULL) {
        vSemaphoreDelete(ctx->mutex_client_addr);
    }

    if (ctx->mutex_socket != NULL) {
        vSemaphoreDelete(ctx->mutex_socket);
    }

    free(ctx);
}

/**
 * @brief Thread-safely close Socket
 * Close socket handle under lock and reset it to -1
 */
static void close_server_socket(app_udp_server_ctx_t *ctx) {
    if (ctx == NULL || ctx->mutex_socket == NULL) {
        return;
    }

    if (xSemaphoreTake(ctx->mutex_socket, portMAX_DELAY) == pdTRUE) {
        if (ctx->socket_server_handle != -1) {
            close(ctx->socket_server_handle);
            ctx->socket_server_handle = -1;
            ESP_LOGI(TAG, "UDP Socket closed");
        }
        xSemaphoreGive(ctx->mutex_socket);
    }
}

/**
 * @brief Update currently recorded client address
 * @param addr Source address structure pointer
 * @param len Address length
 */
static void update_client_address(app_udp_server_ctx_t *ctx, const struct sockaddr_storage *addr, socklen_t len) {
    if (ctx == NULL || ctx->mutex_client_addr == NULL || addr == NULL) {
        return;
    }

    if (xSemaphoreTake(ctx->mutex_client_addr, portMAX_DELAY) != pdTRUE) {
        return;
    }

    memset(&ctx->client_addr_inst, 0, sizeof(ctx->client_addr_inst));
    memcpy(&ctx->client_addr_inst, addr, len);
    xSemaphoreGive(ctx->mutex_client_addr);
}

/**
 * @brief Internal function: create and bind UDP socket
 * Initialize based on configured IP mode (IPv4/IPv6) and port
 * @return true on success, false on failure
 */
static bool udp_server_create(app_udp_server_ctx_t *ctx) {
    struct sockaddr_storage dest_addr;
    int ip_protocol = 0;
    int s_addr_family = AF_INET;

    if (ctx == NULL) {
        return false;
    }

#if CONFIG_LWIP_IPV6
    if (ctx->server_ip_mode == 1) {
        s_addr_family = AF_INET6;
    }
#endif

    if (s_addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(ctx->server_port);
        ip_protocol = IPPROTO_IP;
    }
#if CONFIG_LWIP_IPV6
    else if (s_addr_family == AF_INET6) {
        struct sockaddr_in6 *dest_addr_ip6 = (struct sockaddr_in6 *)&dest_addr;
        bzero(&dest_addr_ip6->sin6_addr.un, sizeof(dest_addr_ip6->sin6_addr.un));
        dest_addr_ip6->sin6_family = AF_INET6;
        dest_addr_ip6->sin6_port = htons(ctx->server_port);
        ip_protocol = IPPROTO_IPV6;
    }
#endif

    ESP_LOGI(TAG, "UDP Socket start (Port %d)", ctx->server_port);

    ctx->socket_server_handle = socket(s_addr_family, SOCK_DGRAM, ip_protocol);
    if (ctx->socket_server_handle < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return false;
    }

    int opt = 1;
    setsockopt(ctx->socket_server_handle, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

#if CONFIG_LWIP_IPV6
    if (s_addr_family == AF_INET6) {
        setsockopt(ctx->socket_server_handle, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
    }
#endif

    if (ctx->sndtimeo >= 0) {
        struct timeval tv = {};
        ms_to_timeval(ctx->sndtimeo, &tv);
        setsockopt(ctx->socket_server_handle, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    }

    int err = bind(ctx->socket_server_handle, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close_server_socket(ctx);
        return false;
    }

    return true;
}

/**
 * @brief UDP server main task
 * Loops receiving data, caches source address, and triggers callback
 */
static void udp_server_task(void *pvParameters) {
    app_udp_server_ctx_t *ctx = (app_udp_server_ctx_t *)pvParameters;

    if (ctx == NULL) {
        vTaskDelete(NULL);
        return;
    }

    while (ctx->task_running) {
        if (ctx->server_stop) {
            if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != 1) {
                continue;
            }
        }

        if (!udp_server_create(ctx)) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "UDP Server listening...");

        while (1) {
            if (ctx->server_stop || !ctx->task_running) {
                break;
            }

            struct sockaddr_storage source_addr;
            socklen_t addr_len = sizeof(source_addr);

            int len = recvfrom(ctx->socket_server_handle, ctx->rx_buffer, CONFIG_CMD_PAYLOAD_SIZE, 0,
                               (struct sockaddr *)&source_addr, &addr_len);

            if (len < 0) {
                ESP_LOGE(TAG, "Recv error: errno %d", errno);
                break;
            } else if (len > 0) {
                update_client_address(ctx, &source_addr, addr_len);
                if (ctx->rx_callback) {
                    ctx->rx_callback(ctx->rx_buffer, len);
                }
            }
        }

        close_server_socket(ctx);
    }

    ctx->task_running = false;
    ctx->server_stop = true;
    // Last ctx access: deinit frees ctx once it sees the handle cleared
    ctx->task_handle = NULL;
    vTaskDelete(NULL);
}

// ================= Public API implementation =================

/**
 * @brief Initialize UDP service task
 * Create mutexes, allocate receive buffer/address cache, and start task
 * @return ESP_OK on success, ESP_ERR_NO_MEM on memory shortage
 */
esp_err_t app_udp_server_init(void) {
    if (s_ctx != NULL && s_ctx->task_running) return ESP_OK;

    if (s_ctx != NULL) {
        app_udp_server_ctx_destroy(s_ctx);
        s_ctx = NULL;
    }

    s_ctx = app_udp_server_ctx_create();
    if (s_ctx == NULL) return ESP_ERR_NO_MEM;

    char *client_ip = NULL;
    uint16_t client_port = 0;
    esp_err_t err = app_nvs_rw_read(NAMESPACE_UDP_SERVER, (app_nvs_rw_read_item_t[]) {
        { .key = KEY_UDP_SERVER_PORT, .type = APP_NVS_RW_TYPE_U16, .data = &s_ctx->server_port, .default_value = DEFAULT_SERVER_PORT },
        { .key = KEY_UDP_IP_MODE, .type = APP_NVS_RW_TYPE_U8, .data = &s_ctx->server_ip_mode, .default_value = DEFAULT_IP_MODE },
        { .key = KEY_UDP_SNDTIMEO, .type = APP_NVS_RW_TYPE_I32, .data = &s_ctx->sndtimeo, .default_value = DEFAULT_SO_SNDTIMEO_VALUE },
        { .key = KEY_UDP_CLIENT_IP, .type = APP_NVS_RW_TYPE_STR, .data = &client_ip, .default_value = 0 },
        { .key = KEY_UDP_CLIENT_PORT, .type = APP_NVS_RW_TYPE_U16, .data = &client_port, .default_value = 0 },
    }, 5);
    if (err != ESP_OK) {
        free(client_ip); // Free any partially loaded client IP string
        app_udp_server_ctx_destroy(s_ctx);
        s_ctx = NULL;
        return err;
    }

    // Print loaded config parameters (except IP address, as it may be NULL)
    ESP_LOGI(TAG, "UDP Server config - Port: %d, IP Mode: %s, Send Timeout: %d ms",
             s_ctx->server_port,
             s_ctx->server_ip_mode == 0 ? "IPv4" : "IPv6",
             s_ctx->sndtimeo);

    // If client IP is persisted in NVS, parse and set it into cached address
    if (client_ip != NULL) {
        ESP_LOGI(TAG, "Loaded UDP Server client IP from NVS: %s", client_ip);
        err = get_sockaddr_storage_from_string(client_ip, client_port, &s_ctx->client_addr_inst, NULL);
        free(client_ip); // Release string memory allocated by NVS library
    }

    s_ctx->task_running = true;
    if (xTaskCreate(udp_server_task, "udp_server", 2048, s_ctx, 5, &s_ctx->task_handle) != pdPASS) {
        s_ctx->task_running = false;
        app_udp_server_ctx_destroy(s_ctx);
        s_ctx = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/**
 * @brief Deinitialize UDP service task
 * Stop task, wait for exit, and release locks/buffers/cache resources
 * @return ESP_OK on success
 */
esp_err_t app_udp_server_deinit(void) {
    if (s_ctx != NULL && s_ctx->task_running) {
        s_ctx->task_running = false;
        s_ctx->server_stop = true;
        if (s_ctx->task_handle != NULL) {
            xTaskNotifyGive(s_ctx->task_handle);
        }
        close_server_socket(s_ctx);

        if (s_ctx->task_handle) {
            wait_for_rtos_task_exit(2000, &s_ctx->task_handle);
        }
    }

    if (s_ctx != NULL) {
        app_udp_server_ctx_destroy(s_ctx);
        s_ctx = NULL;
    }
    return ESP_OK;
}

/**
 * @brief Start UDP service
 * Wake task and begin listening
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if task is not initialized
 */
esp_err_t app_udp_server_start(void) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL || s_ctx->mutex_state == NULL) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_ctx->mutex_state, portMAX_DELAY) != pdTRUE) return ESP_FAIL;
    if (s_ctx->server_stop) {
        s_ctx->server_stop = false;
        xTaskNotifyGive(s_ctx->task_handle);
    }
    xSemaphoreGive(s_ctx->mutex_state);
    return ESP_OK;
}

/**
 * @brief Stop UDP service
 * Set stop flag and close socket to wake blocking recvfrom
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if task is not initialized
 */
esp_err_t app_udp_server_stop(void) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL || s_ctx->mutex_state == NULL) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_ctx->mutex_state, portMAX_DELAY) != pdTRUE) return ESP_FAIL;
    if (!s_ctx->server_stop) {
        s_ctx->server_stop = true;
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
 * @brief Send data (smart routing)
 * Strategy:
 * 1. If fixed target IP/port is configured, parse and send to that address.
 * 2. Otherwise, send to the latest cached client address.
 * 3. If neither is available, return error.
 * @param data Data buffer
 * @param length Data length
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if no destination, ESP_FAIL on send failure
 */
esp_err_t app_udp_server_send(uint8_t *data, size_t length) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL || s_ctx->socket_server_handle < 0) {
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_ctx->mutex_client_addr == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_ctx->mutex_client_addr, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (s_ctx->client_addr_inst.s2_len == 0) {
        xSemaphoreGive(s_ctx->mutex_client_addr);
        ESP_LOGW(TAG, "Send failed: No destination address available.");
        return ESP_ERR_NOT_FOUND;
    }

    ssize_t written = sendto(s_ctx->socket_server_handle, data, length, 0, 
        (struct sockaddr *)&s_ctx->client_addr_inst, s_ctx->client_addr_inst.s2_len);
    if (written < 0) {
        ESP_LOGE(TAG, "Sendto error: errno %d", errno);
        xSemaphoreGive(s_ctx->mutex_client_addr);
        return ESP_FAIL;
    }

    if ((size_t)written != length) {
        xSemaphoreGive(s_ctx->mutex_client_addr);
        ESP_LOGE(TAG, "Sendto partial write: expect %u, actual %d", (unsigned int)length, (int)written);
        return ESP_FAIL;
    }

    xSemaphoreGive(s_ctx->mutex_client_addr);
    ESP_LOGD(TAG, "Sent %u bytes.", (unsigned int)length);
    return ESP_OK;
}

/**
 * @brief Set UDP server RX callback
 * @param callback Callback function pointer
 * @return ESP_OK , ESP_ERR_INVALID_STATE
 */
esp_err_t app_udp_server_set_rx_callback(app_udp_server_rx_callback_t callback) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE;
    s_ctx->rx_callback = callback;
    return ESP_OK;
}

/**
 * @brief Get UDP server running state
 * @param state Output state (0: stopped, 1: running)
 * @return ESP_OK , ESP_ERR_INVALID_STATE
 */
esp_err_t app_udp_server_get_state(uint8_t *state) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE;
    *state = s_ctx->server_stop ? 0 : 1;
    return ESP_OK;
}

esp_err_t app_udp_server_set_ip_mode(uint8_t mode) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE;
    if (mode > 1) return ESP_ERR_INVALID_ARG;
    s_ctx->server_ip_mode = mode;
    return app_nvs_rw_write(NAMESPACE_UDP_SERVER, (app_nvs_rw_write_item_t[]){
        { .key = KEY_UDP_IP_MODE, .type = APP_NVS_RW_TYPE_U8, .data = &mode, .length = sizeof(mode) }
    }, 1);
}

esp_err_t app_udp_server_get_ip_mode(uint8_t *mode) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL || !mode) return ESP_ERR_INVALID_STATE;
    *mode = s_ctx->server_ip_mode;
    return ESP_OK;
}

esp_err_t app_udp_server_set_port(uint16_t port) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE;
    s_ctx->server_port = port;
    return app_nvs_rw_write(NAMESPACE_UDP_SERVER, (app_nvs_rw_write_item_t[]){
        { .key = KEY_UDP_SERVER_PORT, .type = APP_NVS_RW_TYPE_U16, .data = &port, .length = sizeof(port) }
    }, 1);
}

esp_err_t app_udp_server_get_port(uint16_t *port) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL || !port) return ESP_ERR_INVALID_STATE;
    *port = s_ctx->server_port;
    return ESP_OK;
}

esp_err_t app_udp_server_set_so_sndtimeo(int t) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE;
    s_ctx->sndtimeo = t;
    return app_nvs_rw_write(NAMESPACE_UDP_SERVER, (app_nvs_rw_write_item_t[]){
        { .key = KEY_UDP_SNDTIMEO, .type = APP_NVS_RW_TYPE_I32, .data = &t, .length = sizeof(t) }
    }, 1);
}

esp_err_t app_udp_server_get_so_sndtimeo(int *t) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL || !t) return ESP_ERR_INVALID_STATE;
    *t = s_ctx->sndtimeo;
    return ESP_OK;
}

/**
 * @brief Set client destination IP address
 * @param ip_str IP address string (IPv4 or IPv6)
 * @return ESP_OK , ESP_ERR_INVALID_ARG
 */
esp_err_t app_udp_server_set_client_ip(const char *ip_str) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL || !ip_str) return ESP_ERR_INVALID_STATE;
    if (strlen(ip_str) >= UDP_SERVER_IP_STR_LEN) return ESP_ERR_INVALID_ARG;
    if (ip_str[0] == '\0') return ESP_ERR_INVALID_ARG;

    if (s_ctx->mutex_client_addr == NULL) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_ctx->mutex_client_addr, portMAX_DELAY) != pdTRUE) return ESP_ERR_TIMEOUT;

    uint16_t port;
    esp_err_t err = get_port_from_sockaddr_storage(&port, &s_ctx->client_addr_inst);
    if (err != ESP_OK) {
        port = 0;
        ESP_LOGI(TAG, "No previously set port found, defaulting to 0");
    }

    err = get_sockaddr_storage_from_string(ip_str, port, &s_ctx->client_addr_inst, NULL);
    if (err == ESP_OK) {
        err = app_nvs_rw_write(NAMESPACE_UDP_SERVER, (app_nvs_rw_write_item_t[]){
            { .key = KEY_UDP_CLIENT_IP, .type = APP_NVS_RW_TYPE_STR, .data = ip_str, .length = strlen(ip_str) + 1 }
        }, 1);
    } else {
        ESP_LOGE(TAG, "Failed to parse IP address: %d", err);
    }

    xSemaphoreGive(s_ctx->mutex_client_addr);
    return err;
}

/**
 * @brief Get client destination IP address
 * @param ip_str Output buffer for IP string
 * @param max_len Output buffer size
 * @return ESP_OK
 */
esp_err_t app_udp_server_get_client_ip(char *ip_str, size_t max_len) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL || !ip_str || max_len == 0) return ESP_ERR_INVALID_STATE;
    if (s_ctx->mutex_client_addr == NULL) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_ctx->mutex_client_addr, portMAX_DELAY) != pdTRUE) return ESP_ERR_TIMEOUT;
    esp_err_t err = get_ipstr_from_sockaddr_storage(&s_ctx->client_addr_inst, ip_str, max_len);
    xSemaphoreGive(s_ctx->mutex_client_addr);
    return err;
}

/**
 * @brief Set client destination port
 * @param port UDP destination port (must be non-zero)
 * @return ESP_OK , ESP_ERR_INVALID_ARG
 */
esp_err_t app_udp_server_set_client_port(uint16_t port) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL) return ESP_ERR_INVALID_STATE;
    if (port == 0) return ESP_ERR_INVALID_ARG;
    if (s_ctx->mutex_client_addr == NULL) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_ctx->mutex_client_addr, portMAX_DELAY) != pdTRUE) return ESP_ERR_TIMEOUT;
    esp_err_t err = set_port_in_sockaddr_storage(port, &s_ctx->client_addr_inst);
    if (err == ESP_OK) {
        err = app_nvs_rw_write(NAMESPACE_UDP_SERVER, (app_nvs_rw_write_item_t[]){
            { .key = KEY_UDP_CLIENT_PORT, .type = APP_NVS_RW_TYPE_U16, .data = &port, .length = sizeof(port) }
        }, 1);
    }
    xSemaphoreGive(s_ctx->mutex_client_addr);
    return err;
}

/**
 * @brief Get client destination port
 * @param port Output pointer for UDP destination port
 * @return ESP_OK
 */
esp_err_t app_udp_server_get_client_port(uint16_t *port) {
    if (s_ctx == NULL || s_ctx->task_handle == NULL || !port) return ESP_ERR_INVALID_STATE;
    if (s_ctx->mutex_client_addr == NULL) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_ctx->mutex_client_addr, portMAX_DELAY) != pdTRUE) return ESP_ERR_TIMEOUT;
    esp_err_t err = get_port_from_sockaddr_storage(port, &s_ctx->client_addr_inst);
    xSemaphoreGive(s_ctx->mutex_client_addr);
    return err;
}
