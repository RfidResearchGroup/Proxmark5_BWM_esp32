#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_sntp.h"
#include "app_nvs_rw.h"
#include "app_wifi_sntp.h"



#define TAG                             "app_wifi_sntp"
#define APP_WIFI_SNTP_DEFAULT_SERVER    "pool.ntp.org"
#define NAMESPACE_WIFI                  "app_wifi"
#define KEY_WIFI_SNTP_ENABLE            "wifi_sntp_en"
#define KEY_WIFI_SNTP_SERVER            "wifi_sntp_srv"
#define KEY_WIFI_SNTP_INTERVAL          "wifi_sntp_intv"


/**
 * @brief Context structure for the SNTP service wrapper module;
 *        holds SNTP state and configuration.
 */
typedef struct {
    bool enabled;
    bool running;
    char *server;
    uint32_t interval_ms;
} app_wifi_sntp_context_t;

// Module context; allocated in init, freed in deinit
static app_wifi_sntp_context_t *s_ctx = NULL;


/**
 * @brief SNTP time-sync completion callback; logs the event.
 * 
 * @param tv 
 */
static void on_sntp_sync_time(struct timeval *tv) {
    (void)tv;
    ESP_LOGI(TAG, "SNTP time synchronized");
}

/**
 * @brief Helper that duplicates the server string into s_ctx->server.
 *   Ensures the pointer owned by s_ctx is an independent allocation and
 *   correctly frees the previous allocation to avoid memory leaks.
 * 
 * @param server SNTP server address
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
static esp_err_t set_server_dup(const char *server) {
    if (s_ctx == NULL || server == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Duplicate the string so the new address has its own allocation
    char *new_server = strdup(server);
    if (new_server == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // Point to the new allocation
    char *old_server = s_ctx->server;
    s_ctx->server = new_server;

    // lwIP keeps the pointer, not a copy: repoint it before freeing the old string
    if (s_ctx->running) {
        esp_sntp_setservername(0, s_ctx->server);
    }

    // Free the previous string
    free(old_server);
    return ESP_OK;
}

/**
 * @brief Initialises the SNTP service wrapper module; allocates resources
 *        and sets the default server address.
 * 
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
esp_err_t app_wifi_sntp_init(void) {
    if (s_ctx != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Allocate and zero-initialise the context
    s_ctx = (app_wifi_sntp_context_t *)calloc(1, sizeof(app_wifi_sntp_context_t));
    if (s_ctx == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // Load persisted settings from NVS
    esp_err_t err = app_nvs_rw_read(NAMESPACE_WIFI, (app_nvs_rw_read_item_t []) {
        {
            .key = KEY_WIFI_SNTP_ENABLE,
            .type = APP_NVS_RW_TYPE_U8,
            .data = &s_ctx->enabled,
            .default_value = true, // enabled by default
        }, {
            .key = KEY_WIFI_SNTP_SERVER,
            .type = APP_NVS_RW_TYPE_STR,
            .data = &s_ctx->server,
            .default_value = 0, // BLOB/STR does not support a default value
        }, {
            .key = KEY_WIFI_SNTP_INTERVAL,
            .type = APP_NVS_RW_TYPE_U32,
            .data = &s_ctx->interval_ms,
            .default_value = CONFIG_LWIP_SNTP_UPDATE_DELAY, // default sync interval
    }}, 3);
    if (err != ESP_OK) { // Return immediately on read failure
        free(s_ctx->server); // Free any partially loaded server string
        free(s_ctx);
        s_ctx = NULL;
        return err;
    }

    // No server was stored in NVS; copy the compile-time default
    if (s_ctx->server == NULL)  {
        err = set_server_dup(APP_WIFI_SNTP_DEFAULT_SERVER);
        if (err != ESP_OK) {
            free(s_ctx);
            s_ctx = NULL;
            return err;
        }
    }

    // Warn if SNTP has been disabled via configuration
    if (!s_ctx->enabled) {
        ESP_LOGW(TAG, "SNTP is disabled by configuration");
    }

    return ESP_OK;
}

/**
 * @brief Deinitialises the SNTP service wrapper module and frees all resources.
 * 
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
esp_err_t app_wifi_sntp_deinit(void) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Stop the SNTP service before freeing resources
    app_wifi_sntp_stop();

    // Free the server address string
    if (s_ctx->server != NULL) {
        free(s_ctx->server);
        s_ctx->server = NULL;
    }

    // Free the context and clear the global pointer
    free(s_ctx);
    s_ctx = NULL;

    return ESP_OK;
}

/**
 * @brief Starts the SNTP service and begins time synchronisation with the server.
 * 
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
esp_err_t app_wifi_sntp_start(void) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_ctx->server == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_ctx->enabled) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_ctx->running) {
        return ESP_OK;
    }

    esp_sntp_stop();
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, s_ctx->server);
    esp_sntp_set_sync_interval(s_ctx->interval_ms);
    esp_sntp_set_time_sync_notification_cb(on_sntp_sync_time);
    esp_sntp_init();

    s_ctx->running = true;
    ESP_LOGI(TAG, "SNTP started, server=%s", s_ctx->server);
    return ESP_OK;
}

/**
 * @brief Stops the SNTP service.
 * 
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
esp_err_t app_wifi_sntp_stop(void) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_ctx->running) {
        return ESP_OK;
    }

    esp_sntp_stop();
    // Drop lwIP's pointer to s_ctx->server; synchronous, queued after the stop
    esp_sntp_setservername(0, NULL);
    s_ctx->running = false;
    ESP_LOGI(TAG, "SNTP stopped");
    return ESP_OK;
}

/**
 * @brief Sets the SNTP enable flag. When disabled, app_wifi_sntp_start() will fail.
 *
 * @param enable true to enable, false to disable
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
esp_err_t app_wifi_sntp_set_enable(uint8_t enable) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_ctx->enabled = enable;
    ESP_LOGI(TAG, "SNTP enable set to %s", enable ? "true" : "false");

    // Persist the setting
    return app_nvs_rw_write(NAMESPACE_WIFI, (app_nvs_rw_write_item_t[]) {
        {
            .key = KEY_WIFI_SNTP_ENABLE,
            .type = APP_NVS_RW_TYPE_U8,
            .data = &enable,
            .length = sizeof(enable),
        }
    }, 1);
}

/**
 * @brief Gets the SNTP enable flag.
 *
 * @param enable Output: current enable state
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
esp_err_t app_wifi_sntp_get_enable(uint8_t *enable) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (enable == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *enable = s_ctx->enabled;
    return ESP_OK;
}

/**
 * @brief Sets the SNTP server address.
 * 
 * @param server Pointer to the server address string
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
esp_err_t app_wifi_sntp_set_server(const char *server) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (server == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Duplicate the string and free the previous allocation
    esp_err_t err = set_server_dup(server);
    if (err != ESP_OK) {
        return err;
    }

    // Persist to NVS
   return app_nvs_rw_write(NAMESPACE_WIFI, (app_nvs_rw_write_item_t[]) {
        {
            .key = KEY_WIFI_SNTP_SERVER,
            .type = APP_NVS_RW_TYPE_STR,
            .data = server,
            .length = strlen(server) + 1,
        }
    }, 1);
}

/**
 * @brief Gets the configured SNTP server address.
 * 
 * @param server Output: pointer to the server address string
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
esp_err_t app_wifi_sntp_get_server(const char **server) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (server == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *server = s_ctx->server;
    return ESP_OK;
}

/**
 * @brief Sets the SNTP sync interval in milliseconds. Takes effect after
 *        the SNTP service is restarted.
 *        Note: per SNTPv4 RFC 4330, the minimum interval should be no less
 *        than 15 seconds (15000 ms); shorter intervals may cause the server
 *        to refuse the connection.
 * 
 * @param interval_ms Sync interval in milliseconds; minimum value is 15000
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
esp_err_t app_wifi_sntp_set_interval(uint32_t interval_ms) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (interval_ms < 15000) {
        ESP_LOGE(TAG, "SNTP interval too short: %" PRIu32 " ms, must be at least 15000 ms", interval_ms);
        return ESP_ERR_INVALID_ARG; // Minimum interval is 15 seconds
    }

    s_ctx->interval_ms = interval_ms;
    ESP_LOGI(TAG, "SNTP interval set to %" PRIu32 " ms", interval_ms);

    // Persist to NVS
    return app_nvs_rw_write(NAMESPACE_WIFI, (app_nvs_rw_write_item_t[]) {
        {
            .key = KEY_WIFI_SNTP_INTERVAL,
            .type = APP_NVS_RW_TYPE_U32,
            .data = &interval_ms,
            .length = sizeof(interval_ms),
        }
    }, 1);
}

/**
 * @brief Gets the configured SNTP sync interval in milliseconds.
 * 
 * @param interval_ms Output: current sync interval in milliseconds
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
esp_err_t app_wifi_sntp_get_interval(uint32_t *interval_ms) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (interval_ms == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *interval_ms = s_ctx->interval_ms;
    return ESP_OK;
}

/**
 * @brief Gets the SNTP sync status.
 * 
 * @param status Output: 0 = disabled, 1 = idle, 2 = syncing, 3 = synced
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
esp_err_t app_wifi_sntp_get_sync_status(uint8_t *status) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check if disabled
    if (!s_ctx->enabled) {
        *status = 0;
        return ESP_OK;
    }
    // Check if idle
    if (!s_ctx->running) {
        *status = 1;
        return ESP_OK;
    }
    // Map the underlying SNTP sync state
    sntp_sync_status_t sync_status = sntp_get_sync_status();
    switch (sync_status) {
        case SNTP_SYNC_STATUS_COMPLETED:
            *status = 3;
            break;
        case SNTP_SYNC_STATUS_IN_PROGRESS:
        case SNTP_SYNC_STATUS_RESET:
        default:
            *status = 2;
            break;
    }

    return ESP_OK;
}
