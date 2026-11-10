#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_tls.h"
#include "mqtt_client.h"
#include "app_mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "app_nvs_rw.h"


/**
 * @brief MQTT client context structure containing all MQTT connection-related configurations and status information
 * 
 */
static struct mqtt_ctx {

    // --- Certificates ---

    char *cacert;   /*!< Server certificate; in one-way authentication, only server certificate verification is required, server does not authenticate client */
    char *ccert;    /*!< Client certificate, server configuration for verifying client */
    char *cckey;    /*!< Client private key, server configuration for verifying client */

    // --- Pre-defined keys ---
    psk_hint_key_t psk_hint_key; /*!< PSK key verification, this is bidirectional */

    // --- Connection-related configuration ---

    char *host;              /*!< MQTT server address */
    uint16_t port;           /*!< MQTT server port */
    char *path;              /*!< MQTT server path, required for ws and wss protocols */
    uint8_t scheme;          /*!< MQTT protocol: mqtt, mqtts, ws, wss */
    bool is_connected;       /*!< Whether MQTT client is already connected to MQTT server */
    
    char *subscribe_topic;   /*!< Subscribed topic */
    uint8_t subscribe_qos;   /*!< Subscription QoS level */
    bool is_subscribed;      /*!< Whether topic has been subscribed */

    char *publish_topic;     /*!< Published topic */
    uint8_t publish_qos;     /*!< Publishing QoS level */
    uint8_t publish_retain;  /*!< Retain flag for published messages */

    char *client_id;        /*!< MQTT client ID */
    char *username;         /*!< MQTT username */
    char *password;         /*!< MQTT password */

    int keepalive;      /*!< MQTT Keepalive time in seconds */
    uint8_t disable_clean_session; /*!< Whether to disable clean session, 0 = enabled, 1 = disabled */

    char *lwt_topic;        /*!< Last Will topic */
    char *lwt_message;      /*!< Last Will message content */
    uint8_t lwt_qos;        /*!< Last Will QoS level */
    uint8_t lwt_retain;     /*!< Last Will retain flag */

    char *alpn_list[MQTT_ALPN_COUNT_MAX + 1]; /*!< ALPN protocol list, required for mqtts and wss protocols, last element must be NULL */
    uint8_t alpn_count;     /*!< ALPN protocol count */

    char *sni_host;         /*!< SNI server name, required for mqtts and wss protocols */

    // Client-related
    esp_mqtt_client_handle_t client; /*!< MQTT client instance handle */
    bool is_client_started; /*!< Whether MQTT client has been started */
    SemaphoreHandle_t mutex; /*!< Mutex lock to protect MQTT context, avoid data race from multi-thread access */

    app_mqtt_client_rx_callback_t rx_callback; /*!< MQTT client data reception callback function */

} *s_mqtt_ctx = NULL; // MQTT client context instance pointer, initial value NULL means uninitialized


#define TAG                           "app_mqtt_client"
#define NAMESPACE_MQTT_CLIENT         "app_mqtt"
#define KEY_MQTT_HOST                 "mqtt_host"
#define KEY_MQTT_PORT                 "mqtt_port"
#define KEY_MQTT_PATH                 "mqtt_path"
#define KEY_MQTT_SCHEME               "mqtt_scheme"
#define KEY_MQTT_SUBSCRIBE_TOPIC      "mqtt_sub_topic"
#define KEY_MQTT_SUBSCRIBE_QOS        "mqtt_sub_qos"
#define KEY_MQTT_PUBLISH_TOPIC        "mqtt_pub_topic"
#define KEY_MQTT_PUBLISH_QOS          "mqtt_pub_qos"
#define KEY_MQTT_PUBLISH_RETAIN       "mqtt_pub_re"
#define KEY_MQTT_CLIENT_ID            "mqtt_client_id"
#define KEY_MQTT_USERNAME             "mqtt_username"
#define KEY_MQTT_PASSWORD             "mqtt_password"
#define KEY_MQTT_KEEPALIVE            "mqtt_keepalive"
#define KEY_MQTT_CLEAN_SESSION        "mqtt_clean_ses"
#define KEY_MQTT_LWT_TOPIC            "mqtt_lwt_topic"
#define KEY_MQTT_LWT_MESSAGE          "mqtt_lwt_msg"
#define KEY_MQTT_LWT_QOS              "mqtt_lwt_qos"
#define KEY_MQTT_LWT_RETAIN           "mqtt_lwt_re"
#define KEY_MQTT_SNI_HOST             "mqtt_sni_host"
#define KEY_MQTT_CACERT               "mqtt_cacert"
#define KEY_MQTT_CCERT                "mqtt_ccert"
#define KEY_MQTT_CCKEY                "mqtt_cckey"
#define KEY_MQTT_PSK_KEY              "mqtt_psk_key"
#define KEY_MQTT_PSK_HINT             "mqtt_psk_hint"
#define KEY_MQTT_ALPN_COUNT           "mqtt_alpn_cnt"
#define KEY_MQTT_ALPN_PREFIX          "mqtt_alpn_"


// Helper function to print error messages; only prints when error code is non-zero
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            s_mqtt_ctx->is_connected = true; // Mark as connected to MQTT server
            // Core logic: determine if server recovered old session; event->session_present = true means server retained previous subscriptions
            if (event->session_present) {
                ESP_LOGI(TAG, "Session present, resuming previous session");
                s_mqtt_ctx->is_subscribed = true; // Mark as subscribed to topic
            } else {
                ESP_LOGI(TAG, "No session present, starting new session");
                if (s_mqtt_ctx->subscribe_topic) {
                    msg_id = esp_mqtt_client_subscribe(client, s_mqtt_ctx->subscribe_topic, s_mqtt_ctx->subscribe_qos);
                    ESP_LOGI(TAG, "Sent subscribe successful, msg_id=%d", msg_id);
                } else {
                    ESP_LOGW(TAG, "No subscribe topic configured, skipping subscribe");
                }
            }
            break;
        case MQTT_EVENT_DISCONNECTED:
            s_mqtt_ctx->is_subscribed = false; // Mark as unsubscribed from topic
            s_mqtt_ctx->is_connected = false;  // Mark as not connected to MQTT server
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            // After successful subscription, MQTT client can receive messages
            s_mqtt_ctx->is_subscribed = true; // Mark as subscribed to topic
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            s_mqtt_ctx->is_subscribed = false; // Mark as unsubscribed from topic
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGD(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGD(TAG, "MQTT_EVENT_DATA");
            ESP_LOGD(TAG, " > rx data length = %d", event->data_len);
            // After receiving message, pass data through callback to upper application; MQTT client doesn't care about data content
            if (s_mqtt_ctx->rx_callback) {
                s_mqtt_ctx->rx_callback((uint8_t*)event->data, event->data_len);
            }
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
                ESP_LOGE(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
        default:
            ESP_LOGD(TAG, "Other event id:%d", event->event_id);
            break;
    }
}

// Release pre-defined key resources; mainly free memory of key field, set key field to NULL and key_size to 0
static inline void release_psk_data(void) {
    free((uint8_t*)s_mqtt_ctx->psk_hint_key.key);
    s_mqtt_ctx->psk_hint_key.key = NULL;
    s_mqtt_ctx->psk_hint_key.key_size = 0;
}

// MQTT client cleanup function, release all dynamically allocated resources, reset MQTT context pointer to NULL
static void mqtt_client_cleanup(void) {
    if (s_mqtt_ctx == NULL) {
        return; // Already cleaned up, return directly
    }
    // Release dynamically allocated members
    if (s_mqtt_ctx->cacert) free(s_mqtt_ctx->cacert);
    if (s_mqtt_ctx->ccert) free(s_mqtt_ctx->ccert);
    if (s_mqtt_ctx->cckey) free(s_mqtt_ctx->cckey);
    if (s_mqtt_ctx->psk_hint_key.key) release_psk_data();
    if (s_mqtt_ctx->psk_hint_key.hint) free((char*)s_mqtt_ctx->psk_hint_key.hint);
    if (s_mqtt_ctx->host) free(s_mqtt_ctx->host);
    if (s_mqtt_ctx->path) free(s_mqtt_ctx->path);
    if (s_mqtt_ctx->subscribe_topic) free(s_mqtt_ctx->subscribe_topic);
    if (s_mqtt_ctx->publish_topic) free(s_mqtt_ctx->publish_topic);
    if (s_mqtt_ctx->client_id) free(s_mqtt_ctx->client_id);
    if (s_mqtt_ctx->username) free(s_mqtt_ctx->username);
    if (s_mqtt_ctx->password) free(s_mqtt_ctx->password);
    if (s_mqtt_ctx->lwt_topic) free(s_mqtt_ctx->lwt_topic);
    if (s_mqtt_ctx->lwt_message) free(s_mqtt_ctx->lwt_message);
    for (uint8_t i = 0; i < s_mqtt_ctx->alpn_count; i++) {
        if (s_mqtt_ctx->alpn_list[i]) free(s_mqtt_ctx->alpn_list[i]);
    }
    if (s_mqtt_ctx->sni_host) free(s_mqtt_ctx->sni_host);
    if (s_mqtt_ctx->mutex) vSemaphoreDelete(s_mqtt_ctx->mutex);
    // Release MQTT context itself
    free(s_mqtt_ctx);
    s_mqtt_ctx = NULL;
}

/**
 * @brief Initialize MQTT context, allocate memory and set default values
 * 
 * @return esp_err_t Returns ESP_OK on success; otherwise returns a corresponding error code
 */
esp_err_t app_mqtt_init(void)
{
    if (s_mqtt_ctx != NULL) {
        return ESP_ERR_INVALID_STATE; // Already initialized, cannot re-initialize
    }

    // Initialize MQTT context, allocate memory and set default values
    s_mqtt_ctx = calloc(1, sizeof(struct mqtt_ctx));
    if (s_mqtt_ctx == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // Initialize mutex lock
    s_mqtt_ctx->mutex = xSemaphoreCreateMutex();
    if (s_mqtt_ctx->mutex == NULL) {
        free(s_mqtt_ctx);
        s_mqtt_ctx = NULL;
        return ESP_ERR_NO_MEM;
    }

    uint8_t alpn_count = 0;
    // Load persistent parameters from NVS
    esp_err_t err = app_nvs_rw_read(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_read_item_t[]) {
        { .key = KEY_MQTT_HOST, .type = APP_NVS_RW_TYPE_STR, .data = &s_mqtt_ctx->host, .default_value = 0 },
        { .key = KEY_MQTT_PORT, .type = APP_NVS_RW_TYPE_U16, .data = &s_mqtt_ctx->port, .default_value = 1883 },
        { .key = KEY_MQTT_PATH, .type = APP_NVS_RW_TYPE_STR, .data = &s_mqtt_ctx->path, .default_value = 0 },
        { .key = KEY_MQTT_SCHEME, .type = APP_NVS_RW_TYPE_U8, .data = &s_mqtt_ctx->scheme, .default_value = MQTT_SCHEME_TCP },
        { .key = KEY_MQTT_SUBSCRIBE_TOPIC, .type = APP_NVS_RW_TYPE_STR, .data = &s_mqtt_ctx->subscribe_topic, .default_value = 0 },
        { .key = KEY_MQTT_SUBSCRIBE_QOS, .type = APP_NVS_RW_TYPE_U8, .data = &s_mqtt_ctx->subscribe_qos, .default_value = 0 },
        { .key = KEY_MQTT_PUBLISH_TOPIC, .type = APP_NVS_RW_TYPE_STR, .data = &s_mqtt_ctx->publish_topic, .default_value = 0 },
        { .key = KEY_MQTT_PUBLISH_QOS, .type = APP_NVS_RW_TYPE_U8, .data = &s_mqtt_ctx->publish_qos, .default_value = 0 },
        { .key = KEY_MQTT_PUBLISH_RETAIN, .type = APP_NVS_RW_TYPE_U8, .data = &s_mqtt_ctx->publish_retain, .default_value = 0 },
        { .key = KEY_MQTT_CLIENT_ID, .type = APP_NVS_RW_TYPE_STR, .data = &s_mqtt_ctx->client_id, .default_value = 0 },
        { .key = KEY_MQTT_USERNAME, .type = APP_NVS_RW_TYPE_STR, .data = &s_mqtt_ctx->username, .default_value = 0 },
        { .key = KEY_MQTT_PASSWORD, .type = APP_NVS_RW_TYPE_STR, .data = &s_mqtt_ctx->password, .default_value = 0 },
        { .key = KEY_MQTT_KEEPALIVE, .type = APP_NVS_RW_TYPE_I32, .data = &s_mqtt_ctx->keepalive, .default_value = 120 },
        { .key = KEY_MQTT_CLEAN_SESSION, .type = APP_NVS_RW_TYPE_U8, .data = &s_mqtt_ctx->disable_clean_session, .default_value = 0 },
        { .key = KEY_MQTT_LWT_TOPIC, .type = APP_NVS_RW_TYPE_STR, .data = &s_mqtt_ctx->lwt_topic, .default_value = 0 },
        { .key = KEY_MQTT_LWT_MESSAGE, .type = APP_NVS_RW_TYPE_STR, .data = &s_mqtt_ctx->lwt_message, .default_value = 0 },
        { .key = KEY_MQTT_LWT_QOS, .type = APP_NVS_RW_TYPE_U8, .data = &s_mqtt_ctx->lwt_qos, .default_value = 0 },
        { .key = KEY_MQTT_LWT_RETAIN, .type = APP_NVS_RW_TYPE_U8, .data = &s_mqtt_ctx->lwt_retain, .default_value = 0 },
        { .key = KEY_MQTT_SNI_HOST, .type = APP_NVS_RW_TYPE_STR, .data = &s_mqtt_ctx->sni_host, .default_value = 0 },
        { .key = KEY_MQTT_CACERT, .type = APP_NVS_RW_TYPE_STR, .data = &s_mqtt_ctx->cacert, .default_value = 0 },
        { .key = KEY_MQTT_CCERT, .type = APP_NVS_RW_TYPE_STR, .data = &s_mqtt_ctx->ccert, .default_value = 0 },
        { .key = KEY_MQTT_CCKEY, .type = APP_NVS_RW_TYPE_STR, .data = &s_mqtt_ctx->cckey, .default_value = 0 },
        { .key = KEY_MQTT_PSK_HINT, .type = APP_NVS_RW_TYPE_STR, .data = &s_mqtt_ctx->psk_hint_key.hint, .default_value = 0 },
        { .key = KEY_MQTT_ALPN_COUNT, .type = APP_NVS_RW_TYPE_U8, .data = &alpn_count, .default_value = 0 },
    }, 24);
    // If reading config fails return error code directly, only continue on success
    if (err != ESP_OK) {
        mqtt_client_cleanup(); // Clean up already allocated resources
        return err;
    }

    // Because we need out_len, read PSK key separately
    app_nvs_rw_read_item_t psk_item = {
        .key = KEY_MQTT_PSK_KEY,
        .type = APP_NVS_RW_TYPE_BLOB,
        .data = &s_mqtt_ctx->psk_hint_key.key,
        .out_len = 0,
        .default_value = 0,
    };
    err = app_nvs_rw_read(NAMESPACE_MQTT_CLIENT, &psk_item, 1);
    if (err != ESP_OK) {
        mqtt_client_cleanup(); // Clean up already allocated resources    
        return err;
    }
    s_mqtt_ctx->psk_hint_key.key_size = psk_item.out_len;

    // Read ALPN list count, reset to 0 if exceeds max to avoid out-of-bounds access
    if (alpn_count > MQTT_ALPN_COUNT_MAX) alpn_count = 0;
    for (uint8_t i = 0; i < alpn_count; i++) {
        char key_name[16] = {0};
        snprintf(key_name, sizeof(key_name), "%s%u", KEY_MQTT_ALPN_PREFIX, i);
        char *alpn = NULL;
        app_nvs_rw_read_item_t alpn_item = {
            .key = key_name,
            .type = APP_NVS_RW_TYPE_STR,
            .data = &alpn,
            .default_value = 0,
        };
        err = app_nvs_rw_read(NAMESPACE_MQTT_CLIENT, &alpn_item, 1);
        if (alpn) {
            s_mqtt_ctx->alpn_list[s_mqtt_ctx->alpn_count++] = alpn;
            s_mqtt_ctx->alpn_list[s_mqtt_ctx->alpn_count] = NULL;
        }
        if (err != ESP_OK) {
            mqtt_client_cleanup(); // Clean up already allocated resources
            return err;
        }
    }

    return ESP_OK;
}

// Persist the current ALPN list to NVS: write ALPN count first, then each ALPN entry.
// If writing succeeds, also clean up any extra stale ALPN entries.
static esp_err_t persist_mqtt_alpn_list(void) {
    app_nvs_rw_write_item_t items[MQTT_ALPN_COUNT_MAX + 1];
    size_t item_count = 0;

    // First write ALPN count so when reading we know how many ALPN protocols to read
    items[item_count++] = (app_nvs_rw_write_item_t){
        .key = KEY_MQTT_ALPN_COUNT,
        .type = APP_NVS_RW_TYPE_U8,
        .data = &s_mqtt_ctx->alpn_count,
        .length = sizeof(s_mqtt_ctx->alpn_count),
    };

    // Then write each ALPN protocol item, key format: "mqtt_alpn_0", "mqtt_alpn_1", ..., "mqtt_alpn_{n}"
    for (uint8_t i = 0; i < s_mqtt_ctx->alpn_count; i++) {
        char *alpn = s_mqtt_ctx->alpn_list[i];
        char key_name[16];
        snprintf(key_name, sizeof(key_name), "%s%u", KEY_MQTT_ALPN_PREFIX, i);
        items[item_count++] = (app_nvs_rw_write_item_t){
            .key = strdup(key_name),
            .type = APP_NVS_RW_TYPE_STR,
            .data = alpn,
            .length = strlen(alpn) + 1,
        };
    }

    // Execute write and release dynamically allocated key string memory
    esp_err_t err = app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, items, item_count);
    for (size_t i = 1; i < item_count; i++) {
        free((char *)items[i].key);
    }
    if (err == ESP_OK) {
        // Clean up any extra ALPN items that may have existed
        for (uint8_t i = s_mqtt_ctx->alpn_count; i < MQTT_ALPN_COUNT_MAX; i++) {
            char key_name[16];
            snprintf(key_name, sizeof(key_name), "%s%u", KEY_MQTT_ALPN_PREFIX, i);
            app_nvs_rw_erase(NAMESPACE_MQTT_CLIENT, key_name);
        }
    }
    return err;
}

/**
 * @brief Deinitialize MQTT context, release memory
 * 
 * @return esp_err_t Returns ESP_OK on success; otherwise returns a corresponding error code
 */
esp_err_t app_mqtt_deinit(void) {
    // Deinitialize MQTT context, release memory, and release memory for dynamically allocated members in context
    if (s_mqtt_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    // If MQTT client is running, stop it first
    esp_err_t err = app_mqtt_stop();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to stop MQTT client during deinit, error: %s", esp_err_to_name(err));
    }
    // Release dynamically allocated members
    mqtt_client_cleanup();
    
    return ESP_OK;
}

/**
 * @brief Initialize MQTT client configuration based on current user settings
 * 
 * @param mqtt_cfg Reference to the MQTT client configuration
 * @return esp_err_t 
 */
esp_err_t setup_esp_mqtt_client_config(esp_mqtt_client_config_t *mqtt_cfg) {

    // Key connection parameters
    mqtt_cfg->broker.address.hostname = s_mqtt_ctx->host;
    mqtt_cfg->broker.address.port = s_mqtt_ctx->port;
    mqtt_cfg->broker.address.path = s_mqtt_ctx->path;
    
    // Print current connection config for debugging (some parameters may be null)
    ESP_LOGI(TAG, "MQTT broker hostname: %s", mqtt_cfg->broker.address.hostname ? mqtt_cfg->broker.address.hostname : "(null)");
    ESP_LOGI(TAG, "MQTT broker port: %d", mqtt_cfg->broker.address.port);
    ESP_LOGI(TAG, "MQTT broker path: %s", mqtt_cfg->broker.address.path ? mqtt_cfg->broker.address.path : "(null)");

    // MQTT server address is required, return error if not set
    if (mqtt_cfg->broker.address.hostname == NULL) {
        ESP_LOGE(TAG, "MQTT broker hostname is not set");
        return ESP_ERR_INVALID_ARG;
    }

    // Set transport mode based on chosen MQTT protocol: mqtt, mqtts, ws, wss
    switch (s_mqtt_ctx->scheme) {
        case MQTT_SCHEME_TCP:
        default: // Default to MQTT over TCP
            ESP_LOGI(TAG, "Using MQTT over TCP transport");
            mqtt_cfg->broker.address.transport = MQTT_TRANSPORT_OVER_TCP;
            break;
        case MQTT_SCHEME_TLS_NO_VERIFY:
        case MQTT_SCHEME_TLS_VERIFY_SERVER:
        case MQTT_SCHEME_TLS_CLIENT_CERT:
        case MQTT_SCHEME_TLS_VERIFY_SERVER_CLIENT_CERT:
        case MQTT_SCHEME_TLS_PSK:
            ESP_LOGI(TAG, "Using MQTT over SSL transport");
            mqtt_cfg->broker.address.transport = MQTT_TRANSPORT_OVER_SSL;
            break;
        case MQTT_SCHEME_WEBSOCKET:
            ESP_LOGI(TAG, "Using MQTT over WebSocket transport");
            mqtt_cfg->broker.address.transport = MQTT_TRANSPORT_OVER_WS;
            break;
        case MQTT_SCHEME_WEBSOCKET_TLS_NO_VERIFY:
        case MQTT_SCHEME_WEBSOCKET_TLS_VERIFY_SERVER:
        case MQTT_SCHEME_WEBSOCKET_TLS_CLIENT_CERT:
        case MQTT_SCHEME_WEBSOCKET_TLS_VERIFY_SERVER_CLIENT_CERT:
        case MQTT_SCHEME_WEBSOCKET_TLS_PSK:
            ESP_LOGI(TAG, "Using MQTT over WebSocket Secure transport");
            mqtt_cfg->broker.address.transport = MQTT_TRANSPORT_OVER_WSS;
            break;
    }
    // If user hasn't set client ID, use default "Proxmark5"
    if (s_mqtt_ctx->client_id) {
        mqtt_cfg->credentials.client_id = s_mqtt_ctx->client_id;
        ESP_LOGI(TAG, "Using client ID: %s", s_mqtt_ctx->client_id);
        // If user set client ID but it's empty string, use empty string as client ID
        if (s_mqtt_ctx->client_id[0] == '\0') {
            mqtt_cfg->credentials.set_null_client_id = true; // Tell MQTT client to use empty string as client ID
        }
    } else {
        mqtt_cfg->credentials.client_id = CONFIG_DEVICE_IDENTIFIER;
        ESP_LOGI(TAG, "Using default client ID: %s", CONFIG_DEVICE_IDENTIFIER);
    }
    // Username
    if (s_mqtt_ctx->username) {
        ESP_LOGI(TAG, "Using username: %s", s_mqtt_ctx->username);
        mqtt_cfg->credentials.username = s_mqtt_ctx->username;
    }
    // Password
    if (s_mqtt_ctx->password) {
        ESP_LOGI(TAG, "Using password: %s", s_mqtt_ctx->password);
        mqtt_cfg->credentials.authentication.password = s_mqtt_ctx->password;
    }
    // Keepalive time in seconds, default 120s in mqtt client; if user sets <0, disable Keepalive
    if (s_mqtt_ctx->keepalive < 0) {
        ESP_LOGI(TAG, "Disabling Keepalive");
        mqtt_cfg->session.disable_keepalive = true; // Disable Keepalive
    } else {
        // Note: even if Keepalive is set to 0, Keepalive is still enabled,
        // and the MQTT client internals will use 120 seconds.
        ESP_LOGI(TAG, "Using Keepalive: %d seconds", s_mqtt_ctx->keepalive);
        mqtt_cfg->session.keepalive = s_mqtt_ctx->keepalive;
    }
    // Whether to disable clean session, 0 = enabled, 1 = disabled
    mqtt_cfg->session.disable_clean_session = s_mqtt_ctx->disable_clean_session;
    ESP_LOGI(TAG, "Using clean session: %s", s_mqtt_ctx->disable_clean_session ? "disabled" : "enabled");
    // Last Will message configuration
    if (s_mqtt_ctx->lwt_topic) {
        mqtt_cfg->session.last_will.topic = s_mqtt_ctx->lwt_topic;
        mqtt_cfg->session.last_will.qos = s_mqtt_ctx->lwt_qos;
        mqtt_cfg->session.last_will.retain = s_mqtt_ctx->lwt_retain;
        ESP_LOGI(TAG, "Using Last Will topic: %s, QoS: %d, Retain: %d", s_mqtt_ctx->lwt_topic, s_mqtt_ctx->lwt_qos, s_mqtt_ctx->lwt_retain);
        // If user hasn't set LWT message content, use empty string
        if (s_mqtt_ctx->lwt_message) {
            mqtt_cfg->session.last_will.msg = s_mqtt_ctx->lwt_message;
        } else {
            mqtt_cfg->session.last_will.msg = "";
        }
        ESP_LOGI(TAG, "Using Last Will message: %s", mqtt_cfg->session.last_will.msg);
    }
    // Some configurations needed for mqtts and wss protocols
    if (mqtt_cfg->broker.address.transport == MQTT_TRANSPORT_OVER_SSL || mqtt_cfg->broker.address.transport == MQTT_TRANSPORT_OVER_WSS) {
        // ALPN protocol list
        if (s_mqtt_ctx->alpn_count > 0 && s_mqtt_ctx->alpn_count <= MQTT_ALPN_COUNT_MAX) {
            mqtt_cfg->broker.verification.alpn_protos = (const char **)s_mqtt_ctx->alpn_list;
            for (uint8_t i = 0; i < s_mqtt_ctx->alpn_count; i++) {
                ESP_LOGI(TAG, "Using ALPN protocol: %s", s_mqtt_ctx->alpn_list[i]);
            }
        }
        // SNI server name
        if (s_mqtt_ctx->sni_host) {
            mqtt_cfg->broker.verification.common_name = s_mqtt_ctx->sni_host;
            ESP_LOGI(TAG, "Using SNI host: %s", s_mqtt_ctx->sni_host);
        }
        // Disable cert verification; we need to disable cert verification in mqtt client so it won't verify server cert validity when establishing TLS connection
        if (s_mqtt_ctx->scheme == MQTT_SCHEME_TCP || s_mqtt_ctx->scheme == MQTT_SCHEME_WEBSOCKET) {
            mqtt_cfg->broker.verification.skip_cert_common_name_check = true;
            ESP_LOGI(TAG, "Skip any validation of server certificate CN field");
        }
        // Provide certificates based on s_mqtt_ctx->scheme.
        // This branch checks whether server certificates / CA certificates are required.
        if (s_mqtt_ctx->scheme == MQTT_SCHEME_TLS_VERIFY_SERVER || 
            s_mqtt_ctx->scheme == MQTT_SCHEME_TLS_VERIFY_SERVER_CLIENT_CERT || 
            s_mqtt_ctx->scheme == MQTT_SCHEME_WEBSOCKET_TLS_VERIFY_SERVER || 
            s_mqtt_ctx->scheme == MQTT_SCHEME_WEBSOCKET_TLS_VERIFY_SERVER_CLIENT_CERT) 
        {
            // For mqtts/wss modes that require server verification, provide server certificate/CA certificate.
            // The MQTT client internals use this material to verify server identity and mitigate MITM attacks.
            if (s_mqtt_ctx->cacert) {
                mqtt_cfg->broker.verification.certificate = s_mqtt_ctx->cacert;
                ESP_LOGI(TAG, "Using CA certificate");
            } else {
                ESP_LOGE(TAG, "Server certificate verification is enabled, but no CA certificate provided");
                return ESP_ERR_INVALID_ARG; // Missing server certificate/CA certificate; cannot verify server identity
            }
        }
        // Determine whether client certificate and private key are required by current scheme.
        if (s_mqtt_ctx->scheme == MQTT_SCHEME_TLS_CLIENT_CERT || 
            s_mqtt_ctx->scheme == MQTT_SCHEME_TLS_VERIFY_SERVER_CLIENT_CERT || 
            s_mqtt_ctx->scheme == MQTT_SCHEME_WEBSOCKET_TLS_CLIENT_CERT || 
            s_mqtt_ctx->scheme == MQTT_SCHEME_WEBSOCKET_TLS_VERIFY_SERVER_CLIENT_CERT) 
        {
            // For mqtts/wss modes requiring client auth, provide client certificate and private key.
            // The server uses these credentials to verify client identity.
            if (s_mqtt_ctx->ccert) {
                mqtt_cfg->credentials.authentication.certificate = s_mqtt_ctx->ccert;
                ESP_LOGI(TAG, "Using client certificate");
            } else {
                ESP_LOGE(TAG, "Client certificate authentication is enabled, but no client certificate provided");
                return ESP_ERR_INVALID_ARG; // Missing client certificate; cannot verify client identity
            }
            if (s_mqtt_ctx->cckey) {
                mqtt_cfg->credentials.authentication.key = s_mqtt_ctx->cckey;
                ESP_LOGI(TAG, "Using client key");
            } else {
                ESP_LOGE(TAG, "Client certificate authentication is enabled, but no client key provided");
                return ESP_ERR_INVALID_ARG; // Missing client private key; cannot verify client identity
            }
        }
        // Determine whether a pre-shared key is required by current scheme.
        if (s_mqtt_ctx->scheme == MQTT_SCHEME_TLS_PSK || s_mqtt_ctx->scheme == MQTT_SCHEME_WEBSOCKET_TLS_PSK) {
            // Provide pre-shared key material
            if (s_mqtt_ctx->psk_hint_key.key != NULL) {
                mqtt_cfg->broker.verification.psk_hint_key = &s_mqtt_ctx->psk_hint_key;
                ESP_LOGI(TAG, "Using PSK hint key");
            } else {
                ESP_LOGE(TAG, "PSK authentication is enabled, but no PSK hint key provided");
                return ESP_ERR_INVALID_ARG; // Missing pre-shared key; cannot perform PSK authentication
            }
        }
    }

    return ESP_OK;
}

/**
 * @brief Start MQTT client; if already started, do not start again
 * Note: this function is thread-safe
 * 
 * @return esp_err_t 
 */
esp_err_t app_mqtt_start(void) {
    if (s_mqtt_ctx == NULL) {
        // MQTT context is not initialized; call app_mqtt_init() first
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err;
    // Acquire lock first to avoid races when app_mqtt_start is called concurrently
    if (xSemaphoreTake(s_mqtt_ctx->mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take MQTT context mutex");
        return ESP_ERR_TIMEOUT; // Failed to acquire mutex lock
    }
    // Already started; do not start again
    if (s_mqtt_ctx->is_client_started) {
        err = ESP_OK;
        goto start_done; // Already started
    }
    // Create the client only when handle is null
    if (s_mqtt_ctx->client == NULL) {
        // Initialize MQTT client configuration
        esp_mqtt_client_config_t mqtt_cfg;
        memset(&mqtt_cfg, 0, sizeof(mqtt_cfg));
        err = setup_esp_mqtt_client_config(&mqtt_cfg);
        if (err != ESP_OK) {
            goto start_done; // Configuration invalid; jump to cleanup
        }
        // Initialize MQTT client instance
        s_mqtt_ctx->client = esp_mqtt_client_init(&mqtt_cfg);
        if (s_mqtt_ctx->client == NULL) {
            ESP_LOGE(TAG, "Failed to initialize MQTT client");
            err = ESP_FAIL;  // Initialization failed for unspecified internal reason
            goto start_done; // MQTT client init failed; jump to cleanup
        }
    }
    // esp_mqtt_client_register_event internally guards against duplicate registration
    err = esp_mqtt_client_register_event(s_mqtt_ctx->client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    if (err != ESP_OK) {
        goto start_done; // Event handler registration failed
    }
    // Start client (internally starts MQTT protocol handling task)
    err = esp_mqtt_client_start(s_mqtt_ctx->client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "MQTT client started successfully");
        s_mqtt_ctx->is_client_started = true;
    } else {
        ESP_LOGE(TAG, "Failed to start MQTT client, error: %s", esp_err_to_name(err));
    }
start_done:
    // If start failed and instance was created, destroy it to avoid leaking unusable resources
    if (err != ESP_OK && s_mqtt_ctx->client) {
        esp_mqtt_client_destroy(s_mqtt_ctx->client);
        s_mqtt_ctx->client = NULL;
    }
    // Release lock at exit
    xSemaphoreGive(s_mqtt_ctx->mutex);
    return err;
}

/**
 * @brief Stop MQTT client; if already stopped, do not stop again
 * Note: this function is thread-safe
 * 
 * @return esp_err_t 
 */
esp_err_t app_mqtt_stop(void) {
    if (s_mqtt_ctx == NULL || s_mqtt_ctx->client == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err;
    // Acquire lock first to avoid races when app_mqtt_stop is called concurrently
    if (xSemaphoreTake(s_mqtt_ctx->mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take MQTT context mutex");
        return ESP_ERR_TIMEOUT; // Failed to acquire mutex lock
    }
    // Stop client if needed
    if (s_mqtt_ctx->is_client_started) {
        err = esp_mqtt_client_stop(s_mqtt_ctx->client);
        if (err != ESP_OK) {
            xSemaphoreGive(s_mqtt_ctx->mutex);
            ESP_LOGE(TAG, "Failed to stop MQTT client, error: %s", esp_err_to_name(err));
            return err;
        }
        s_mqtt_ctx->is_client_started = false;
    }
    // Destroy client if needed
    if (s_mqtt_ctx->client) {
        // Then destroy the client
        err = esp_mqtt_client_destroy(s_mqtt_ctx->client);
        if (err != ESP_OK) {
            xSemaphoreGive(s_mqtt_ctx->mutex);
            ESP_LOGE(TAG, "Failed to destroy MQTT client, error: %s", esp_err_to_name(err));
            return err;
        }
        s_mqtt_ctx->client = NULL;
    }
    s_mqtt_ctx->is_subscribed = false; // Reset to unsubscribed
    s_mqtt_ctx->is_connected = false;  // Mark as disconnected from MQTT broker
    xSemaphoreGive(s_mqtt_ctx->mutex);
    return ESP_OK;
}

/**
 * @brief Send MQTT message
 * 
 * @param data Message payload
 * @param length Message length
 * @return esp_err_t Returns ESP_OK on success, otherwise failure code
 */
esp_err_t app_mqtt_send(uint8_t *data, size_t length) {
    // To send MQTT messages, client must be initialized and publish topic must be configured
    if (s_mqtt_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    // app_mqtt_stop() destroys the client under this mutex (e.g. on WiFi drop).
    // Short wait: called from the UART task, and data sent while stopping is lost anyway.
    if (xSemaphoreTake(s_mqtt_ctx->mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    if (s_mqtt_ctx->client == NULL) {
        xSemaphoreGive(s_mqtt_ctx->mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (s_mqtt_ctx->publish_topic == NULL) {
        xSemaphoreGive(s_mqtt_ctx->mutex);
        return ESP_ERR_INVALID_ARG; // No publish topic configured
    }
    // Publish via esp_mqtt_client_publish; this wrapper publishes to a single configured topic
    int msg_id = esp_mqtt_client_publish(
        s_mqtt_ctx->client, 
        s_mqtt_ctx->publish_topic,  // Publish to configured topic
        (const char *)data,         // Payload data (treated as text by default)
        length,                     // Payload length
        s_mqtt_ctx->publish_qos,    // QoS level
        s_mqtt_ctx->publish_retain  // Retain flag
    );
    xSemaphoreGive(s_mqtt_ctx->mutex);
    if (msg_id < 0) {
        return ESP_FAIL; // Publish failed (e.g., disconnected client, memory pressure)
    }
    return ESP_OK;
}

/**
 * @brief Set callback for MQTT received messages
 * 
 * @param callback Callback function pointer
 * @return esp_err_t Returns ESP_OK on success, otherwise failure code
 */
esp_err_t app_mqtt_set_rx_callback(app_mqtt_client_rx_callback_t callback) {
    if (s_mqtt_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    s_mqtt_ctx->rx_callback = callback;
    return ESP_OK;
}

/**
 * @brief Get current MQTT client state
 * 
 * @param state State output pointer
 *  0: client not started
 *  1: client started but not connected to broker
 *  2: client connected but not subscribed
 *  3: client connected and subscribed
 * @return esp_err_t 
 */
esp_err_t app_mqtt_get_state(uint8_t *state) {
    if (s_mqtt_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_mqtt_ctx->is_client_started) {
        *state = 0; // Client not started
    } else if (!s_mqtt_ctx->is_connected) {
        *state = 1; // Started but not connected to broker
    } else if (!s_mqtt_ctx->is_subscribed) {
        *state = 2; // Connected but not subscribed
    } else {
        *state = 3; // Connected and subscribed
    }
    return ESP_OK;
}

/**
 * @brief Duplicate a string and free existing destination memory before replacement
 * 
 * @param dest Address of destination pointer
 * @param src Source string
 * @return esp_err_t Returns ESP_OK on success, ESP_ERR_NO_MEM if allocation fails
 */
esp_err_t strdup_with_free(char **dest, const char *src)
{
    // Free old memory first when destination is non-null
    if (*dest) {
        free(*dest);
    }
    // Duplicate new string; if source is NULL set destination to NULL
    if (src == NULL) {
        *dest = NULL;
        // ESP_LOGI(TAG, "Source string is NULL, setting destination to NULL");
        return ESP_OK;
    }
    // Check allocation result after duplication
    *dest = strdup(src);
    if (*dest == NULL) {
        return ESP_ERR_NO_MEM;
    }
    // ESP_LOGI(TAG, "String duplicated successfully, length %d, value: %s", strlen(*dest), *dest);
    return ESP_OK;
}

// This macro checks whether MQTT context is initialized; returns error if not.
#define CHECK_MQTT_CTX() do { \
    if (s_mqtt_ctx == NULL) { \
        return ESP_ERR_INVALID_STATE; \
    } \
} while(0)

// This macro ensures context exists and client is not started before mutating configuration.
// It is intended for setters; getters can still be called while running.
#define SET_CHECK_NOT_STARTED() do { \
    CHECK_MQTT_CTX(); \
    if (s_mqtt_ctx->is_client_started) { \
        return ESP_ERR_INVALID_STATE; \
    } \
} while(0)

esp_err_t app_mqtt_set_host(const char *host)
{
    SET_CHECK_NOT_STARTED();
    esp_err_t err = strdup_with_free(&s_mqtt_ctx->host, host);
    if (err != ESP_OK) {
        return err;
    }
    if (host) {
        return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
            { .key = KEY_MQTT_HOST, .type = APP_NVS_RW_TYPE_STR, .data = host, .length = strlen(host) + 1 }
        }, 1);
    }
    return app_nvs_rw_erase(NAMESPACE_MQTT_CLIENT, KEY_MQTT_HOST);
}

esp_err_t app_mqtt_get_host(char **host)
{
    CHECK_MQTT_CTX();
    *host = s_mqtt_ctx->host;
    return ESP_OK;
}

esp_err_t app_mqtt_set_port(uint16_t port)
{
    SET_CHECK_NOT_STARTED();
    s_mqtt_ctx->port = port;
    return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
        { .key = KEY_MQTT_PORT, .type = APP_NVS_RW_TYPE_U16, .data = &port, .length = sizeof(port) }
    }, 1);
}

esp_err_t app_mqtt_get_port(uint16_t *port)
{
    CHECK_MQTT_CTX();
    *port = s_mqtt_ctx->port;
    return ESP_OK;
}

esp_err_t app_mqtt_set_path(const char *path)
{
    SET_CHECK_NOT_STARTED();
    esp_err_t err = strdup_with_free(&s_mqtt_ctx->path, path);
    if (err != ESP_OK) {
        return err;
    }
    if (path) {
        return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
            { .key = KEY_MQTT_PATH, .type = APP_NVS_RW_TYPE_STR, .data = path, .length = strlen(path) + 1 }
        }, 1);
    }
    return app_nvs_rw_erase(NAMESPACE_MQTT_CLIENT, KEY_MQTT_PATH);
}

esp_err_t app_mqtt_get_path(char **path)
{
    CHECK_MQTT_CTX();
    *path = s_mqtt_ctx->path;
    return ESP_OK;
}

/**
 * @brief Set MQTT scheme (mqtt, mqtts, ws, wss)
 * 
 * @param scheme MQTT protocol: mqtt, mqtts, ws, wss
 *  See mqtt_scheme_t for details
 * @return esp_err_t 
 */
esp_err_t app_mqtt_set_scheme(uint8_t scheme)
{
    SET_CHECK_NOT_STARTED();
    s_mqtt_ctx->scheme = scheme;
    return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
        { .key = KEY_MQTT_SCHEME, .type = APP_NVS_RW_TYPE_U8, .data = &scheme, .length = sizeof(scheme) }
    }, 1);
}

esp_err_t app_mqtt_get_scheme(uint8_t *scheme)
{
    CHECK_MQTT_CTX();
    *scheme = s_mqtt_ctx->scheme;
    return ESP_OK;
}

esp_err_t app_mqtt_set_subscribe_topic(const char *subscribe_topic)
{
    SET_CHECK_NOT_STARTED();
    esp_err_t err = strdup_with_free(&s_mqtt_ctx->subscribe_topic, subscribe_topic);
    if (err != ESP_OK) {
        return err;
    }
    if (subscribe_topic) {
        return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
            { .key = KEY_MQTT_SUBSCRIBE_TOPIC, .type = APP_NVS_RW_TYPE_STR, .data = subscribe_topic, .length = strlen(subscribe_topic) + 1 }
        }, 1);
    }
    return app_nvs_rw_erase(NAMESPACE_MQTT_CLIENT, KEY_MQTT_SUBSCRIBE_TOPIC);
}

esp_err_t app_mqtt_get_subscribe_topic(char **subscribe_topic)
{
    CHECK_MQTT_CTX();
    *subscribe_topic = s_mqtt_ctx->subscribe_topic;
    return ESP_OK;
}

esp_err_t app_mqtt_set_subscribe_qos(uint8_t subscribe_qos)
{
    SET_CHECK_NOT_STARTED();
    s_mqtt_ctx->subscribe_qos = subscribe_qos;
    return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
        { .key = KEY_MQTT_SUBSCRIBE_QOS, .type = APP_NVS_RW_TYPE_U8, .data = &subscribe_qos, .length = sizeof(subscribe_qos) }
    }, 1);
}

esp_err_t app_mqtt_get_subscribe_qos(uint8_t *subscribe_qos)
{
    CHECK_MQTT_CTX();
    *subscribe_qos = s_mqtt_ctx->subscribe_qos;
    return ESP_OK;
}

esp_err_t app_mqtt_set_publish_topic(const char *publish_topic)
{
    SET_CHECK_NOT_STARTED();
    esp_err_t err = strdup_with_free(&s_mqtt_ctx->publish_topic, publish_topic);
    if (err != ESP_OK) {
        return err;
    }
    if (publish_topic) {
        return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
            { .key = KEY_MQTT_PUBLISH_TOPIC, .type = APP_NVS_RW_TYPE_STR, .data = publish_topic, .length = strlen(publish_topic) + 1 }
        }, 1);
    }
    return app_nvs_rw_erase(NAMESPACE_MQTT_CLIENT, KEY_MQTT_PUBLISH_TOPIC);
}

esp_err_t app_mqtt_get_publish_topic(char **publish_topic)
{
    CHECK_MQTT_CTX();
    *publish_topic = s_mqtt_ctx->publish_topic;
    return ESP_OK;
}

esp_err_t app_mqtt_set_publish_qos(uint8_t publish_qos)
{
    SET_CHECK_NOT_STARTED();
    s_mqtt_ctx->publish_qos = publish_qos;
    return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
        { .key = KEY_MQTT_PUBLISH_QOS, .type = APP_NVS_RW_TYPE_U8, .data = &publish_qos, .length = sizeof(publish_qos) }
    }, 1);
}

esp_err_t app_mqtt_get_publish_qos(uint8_t *publish_qos)
{
    CHECK_MQTT_CTX();
    *publish_qos = s_mqtt_ctx->publish_qos;
    return ESP_OK;
}

esp_err_t app_mqtt_set_publish_retain(uint8_t publish_retain)
{
    SET_CHECK_NOT_STARTED();
    s_mqtt_ctx->publish_retain = publish_retain;
    return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
        { .key = KEY_MQTT_PUBLISH_RETAIN, .type = APP_NVS_RW_TYPE_U8, .data = &publish_retain, .length = sizeof(publish_retain) }
    }, 1);
}

esp_err_t app_mqtt_get_publish_retain(uint8_t *publish_retain)
{
    CHECK_MQTT_CTX();
    *publish_retain = s_mqtt_ctx->publish_retain;
    return ESP_OK;
}

esp_err_t app_mqtt_set_client_id(const char *client_id)
{
    SET_CHECK_NOT_STARTED();
    esp_err_t err = strdup_with_free(&s_mqtt_ctx->client_id, client_id);
    if (err != ESP_OK) {
        return err;
    }
    if (client_id) {
        return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
            { .key = KEY_MQTT_CLIENT_ID, .type = APP_NVS_RW_TYPE_STR, .data = client_id, .length = strlen(client_id) + 1 }
        }, 1);
    }
    return app_nvs_rw_erase(NAMESPACE_MQTT_CLIENT, KEY_MQTT_CLIENT_ID);
}

esp_err_t app_mqtt_get_client_id(char **client_id)
{
    CHECK_MQTT_CTX();
    *client_id = s_mqtt_ctx->client_id;
    return ESP_OK;
}

esp_err_t app_mqtt_set_username(const char *username)
{
    SET_CHECK_NOT_STARTED();
    esp_err_t err = strdup_with_free(&s_mqtt_ctx->username, username);
    if (err != ESP_OK) {
        return err;
    }
    if (username) {
        return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
            { .key = KEY_MQTT_USERNAME, .type = APP_NVS_RW_TYPE_STR, .data = username, .length = strlen(username) + 1 }
        }, 1);
    }
    return app_nvs_rw_erase(NAMESPACE_MQTT_CLIENT, KEY_MQTT_USERNAME);
}

esp_err_t app_mqtt_get_username(char **username)
{
    CHECK_MQTT_CTX();
    *username = s_mqtt_ctx->username;
    return ESP_OK;
}

esp_err_t app_mqtt_set_password(const char *password)
{
    SET_CHECK_NOT_STARTED();
    esp_err_t err = strdup_with_free(&s_mqtt_ctx->password, password);
    if (err != ESP_OK) {
        return err;
    }
    if (password) {
        return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
            { .key = KEY_MQTT_PASSWORD, .type = APP_NVS_RW_TYPE_STR, .data = password, .length = strlen(password) + 1 }
        }, 1);
    }
    return app_nvs_rw_erase(NAMESPACE_MQTT_CLIENT, KEY_MQTT_PASSWORD);
}

esp_err_t app_mqtt_get_password(char **password)
{
    CHECK_MQTT_CTX();
    *password = s_mqtt_ctx->password;
    return ESP_OK;
}

esp_err_t app_mqtt_set_keepalive(int keepalive)
{
    SET_CHECK_NOT_STARTED();
    s_mqtt_ctx->keepalive = keepalive;
    return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
        { .key = KEY_MQTT_KEEPALIVE, .type = APP_NVS_RW_TYPE_I32, .data = &keepalive, .length = sizeof(keepalive) }
    }, 1);
}

esp_err_t app_mqtt_get_keepalive(int *keepalive)
{
    CHECK_MQTT_CTX();
    *keepalive = s_mqtt_ctx->keepalive;
    return ESP_OK;
}

esp_err_t app_mqtt_set_disable_clean_session(uint8_t disable_clean_session)
{
    SET_CHECK_NOT_STARTED();
    s_mqtt_ctx->disable_clean_session = disable_clean_session;
    return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
        { .key = KEY_MQTT_CLEAN_SESSION, .type = APP_NVS_RW_TYPE_U8, .data = &disable_clean_session, .length = sizeof(disable_clean_session) }
    }, 1);
}

esp_err_t app_mqtt_get_disable_clean_session(uint8_t *disable_clean_session)
{
    CHECK_MQTT_CTX();
    *disable_clean_session = s_mqtt_ctx->disable_clean_session;
    return ESP_OK;
}

esp_err_t app_mqtt_set_lwt_topic(const char *lwt_topic)
{
    SET_CHECK_NOT_STARTED();
    esp_err_t err = strdup_with_free(&s_mqtt_ctx->lwt_topic, lwt_topic);
    if (err != ESP_OK) {
        return err;
    }
    if (lwt_topic) {
        return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
            { .key = KEY_MQTT_LWT_TOPIC, .type = APP_NVS_RW_TYPE_STR, .data = lwt_topic, .length = strlen(lwt_topic) + 1 }
        }, 1);
    }
    return app_nvs_rw_erase(NAMESPACE_MQTT_CLIENT, KEY_MQTT_LWT_TOPIC);
}

esp_err_t app_mqtt_get_lwt_topic(char **lwt_topic)
{
    CHECK_MQTT_CTX();
    *lwt_topic = s_mqtt_ctx->lwt_topic;
    return ESP_OK;
}

esp_err_t app_mqtt_set_lwt_message(const char *lwt_message)
{
    SET_CHECK_NOT_STARTED();
    esp_err_t err = strdup_with_free(&s_mqtt_ctx->lwt_message, lwt_message);
    if (err != ESP_OK) {
        return err;
    }
    if (lwt_message) {
        return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
            { .key = KEY_MQTT_LWT_MESSAGE, .type = APP_NVS_RW_TYPE_STR, .data = lwt_message, .length = strlen(lwt_message) + 1 }
        }, 1);
    }
    return app_nvs_rw_erase(NAMESPACE_MQTT_CLIENT, KEY_MQTT_LWT_MESSAGE);
}

esp_err_t app_mqtt_get_lwt_message(char **lwt_message)
{
    CHECK_MQTT_CTX();
    *lwt_message = s_mqtt_ctx->lwt_message;
    return ESP_OK;
}

esp_err_t app_mqtt_set_lwt_qos(uint8_t lwt_qos)
{
    SET_CHECK_NOT_STARTED();
    s_mqtt_ctx->lwt_qos = lwt_qos;
    return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
        { .key = KEY_MQTT_LWT_QOS, .type = APP_NVS_RW_TYPE_U8, .data = &lwt_qos, .length = sizeof(lwt_qos) }
    }, 1);
}

esp_err_t app_mqtt_get_lwt_qos(uint8_t *lwt_qos)
{
    CHECK_MQTT_CTX();
    *lwt_qos = s_mqtt_ctx->lwt_qos;
    return ESP_OK;
}

esp_err_t app_mqtt_set_lwt_retain(uint8_t lwt_retain)
{
    SET_CHECK_NOT_STARTED();
    s_mqtt_ctx->lwt_retain = lwt_retain;
    return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
        { .key = KEY_MQTT_LWT_RETAIN, .type = APP_NVS_RW_TYPE_U8, .data = &lwt_retain, .length = sizeof(lwt_retain) }
    }, 1);
}

esp_err_t app_mqtt_get_lwt_retain(uint8_t *lwt_retain)
{
    CHECK_MQTT_CTX();
    *lwt_retain = s_mqtt_ctx->lwt_retain;
    return ESP_OK;
}

esp_err_t app_mqtt_alpn_add(const char *alpn)
{
    SET_CHECK_NOT_STARTED();
    if (s_mqtt_ctx->alpn_count >= MQTT_ALPN_COUNT_MAX) {
        return ESP_ERR_NO_MEM;
    }
    size_t alpn_len = strlen(alpn);
    char *alpn_copy = malloc(alpn_len + 1);
    if (alpn_copy == NULL) {
        return ESP_ERR_NO_MEM;
    }
    strcpy(alpn_copy, alpn);
    s_mqtt_ctx->alpn_list[s_mqtt_ctx->alpn_count++] = alpn_copy;
    s_mqtt_ctx->alpn_list[s_mqtt_ctx->alpn_count] = NULL; // Ensure the element after last valid ALPN entry is NULL
    return persist_mqtt_alpn_list();
}

esp_err_t app_mqtt_alpn_get_count(uint8_t *alpn_count) {
    CHECK_MQTT_CTX();
    *alpn_count = s_mqtt_ctx->alpn_count;
    return ESP_OK;
}

esp_err_t app_mqtt_alpn_get_list(char ***alpn_list, uint8_t *alpn_count)
{
    CHECK_MQTT_CTX();
    *alpn_list = s_mqtt_ctx->alpn_list;
    *alpn_count = s_mqtt_ctx->alpn_count;
    return ESP_OK;
}

esp_err_t app_mqtt_alpn_remove(uint8_t index)
{
    SET_CHECK_NOT_STARTED();
    if (index >= s_mqtt_ctx->alpn_count) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_mqtt_ctx->alpn_list[index]) {
        free(s_mqtt_ctx->alpn_list[index]);
    }
    for (uint8_t i = index; i < s_mqtt_ctx->alpn_count - 1; i++) {
        s_mqtt_ctx->alpn_list[i] = s_mqtt_ctx->alpn_list[i + 1];
    }
    s_mqtt_ctx->alpn_list[--s_mqtt_ctx->alpn_count] = NULL;
    return persist_mqtt_alpn_list();
}

esp_err_t app_mqtt_alpn_clear(void)
{
    SET_CHECK_NOT_STARTED();
    for (uint8_t i = 0; i < s_mqtt_ctx->alpn_count; i++) {
        if (s_mqtt_ctx->alpn_list[i]) {
            free(s_mqtt_ctx->alpn_list[i]);
            s_mqtt_ctx->alpn_list[i] = NULL;
        }
    }
    s_mqtt_ctx->alpn_count = 0;
    s_mqtt_ctx->alpn_list[0] = NULL;
    return persist_mqtt_alpn_list();
}

esp_err_t app_mqtt_set_sni_host(const char *sni_host)
{
    SET_CHECK_NOT_STARTED();
    esp_err_t err = strdup_with_free(&s_mqtt_ctx->sni_host, sni_host);
    if (err != ESP_OK) {
        return err;
    }
    if (sni_host) {
        return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
            { .key = KEY_MQTT_SNI_HOST, .type = APP_NVS_RW_TYPE_STR, .data = sni_host, .length = strlen(sni_host) + 1 }
        }, 1);
    }
    return app_nvs_rw_erase(NAMESPACE_MQTT_CLIENT, KEY_MQTT_SNI_HOST);
}

esp_err_t app_mqtt_get_sni_host(char **sni_host)
{
    CHECK_MQTT_CTX();
    *sni_host = s_mqtt_ctx->sni_host;
    return ESP_OK;
}

esp_err_t app_mqtt_set_cacert(const char *cacert)
{
    SET_CHECK_NOT_STARTED();
    esp_err_t err = strdup_with_free(&s_mqtt_ctx->cacert, cacert);
    if (err != ESP_OK) {
        return err;
    }
    if (cacert) {
        return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
            { .key = KEY_MQTT_CACERT, .type = APP_NVS_RW_TYPE_STR, .data = cacert, .length = strlen(cacert) + 1 }
        }, 1);
    }
    return app_nvs_rw_erase(NAMESPACE_MQTT_CLIENT, KEY_MQTT_CACERT);
}

esp_err_t app_mqtt_get_cacert(char **cacert)
{
    CHECK_MQTT_CTX();
    *cacert = s_mqtt_ctx->cacert;
    return ESP_OK;
}

esp_err_t app_mqtt_set_ccert(const char *ccert)
{
    SET_CHECK_NOT_STARTED();
    esp_err_t err = strdup_with_free(&s_mqtt_ctx->ccert, ccert);
    if (err != ESP_OK) {
        return err;
    }
    if (ccert) {
        return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
            { .key = KEY_MQTT_CCERT, .type = APP_NVS_RW_TYPE_STR, .data = ccert, .length = strlen(ccert) + 1 }
        }, 1);
    }
    return app_nvs_rw_erase(NAMESPACE_MQTT_CLIENT, KEY_MQTT_CCERT);
}

esp_err_t app_mqtt_get_ccert(char **ccert)
{
    CHECK_MQTT_CTX();
    *ccert = s_mqtt_ctx->ccert;
    return ESP_OK;
}

esp_err_t app_mqtt_set_cckey(const char *cckey)
{
    SET_CHECK_NOT_STARTED();
    esp_err_t err = strdup_with_free(&s_mqtt_ctx->cckey, cckey);
    if (err != ESP_OK) {
        return err;
    }
    if (cckey) {
        return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
            { .key = KEY_MQTT_CCKEY, .type = APP_NVS_RW_TYPE_STR, .data = cckey, .length = strlen(cckey) + 1 }
        }, 1);
    }
    return app_nvs_rw_erase(NAMESPACE_MQTT_CLIENT, KEY_MQTT_CCKEY);
}

esp_err_t app_mqtt_get_cckey(char **cckey)
{
    CHECK_MQTT_CTX();
    *cckey = s_mqtt_ctx->cckey;
    return ESP_OK;
}

esp_err_t app_mqtt_set_psk_key(const uint8_t *key, size_t key_size) {
    SET_CHECK_NOT_STARTED();
    if (key_size > 0 && key == NULL) {
        return ESP_ERR_INVALID_ARG; // Invalid args: key_size > 0 but key is NULL
    }
    if (key == NULL || key_size == 0) {
        if (s_mqtt_ctx->psk_hint_key.key) {
            release_psk_data();
        }
        return app_nvs_rw_erase(NAMESPACE_MQTT_CLIENT, KEY_MQTT_PSK_KEY);
    }
    if (s_mqtt_ctx->psk_hint_key.key) {
        if (s_mqtt_ctx->psk_hint_key.key_size == key_size) {
            memcpy((uint8_t*)s_mqtt_ctx->psk_hint_key.key, key, key_size);
            return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
                { .key = KEY_MQTT_PSK_KEY, .type = APP_NVS_RW_TYPE_BLOB, .data = key, .length = key_size }
            }, 1);
        } else {
            release_psk_data();
        }
    }
    uint8_t *key_copy = malloc(key_size);
    if (key_copy == NULL) {
        return ESP_ERR_NO_MEM;
    }
    memcpy(key_copy, key, key_size);
    s_mqtt_ctx->psk_hint_key.key = key_copy;
    s_mqtt_ctx->psk_hint_key.key_size = key_size;
    return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
        { .key = KEY_MQTT_PSK_KEY, .type = APP_NVS_RW_TYPE_BLOB, .data = key, .length = key_size }
    }, 1);
}

esp_err_t app_mqtt_get_psk_key(uint8_t **key, size_t *key_size) {
    CHECK_MQTT_CTX();
    *key = (uint8_t *)s_mqtt_ctx->psk_hint_key.key;
    *key_size = s_mqtt_ctx->psk_hint_key.key_size;
    return ESP_OK;
}

esp_err_t app_mqtt_set_psk_hint(const char *hint) {
    SET_CHECK_NOT_STARTED();
    esp_err_t err = strdup_with_free((char**)&s_mqtt_ctx->psk_hint_key.hint, hint);
    if (err != ESP_OK) {
        return err;
    }
    if (hint) {
        return app_nvs_rw_write(NAMESPACE_MQTT_CLIENT, (app_nvs_rw_write_item_t[]){
            { .key = KEY_MQTT_PSK_HINT, .type = APP_NVS_RW_TYPE_STR, .data = hint, .length = strlen(hint) + 1 }
        }, 1);
    }
    return app_nvs_rw_erase(NAMESPACE_MQTT_CLIENT, KEY_MQTT_PSK_HINT);
}

esp_err_t app_mqtt_get_psk_hint(char **hint) {
    CHECK_MQTT_CTX();
    *hint = (char *)s_mqtt_ctx->psk_hint_key.hint;
    return ESP_OK;
}
