#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_att.h"
#include "host/util/util.h"
#include "host/ble_store.h"
#include "host/ble_sm.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "os/os_mbuf.h"

#include "esp_mac.h"
#include "esp_bt.h"
#include "console/console.h"
#include "ble_spp_server.h"
#include "app_ble_spp.h"
#include "app_nvs_rw.h"


#define NOTIFY_NOMEM_RETRY_MAX          250
#define NOTIFY_FAIL_RETRY_MAX           250
#define DEFAULT_BONDING_KEY             123456U // default 6-digit pairing passkey, avoids phones misinterpreting leading-zero passkeys as a short PIN
#define DEFAULT_BAS_VALUE               0xFFU // 0~100 represents battery level; 255 means unknown battery level
#define NAMESPACE_BLE_SPP               "app_ble"
#define KEY_BLE_BONDING_ENABLE          "ble_bond_en"
#define KEY_BLE_BONDING_PASSKEY         "ble_bond_key"
#define KEY_BLE_ADV_MFG_DATA            "ble_adv_mfg"
#define KEY_BLE_DEVICE_NAME             "ble_name"
#define KEY_BLE_DEVICE_ADDR             "ble_addr"
// Advertising duty. Zeroed interval params = NimBLE's "fast" default (30-60 ms):
// quick to find, but kept up forever it is the module's largest idle radio cost.
// Low duty: fast for ADV_FAST_DURATION_MS after boot, a disconnect or a duty
// change, then ADV_SLOW_ITVL, under the 1.285 s that phones scan for reliably.
#define ADV_FAST_DURATION_MS            30000
#define ADV_SLOW_ITVL                   1636    // 1022.5 ms in 0.625 ms units
#define KEY_BLE_NOTIFY_RETRY_NOMEM      "ble_ntf_nmem"
#define KEY_BLE_NOTIFY_RETRY_FAIL       "ble_ntf_fail"
#define KEY_BLE_TX_POWER_ADV            "ble_txp_adv"
#define KEY_BLE_TX_POWER_CONN           "ble_txp_conn"


typedef struct {
    // State machine for the BLE wrapper module; indicates the current BLE module state
    // so different logic can be applied in each state.
    enum {
        APP_BLE_STATE_CTX_READY = 0,   // Context created; NimBLE stack not yet initialized
        APP_BLE_STATE_STACK_BOOTING,   // NimBLE started; waiting for sync callback
        APP_BLE_STATE_RUNNING,         // NimBLE synced; advertising/communication available
    } state;

    // Whether Notify has been enabled. If not enabled, app_ble_send() returns an error
    // immediately because data cannot be sent without an active notification subscription.
    bool data_notify_enabled;
    // Whether the Battery Level characteristic has Notify enabled.
    bool battery_notify_enabled;
    // Handle of the current connection; BLE_HS_CONN_HANDLE_NONE when not connected.
    uint16_t conn_handle;
    // Value handle of the SPP characteristic; required when sending data to specify the target.
    uint16_t spp_chr_val_handle;
    // Value handle of the Battery Level characteristic.
    uint16_t battery_chr_val_handle;
    // RX data callback pointer; set via app_ble_set_rx_callback() to be notified when data arrives.
    app_ble_rx_callback_t rx_callback;
    // Link callback; set via app_ble_set_link_callback(), called on connect / disconnect.
    app_ble_link_callback_t link_callback;
    // Pairing security config (dynamic): MITM is enabled by default when bonding is enabled
    uint8_t bonding;
    // Pairing passkey
    uint32_t static_passkey;
    // Manufacturer-specific data field in advertising packets
    uint8_t adv_mfg_data_len;
    uint8_t adv_mfg_data[APP_BLE_ADV_MFG_DATA_MAX_LEN];
    // Cached device name (can be set/read before start)
    char device_name[APP_BLE_DEVICE_NAME_MAX_LEN];
    // Battery level; range 0~100; 255 means unknown.
    uint8_t battery_level;
    // Three bytes are reserved for the ATT header, so max data transfer size is BLE_ATT_ATTR_MAX_LEN - 3
    uint8_t rx_static_buf[BLE_ATT_ATTR_MAX_LEN - 3];
    // Upper limit for notify retry counts
    uint16_t notify_nomem_retry_max;
    uint16_t notify_fail_retry_max;
    // TX power level
    uint8_t tx_power_level_for_advertising;
    uint8_t tx_power_level_for_connection;
} app_ble_ctx_t;

static app_ble_ctx_t *s_ctx = NULL;
// power-save advertising (two-phase) or NimBLE's default; see ADV_FAST_DURATION_MS
static bool s_adv_low_duty = true;
// true once the fast phase expired without a connection
static bool s_adv_slow = false;
static const char *TAG = "ble_spp";


/**
 * @brief Reads the default TX power level from sdkconfig and converts it to an
 * esp_power_level_t enum value.
 * 
 * @return esp_power_level_t The default TX power level as esp_power_level_t,
 *  or a reasonable default if the sdkconfig value is invalid.
 */
static esp_power_level_t from_sdkconfig_get_default_tx_power_level(void) {
    // Read the default TX power level from sdkconfig (in dBm as an integer) and
    // convert it to esp_power_level_t. Levels increment by 3 dBm per step.
    int default_tx_power_dbm = CONFIG_BT_LE_DFT_TX_POWER_LEVEL_DBM_EFF;
    if (default_tx_power_dbm < -24) {
        default_tx_power_dbm = -24;
    } else if (default_tx_power_dbm > 20) {
        default_tx_power_dbm = 20;
    }
    // Convert dBm to level: -24 dBm maps to level 0; each +3 dBm increments level by 1
    int level = (default_tx_power_dbm + 24) / 3;
    // Clamp to the valid esp_power_level_t range: ESP_PWR_LVL_N24=0, ESP_PWR_LVL_P20=15
    if (level < 0) {
        level = ESP_PWR_LVL_P3;
    } else if (level > ESP_PWR_LVL_P20) {
        level = ESP_PWR_LVL_P20;
    }
    return (esp_power_level_t)level;
}

/**
 * @brief Converts an esp_power_level_t enum value to its corresponding dBm value,
 * using the rule of +3 dBm per level step.
 * @param level esp_power_level_t enum value representing the TX power level
 * @return int8_t The corresponding dBm value, or a reasonable default if the level is invalid.
 */
static int8_t to_tx_power_dbm(esp_power_level_t level) {
    // Convert esp_power_level_t to dBm: level 0 = -24 dBm, each level adds 3 dBm
    if (level > ESP_PWR_LVL_P20) {
        level = ESP_PWR_LVL_P20;
    }
    int8_t dbm = (int8_t)(level * 3 - 24); // level 0 = -24 dBm; each +1 level adds 3 dBm
    // Clamp to valid range: ESP_PWR_LVL_N24 = -24 dBm, ESP_PWR_LVL_P20 = 20 dBm
    if (dbm < -24) {
        dbm = -24;
    } else if (dbm > 20) {
        dbm = 20;
    }
    return dbm;
}

/**
 * @brief Loads persisted BLE parameters.
 * 
 * @param ctx Pointer to the BLE context
 * @return esp_err_t ESP_OK on success, other value on failure
 */
static esp_err_t app_ble_load_persisted_params(app_ble_ctx_t *ctx) {
    if (ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    char *persisted_name = NULL;
    uint8_t *persisted_adv_mfg_data = NULL;
    uint8_t *persisted_device_addr = NULL;
    app_nvs_rw_read_item_t items[] = {
        {
            .key = KEY_BLE_BONDING_ENABLE,
            .type = APP_NVS_RW_TYPE_U8,
            .data = &ctx->bonding,
            .default_value = 0,
        }, {
            .key = KEY_BLE_BONDING_PASSKEY,
            .type = APP_NVS_RW_TYPE_U32,
            .data = &ctx->static_passkey,
            .default_value = DEFAULT_BONDING_KEY,
        }, {
            .key = KEY_BLE_ADV_MFG_DATA,
            .type = APP_NVS_RW_TYPE_BLOB,
            .data = &persisted_adv_mfg_data,
            .default_value = 0,
        }, {
            .key = KEY_BLE_DEVICE_NAME,
            .type = APP_NVS_RW_TYPE_STR,
            .data = &persisted_name,
            .default_value = 0,
        }, {
            .key = KEY_BLE_DEVICE_ADDR,
            .type = APP_NVS_RW_TYPE_BLOB,
            .data = &persisted_device_addr,
            .default_value = 0,
        }, {
            .key = KEY_BLE_NOTIFY_RETRY_NOMEM,
            .type = APP_NVS_RW_TYPE_U16,
            .data = &ctx->notify_nomem_retry_max,
            .default_value = NOTIFY_NOMEM_RETRY_MAX,
        }, {
            .key = KEY_BLE_NOTIFY_RETRY_FAIL,
            .type = APP_NVS_RW_TYPE_U16,
            .data = &ctx->notify_fail_retry_max,
            .default_value = NOTIFY_FAIL_RETRY_MAX,
        }, {
            .key = KEY_BLE_TX_POWER_ADV,
            .type = APP_NVS_RW_TYPE_U8,
            .data = &ctx->tx_power_level_for_advertising,
            .default_value = (uint64_t)from_sdkconfig_get_default_tx_power_level(),
        }, {
            .key = KEY_BLE_TX_POWER_CONN,
            .type = APP_NVS_RW_TYPE_U8,
            .data = &ctx->tx_power_level_for_connection,
            .default_value = (uint64_t)from_sdkconfig_get_default_tx_power_level(),
        },
    };

    // Read persisted parameters; if any read fails, return an error and free any dynamically allocated memory.
    const esp_err_t err = APP_NVS_RW_READ(NAMESPACE_BLE_SPP, items);
    if (err != ESP_OK) {
        free(persisted_name);
        free(persisted_adv_mfg_data);
        free(persisted_device_addr);
        return err;
    }

    // Copy device name
    if (persisted_name != NULL) {
        const size_t persisted_name_len = strlen(persisted_name);
        if (persisted_name_len < sizeof(ctx->device_name)) {
            memcpy(ctx->device_name, persisted_name, persisted_name_len + 1);
        } else {
            ESP_LOGW(TAG, "Persisted BLE name is too long(%u), keep current default",
                     (unsigned int)persisted_name_len);
        }
    }

    // Copy advertising manufacturer data
    if (persisted_adv_mfg_data != NULL) {
        if (items[2].out_len <= APP_BLE_ADV_MFG_DATA_MAX_LEN) {
            memcpy(ctx->adv_mfg_data, persisted_adv_mfg_data, items[2].out_len);
            ctx->adv_mfg_data_len = (uint8_t)items[2].out_len;
        } else {
            ESP_LOGW(TAG, "Persisted advertising manufacturer data is too long(%u), ignore",
                     (unsigned int)items[2].out_len);
        }
    }

    // Apply persisted device address
    if (persisted_device_addr != NULL) {
        if (items[4].out_len == BLE_DEV_ADDR_LEN) {
            const esp_err_t set_addr_err = esp_iface_mac_addr_set(persisted_device_addr, ESP_MAC_BT);
            if (set_addr_err != ESP_OK) {
                ESP_LOGW(TAG, "Failed to apply persisted BT MAC address: %s", esp_err_to_name(set_addr_err));
            }
        } else {
            ESP_LOGW(TAG, "Persisted BLE address length is invalid(%u), ignore",
                     (unsigned int)items[4].out_len);
        }
    }

    // Must free dynamically allocated memory; it will not be used again
    free(persisted_name);
    free(persisted_adv_mfg_data);
    free(persisted_device_addr);

    // Validate parameters; reset to defaults if out of range
    if (ctx->static_passkey > 999999U) {
        ctx->static_passkey = DEFAULT_BONDING_KEY;
        ESP_LOGW(TAG, "Persisted passkey is invalid, fallback to default");
    }
    if (ctx->tx_power_level_for_advertising > ESP_PWR_LVL_P20) {
        ctx->tx_power_level_for_advertising = (uint8_t)from_sdkconfig_get_default_tx_power_level();
        ESP_LOGW(TAG, "Persisted advertising tx power is invalid, fallback to default");
    }
    if (ctx->tx_power_level_for_connection > ESP_PWR_LVL_P20) {
        ctx->tx_power_level_for_connection = (uint8_t)from_sdkconfig_get_default_tx_power_level();
        ESP_LOGW(TAG, "Persisted connection tx power is invalid, fallback to default");
    }

    return ESP_OK;
}


static int ble_spp_server_gap_event(struct ble_gap_event *event, void *arg);


/**
 * @brief Infers a suitable address type based on the current BLE configuration.
 * 
 * @param addr_type Output parameter for the inferred address type.
 * @return int 0 on success; non-zero on failure (see ble_hs_id_infer_auto return values).
 */
static int app_get_addr_type(uint8_t *addr_type, char* msg_on_error) {
    // Infer a suitable address type from the current BLE configuration
    int rc = ble_hs_id_infer_auto(0, addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to infer own address type(%s): rc=%d", msg_on_error, rc);
        return rc;
    }
    // BLE_ADDR_RANDOM or BLE_ADDR_PUBLIC
    ESP_LOGI(TAG, "Inferred own address type: %d", *addr_type);
    return 0;
}

/**
 * @brief Returns the currently active connection handle.
 * 
 * @return uint16_t Active connection handle, or BLE_HS_CONN_HANDLE_NONE if not connected.
 */
static inline uint16_t app_ble_get_active_conn_handle(void) {
    if (s_ctx == NULL) {
        return BLE_HS_CONN_HANDLE_NONE;
    }
    return s_ctx->conn_handle;
}

/**
 * @brief Converts a security manager error code to a string representation.
 * 
 * @param sm_err Security manager error code
 * @return const char* String representation of the error code
 */
static const char *app_ble_sm_err_to_str(int sm_err) {
    switch (sm_err) {
    case BLE_SM_ERR_SUCCESS:
        return "success";
    case BLE_SM_ERR_PASSKEY:
        return "passkey entry failed";
    case BLE_SM_ERR_OOB:
        return "oob data invalid";
    case BLE_SM_ERR_AUTHREQ:
        return "authentication requirements not met";
    case BLE_SM_ERR_CONFIRM_MISMATCH:
        return "confirm mismatch";
    case BLE_SM_ERR_PAIR_NOT_SUPP:
        return "pairing not supported";
    case BLE_SM_ERR_ENC_KEY_SZ:
        return "invalid encryption key size";
    case BLE_SM_ERR_CMD_NOT_SUPP:
        return "security manager command not supported";
    case BLE_SM_ERR_UNSPECIFIED:
        return "unspecified security manager error";
    case BLE_SM_ERR_REPEATED:
        return "repeated pairing attempt";
    case BLE_SM_ERR_INVAL:
        return "invalid security manager parameters";
    case BLE_SM_ERR_DHKEY:
        return "dhkey check failed";
    case BLE_SM_ERR_NUMCMP:
        return "numeric comparison failed";
    case BLE_SM_ERR_ALREADY:
        return "pairing already in progress";
    case BLE_SM_ERR_CROSS_TRANS:
        return "cross transport key generation not allowed";
    case BLE_SM_ERR_KEY_REJ:
        return "key rejected";
    default:
        return "unknown security manager error";
    }
}

/**
 * @brief Logs the connection encryption status.
 * 
 * @param status Encryption status code
 */
static void app_ble_log_enc_status(int status) {
    if (status >= BLE_HS_ERR_SM_US_BASE &&
        status < (BLE_HS_ERR_SM_US_BASE + BLE_SM_ERR_MAX_PLUS_1)) {
        const int sm_err = status - BLE_HS_ERR_SM_US_BASE;
        ESP_LOGE(TAG,
                 "connection encryption failed, status: %d (local security manager: %s)",
                 status,
                 app_ble_sm_err_to_str(sm_err));
        return;
    }

    if (status >= BLE_HS_ERR_SM_PEER_BASE &&
        status < (BLE_HS_ERR_SM_PEER_BASE + BLE_SM_ERR_MAX_PLUS_1)) {
        const int sm_err = status - BLE_HS_ERR_SM_PEER_BASE;
        ESP_LOGE(TAG,
                 "connection encryption failed, status: %d (peer security manager: %s)",
                 status,
                 app_ble_sm_err_to_str(sm_err));
        return;
    }

    ESP_LOGE(TAG, "connection encryption failed, status: %d", status);
}

/**
 * @brief Applies the security manager configuration.
 * 
 * @param bonding Whether bonding is enabled
 */
static void app_ble_apply_sm_cfg(uint8_t bonding) {
    ble_hs_cfg.sm_io_cap = bonding ? BLE_HS_IO_DISPLAY_ONLY : BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_oob_data_flag = 0;
    ble_hs_cfg.sm_bonding = bonding;
    ble_hs_cfg.sm_mitm = bonding ? 1 : 0;
    ble_hs_cfg.sm_sc = bonding ? 1 : 0;
    // sm_sec_lvl=1 means "security not supported": NimBLE immediately replies with
    // Pairing Failed (Command Not Supported) when it receives a Pairing Request,
    // rejecting before sending a Pairing Response — the phone never sees a response
    // and will not display any pairing dialog.
    ble_hs_cfg.sm_sec_lvl = bonding ? 0 : 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
}

/**
 * @brief Creates a BLE context.
 * 
 * @return app_ble_ctx_t* Pointer to the created context, or NULL on failure
 */
static app_ble_ctx_t *app_ble_ctx_create(void) {
    app_ble_ctx_t *ctx = calloc(1, sizeof(app_ble_ctx_t));
    if (ctx == NULL) {
        return NULL;
    }

    ctx->notify_nomem_retry_max = NOTIFY_NOMEM_RETRY_MAX;
    ctx->notify_fail_retry_max = NOTIFY_FAIL_RETRY_MAX;
    ctx->conn_handle = BLE_HS_CONN_HANDLE_NONE;
    ctx->state = APP_BLE_STATE_CTX_READY;
    ctx->static_passkey = DEFAULT_BONDING_KEY; // default pairing passkey; user can change via app_ble_set_bonding_key
    ctx->battery_level = DEFAULT_BAS_VALUE;
    memcpy(ctx->device_name, CONFIG_DEVICE_IDENTIFIER, sizeof(CONFIG_DEVICE_IDENTIFIER));
    ctx->tx_power_level_for_advertising = from_sdkconfig_get_default_tx_power_level();
    ctx->tx_power_level_for_connection = from_sdkconfig_get_default_tx_power_level();

    return ctx;
}

/**
 * @brief Converts a BLE address to its string representation.
 * 
 * @param addr Pointer to the BLE address
 * @return char* String representation of the address
 */
static char* app_addr_to_str(const uint8_t *addr) {
    static char app_addr_to_str[18]; // 6-byte address: 2 chars per byte + 5 colons + 1 null terminator = 18
    sprintf(app_addr_to_str, "%02X:%02X:%02X:%02X:%02X:%02X",
             addr[5], addr[4], addr[3],
             addr[2], addr[1], addr[0]);
    return app_addr_to_str;
}

/**
 * @brief Prints connection descriptor information.
 * 
 * @param desc Pointer to the connection descriptor
 */
static void ble_spp_server_print_conn_desc(struct ble_gap_conn_desc *desc) {
    ESP_LOGI(TAG, "--------------------------------");
    ESP_LOGI(TAG, " * our_ota_addr_type   = %d, our_ota_addr  = %s", desc->our_ota_addr.type, app_addr_to_str(desc->our_ota_addr.val));
    ESP_LOGI(TAG, " * our_id_addr_type    = %d, our_id_addr   = %s", desc->our_id_addr.type, app_addr_to_str(desc->our_id_addr.val));
    ESP_LOGI(TAG, " * peer_ota_addr_type  = %d, peer_ota_addr = %s", desc->peer_ota_addr.type, app_addr_to_str(desc->peer_ota_addr.val));
    ESP_LOGI(TAG, " * peer_id_addr_type   = %d, peer_id_addr  = %s", desc->peer_id_addr.type, app_addr_to_str(desc->peer_id_addr.val));
    ESP_LOGI(TAG, " * handle=%d, conn_itvl=%d, conn_latency=%d, supervision_timeout=%d, encrypted=%d, authenticated=%d, bonded=%d",
             desc->conn_handle,
             desc->conn_itvl,
             desc->conn_latency,
             desc->supervision_timeout,
             desc->sec_state.encrypted,
             desc->sec_state.authenticated,
             desc->sec_state.bonded);
    ESP_LOGI(TAG, "--------------------------------");
}

/**
 * @brief Configures BLE advertising fields.
 * 
 * @return int 0 on success, non-zero on failure
 */
static int app_ble_config_adv_fields(void) {
    if (s_ctx == NULL) {
        return BLE_HS_EINVAL;
    }

    struct ble_hs_adv_fields fields = {0};
    
    // Set advertising flags: device is generally discoverable and does not support BR/EDR (classic Bluetooth).
    // This causes the device to appear as a connectable BLE device in scan results.
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    // Include Tx Power Level in advertising data
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = to_tx_power_dbm(s_ctx->tx_power_level_for_advertising);

    // Include device name in advertising data
    fields.name = (uint8_t *)s_ctx->device_name;
    fields.name_len = strlen(s_ctx->device_name);
    fields.name_is_complete = 1;

    // Include 16-bit service UUIDs in advertising data
    fields.uuids16 = (ble_uuid16_t[]) {
        BLE_UUID16_INIT(BLE_SVC_SPP_UUID16),
        BLE_UUID16_INIT(BLE_SVC_BAS_UUID16),
    };
    fields.num_uuids16 = 2;
    fields.uuids16_is_complete = 1;

    return ble_gap_adv_set_fields(&fields);
}

/**
 * @brief Configures BLE advertising response fields.
 * 
 * @return int 0 on success, non-zero on failure
 */
static int app_ble_config_adv_rsp_fields(void) {
    if (s_ctx == NULL) {
        return BLE_HS_EINVAL;
    }

    struct ble_hs_adv_fields fields = {0};

    if (s_ctx->adv_mfg_data_len > 0U) {
        fields.mfg_data = s_ctx->adv_mfg_data;
        fields.mfg_data_len = s_ctx->adv_mfg_data_len;
    }

    return ble_gap_adv_rsp_set_fields(&fields);
}

/**
 * @brief Starts BLE advertising.
 */
static void ble_spp_server_advertise(void) {
    // Check if advertising can be started: if the BLE stack is not ready, a device is already
    // connected, or advertising is already active, no action is needed.
    if (s_ctx == NULL) {
        return;
    }
    if (s_ctx->state != APP_BLE_STATE_RUNNING) {
        ESP_LOGW(TAG, "BLE stack is not ready, cannot start advertising, current state: %d", s_ctx->state);
        return;
    }
    if (s_ctx->conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGW(TAG, "Already connected, no need to start advertising");
        return;
    }
    if (ble_gap_adv_active()) {
        ESP_LOGW(TAG, "Advertising is already active, no need to start it again");
        return;
    }
    
    // Set advertising data (device name, service UUIDs, etc.) so they appear in scan results
    int rc = app_ble_config_adv_fields();
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertisement data: rc=%d", rc);
        return;
    }

    // Set advertising response data (manufacturer-specific data) for scan responses
    rc = app_ble_config_adv_rsp_fields();
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertisement response data: rc=%d", rc);
        return;
    }
    
    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    int32_t duration_ms = BLE_HS_FOREVER;
    if (s_adv_low_duty) {
        if (s_adv_slow) {
            adv_params.itvl_min = ADV_SLOW_ITVL;
            adv_params.itvl_max = ADV_SLOW_ITVL;
        } else {
            duration_ms = ADV_FAST_DURATION_MS;
        }
    }

    // Retrieve current address type
    uint8_t own_addr_type;
    rc = app_get_addr_type(&own_addr_type, "before advertising");
    if (rc != 0) {
        return;
    }

    // Set advertising TX power before starting
    rc = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, s_ctx->tx_power_level_for_advertising);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertising tx power: rc=%d", rc);
        return;
    }

    // Start advertising
    rc = ble_gap_adv_start(own_addr_type, NULL, duration_ms, &adv_params, ble_spp_server_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start advertising: rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "advertising (%s interval)", (s_adv_low_duty && s_adv_slow) ? "slow" : "fast");
    }
}

esp_err_t app_ble_set_adv_low_duty(bool low_duty) {
    if (s_adv_low_duty == low_duty) {
        return ESP_OK;
    }
    s_adv_low_duty = low_duty;
    s_adv_slow = false;   // either way the next run starts with the fast phase
    // The interval is fixed when advertising starts: if we are advertising now,
    // stop and start again. An explicit stop raises no ADV_COMPLETE, so restart
    // here; ble_spp_server_advertise() is a no-op while connected.
    if (s_ctx != NULL && s_ctx->state == APP_BLE_STATE_RUNNING && ble_gap_adv_active()) {
        int rc = ble_gap_adv_stop();
        if (rc != 0 && rc != BLE_HS_EALREADY) {
            ESP_LOGW(TAG, "Failed to stop advertising for interval change: rc=%d", rc);
            return ESP_FAIL;
        }
        ble_spp_server_advertise();
    }
    return ESP_OK;
}

/**
 * @brief GATT service handler.
 * 
 * @param conn_handle Connection handle
 * @param attr_handle Attribute handle
 * @param ctxt GATT access context
 * @param arg User-defined argument
 * @return int 0 on success, non-zero on failure
 */
static int ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    (void)arg;
    (void)conn_handle;
    (void)attr_handle;

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        return 0;

    case BLE_GATT_ACCESS_OP_WRITE_CHR: {
        if (s_ctx == NULL) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        // Get the length of received data; if 0, nothing to process.
        const uint16_t rx_len = (uint16_t)OS_MBUF_PKTLEN(ctxt->om);
        if (rx_len == 0) {
            return 0;
        }
        // Copy received data from mbuf into a contiguous memory region for processing.
        // First try the static buffer; if data exceeds its size, fall back to heap allocation.
        uint8_t *rx_data = NULL;
        bool use_heap = false;
        if (rx_len <= sizeof(s_ctx->rx_static_buf)) {
            rx_data = s_ctx->rx_static_buf;
        } else {
            rx_data = malloc(rx_len);
            use_heap = true;
            if (rx_data == NULL) {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
        }
        if (os_mbuf_copydata(ctxt->om, 0, rx_len, rx_data) != 0) {
            if (use_heap) {
                free(rx_data);
            }
            return BLE_ATT_ERR_UNLIKELY;
        }
        // Invoke the user-registered RX callback to handle received data
        app_ble_rx_callback_t cb = s_ctx->rx_callback;
        if (cb != NULL) {
            cb(rx_data, rx_len);
        }
        // Free heap-allocated buffer when done
        if (use_heap) {
            free(rx_data);
        }
        ESP_LOGI(TAG, "Received %u bytes from BLE central", rx_len);
        return 0;
    }

    default:
        return 0;
    }
}

/**
 * @brief Battery Service characteristic handler.
 */
static int ble_bas_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    (void)arg;
    (void)conn_handle;
    (void)attr_handle;

    if (s_ctx == NULL) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    const int rc = os_mbuf_append(ctxt->om, &s_ctx->battery_level, sizeof(s_ctx->battery_level));
    return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

/**
 * @brief BLE SPP characteristic definitions.
 */
static struct ble_gatt_chr_def s_ble_spp_chr_defs[] = {
    {
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_SPP_CHR_UUID16),
        .access_cb = ble_svc_gatt_handler,
        .val_handle = NULL,
        .flags = 0, // Set dynamically at start; no value needed here
    },
    {
        0,
    }
};

static struct ble_gatt_chr_def s_ble_bas_chr_defs[] = {
    {
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_BAS_CHR_UUID16),
        .access_cb = ble_bas_gatt_handler,
        .val_handle = NULL,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
    },
    {
        0,
    }
};

/**
 * @brief BLE SPP service definitions.
 */
static struct ble_gatt_svc_def s_ble_spp_svc_defs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_BAS_UUID16),
        .characteristics = s_ble_bas_chr_defs,
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_SPP_UUID16),
        .characteristics = s_ble_spp_chr_defs,
    },
    {
        0,
    },
};

/**
 * @brief GATT registration callback.
 * 
 * @param ctxt GATT registration context
 * @param arg User-defined argument
 */
static void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
    (void)arg;
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGD(TAG, "registered service %s with handle=%d", ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf), ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGD(TAG,
                 "registered characteristic %s with def_handle=%d val_handle=%d",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                 ctxt->chr.def_handle,
                 ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGD(TAG, "registered descriptor %s with handle=%d", ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf), ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

/**
 * @brief Initializes the GATT server.
 * 
 * @return int 0 on success, non-zero on failure
 */
static int gatt_svr_init(void) {
    int rc = 0;
    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(s_ble_spp_svc_defs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to count GATT configuration: rc=%d", rc);
        return rc;
    }

    rc = ble_gatts_add_svcs(s_ble_spp_svc_defs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to add GATT services: rc=%d", rc);
        return rc;
    }

    return 0;
}

/**
 * @brief GAP event handler for the BLE SPP server.
 * 
 * @param event GAP event
 * @param arg User-defined argument
 * @return int 0 on success, non-zero on failure
 */
static int ble_spp_server_gap_event(struct ble_gap_event *event, void *arg) {
    // Context is invalid; cannot process BLE events.
    if (s_ctx == NULL) {
        return 0;
    }

    struct ble_gap_conn_desc desc;
    esp_err_t err;
    int rc;
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "connection %s; status=%d", event->connect.status == 0 ? "established" : "failed", event->connect.status);
        // Check whether the connection succeeded or failed
        if (event->connect.status == 0) {
            // Cache connection handle; Notify is not enabled by default after connection
            s_ctx->conn_handle = event->connect.conn_handle;
            s_ctx->data_notify_enabled = false;
            s_ctx->battery_notify_enabled = false;

            if (s_ctx->link_callback) {
                s_ctx->link_callback(true);
            }

            // Set the connection TX power level
            err = esp_ble_tx_power_set_enhanced(ESP_BLE_ENHANCED_PWR_TYPE_CONN, 
                s_ctx->conn_handle, s_ctx->tx_power_level_for_connection);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Failed to set connection tx power: rc=%d", err);
            }

            // Log connection descriptor info; it contains connection parameters and state
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            if (rc == 0) {
                ble_spp_server_print_conn_desc(&desc);
            }

            // Request larger MTU to improve data transfer efficiency
            ble_att_set_preferred_mtu(CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU);
            rc = ble_gattc_exchange_mtu(event->connect.conn_handle, NULL, NULL);
            if (rc != 0) {
                ESP_LOGW(TAG, "MTU exchange failed: rc=%d", rc);
            }

            // Request 2M PHY to improve throughput (if both sides support it)
            rc = ble_gap_set_prefered_le_phy(event->connect.conn_handle, 
                BLE_HCI_LE_PHY_2M_PREF_MASK, BLE_HCI_LE_PHY_2M_PREF_MASK, 0);
            if (rc != 0) {
                ESP_LOGW(TAG, "Set 2M PHY failed: rc=%d", rc);
            }
        } else {
            s_ctx->conn_handle = BLE_HS_CONN_HANDLE_NONE;
            s_ctx->data_notify_enabled = false;
            s_ctx->battery_notify_enabled = false;
            s_adv_slow = false;   // someone is trying: be quick to find
            ble_spp_server_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnect; reason=%d", event->disconnect.reason);
        ble_spp_server_print_conn_desc(&event->disconnect.conn);
        s_ctx->conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_ctx->data_notify_enabled = false;
        s_ctx->battery_notify_enabled = false;
        if (s_ctx->link_callback) {
            s_ctx->link_callback(false);
        }
        s_adv_slow = false;   // just been in use: be quick to find again
        ble_spp_server_advertise();
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        // Encryption has been enabled or disabled for this connection.
        if (event->enc_change.status == 0) {
            ESP_LOGI(TAG, "connection encrypted!");
        } else {
            app_ble_log_enc_status(event->enc_change.status);
            // When bonding is disabled: the phone may hold a saved LTK and try to resume
            // encryption using it. After LTK verification fails, Android re-initiates pairing
            // and shows a pairing dialog. Terminate here to block that re-pairing path.
            if (!s_ctx->bonding) {
                ESP_LOGW(TAG, "enc failed while bonding disabled, terminating to prevent re-pairing");
                ble_gap_terminate(event->enc_change.conn_handle, BLE_ERR_AUTH_FAIL);
            }
        }
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        // When bonding is disabled, ignore repeat pairing requests
        if (!s_ctx->bonding) {
            ESP_LOGW(TAG, "bonding disabled, ignoring repeat pairing request");
            return BLE_GAP_REPEAT_PAIRING_IGNORE;
        }
        // Repeat pairing event, delete the old bond is required
        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        if (rc != 0) {
            ESP_LOGE(TAG, "failed to find connection, error code %d", rc);
            return rc;
        }
        ble_store_util_delete_peer(&desc.peer_id_addr);
        // Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should
        // continue with pairing operation
        ESP_LOGI(TAG, "repairing...");
        return BLE_GAP_REPEAT_PAIRING_RETRY; // return BLE_GAP_REPEAT_PAIRING_IGNORE if you don't want to repair

    case BLE_GAP_EVENT_PASSKEY_ACTION: {
        // When bonding is disabled, terminate connection to reject pairing
        if (!s_ctx->bonding) {
            ESP_LOGW(TAG, "bonding disabled, terminating pairing attempt (conn_handle=%u)",
                     event->passkey.conn_handle);
            ble_gap_terminate(event->passkey.conn_handle, BLE_ERR_AUTH_FAIL);
            return 0;
        }

        const uint8_t action = event->passkey.params.action;
        const uint32_t numcmp = event->passkey.params.numcmp;

        ESP_LOGI(TAG, "passkey action=%u conn_handle=%u numcmp=%lu",
                 action,
                 event->passkey.conn_handle,
                 (unsigned long)numcmp);
        
        // Display action
        if (action == BLE_SM_IOACT_DISP) {
            struct ble_sm_io io = {0};
            io.action = action;
            io.passkey = s_ctx->static_passkey % 1000000U;
            ESP_LOGI(TAG, "enter passkey %06" PRIu32 " on the peer side", io.passkey);
            const int rc_inject = ble_sm_inject_io(event->passkey.conn_handle, &io);
            if (rc_inject != 0) {
                ESP_LOGE(TAG, "inject static passkey failed: rc=%d", rc_inject);
                return rc_inject;
            }
            ESP_LOGI(TAG, "static passkey applied");
        } else if (action == BLE_SM_IOACT_INPUT) {
            ESP_LOGE(TAG,
                     "passkey input requested by peer, but device is configured as DISPLAY_ONLY; reject automatic static passkey injection");
            return BLE_HS_EINVAL;
        }
        return 0;
    }

    case BLE_GAP_EVENT_CONN_UPDATE:
        ESP_LOGI(TAG, "connection updated; status=%d", event->conn_update.status);
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        if (rc == 0) {
            ble_spp_server_print_conn_desc(&desc);
        }
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "advertise complete; reason=%d", event->adv_complete.reason);
        if (s_ctx->conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            if (event->adv_complete.reason == BLE_HS_ETIMEOUT) {
                s_adv_slow = true;   // fast phase ran out with nobody connecting
            }
            ble_spp_server_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "mtu update event; conn_handle=%d cid=%d mtu=%d", event->mtu.conn_handle, event->mtu.channel_id, event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_ctx->spp_chr_val_handle) {
            s_ctx->data_notify_enabled = event->subscribe.cur_notify;
        } else if (event->subscribe.attr_handle == s_ctx->battery_chr_val_handle) {
            s_ctx->battery_notify_enabled = event->subscribe.cur_notify;
        }
        ESP_LOGI(TAG,
                 "subscribe event; conn_handle=%d attr_handle=%d prevn=%d curn=%d",
                 event->subscribe.conn_handle,
                 event->subscribe.attr_handle,
                 event->subscribe.prev_notify,
                 event->subscribe.cur_notify);
        return 0;

    default:
        ESP_LOGI(TAG, "unhandled GAP event: %d", event->type);
        return 0;
    }
}

/**
 * @brief BLE SPP server reset callback.
 * 
 * @param reason Reset reason
 */
static void ble_spp_server_on_reset(int reason) {
    ESP_LOGW(TAG, "NimBLE reset; reason=%d", reason);
}

/**
 * @brief BLE SPP server sync callback.
 */
static void ble_spp_server_on_sync(void) {
    if (s_ctx == NULL) {
        return;
    }

    int rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    // Infer a suitable address type from the current BLE configuration
    uint8_t own_addr_type;
    rc = app_get_addr_type(&own_addr_type, "on ble spp server sync");
    if (rc != 0) {
        return;
    }

    // Copy and log the final address in use
    uint8_t addr_val[BLE_DEV_ADDR_LEN] = {0};
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    if (rc == 0) {
        ESP_LOGI(TAG, "Device Address: %s", app_addr_to_str(addr_val));
    }

    s_ctx->state = APP_BLE_STATE_RUNNING;
    s_adv_slow = false;   // fresh (re)start of the stack: fast phase
    ble_spp_server_advertise();
}

/**
 * @brief BLE SPP server host task.
 * 
 * @param param User-defined parameter
 */
static void ble_spp_server_host_task(void *param) {
    (void)param;
    ESP_LOGI(TAG, "BLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/**
 * @brief Initializes the BLE SPP server.
 * 
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_init(void) {
    if (s_ctx != NULL) {
        return ESP_OK;
    }

    s_ctx = app_ble_ctx_create();
    if (s_ctx == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = app_ble_load_persisted_params(s_ctx);
    if (err != ESP_OK) {
        free(s_ctx);
        s_ctx = NULL;
        return err;
    }

    return ESP_OK;
}

/**
 * @brief Starts the BLE SPP server.
 * 
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_start(void) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_ctx->state == APP_BLE_STATE_CTX_READY) {
        s_ctx->conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_ctx->data_notify_enabled = false;
        s_ctx->battery_notify_enabled = false;

        esp_err_t err = nimble_port_init();
        if (err != ESP_OK) {
            return err;
        }

        ble_hs_cfg.reset_cb = ble_spp_server_on_reset;
        ble_hs_cfg.sync_cb = ble_spp_server_on_sync;
        ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
        ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
        app_ble_apply_sm_cfg(s_ctx->bonding);

        // Reset spp_chr_val_handle to 0 to indicate the characteristic is not yet registered;
        // it will be updated when the GATT server registers the characteristic.
        s_ctx->spp_chr_val_handle = 0;
        s_ctx->battery_chr_val_handle = 0;
        // Assign the address of spp_chr_val_handle to s_ble_spp_chr_defs[0].val_handle so that
        // when the GATT server registers this characteristic, it writes the value handle into
        // s_ctx->spp_chr_val_handle, allowing us to reference it later when sending data.
        s_ble_spp_chr_defs[0].val_handle = &s_ctx->spp_chr_val_handle;
        s_ble_bas_chr_defs[0].val_handle = &s_ctx->battery_chr_val_handle;
        // Each start resets to default permissions, then adds encryption flags if bonding is enabled
        s_ble_spp_chr_defs[0].flags =
            BLE_GATT_CHR_F_READ |
            BLE_GATT_CHR_F_NOTIFY |
            BLE_GATT_CHR_F_WRITE_NO_RSP |
            BLE_GATT_CHR_F_WRITE;
        if (s_ctx->bonding) {
            s_ble_spp_chr_defs[0].flags |=
                (BLE_GATT_CHR_F_READ_ENC |
                BLE_GATT_CHR_F_NOTIFY_INDICATE_ENC |
                BLE_GATT_CHR_F_WRITE_ENC);
        }

        int rc = gatt_svr_init();
        if (rc != 0) {
            nimble_port_deinit();
            return ESP_FAIL;
        }

        rc = ble_svc_gap_device_name_set(s_ctx->device_name);
        if (rc != 0) {
            nimble_port_deinit();
            return ESP_FAIL;
        }

        s_ctx->state = APP_BLE_STATE_STACK_BOOTING;
        ESP_LOGI(TAG, "Starting BLE stack...");
        ble_store_config_init();
        nimble_port_freertos_init(ble_spp_server_host_task);
        ESP_LOGI(TAG, "BLE stack started");
    }

    // For test!!!
    // ble_store_clear();
    // ESP_LOGW(TAG, "Cleared bonded devices for testing");

    return ESP_OK;
}

/**
 * @brief Stops the BLE SPP server.
 * 
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_stop(void) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_ctx->state == APP_BLE_STATE_CTX_READY) {
        return ESP_OK;
    }

    if (ble_gap_adv_active()) {
        ble_gap_adv_stop();
    }

    if (s_ctx->conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(s_ctx->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        s_ctx->conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_ctx->data_notify_enabled = false;
        s_ctx->battery_notify_enabled = false;
        // The stack stops before the DISCONNECT event could be delivered.
        if (s_ctx->link_callback) {
            s_ctx->link_callback(false);
        }
    }

    esp_err_t err = nimble_port_stop();
    if (err != ESP_OK) {
        return err;
    }

    err = nimble_port_deinit();
    if (err != ESP_OK) {
        return err;
    }

    s_ctx->state = APP_BLE_STATE_CTX_READY;
    s_ctx->conn_handle = BLE_HS_CONN_HANDLE_NONE;
    s_ctx->data_notify_enabled = false;
    s_ctx->battery_notify_enabled = false;
    s_ctx->spp_chr_val_handle = 0;
    s_ctx->battery_chr_val_handle = 0;
    return ESP_OK;
}

/**
 * @brief Proactively notifies the central device of an updated battery level.
 * 
 */
static void app_ble_notify_battery_level(void) {
    // If connected and battery notify is enabled, send a notification so the central
    // device receives the updated battery level immediately
    const uint16_t conn_handle = app_ble_get_active_conn_handle();
    if (s_ctx->state != APP_BLE_STATE_CTX_READY 
            && conn_handle != BLE_HS_CONN_HANDLE_NONE 
            && s_ctx->battery_chr_val_handle != 0
            && s_ctx->battery_notify_enabled) {
        struct os_mbuf *txom = ble_hs_mbuf_from_flat(&s_ctx->battery_level, sizeof(s_ctx->battery_level));
        if (txom == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory(ble_hs_mbuf_from_flat) for battery level notification, cannot notify battery level update");
            return;
        }
        const int rc = ble_gatts_notify_custom(conn_handle, s_ctx->battery_chr_val_handle, txom);
        if (rc != 0) {
            ESP_LOGW(TAG, "Failed to send battery level notification: rc=%d", rc);
        }
    }
}

/**
 * @brief De-initializes the BLE SPP server.
 * 
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_deinit(void) {
    if (s_ctx == NULL) {
        return ESP_OK;
    }

    if (s_ctx->state != APP_BLE_STATE_CTX_READY) {
        const esp_err_t err = app_ble_stop();
        if (err != ESP_OK) {
            return err;
        }
    }

    free(s_ctx);
    s_ctx = NULL;
    return ESP_OK;
}

/**
 * @brief Notify interface with retry logic to handle BLE stack memory exhaustion
 * and avoid data loss on transient send failures.
 * 
 * @param buf Data buffer to send
 * @param len Length of data to send
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t notify_at_retry(const void *buf, uint16_t len) {
    const uint16_t conn_handle = app_ble_get_active_conn_handle();
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return ESP_ERR_INVALID_STATE;
    }
    // Retry loop: keep trying until send succeeds or retry counts exceed the threshold
    int nomem_fail_count = 0;
    int notify_fail_count = 0;
    while (1) {
        // Allocate an os_mbuf for the data chunk. NULL means the BLE stack is out of memory;
        // wait and retry. On success, call the underlying send API.
        struct os_mbuf *txom = ble_hs_mbuf_from_flat(buf, len);
        if (txom == NULL) {
            if (++nomem_fail_count > s_ctx->notify_nomem_retry_max) {
                ESP_LOGW(TAG, "Failed to allocate memory(ble_hs_mbuf_from_flat) for BLE notification after %d retries, data will be dropped", nomem_fail_count);
                return ESP_ERR_NO_MEM;
            }
            vTaskDelay(pdMS_TO_TICKS(10));  // ~1 connection interval; long enough for a TX buffer to free, short enough not to overflow the UART (was 100ms)
            continue;
        }
        // os_mbuf allocated successfully; reset nomem counter
        nomem_fail_count = 0;
        // Call the underlying send API. BLE_HS_ENOMEM means the stack is still out of memory;
        // wait and retry. Any other error is a hard failure.
        int rc = ble_gatts_notify_custom(conn_handle, s_ctx->spp_chr_val_handle, txom);
        if (rc == BLE_HS_ENOMEM) {
            if (++notify_fail_count > s_ctx->notify_fail_retry_max) {
                ESP_LOGW(TAG, "Failed to send BLE notification due to insufficient memory after %d retries, data will be dropped", notify_fail_count);
                return ESP_ERR_NO_MEM;
            }
            vTaskDelay(pdMS_TO_TICKS(10));  // ~1 connection interval; long enough for a TX buffer to free, short enough not to overflow the UART (was 100ms)
            continue;
        } else if (rc != 0) {
            return ESP_FAIL;
        }
        // Chunk sent successfully; exit retry loop
        break;
    }
    // Return success
    return ESP_OK;
}

/**
 * @brief Sends data to the BLE central device.
 * 
 * @param data Data buffer to send
 * @param length Length of data to send
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_send(uint8_t *data, size_t length) {
    if (data == NULL || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check that context and BLE stack are properly initialized
    if (s_ctx == NULL || s_ctx->state == APP_BLE_STATE_CTX_READY) {
        return ESP_ERR_INVALID_STATE;
    }

    // Get current connection handle; return error if not connected or Notify not enabled
    const uint16_t conn_handle = app_ble_get_active_conn_handle();
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_ctx->data_notify_enabled) {
        return ESP_ERR_INVALID_STATE;
    }

    // Packet segmentation loop
    const uint16_t att_mtu = ble_att_mtu(conn_handle);
    const size_t max_payload = (att_mtu > 3U) ? (size_t)(att_mtu - 3U) : 1U;
    size_t offset = 0;
    while (offset < length) {
        // Each chunk is at most max_payload bytes; the final chunk may be smaller
        const size_t chunk_len = ((length - offset) > max_payload) ? max_payload : (length - offset);
        const uint8_t *chunk_data = data + offset;
        // Send via notify_at_retry; on failure return immediately — retry logic is inside notify_at_retry
        esp_err_t err = notify_at_retry(chunk_data, (uint16_t)chunk_len);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Notify failed: rc=%s offset=%u chunk=%u mtu=%u",
                esp_err_to_name(err),
                (unsigned int)offset,
                (unsigned int)chunk_len,
                (unsigned int)att_mtu);
            return err;
        }
        // Chunk sent; continue to next chunk
        offset += chunk_len;
    }

    ESP_LOGI(TAG, "Data sent to BLE central successfully");
    return ESP_OK;
}

/**
 * @brief Sets the RX callback for the BLE SPP server.
 * 
 * @param callback RX callback function
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_set_rx_callback(app_ble_rx_callback_t callback) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    s_ctx->rx_callback = callback;
    return ESP_OK;
}

esp_err_t app_ble_set_link_callback(app_ble_link_callback_t callback) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    s_ctx->link_callback = callback;
    return ESP_OK;
}

/**
 * @brief Gets the current state of the BLE SPP server.
 * 
 * @param state Output pointer for the state value: 0=not started, 1=started but not connected, 2=connected
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_get_state(uint8_t *state) {
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_ctx->state == APP_BLE_STATE_CTX_READY) { // CTX_READY: BLE stack not initialized = not started
        *state = 0;
    } else if (s_ctx->conn_handle == BLE_HS_CONN_HANDLE_NONE) { // handle invalid = not connected
        *state = 1;
    } else { // connected
        *state = 2;
    }

    return ESP_OK;
}

/**
 * @brief Sets the manufacturer-specific advertising data for the BLE SPP server.
 * 
 * @param data Manufacturer data buffer
 * @param length Length of manufacturer data
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_set_adv_mfg_data(uint8_t *data, size_t length) {
    if ((data == NULL && length > 0U) || length > APP_BLE_ADV_MFG_DATA_MAX_LEN) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (length > 0U) {
        memcpy(s_ctx->adv_mfg_data, data, length);
    }
    s_ctx->adv_mfg_data_len = length;

    if (length == 0U) {
        return app_nvs_rw_erase(NAMESPACE_BLE_SPP, KEY_BLE_ADV_MFG_DATA);
    }

    return app_nvs_rw_write(NAMESPACE_BLE_SPP, (app_nvs_rw_write_item_t[]) {
        {
            .key = KEY_BLE_ADV_MFG_DATA,
            .type = APP_NVS_RW_TYPE_BLOB,
            .data = data,
            .length = length,
        }
    }, 1);
}

/**
 * @brief Gets the manufacturer-specific advertising data of the BLE SPP server.
 * 
 * @param data Output buffer for manufacturer data
 * @param length Length of the output buffer
 * @param length_out Output pointer for the actual data length
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_get_adv_mfg_data(uint8_t *data, size_t length, uint8_t *length_out) {
    if (data == NULL || length == 0U || length_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (length < s_ctx->adv_mfg_data_len) {
        return ESP_ERR_NO_MEM;
    }
    *length_out = s_ctx->adv_mfg_data_len;
    if (s_ctx->adv_mfg_data_len > 0U) {
        memcpy(data, s_ctx->adv_mfg_data, s_ctx->adv_mfg_data_len);
    }
    return ESP_OK;
}

/**
 * @brief Sets the device name for the BLE SPP server.
 * 
 * @param name Device name string
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_set_device_name(const char *name) {
    if (name == NULL || s_ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Validate name length to prevent overflow in the device_name buffer
    size_t name_len = strlen(name);
    if (name_len >= sizeof(s_ctx->device_name)) { // empty name allowed; reject if exceeds max length
        return ESP_ERR_INVALID_ARG;
    }

    // If length is 0, set first byte to '\0'; otherwise copy name and null-terminate
    if (name_len == 0U) {
        s_ctx->device_name[0] = '\0';
    } else {
        memcpy(s_ctx->device_name, name, name_len);
        s_ctx->device_name[name_len] = '\0';
    }

    return app_nvs_rw_write(NAMESPACE_BLE_SPP, (app_nvs_rw_write_item_t[]) {
        {
            .key = KEY_BLE_DEVICE_NAME,
            .type = APP_NVS_RW_TYPE_STR,
            .data = name,
            .length = name_len + 1,
        }
    }, 1);
}

/**
 * @brief Gets the device name of the BLE SPP server.
 * 
 * @param name_buf Output buffer for the device name
 * @param buf_len Length of the output buffer
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_get_device_name(char *name_buf, size_t buf_len) {
    if (name_buf == NULL || buf_len == 0 || s_ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Read device name from cache; not restricted by BLE stack state
    const size_t name_len = strlen(s_ctx->device_name);
    if (name_len + 1 > buf_len) { // name + null terminator must fit in the buffer
        return ESP_ERR_NO_MEM;
    }

    // Copy name to output buffer and ensure null termination
    memcpy(name_buf, s_ctx->device_name, name_len);
    name_buf[name_len] = '\0';
    return ESP_OK;
}

/**
 * @brief Sets the notify retry count thresholds for the BLE SPP server to control
 * retry behavior when the BLE stack runs out of memory during a send.
 * 
 * @param nomem_retry_max Maximum retry count on out-of-memory errors
 * @param fail_retry_max Maximum retry count on send failures
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_set_notify_retry_max(uint16_t nomem_retry_max, uint16_t fail_retry_max) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    s_ctx->notify_nomem_retry_max = nomem_retry_max;
    s_ctx->notify_fail_retry_max = fail_retry_max;
    
    ESP_LOGI(TAG, "Notify retry max updated: nomem_retry_max=%u, fail_retry_max=%u",
             nomem_retry_max, fail_retry_max);

    return app_nvs_rw_write(NAMESPACE_BLE_SPP, (app_nvs_rw_write_item_t[]) {
        {
            .key = KEY_BLE_NOTIFY_RETRY_NOMEM,
            .type = APP_NVS_RW_TYPE_U16,
            .data = &nomem_retry_max,
            .length = sizeof(nomem_retry_max),
        }, {
            .key = KEY_BLE_NOTIFY_RETRY_FAIL,
            .type = APP_NVS_RW_TYPE_U16,
            .data = &fail_retry_max,
            .length = sizeof(fail_retry_max),
        }
    }, 2);
}

/**
 * @brief Gets the notify retry count thresholds of the BLE SPP server.
 * 
 * @param nomem_retry_max Output pointer for the out-of-memory retry max
 * @param fail_retry_max Output pointer for the send-failure retry max
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_get_notify_retry_max(uint16_t *nomem_retry_max, uint16_t *fail_retry_max) {
    if (nomem_retry_max == NULL || fail_retry_max == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    *nomem_retry_max = s_ctx->notify_nomem_retry_max;
    *fail_retry_max = s_ctx->notify_fail_retry_max;
    return ESP_OK;
}

/**
 * @brief Sets the device Bluetooth address for the BLE SPP server.
 * 
 * @param addr Device address buffer
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_set_device_addr(uint8_t addr[BLE_DEV_ADDR_LEN]) {
    if (addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // Modifying the MAC address while the BLE stack is running is not permitted
    if (s_ctx == NULL || s_ctx->state == APP_BLE_STATE_RUNNING) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = esp_iface_mac_addr_set(addr, ESP_MAC_BT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set BT MAC address: %s", esp_err_to_name(err));
        return err;
    }

    return app_nvs_rw_write(NAMESPACE_BLE_SPP, (app_nvs_rw_write_item_t[]) {
        {
            .key = KEY_BLE_DEVICE_ADDR,
            .type = APP_NVS_RW_TYPE_BLOB,
            .data = addr,
            .length = BLE_DEV_ADDR_LEN,
        }
    }, 1);
}

/**
 * @brief Gets the device Bluetooth address of the BLE SPP server.
 * 
 * @param addr Output buffer for the device address
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_get_device_addr(uint8_t addr[BLE_DEV_ADDR_LEN]) {
    if (addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    // Reads directly from the lower-layer interface, so s_ctx->state check is not needed
    return esp_read_mac(addr, ESP_MAC_BT);
}

/**
 * @brief Sets the bonding (pairing) enable state for the BLE SPP server.
 * 
 * @param enable Bonding enable flag: 0=disabled, 1=enabled
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_set_bonding_enable(uint8_t enable) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_ctx->bonding = enable ? 1 : 0;
    uint8_t persisted_enable = s_ctx->bonding;
    return app_nvs_rw_write(NAMESPACE_BLE_SPP, (app_nvs_rw_write_item_t[]) {
        {
            .key = KEY_BLE_BONDING_ENABLE,
            .type = APP_NVS_RW_TYPE_U8,
            .data = &persisted_enable,
            .length = sizeof(persisted_enable),
        }
    }, 1);
}

/**
 * @brief Gets the bonding (pairing) enable state of the BLE SPP server.
 * 
 * @param enable Output pointer for the bonding enable flag: 0=disabled, 1=enabled
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_get_bonding_enable(uint8_t *enable) {
    if (enable == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    *enable = s_ctx->bonding;
    return ESP_OK;
}

/**
 * @brief Sets the bonding passkey for the BLE SPP server.
 * 
 * @param passkey Passkey string; must be exactly 6 digits ("000000" to "999999")
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_set_bonding_key(const char *passkey) {
    if (passkey == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    const size_t len = strlen(passkey);
    if (len != 6) {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < 6; ++i) {
        if (passkey[i] < '0' || passkey[i] > '9') {
            return ESP_ERR_INVALID_ARG;
        }
    }

    s_ctx->static_passkey = (uint32_t)strtoul(passkey, NULL, 10);
    ESP_LOGI(TAG, "bonding passkey updated to %06" PRIu32, s_ctx->static_passkey);
    if (s_ctx->static_passkey < 100000U) {
        ESP_LOGW(TAG,
                 "passkey has leading zeros when displayed as BLE PIN; peer must enter %06" PRIu32,
                 s_ctx->static_passkey);
    }

    return app_nvs_rw_write(NAMESPACE_BLE_SPP, (app_nvs_rw_write_item_t[]) {
        {
            .key = KEY_BLE_BONDING_PASSKEY,
            .type = APP_NVS_RW_TYPE_U32,
            .data = &s_ctx->static_passkey,
            .length = sizeof(s_ctx->static_passkey),
        }
    }, 1);
}

/**
 * @brief Gets the bonding passkey of the BLE SPP server.
 * 
 * @param passkey Output buffer for the passkey string
 * @param buf_len Buffer length; must be at least 7 bytes to hold 6 digits plus null terminator
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_get_bonding_key(char *passkey, size_t buf_len) {
    if (passkey == NULL || buf_len < 7U) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    (void)snprintf(passkey, buf_len, "%06" PRIu32, s_ctx->static_passkey);
    return ESP_OK;
}

/**
 * @brief Gets the number of bonded devices for the BLE SPP server.
 * 
 * @param count Output pointer for the bonded device count
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_get_bonded_count(uint8_t *count) {
    if (count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Requires BLE stack to be initialized (calls ble_xxx APIs)
    if (s_ctx == NULL || s_ctx->state == APP_BLE_STATE_CTX_READY) {
        return ESP_ERR_INVALID_STATE;
    }

    int num_peers = 0;
    const int rc = ble_store_util_count(BLE_STORE_OBJ_TYPE_OUR_SEC, &num_peers);
    if (rc != 0) {
        return ESP_FAIL;
    }

    // If bonded count exceeds the output type limit, report error (should not happen in practice)
    if (num_peers > UINT8_MAX) {
        return ESP_ERR_NO_MEM;
    }

    // Return the queried bonded count
    *count = (uint8_t)num_peers;
    return ESP_OK;
}

/**
 * @brief Gets the address and type of a bonded device by index.
 * 
 * @param index Zero-based index of the bonded device
 * @param addr Output buffer for the device address
 * @param type Output pointer for the device address type
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_get_bonded_device_address(uint8_t index, uint8_t addr[BLE_DEV_ADDR_LEN], uint8_t *type) {
    if (addr == NULL || type == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Requires BLE stack to be initialized (calls ble_xxx APIs)
    if (s_ctx == NULL || s_ctx->state == APP_BLE_STATE_CTX_READY) {
        return ESP_ERR_INVALID_STATE;
    }

    // Get bonded count first; needed for buffer allocation and index validation
    uint8_t num_peers = 0;
    esp_err_t err = app_ble_get_bonded_count(&num_peers);
    if (err != ESP_OK) {
        return err;
    }

    // No bonded devices; return not found
    if (num_peers == 0U) {
        return ESP_ERR_NOT_FOUND;
    }

    // Allocate a temporary buffer for ble_store_util_bonded_peers
    ble_addr_t *bonded_addrs = (ble_addr_t *)malloc(sizeof(ble_addr_t) * num_peers);
    if (bonded_addrs == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // Fetch the persisted bonded address list and locate the entry at the given index
    int num_peers_out = 0;
    const int rc = ble_store_util_bonded_peers(bonded_addrs, &num_peers_out, num_peers);
    if (rc != 0) {
        free(bonded_addrs);
        return ESP_FAIL;
    }

    // Index out of range; caller passed an invalid index
    if (index >= num_peers_out) {
        free(bonded_addrs);
        return ESP_ERR_INVALID_ARG;
    }

    // Valid index; copy out the address and type
    memcpy(addr, bonded_addrs[index].val, BLE_DEV_ADDR_LEN);
    *type = bonded_addrs[index].type;
    free(bonded_addrs);
    return ESP_OK;
}

/**
 * @brief Removes a bonded device by index.
 * 
 * @param index Zero-based index of the bonded device to remove
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_del_bonded_device(uint8_t index) {
    if (s_ctx == NULL || s_ctx->state == APP_BLE_STATE_CTX_READY) {
        return ESP_ERR_INVALID_STATE;
    }

    // Retrieve the address and type for the given index, then delete the bond entry
    ble_addr_t addr;
    esp_err_t err = app_ble_get_bonded_device_address(index, addr.val, &addr.type);
    if (err != ESP_OK) {
        return err;
    }

    // Delete the bonded device at the resolved address and type
    const int rc = ble_store_util_delete_peer(&addr);
    return (rc == 0) ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Clears all bonded devices from the BLE SPP server.
 * 
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_clear_bonded(void) {
    // Requires BLE stack to be initialized (calls ble_xxx APIs)
    if (s_ctx == NULL || s_ctx->state == APP_BLE_STATE_CTX_READY) {
        return ESP_ERR_INVALID_STATE;
    }

    const int rc = ble_store_clear();
    return (rc == 0) ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Sets the battery level for the BLE SPP server.
 * 
 * @param level Battery level 0–100, or 0xFF for unknown
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_set_battery_level(uint8_t level) {
    // Valid range is 0-100 or 0xFF (unknown); all other values are invalid
    if (level > 100U && level != 0xFFU) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_ctx->battery_level = level;
    // Notify the central device each time the battery level is updated
    app_ble_notify_battery_level();
    return ESP_OK;
}

/**
 * @brief Gets the battery level of the BLE SPP server.
 * 
 * @param level Output pointer for the current battery level (0–100 or 0xFF for unknown)
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_get_battery_level(uint8_t *level) {
    if (level == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    *level = s_ctx->battery_level;
    return ESP_OK;
}

/**
 * @brief Sets the TX power level for the BLE SPP server.
 *
 * @param type Power type: 0=advertising TX power, 1=connection TX power
 * @param level TX power level using the esp_power_level_t enum values from ESP-IDF
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_set_tx_power(uint8_t type, uint8_t level) {
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Parameter is u8 so only check the upper bound; no need to check < 0
    if (level > ESP_PWR_LVL_P20) {
        return ESP_ERR_INVALID_ARG;
    }

    // Cache the level in the context; it is applied via esp_ble_tx_power_set_enhanced
    // in the GAP callback after a connection is established
    if (type == 0) {
        s_ctx->tx_power_level_for_advertising = level;
        return app_nvs_rw_write(NAMESPACE_BLE_SPP, (app_nvs_rw_write_item_t[]) {
            {
                .key = KEY_BLE_TX_POWER_ADV,
                .type = APP_NVS_RW_TYPE_U8,
                .data = &level,
                .length = sizeof(level),
            }
        }, 1);
    } else if (type == 1) {
        s_ctx->tx_power_level_for_connection = level;
        return app_nvs_rw_write(NAMESPACE_BLE_SPP, (app_nvs_rw_write_item_t[]) {
            {
                .key = KEY_BLE_TX_POWER_CONN,
                .type = APP_NVS_RW_TYPE_U8,
                .data = &level,
                .length = sizeof(level),
            }
        }, 1);
    } else {
        return ESP_ERR_INVALID_ARG;
    }
}

/**
 * @brief Gets the TX power level of the BLE SPP server.
 *
 * @param type Power type: 0=advertising TX power, 1=connection TX power
 * @param level Output pointer for the current TX power level
 * @return esp_err_t ESP_OK on success, other value on failure
 */
esp_err_t app_ble_get_tx_power(uint8_t type, uint8_t *level) {
    if (level == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Return the cached TX power level from context; no need to query the lower-layer API
    // because we already cache it when the level is set
    if (type == 0) {
        *level = s_ctx->tx_power_level_for_advertising;
    } else if (type == 1) {
        *level = s_ctx->tx_power_level_for_connection;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}
