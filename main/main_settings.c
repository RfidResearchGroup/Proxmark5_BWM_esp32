#include <esp_log.h>
#include "main_settings.h"
#include "app_nvs_rw.h"

// NOTE: NVS namespace and key names must satisfy the NVS_KEY_NAME_MAX_SIZE limit,
//  which is typically 15 bytes (excluding the terminating '\0'). Keep all names within this limit.

#define TAG                         "main_settings"

// Namespaces
#define NAMESPACE_SYS               "app_sys"
#define NAMESPACE_WIFI              "app_wifi"

// Key names
#define KEY_TIME_ZONE               "timezone"
#define KEY_POWER_SAVE              "pwr_save"
#define KEY_BLE_ENABLE              "ble_en"
#define KEY_WIFI_MODE               "wifi_mode"
#define KEY_WIFI_FWD_TYPE           "wifi_fwd_type"
#define KEY_WIFI_TX_PWR             "wifi_tx_pwr"
#define KEY_WIFI_PS_MODE            "wifi_ps"
#define KEY_WIFI_INACTIVE_TIME      "wifi_inact_tm"
#define KEY_WIFI_DHCP_ENABLE        "wifi_dhcp_en"
#define KEY_WIFI_MAC_ADDR           "wifi_mac_addr"
#define KEY_WIFI_IP_ADDR            "wifi_ip_addr"
#define KEY_WIFI_HOST_NAME          "wifi_host_name"


/**
 * @brief Save the timezone setting to NVS for loading on the next boot.
 * 
 * @param tz Timezone string, must be null-terminated
 * @return esp_err_t
 */
esp_err_t settings_time_zone_save(const char *tz) {
    return app_nvs_rw_write(NAMESPACE_SYS, (app_nvs_rw_write_item_t[]) {
        {
            .key = KEY_TIME_ZONE,
            .type = APP_NVS_RW_TYPE_STR,
            .data = tz,
            .length = strlen(tz),
        }
    }, 1);
}

/**
 * @brief Load the timezone setting from NVS. If no timezone has been saved or reading fails, sets *tz_buf = NULL.
 * 
 * @param tz_buf Pointer to where the timezone string pointer will be stored; set to NULL on failure
 * @return esp_err_t
 */
esp_err_t settings_time_zone_load(char **tz_buf) {
    app_nvs_rw_read_item_t nvs_item = {
        .key = KEY_TIME_ZONE,
        .type = APP_NVS_RW_TYPE_STR,
        .data = tz_buf,
        .default_value = 0, // STR type does not use a default value
    };
    return app_nvs_rw_read(NAMESPACE_SYS, &nvs_item, 1);
}

/**
 * @brief Save the power-save switch (app_power.c) to NVS.
 * @param enabled 0 = off, 1 = on
 * @return esp_err_t
 */
esp_err_t settings_power_save_save(uint8_t enabled) {
    return app_nvs_rw_write(NAMESPACE_SYS, (app_nvs_rw_write_item_t[]) {
        {
            .key = KEY_POWER_SAVE,
            .type = APP_NVS_RW_TYPE_U8,
            .data = &enabled,
            .length = sizeof(enabled),
        }
    }, 1);
}

/**
 * @brief Load the power-save switch from NVS.
 * @param enabled Pointer to uint8_t where the loaded value will be stored
 * @param default_enabled Default value to use if no value is stored in NVS
 * @return esp_err_t
 */
esp_err_t settings_power_save_load(uint8_t *enabled, uint8_t default_enabled) {
    app_nvs_rw_read_item_t nvs_item = {
        .key = KEY_POWER_SAVE,
        .type = APP_NVS_RW_TYPE_U8,
        .data = enabled,
        .default_value = default_enabled,
    };
    return app_nvs_rw_read(NAMESPACE_SYS, &nvs_item, 1);
}

/**
 * @brief Save the BLE switch (start the BLE SPP stack at boot or not) to NVS.
 * @param enabled 0 = off, 1 = on
 * @return esp_err_t
 */
esp_err_t settings_ble_enable_save(uint8_t enabled) {
    return app_nvs_rw_write(NAMESPACE_SYS, (app_nvs_rw_write_item_t[]) {
        {
            .key = KEY_BLE_ENABLE,
            .type = APP_NVS_RW_TYPE_U8,
            .data = &enabled,
            .length = sizeof(enabled),
        }
    }, 1);
}

/**
 * @brief Load the BLE switch from NVS.
 * @param enabled Pointer to uint8_t where the loaded value will be stored
 * @param default_enabled Default value to use if no value is stored in NVS
 * @return esp_err_t
 */
esp_err_t settings_ble_enable_load(uint8_t *enabled, uint8_t default_enabled) {
    app_nvs_rw_read_item_t nvs_item = {
        .key = KEY_BLE_ENABLE,
        .type = APP_NVS_RW_TYPE_U8,
        .data = enabled,
        .default_value = default_enabled,
    };
    return app_nvs_rw_read(NAMESPACE_SYS, &nvs_item, 1);
}

/**
 * @brief Save the WiFi functional mode to NVS.
 * @param mode WiFi functional mode (int, corresponds to the wifi_function_mode_t enum)
 * @return esp_err_t
 */
esp_err_t settings_wifi_mode_save(int mode) {
    return app_nvs_rw_write(NAMESPACE_WIFI, (app_nvs_rw_write_item_t[]) {
        {
            .key = KEY_WIFI_MODE,
            .type = APP_NVS_RW_TYPE_I8,
            .data = &mode,
            .length = sizeof(mode),
        }
    }, 1);
}

/**
 * @brief Load the WiFi functional mode from NVS.
 * @param mode Pointer to int where the loaded mode will be stored
 * @param default_mode Default value to use if no value is stored in NVS
 * @return esp_err_t
 */
esp_err_t settings_wifi_mode_load(int *mode, int default_mode) {
    app_nvs_rw_read_item_t nvs_item = {
        .key = KEY_WIFI_MODE,
        .type = APP_NVS_RW_TYPE_I8,
        .data = mode,
        .default_value = default_mode,
    };
    return app_nvs_rw_read(NAMESPACE_WIFI, &nvs_item, 1);
}

/**
 * @brief Save the WiFi forwarding type to NVS.
 * @param forward_type A wifi_forward_type_t enum value
 * @return esp_err_t
 */
esp_err_t settings_wifi_forward_type_save(int forward_type) {
    return app_nvs_rw_write(NAMESPACE_WIFI, (app_nvs_rw_write_item_t []) {
        {
            .key = KEY_WIFI_FWD_TYPE,
            .type = APP_NVS_RW_TYPE_U8,
            .data = &forward_type,
            .length = sizeof(forward_type),
        }
    }, 1);
}

/**
 * @brief Load the WiFi forwarding type from NVS.
 * @param forward_type Pointer to int where the loaded type will be stored
 * @param default_type Default value to use if no value is stored in NVS
 * @return esp_err_t
 */
esp_err_t settings_wifi_forward_type_load(int *forward_type, int default_type) {
    app_nvs_rw_read_item_t nvs_item = {
        .key = KEY_WIFI_FWD_TYPE,
        .type = APP_NVS_RW_TYPE_U8,
        .data = forward_type,
        .default_value = default_type,
    };
    return app_nvs_rw_read(NAMESPACE_WIFI, &nvs_item, 1);
}

/**
 * @brief Save the WiFi modem power-save type to NVS.
 * @param ps_mode wifi_ps_type_t as uint8_t (0=none, 1=min modem, 2=max modem)
 * @return esp_err_t
 */
esp_err_t settings_wifi_ps_mode_save(uint8_t ps_mode) {
    return app_nvs_rw_write(NAMESPACE_WIFI, (app_nvs_rw_write_item_t []) {
        {
            .key = KEY_WIFI_PS_MODE,
            .type = APP_NVS_RW_TYPE_U8,
            .data = &ps_mode,
            .length = sizeof(ps_mode),
        }
    }, 1);
}

/**
 * @brief Load the WiFi modem power-save type from NVS.
 * @param ps_mode Pointer to uint8_t where the loaded value will be stored
 * @param default_mode Default value to use if no value is stored in NVS
 * @return esp_err_t
 */
esp_err_t settings_wifi_ps_mode_load(uint8_t *ps_mode, uint8_t default_mode) {
    app_nvs_rw_read_item_t nvs_item = {
        .key = KEY_WIFI_PS_MODE,
        .type = APP_NVS_RW_TYPE_U8,
        .data = ps_mode,
        .default_value = default_mode,
    };
    return app_nvs_rw_read(NAMESPACE_WIFI, &nvs_item, 1);
}

/**
 * @brief Save the WiFi TX power to NVS.
 * @param tx_pwr Maximum transmit power in dBm (int8_t)
 * @return esp_err_t
 */
esp_err_t settings_wifi_tx_pwr_save(int8_t tx_pwr) {
    return app_nvs_rw_write(NAMESPACE_WIFI, (app_nvs_rw_write_item_t []) {
        {
            .key = KEY_WIFI_TX_PWR,
            .type = APP_NVS_RW_TYPE_I8,
            .data = &tx_pwr,
            .length = sizeof(tx_pwr),
        }
    }, 1);
}

/**
 * @brief Load the WiFi TX power from NVS.
 * @param tx_pwr Pointer to int8_t where the loaded power will be stored
 * @param default_pwr Default value to use if no value is stored in NVS
 * @return esp_err_t
 */
esp_err_t settings_wifi_tx_pwr_load(int8_t *tx_pwr, int8_t default_pwr) {
    app_nvs_rw_read_item_t nvs_item = {
        .key = KEY_WIFI_TX_PWR,
        .type = APP_NVS_RW_TYPE_I8,
        .data = tx_pwr,
        .default_value = default_pwr,
    };
    return app_nvs_rw_read(NAMESPACE_WIFI, &nvs_item, 1);
}

/**
 * @brief Save the WiFi inactive time to NVS.
 * @param inactive_time Inactivity timeout in seconds (uint16_t)
 * @return esp_err_t
 */
esp_err_t settings_wifi_inactive_time_save(uint16_t inactive_time) {
    return app_nvs_rw_write(NAMESPACE_WIFI, (app_nvs_rw_write_item_t []) {
        {
            .key = KEY_WIFI_INACTIVE_TIME,
            .type = APP_NVS_RW_TYPE_U16,
            .data = &inactive_time,
            .length = sizeof(inactive_time),
        }
    }, 1);
}

/**
 * @brief Load the WiFi inactive time from NVS.
 * @param inactive_time Pointer to uint16_t where the loaded value will be stored
 * @param default_time Default value to use if no value is stored in NVS
 * @return esp_err_t
 */
esp_err_t settings_wifi_inactive_time_load(uint16_t *inactive_time, uint16_t default_time) {
    app_nvs_rw_read_item_t nvs_item = {
        .key = KEY_WIFI_INACTIVE_TIME,
        .type = APP_NVS_RW_TYPE_U16,
        .data = inactive_time,
        .default_value = default_time,
    };
    return app_nvs_rw_read(NAMESPACE_WIFI, &nvs_item, 1);
}

/**
 * @brief Save the WiFi DHCP enable flag to NVS.
 * @param dhcp_enable Enable flag (0 or 1)
 * @return esp_err_t
 */
esp_err_t settings_wifi_dhcp_enable_save(uint8_t dhcp_enable) {
    return app_nvs_rw_write(NAMESPACE_WIFI, (app_nvs_rw_write_item_t []) {
        {
            .key = KEY_WIFI_DHCP_ENABLE,
            .type = APP_NVS_RW_TYPE_U8,
            .data = &dhcp_enable,
            .length = sizeof(dhcp_enable),
        }
    }, 1);
}

/**
 * @brief Load the WiFi DHCP enable flag from NVS.
 * @param dhcp_enable Pointer to uint8_t where the loaded value will be stored
 * @param default_enable Default value to use if no value is stored in NVS
 * @return esp_err_t
 */
esp_err_t settings_wifi_dhcp_enable_load(uint8_t *dhcp_enable, uint8_t default_enable) {
    app_nvs_rw_read_item_t nvs_item = {
        .key = KEY_WIFI_DHCP_ENABLE,
        .type = APP_NVS_RW_TYPE_U8,
        .data = dhcp_enable,
        .default_value = default_enable,
    };
    return app_nvs_rw_read(NAMESPACE_WIFI, &nvs_item, 1);
}

/**
 * @brief Save the WiFi MAC address to NVS.
 * 
 * @param mac Pointer to the 6-byte MAC address
 * @return esp_err_t
 */
esp_err_t settings_wifi_mac_addr_save(const uint8_t *mac) {
    return app_nvs_rw_write(NAMESPACE_WIFI, (app_nvs_rw_write_item_t []) {
        {
            .key = KEY_WIFI_MAC_ADDR,
            .type = APP_NVS_RW_TYPE_BLOB,
            .data = mac,
            .length = 6,
        }
    }, 1);
}

/**
 * @brief Load the WiFi MAC address from NVS. WARNING: remember to free the returned pointer to avoid a memory leak.
 * 
 * @param mac Pointer to a 6-byte buffer pointer; set to NULL if no address has been saved
 * @return esp_err_t
 */
esp_err_t settings_wifi_mac_addr_load(uint8_t **mac) {
    app_nvs_rw_read_item_t nvs_item = {
        .key = KEY_WIFI_MAC_ADDR,
        .type = APP_NVS_RW_TYPE_BLOB,
        .data = mac,
        .default_value = 0, // BLOB type does not use a default value
    };
    return app_nvs_rw_read(NAMESPACE_WIFI, &nvs_item, 1);
}

/**
 * @brief Save the WiFi IP address configuration to NVS.
 * 
 * @param info Pointer to a uint32_t array (length >= 3) containing the IP address, gateway, and netmask in little-endian format
 * @return esp_err_t
 */
esp_err_t settings_wifi_ip_addr_save(uint32_t ip, uint32_t gateway, uint32_t netmask) {
    uint32_t info[3] = {ip, gateway, netmask};
    return app_nvs_rw_write(NAMESPACE_WIFI, (app_nvs_rw_write_item_t[]) {
        {
            .key = KEY_WIFI_IP_ADDR,
            .type = APP_NVS_RW_TYPE_BLOB,
            .data = info,
            .length = sizeof(info),
        }
    }, 1);
}

/**
 * @brief Load the WiFi IP address configuration from NVS. WARNING: remember to free the returned pointer to avoid a memory leak.
 * 
 * @param info Pointer to a uint32_t pointer; the loaded array (length 3) is stored here; set to NULL if no address has been saved
 * @return esp_err_t
 */
esp_err_t settings_wifi_ip_addr_load(uint32_t **info) {
    app_nvs_rw_read_item_t nvs_item = {
        .key = KEY_WIFI_IP_ADDR,
        .type = APP_NVS_RW_TYPE_BLOB,
        .data = info,
        .default_value = 0, // BLOB type does not use a default value
    };
    return app_nvs_rw_read(NAMESPACE_WIFI, &nvs_item, 1);
}

/**
 * @brief Save the WiFi hostname to NVS.
 * 
 * @param host_name Hostname string, must be null-terminated
 * @return esp_err_t
 */
esp_err_t settings_wifi_host_name_save(const char *host_name) {
    return app_nvs_rw_write(NAMESPACE_WIFI, (app_nvs_rw_write_item_t []) {
        {
            .key = KEY_WIFI_HOST_NAME,
            .type = APP_NVS_RW_TYPE_STR,
            .data = host_name,
            .length = strlen(host_name) + 1,
        }
    }, 1);
}

/**
 * @brief Load the WiFi hostname from NVS. If no hostname has been saved or reading fails, sets *host_name_buf = NULL.
 * 
 * @param host_name_buf Pointer to where the hostname string pointer will be stored; set to NULL on failure
 * @return esp_err_t
 */

esp_err_t settings_wifi_host_name_load(char **host_name_buf) {
    app_nvs_rw_read_item_t nvs_item = {
        .key = KEY_WIFI_HOST_NAME,
        .type = APP_NVS_RW_TYPE_STR,
        .data = host_name_buf,
        .default_value = 0, // STR type does not use a default value
    };
    return app_nvs_rw_read(NAMESPACE_WIFI, &nvs_item, 1);
}
