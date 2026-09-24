#ifndef APP_BLE_H_
#define APP_BLE_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifndef BLE_DEV_ADDR_LEN
#define BLE_DEV_ADDR_LEN 6
#endif

#define APP_BLE_ADV_MFG_DATA_MAX_LEN    24U
#define APP_BLE_DEVICE_NAME_MAX_LEN     32U // Including the terminating null character; usable length is 31 characters

typedef void (*app_ble_rx_callback_t)(uint8_t *data, uint16_t length);
// Called from the NimBLE host task when a central connects (true) or the
// connection ends (false), including a stop of the stack while connected.
typedef void (*app_ble_link_callback_t)(bool connected);

esp_err_t app_ble_init(void);
esp_err_t app_ble_deinit(void);
esp_err_t app_ble_start(void);
esp_err_t app_ble_stop(void);

esp_err_t app_ble_send(uint8_t *data, size_t length);
esp_err_t app_ble_set_rx_callback(app_ble_rx_callback_t callback);
esp_err_t app_ble_set_link_callback(app_ble_link_callback_t callback);
esp_err_t app_ble_get_state(uint8_t *state);

esp_err_t app_ble_set_adv_mfg_data(uint8_t *data, size_t length);
esp_err_t app_ble_get_adv_mfg_data(uint8_t *data, size_t length, uint8_t *length_out);

esp_err_t app_ble_set_device_name(const char *name);
esp_err_t app_ble_get_device_name(char *name_buf, size_t buf_len);

esp_err_t app_ble_set_notify_retry_max(uint16_t nomem_retry_max, uint16_t fail_retry_max);
esp_err_t app_ble_get_notify_retry_max(uint16_t *nomem_retry_max, uint16_t *fail_retry_max);

esp_err_t app_ble_set_device_addr(uint8_t addr[BLE_DEV_ADDR_LEN]);
esp_err_t app_ble_get_device_addr(uint8_t addr[BLE_DEV_ADDR_LEN]);

esp_err_t app_ble_set_bonding_enable(uint8_t enable);
esp_err_t app_ble_get_bonding_enable(uint8_t *enable);
esp_err_t app_ble_set_bonding_key(const char *passkey);
esp_err_t app_ble_get_bonding_key(char *passkey, size_t buf_len);

esp_err_t app_ble_set_battery_level(uint8_t level);
esp_err_t app_ble_get_battery_level(uint8_t *level);

esp_err_t app_ble_set_tx_power(uint8_t type, uint8_t level);
esp_err_t app_ble_get_tx_power(uint8_t type, uint8_t *level);

// Advertising duty: low (two-phase, fast then slow) or NimBLE's fast default
// throughout. Takes effect at once if the stack is advertising; safe to call
// before app_ble_init(). Driven by app_power.c.
esp_err_t app_ble_set_adv_low_duty(bool low_duty);

// --- The following APIs can only be called after app_ble_start() has been invoked to start the BLE stack

esp_err_t app_ble_get_bonded_count(uint8_t *count);
esp_err_t app_ble_get_bonded_device_address(uint8_t index, uint8_t addr[BLE_DEV_ADDR_LEN], uint8_t *type);
esp_err_t app_ble_del_bonded_device(uint8_t index);
esp_err_t app_ble_clear_bonded(void);

#endif /* APP_BLE_H_ */
