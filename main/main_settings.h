#ifndef SETTINGS_H
#define SETTINGS_H

#include <esp_err.h>

esp_err_t settings_time_zone_save(const char *tz);
esp_err_t settings_time_zone_load(char **tz_buf);

esp_err_t settings_power_save_save(uint8_t enabled);
esp_err_t settings_power_save_load(uint8_t *enabled, uint8_t default_enabled);

esp_err_t settings_ble_enable_save(uint8_t enabled);
esp_err_t settings_ble_enable_load(uint8_t *enabled, uint8_t default_enabled);

esp_err_t settings_wifi_mode_save(int mode);
esp_err_t settings_wifi_mode_load(int *mode, int default_mode);

esp_err_t settings_wifi_forward_type_save(int forward_type);
esp_err_t settings_wifi_forward_type_load(int *forward_type, int default_type);

esp_err_t settings_wifi_ps_mode_save(uint8_t ps_mode);
esp_err_t settings_wifi_ps_mode_load(uint8_t *ps_mode, uint8_t default_mode);

esp_err_t settings_wifi_tx_pwr_save(int8_t tx_pwr);
esp_err_t settings_wifi_tx_pwr_load(int8_t *tx_pwr, int8_t default_pwr);

esp_err_t settings_wifi_inactive_time_save(uint16_t inactive_time);
esp_err_t settings_wifi_inactive_time_load(uint16_t *inactive_time, uint16_t default_time);

esp_err_t settings_wifi_dhcp_enable_save(uint8_t dhcp_enable);
esp_err_t settings_wifi_dhcp_enable_load(uint8_t *dhcp_enable, uint8_t default_enable);

esp_err_t settings_wifi_mac_addr_save(const uint8_t *mac);
esp_err_t settings_wifi_mac_addr_load(uint8_t **mac);

esp_err_t settings_wifi_ip_addr_save(uint32_t ip, uint32_t gateway, uint32_t netmask);
esp_err_t settings_wifi_ip_addr_load(uint32_t **info);

esp_err_t settings_wifi_host_name_save(const char *host_name);
esp_err_t settings_wifi_host_name_load(char **host_name);

#endif // SETTINGS_H
