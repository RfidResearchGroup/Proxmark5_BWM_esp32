#ifndef APP_WIFI_MDNS_H_
#define APP_WIFI_MDNS_H_

#include "esp_err.h"

// Init once at boot; mdns follows STA up/down itself
esp_err_t app_wifi_mdns_start(const char *hostname);
// Rename host; no-op if not started
esp_err_t app_wifi_mdns_set_hostname(const char *hostname);

#endif
