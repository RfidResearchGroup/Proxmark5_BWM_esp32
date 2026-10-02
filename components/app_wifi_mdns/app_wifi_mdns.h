#ifndef APP_WIFI_MDNS_H_
#define APP_WIFI_MDNS_H_

#include "esp_err.h"

// Needs the default event loop and STA netif; call after app_wifi_connect_init()
esp_err_t app_wifi_mdns_start(const char *hostname);
// Call before app_wifi_connect_deinit()
void app_wifi_mdns_stop(void);
// Rename host; no-op if not started
esp_err_t app_wifi_mdns_set_hostname(const char *hostname);

#endif
