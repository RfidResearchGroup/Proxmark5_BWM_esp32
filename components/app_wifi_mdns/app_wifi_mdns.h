#ifndef APP_WIFI_MDNS_H_
#define APP_WIFI_MDNS_H_

#include <stdint.h>
#include "esp_err.h"

// Start responder for <hostname>.local; port != 0 also advertises _proxmark5._tcp
esp_err_t app_wifi_mdns_start(const char *hostname, uint16_t port);
// Rename host; no-op if not running
esp_err_t app_wifi_mdns_set_hostname(const char *hostname);
esp_err_t app_wifi_mdns_stop(void);

#endif
