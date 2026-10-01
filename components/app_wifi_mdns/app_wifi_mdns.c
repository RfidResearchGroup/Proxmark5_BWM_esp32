#include <ctype.h>
#include <string.h>
#include <stdbool.h>
#include "esp_log.h"
#include "sdkconfig.h"
#include "app_wifi_mdns.h"

#ifdef CONFIG_PM5_MDNS_ENABLE

#include "mdns.h"

#define MDNS_SERVICE_TYPE  "_proxmark5"
#define MDNS_SERVICE_PROTO "_tcp"

static const char *TAG = "app_mdns";
static bool s_running = false;

static void to_lower_copy(char *dst, size_t dst_size, const char *src) {
    size_t i = 0;
    for (; src[i] && i < dst_size - 1; i++) {
        dst[i] = (char)tolower((unsigned char)src[i]);
    }
    dst[i] = '\0';
}

esp_err_t app_wifi_mdns_start(const char *hostname, uint16_t port) {
    if (!hostname || !hostname[0]) {
        return ESP_ERR_INVALID_ARG;
    }
    // mdns re-announces on IP change itself
    if (s_running) {
        return ESP_OK;
    }

    char name[33];
    to_lower_copy(name, sizeof(name), hostname);

    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mdns_init failed: %s", esp_err_to_name(err));
        return err;
    }
    err = mdns_hostname_set(name);
    if (err == ESP_OK) {
        err = mdns_instance_name_set(hostname);
    }
    if (err == ESP_OK && port != 0) {
        err = mdns_service_add(NULL, MDNS_SERVICE_TYPE, MDNS_SERVICE_PROTO, port, NULL, 0);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mdns setup failed: %s", esp_err_to_name(err));
        mdns_free();
        return err;
    }
    s_running = true;
    ESP_LOGI(TAG, "mDNS responder up: %s.local (service port %u)", name, (unsigned)port);
    return ESP_OK;
}

esp_err_t app_wifi_mdns_set_hostname(const char *hostname) {
    if (!s_running) {
        return ESP_OK;
    }
    if (!hostname || !hostname[0]) {
        return ESP_ERR_INVALID_ARG;
    }
    char name[33];
    to_lower_copy(name, sizeof(name), hostname);
    esp_err_t err = mdns_hostname_set(name);
    if (err == ESP_OK) {
        err = mdns_instance_name_set(hostname);
    }
    return err;
}

esp_err_t app_wifi_mdns_stop(void) {
    if (!s_running) {
        return ESP_OK;
    }
    mdns_free();
    s_running = false;
    return ESP_OK;
}

#else /* !CONFIG_PM5_MDNS_ENABLE */

esp_err_t app_wifi_mdns_start(const char *hostname, uint16_t port) { (void)hostname; (void)port; return ESP_OK; }
esp_err_t app_wifi_mdns_set_hostname(const char *hostname) { (void)hostname; return ESP_OK; }
esp_err_t app_wifi_mdns_stop(void) { return ESP_OK; }

#endif
