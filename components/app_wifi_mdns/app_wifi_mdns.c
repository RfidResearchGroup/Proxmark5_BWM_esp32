#include <ctype.h>
#include <stdbool.h>
#include "esp_log.h"
#include "sdkconfig.h"
#include "app_wifi_mdns.h"

#ifdef CONFIG_PM5_MDNS_ENABLE

#include "mdns.h"

static const char *TAG = "app_mdns";
static bool s_started = false;

static void to_lower_copy(char *dst, size_t dst_size, const char *src) {
    size_t i = 0;
    for (; src[i] && i < dst_size - 1; i++) {
        dst[i] = (char)tolower((unsigned char)src[i]);
    }
    dst[i] = '\0';
}

static esp_err_t set_names(const char *hostname) {
    char name[33];
    to_lower_copy(name, sizeof(name), hostname);
    esp_err_t err = mdns_hostname_set(name);
    if (err == ESP_OK) {
        err = mdns_instance_name_set(hostname);
    }
    return err;
}

esp_err_t app_wifi_mdns_start(const char *hostname) {
    if (!hostname || !hostname[0]) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_started) {
        return ESP_OK;
    }
    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mdns_init: %s", esp_err_to_name(err));
        return err;
    }
    s_started = true;
    err = set_names(hostname);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set name: %s", esp_err_to_name(err));
    }
    return err;
}

void app_wifi_mdns_stop(void) {
    if (s_started) {
        mdns_free();
        s_started = false;
    }
}

esp_err_t app_wifi_mdns_set_hostname(const char *hostname) {
    if (!s_started) {
        return ESP_OK;
    }
    if (!hostname || !hostname[0]) {
        return ESP_ERR_INVALID_ARG;
    }
    return set_names(hostname);
}

#else

esp_err_t app_wifi_mdns_start(const char *hostname) { (void)hostname; return ESP_OK; }
esp_err_t app_wifi_mdns_set_hostname(const char *hostname) { (void)hostname; return ESP_OK; }
void app_wifi_mdns_stop(void) {}

#endif
