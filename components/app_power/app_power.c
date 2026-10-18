// Runtime power-save switch. See app_power.h.
#include "app_power.h"
#include "esp_log.h"
#include "sdkconfig.h"
#if CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif
#include "app_ble_spp.h"

static const char *TAG = "power";
static bool s_enabled = true;

#if CONFIG_PM_ENABLE
// Held while power save is OFF: pins the clock at max_freq_mhz, which together
// with light_sleep_enable = false in apply() gives the stock always-on behaviour.
static esp_pm_lock_handle_t s_cpu_lock = NULL;
static bool s_cpu_lock_held = false;

static void hold_cpu_lock(bool hold) {
    if (hold == s_cpu_lock_held) {
        return;
    }
    if (hold) {
        esp_pm_lock_acquire(s_cpu_lock);
    } else {
        esp_pm_lock_release(s_cpu_lock);
    }
    s_cpu_lock_held = hold;
}
#endif

static esp_err_t apply(bool enabled) {
#if CONFIG_PM_ENABLE
    // CONFIG_PM_DFS_INIT_AUTO configured DFS (max = default CPU clock, min = the
    // crystal) with light sleep off; this is the only place that reconfigures it.
    esp_pm_config_t cfg = {
        .max_freq_mhz = enabled ? APP_POWER_SAVE_CPU_MHZ : CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_XTAL_FREQ,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = enabled,
#endif
    };
    esp_err_t err = esp_pm_configure(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_pm_configure failed: %s", esp_err_to_name(err));
        return err;
    }
    hold_cpu_lock(!enabled);
#endif
    app_ble_set_adv_low_duty(enabled);
    return ESP_OK;
}

esp_err_t app_power_init(bool enabled) {
#if CONFIG_PM_ENABLE
    esp_err_t err = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "pwrsave_off", &s_cpu_lock);
    if (err != ESP_OK) {
        return err;
    }
#endif
    return app_power_set_enabled(enabled) == ESP_OK ? ESP_OK : ESP_FAIL;
}

esp_err_t app_power_set_enabled(bool enabled) {
    esp_err_t err = apply(enabled);
    if (err != ESP_OK) {
        return err;
    }
    s_enabled = enabled;
    ESP_LOGI(TAG, "power save %s", enabled ? "on" : "off");
    return ESP_OK;
}

bool app_power_get_enabled(void) {
    return s_enabled;
}
