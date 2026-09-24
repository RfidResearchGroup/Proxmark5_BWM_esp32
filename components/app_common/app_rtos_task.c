#include <stdint.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_rtos_task.h"

// Log tag
#define TAG "rtos_task"


/**
 * @brief Waits up to timeout_ms for the FreeRTOS task to exit.
 * 
 */
void wait_for_rtos_task_exit(int timeout_ms, TaskHandle_t *task_handle) {
    int64_t start_time = esp_timer_get_time();
    do {
        // Task already exited; return immediately
        if (*task_handle == NULL) {
            return;
        }
        // Task still running; check for timeout
        if (esp_timer_get_time() - start_time > (timeout_ms * 1000)) {
            break; // Timed out; exit loop and force-delete below
        }
        // Yield: a busy spin here burns the CPU at full clock for the whole wait
        // and, at equal priority, starves the very task we are waiting on.
        vTaskDelay(1);
    } while(1);
    // Task did not exit within the timeout; force-delete to avoid a deadlock
    ESP_LOGW(TAG, "Connect task did not exit gracefully, forcing delete");
    vTaskDelete(*task_handle);
    *task_handle = NULL;
}
