#include "esp_log.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "esp_rom_uart.h"
#include "app_cmd_uart.h"
#if CONFIG_PM_ENABLE && CONFIG_FREERTOS_USE_TICKLESS_IDLE
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#define UART_LINK_SLEEP 1
#endif


static const char *TAG = "uart";
static QueueHandle_t s_spp_common_uart_queue = NULL;
static app_uart_command_callback_t s_uart_command_callback = NULL;
static app_uart_baudrate_change_callback_t s_baudrate_change_callback = NULL;
static UartParserCtx_t s_ctx = { .state = STATE_IDLE };
static TickType_t s_ticks_wait_event = portMAX_DELAY;
static uint32_t s_uart_baud_rate = UART_BAUD_RATE_DEFAULT;

#if UART_LINK_SLEEP
// Light sleep vs. the host link. The host (AT32) sends frames unsolicited and the
// UART can only wake the chip on RX edges, losing the bytes that carried them, so
// after a quiet spell the host leads with a throw-away preamble and waits a few ms
// before the real frame (bwm_uart_at32.c in the proxmark3 tree); the parser drops
// it as noise. Any traffic holds a no-light-sleep lock for UART_LINK_AWAKE_MS; the
// lock is only dropped once a preamble was seen, since a host that sends none
// would lose every frame after a quiet spell.
// DFS is unaffected by this lock: the driver holds APB_FREQ_MAX during transfers.
#define UART_LINK_AWAKE_MS      2000    // keep in step with BWM_ESP_AWAKE_MS on the host
#define UART_LINK_WAKEUP_EDGES  3       // RX rising edges that end light sleep (hw minimum)
#define UART_LINK_PREAMBLE_BYTE 0x55    // the host sends 4 of these; never a frame header byte
#define UART_LINK_PREAMBLE_RUN  3       // consecutive ones that count as a preamble (margin of one)
static esp_pm_lock_handle_t s_link_lock = NULL;
static esp_timer_handle_t s_link_timer = NULL;
static portMUX_TYPE s_link_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_link_awake = false;
static volatile bool s_host_preamble = false;   // set once, never cleared: false = never light sleep

// esp_timer task: nothing on the link for UART_LINK_AWAKE_MS, allow light sleep.
// No preamble seen yet: the lock stays held, s_link_awake stays true, and the
// next link_touch() re-arms the timer.
static void link_idle_cb(void *arg) {
    portENTER_CRITICAL(&s_link_mux);
    if (s_link_awake && s_host_preamble) {
        s_link_awake = false;
        esp_pm_lock_release(s_link_lock);   // ISR-safe, may nest in a critical section
    }
    portEXIT_CRITICAL(&s_link_mux);
}

// Fed every STATE_IDLE byte; the parser itself still drops the preamble as noise.
static inline void link_preamble_scan(uint8_t byte) {
    static uint8_t run = 0;
    if (byte != UART_LINK_PREAMBLE_BYTE) {
        run = 0;
    } else if (++run >= UART_LINK_PREAMBLE_RUN) {
        run = 0;
        s_host_preamble = true;
    }
}

// Any task that moves bytes on the link: hold the chip awake a while longer.
static void link_touch(void) {
    if (s_link_lock == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_link_mux);
    if (!s_link_awake) {
        s_link_awake = true;
        esp_pm_lock_acquire(s_link_lock);
    }
    // Re-arm inside the critical section: an expiry landing between the lock
    // acquire and the restart would otherwise release the lock we just took.
    esp_timer_stop(s_link_timer);   // ESP_ERR_INVALID_STATE if not running: fine
    esp_timer_start_once(s_link_timer, (uint64_t)UART_LINK_AWAKE_MS * 1000ULL);
    portEXIT_CRITICAL(&s_link_mux);
}

static esp_err_t link_sleep_init(void) {
    esp_err_t err = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "uart_link", &s_link_lock);
    if (err != ESP_OK) {
        return err;
    }
    const esp_timer_create_args_t targs = { .callback = link_idle_cb, .name = "uart_link" };
    err = esp_timer_create(&targs, &s_link_timer);
    if (err != ESP_OK) {
        return err;
    }
    err = uart_set_wakeup_threshold(UART_SPP_NUM, UART_LINK_WAKEUP_EDGES);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_sleep_enable_uart_wakeup(UART_SPP_NUM);
    if (err != ESP_OK) {
        return err;
    }
    link_touch();   // stay awake through boot and the host's link bring-up
    return ESP_OK;
}
#else
static inline void link_touch(void) {}
static inline void link_preamble_scan(uint8_t byte) { (void)byte; }
#endif


/**
 * @brief Calculate CRC16_CCITT
 */
static uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t init_crc) {
    uint16_t crc = init_crc;
    while (len--) {
        crc ^= ((uint16_t)(*data++) << 8);
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ CRC16_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Internal universal packet building and sending function
 * @param hdr1 Packet header byte 1
 * @param hdr2 Packet header byte 2
 * @param cmd Instruction/type code
 * @param data Payload
 * @param len Payload length
 * @return ESP_OK or ESP_FAIL
 */
static esp_err_t uart_build_and_send(uint8_t hdr1, uint8_t hdr2, uint16_t cmd, const uint8_t *data, uint16_t len) {
    if (len > MAX_PAYLOAD_LEN) {
        ESP_LOGE(TAG, "Payload too large: %d", len);
        return ESP_ERR_INVALID_ARG;
    }

    // Calculate total length: Header(2) + Cmd(2) + Len(2) + Payload(N) + CRC(2)
    size_t total_len = 2 + 2 + 2 + len + 2;

    // Adaptive buffer storage area
    bool use_heap = (total_len > 256);
    uint8_t *pkt_buf;
    uint8_t buffer_stack[256]; // Allocate max 256 bytes on stack, sufficient for most small packets
    if (use_heap) {
        pkt_buf = (uint8_t *)malloc(total_len);
        if (!pkt_buf) {
            ESP_LOGE(TAG, "Malloc failed for TX buffer");
            return ESP_ERR_NO_MEM;
        }
    } else {
        pkt_buf = buffer_stack;
    }

    size_t idx = 0;
    
    // 1. Header
    pkt_buf[idx++] = hdr1;
    pkt_buf[idx++] = hdr2;
    
    // 2. Cmd/Type (Little-Endian)
    pkt_buf[idx++] = cmd & 0xFF;
    pkt_buf[idx++] = (cmd >> 8) & 0xFF;
    
    // 3. Length (Little-Endian)
    pkt_buf[idx++] = len & 0xFF;
    pkt_buf[idx++] = (len >> 8) & 0xFF;
    
    // 4. Payload
    if (len > 0 && data != NULL) {
        memcpy(&pkt_buf[idx], data, len);
        idx += len;
    }
    
    // 5. CRC (calculate all previous bytes)
    uint16_t crc = crc16_ccitt(pkt_buf, idx, CRC16_INIT);
    pkt_buf[idx++] = crc & 0xFF;
    pkt_buf[idx++] = (crc >> 8) & 0xFF;

    // 6. Send
    link_touch();   // a reply usually follows: keep the chip out of light sleep
    int written = uart_write_bytes(UART_SPP_NUM, pkt_buf, total_len);
    
    // After uart_write_bytes, data is copied to FIFO or sent directly via peripheral, can now free allocated memory
    if (use_heap && pkt_buf) {
        free(pkt_buf);
    }

    // Check if send was successful
    if (written == total_len) {
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "UART Write incomplete: %d/%d", written, total_len);
        return ESP_FAIL;
    }
}

// Parse function
static void uart_rx_parser(uint8_t *data, size_t size) {
    UartParserCtx_t *ctx = &s_ctx;

    for (size_t i = 0; i < size; i++) {
        uint8_t byte = data[i];

        switch (ctx->state) {
            case STATE_IDLE:
                link_preamble_scan(byte);
                if (byte == HDR_HOST_CMD_1) { 
                    ctx->pkt_type = TYPE_HOST_CMD; 
                    ctx->first_hdr_byte = byte; 
                    ctx->state = STATE_HDR_2; 
                }
                else if (byte == HDR_SLAVE_RESP_1) { 
                    ctx->pkt_type = TYPE_SLAVE_RESP; 
                    ctx->first_hdr_byte = byte; 
                    ctx->state = STATE_HDR_2; 
                }
                else if (byte == HDR_SLAVE_BCAST_1) { 
                    ctx->pkt_type = TYPE_SLAVE_BCAST; 
                    ctx->first_hdr_byte = byte; 
                    ctx->state = STATE_HDR_2; 
                }
                break;

            case STATE_HDR_2: {
                bool match = false;
                if (ctx->pkt_type == TYPE_HOST_CMD && byte == HDR_HOST_CMD_2) match = true;
                else if (ctx->pkt_type == TYPE_SLAVE_RESP && byte == HDR_SLAVE_RESP_2) match = true;
                else if (ctx->pkt_type == TYPE_SLAVE_BCAST && byte == HDR_SLAVE_BCAST_2) match = true;

                if (match) {
                    uint8_t hdr[2] = {ctx->first_hdr_byte, byte};
                    ctx->crc_calc = crc16_ccitt(hdr, 2, CRC16_INIT);
                    ctx->state = STATE_CMD_LO;
                } else {
                    // Header mismatch, reset and re-check current byte
                    ctx->state = STATE_IDLE;
                    i--; 
                }
                break;
            }

            case STATE_CMD_LO:
                ctx->cmd_val = (uint16_t)byte;
                ctx->crc_calc = crc16_ccitt(&byte, 1, ctx->crc_calc);
                ctx->state = STATE_CMD_HI;
                break;

            case STATE_CMD_HI:
                ctx->cmd_val |= ((uint16_t)byte) << 8;
                ctx->crc_calc = crc16_ccitt(&byte, 1, ctx->crc_calc);
                ctx->state = STATE_LEN_LO;
                break;

            case STATE_LEN_LO:
                ctx->len_val = (uint16_t)byte;
                ctx->crc_calc = crc16_ccitt(&byte, 1, ctx->crc_calc);
                ctx->state = STATE_LEN_HI;
                break;

            case STATE_LEN_HI:
                ctx->len_val |= ((uint16_t)byte) << 8;
                ctx->crc_calc = crc16_ccitt(&byte, 1, ctx->crc_calc);
                
                if (ctx->len_val > MAX_PAYLOAD_LEN) {
                    ESP_LOGW(TAG, "Length overflow: %d", ctx->len_val);
                    ctx->state = STATE_IDLE;
                } else if (ctx->len_val == 0) {
                    ctx->state = STATE_CRC_LO;
                } else {
                    ctx->payload_idx = 0;
                    ctx->state = STATE_PAYLOAD;
                }
                break;

            case STATE_PAYLOAD:
                ctx->payload_buf[ctx->payload_idx++] = byte;
                ctx->crc_calc = crc16_ccitt(&byte, 1, ctx->crc_calc);
                if (ctx->payload_idx >= ctx->len_val) {
                    ctx->state = STATE_CRC_LO;
                }
                break;

            case STATE_CRC_LO:
                ctx->crc_recv = (uint16_t)byte;
                ctx->state = STATE_CRC_HI;
                break;

            case STATE_CRC_HI:
                ctx->crc_recv |= ((uint16_t)byte) << 8;
                if (ctx->crc_calc == ctx->crc_recv) {
                    // CRC check passed, call callback to notify upper layer
                    // ESP_LOGI(TAG, "Packet received: Type=%d, CMD=0x%04X, Len=%d, CRC valid", ctx->pkt_type, ctx->cmd_val, ctx->len_val);
                    if (s_uart_command_callback) {
                        // Before callback notification, set the last byte of Payload data to \0, ensure if Payload is string format, it's more convenient to use in callback function
                        if (ctx->len_val > 0) {
                            ctx->payload_buf[ctx->len_val] = 0x00; // 0x00 is '\0', string terminator
                        }
                        // Call command processing callback, notify upper layer of command data packet arrival
                        s_uart_command_callback(ctx->pkt_type, ctx->cmd_val, ctx->payload_buf, ctx->len_val);
                    }
                } else {
                    ESP_LOGW(TAG, "CRC mismatch: Calc=0x%04X, Recv=0x%04X", ctx->crc_calc, ctx->crc_recv);
                }
                ctx->state = STATE_IDLE;
                break;
                
            default:
                ctx->state = STATE_IDLE;
                break;
        }
    }
}

/**
 * @brief Reset UART receive state machine, clear buffers and event queue, prepare for next packet
 * 
 */
static inline void reset_uart_rx_state(void) {
    // Clear UART buffers and event queue, reset state machine, prepare for next packet
    uart_flush_input(UART_SPP_NUM);
    if (s_spp_common_uart_queue != NULL) xQueueReset(s_spp_common_uart_queue);
    s_ctx.state = STATE_IDLE;
    s_ticks_wait_event = portMAX_DELAY;
}

/**
 * @brief Reset handling on error, mainly to clear UART buffers and event queue and reset state machine when encountering exceptions during reception, prepare for next packet
 * 
 * @param error_msg Error message to print in logs
 */
static inline void reset_on_error(const char *error_msg) {
    // Print error log
    ESP_LOGE(TAG, "%s", error_msg);
    // Reuse reset function
    reset_uart_rx_state();
}

/**
 * @brief FreeRTOS task for continuously receiving and processing UART data
 * 
 * @param pvParameters 
 */
static void uart_rx_task(void *pvParameters) {
    uart_event_t event;

    ESP_LOGI(TAG, "UART RX Task started (Baud: %lu)", (unsigned long)s_uart_baud_rate);
    // Last buffer byte as reserved byte to ensure string data ends with \0
    s_ctx.payload_buf[MAX_PAYLOAD_LEN] = 0x00;
    // Continuously receive and process UART events
    for (;;) {
        // Design: when receiving but not complete, set timeout to packet receive timeout value
        // So if timeout occurs during reception, the packet being received is incomplete, must discard and reset state machine, wait for next packet
        // After completing reception, can set xQueueReceive timeout to portMAX_DELAY to wait for next event, avoid wasting CPU on unnecessary timeout checks
        if (xQueueReceive(s_spp_common_uart_queue, (void *)&event, s_ticks_wait_event)) {
            switch (event.type) {
                case UART_DATA: {
                    link_touch();
                    // Drain the driver's ring in UART_RX_CHUNK pieces. The chunk bounds
                    // memory (this task is the only reader, so one static buffer); the
                    // loop matters because the driver queues UART_DATA on arrival only,
                    // so bytes left behind by a partial read would wait for the next burst.
                    static uint8_t rbuf[UART_RX_CHUNK];
                    size_t buffered = 0;
                    while ((uart_get_buffered_data_len(UART_SPP_NUM, &buffered) == ESP_OK) && (buffered > 0)) {
                        size_t want = (buffered < sizeof(rbuf)) ? buffered : sizeof(rbuf);
                        int read_len = uart_read_bytes(UART_SPP_NUM, rbuf, want, 0);   // already buffered: no wait
                        if (read_len <= 0) {
                            break;
                        }
                        uart_rx_parser(rbuf, read_len);
                    }
                    // If last buffer byte is used by user data, may have stack overflow, very serious error
                    if (s_ctx.payload_buf[MAX_PAYLOAD_LEN] != 0x00) {
                        ESP_LOGE(TAG, "Critical Error: Payload buffer overflow detected!");
                        // Force reset to safe defaults after overflow detection
                        s_ctx.payload_buf[MAX_PAYLOAD_LEN] = 0x00;
                        s_ctx.state = STATE_IDLE;
                    }
                    // Keep timeout polling while a frame is still being assembled.
                    // If state returns to IDLE, switch back to blocking wait mode.
                    if (s_ctx.state != STATE_IDLE) {
                        s_ticks_wait_event = pdMS_TO_TICKS(UART_RX_TIMEOUT); // Set timeout checking during reception
                    } else {
                        s_ticks_wait_event = portMAX_DELAY; // Wait indefinitely for next event after completing reception
                    }
                    break;
                }
                case UART_FIFO_OVF:
                case UART_BUFFER_FULL:
                    reset_on_error("UART Overflow or Buffer Full");
                    break;
#if SOC_UART_SUPPORT_WAKEUP_INT
                case UART_WAKEUP:
                    // RX edges ended light sleep; the bytes that did it are gone
                    // (the host sent a disposable preamble for that reason).
                    link_touch();
                    break;
#endif
                default:
                    break;
            }
        } else {
            // Receive event timeout, consider current packet incomplete, reset state machine
            if (s_ctx.state != STATE_IDLE) {
                reset_on_error("UART RX Timeout - Incomplete packet");
            }
        }
    }
    vTaskDelete(NULL);
}

/**
 * @brief Initialize pass-through/control communication UART
 */
esp_err_t app_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = s_uart_baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        // XTAL, not the default PLL_F40M: under DFS the PLL is powered down whenever
        // no APB lock is held and the driver only raises it inside a read/write call,
        // so a frame the host sends unsolicited would arrive with a dead baud clock.
        // The 26 MHz crystal runs at every CPU frequency and clocks 460800 and 921600.
        .source_clk = UART_SCLK_XTAL,
    };

    int intr_alloc_flags = 0; // UART 
#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    // Install UART driver and get queue
    esp_err_t err = uart_driver_install(UART_SPP_NUM, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, UART_EVENT_QUEUE_SZ, &s_spp_common_uart_queue, intr_alloc_flags);
    if (err != ESP_OK) {
        return err;
    }
    // Set UART parameters
    err = uart_param_config(UART_SPP_NUM, &uart_config);
    if (err != ESP_OK) {
        return err;
    }
    // Set UART pins
    err = uart_set_pin(UART_SPP_NUM, UART_TXD_PIN, UART_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        return err;
    }
#if UART_LINK_SLEEP
    err = link_sleep_init();
    if (err != ESP_OK) {
        // Not fatal for the link itself, only for the power saving.
        ESP_LOGW(TAG, "UART light-sleep wakeup setup failed: %s", esp_err_to_name(err));
    }
#endif
    // Start instruction UART receive task to handle instruction data anytime
    xTaskCreate(uart_rx_task, "uTask", 4096, (void *)UART_SPP_NUM, 8, NULL);
    ESP_LOGI(TAG, "UART Initialized");
    return ESP_OK;
}

/**
 * @brief Set instruction receive callback, after receiving instruction packet on UART, notify through this callback
 * 
 * @param callback 
 */
esp_err_t app_uart_set_command_callback(app_uart_command_callback_t callback) {
    s_uart_command_callback = callback;
    return ESP_OK;
}

/**
 * @brief Set baud rate change callback function
 * @param callback Callback function pointer
 */
esp_err_t app_uart_set_baudrate_change_callback(app_uart_baudrate_change_callback_t callback) {
    s_baudrate_change_callback = callback;
    return ESP_OK;
}

/**
 * @brief Get current instruction communication UART baud rate
 * 
 * @return uint32_t UART
 */
uint32_t app_uart_get_baud_rate(void) {
    return s_uart_baud_rate;
}

/**
 * @brief Check if specified baud rate is within current chip's UART support range
 * 
 * @param baud_rate 
 * @return true 0UART
 * @return false 
 */
bool app_uart_is_baud_rate_supported(uint32_t baud_rate) {
    return baud_rate > 0 && baud_rate <= CONFIG_SOC_UART_BITRATE_MAX;
}

/**
 * @brief Internal function to execute baud rate switching, wait for current transmission to complete before switching to avoid data loss
 * 
 * @param baud_rate Baud rate to switch to, must be within current chip's UART support range or return error
 * @return esp_err_t Status code, ESP_OK means success, other values mean failure
 */
static esp_err_t app_change_baudrate(uint32_t baud_rate) {
    // Wait for current transmission to complete before switching baud rate to avoid data loss
    esp_err_t err = app_uart_wait_for_tx_done(portMAX_DELAY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[BAUDRATE_UPDATE] UART wait tx done failed: %s", esp_err_to_name(err));
        return err;
    }
    // Switch to new baud rate, if switch succeeds can continue execution
    err = uart_set_baudrate(UART_SPP_NUM, baud_rate);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[BAUDRATE_UPDATE] UART set baudrate failed: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

// Wrap macro function to call baud rate change callback, avoid writing code every time to check if callback exists
#define NOTIFY_BAUDRATE_CHANGE() do { \
    if (s_baudrate_change_callback) s_baudrate_change_callback(); \
} while(0)

/**
 * @brief Set instruction communication UART baud rate; note this interface waits for current transmission to complete before switching to avoid data loss
 * 
 * @param baud_rate Baud rate to set, must be within current chip's UART support range or return error
 * @return esp_err_t Status code, ESP_OK means success, other values mean failure
 */
esp_err_t app_uart_set_baud_rate(uint32_t baud_rate) {
    if (!app_uart_is_baud_rate_supported(baud_rate)) {
        return ESP_ERR_INVALID_ARG;
    }

    // If current baud rate already equals target, no need to switch, return success
    if (baud_rate == s_uart_baud_rate) {
        // Even if baud rate unchanged, notify upper layer once to prepare, avoid upper layer waiting for notification that never comes
        NOTIFY_BAUDRATE_CHANGE();
        return ESP_OK;
    }

    // Switch to new baud rate, if fails stay on old rate
    esp_err_t err = app_change_baudrate(baud_rate);
    if (err != ESP_OK) {
        return err;
    }

    // Switch back to old baud rate, very unlikely to fail; if it does, it's serious design error or hardware issue
    // Can't continue if rollback fails, means UART interface is very unstable
    // Must restart chip, UART baud rate wasn't designed to be persistent, so restart recovers default rate
    // This ensures device doesn't brick from incorrect baud rate setting
    err = uart_set_baudrate(UART_SPP_NUM, s_uart_baud_rate);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART set baudrate rollback failed: %s", esp_err_to_name(err));
        esp_restart(); // Directly restart chip to recover default baud rate, avoid bricking
    }

    // Callback to upper layer to notify baud rate will change, let upper layer prepare (e.g., reply to host)
    NOTIFY_BAUDRATE_CHANGE();

    // Switch to new baud rate, this step must not fail or host will lose communication with device
    // But there's no way around it; host may change baud rate immediately after callback, we stay on old rate and retry indefinitely
    while(app_change_baudrate(baud_rate) != ESP_OK) {
        ESP_LOGE(TAG, "UART set baudrate to new value failed: %s, retrying...", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(200)); // Wait before retrying to avoid excessive retries causing system overload
    }

    // Update global baud rate variable and reset receive state machine to receive with new rate
    s_uart_baud_rate = baud_rate;
    reset_uart_rx_state();
    ESP_LOGI(TAG, "UART baudrate updated to %lu (max %d)", (unsigned long)s_uart_baud_rate, CONFIG_SOC_UART_BITRATE_MAX);
    return ESP_OK;
}

/**
 * @brief Send response data packet, this is thread-safe interface
 * 
 * @param cmd Response instruction
 * @param data Data payload, can pass NULL if no payload
 * @param len Data length, no payload returned if length is 0
 * @return esp_err_t Status code
 */
esp_err_t app_uart_send_response(uint16_t cmd, const uint8_t *data, uint16_t len) {
    return uart_build_and_send(HDR_SLAVE_RESP_1, HDR_SLAVE_RESP_2, cmd, data, len);
}

/**
 * @brief Send broadcast data packet, this is thread-safe interface
 * 
 * @param type 
 * @param data Data payload, can pass NULL if no payload
 * @param len Data length, no payload returned if length is 0
 * @return esp_err_t Status code
 */
esp_err_t app_uart_send_broadcast(uint16_t type, const uint8_t *data, uint16_t len) {
    return uart_build_and_send(HDR_SLAVE_BCAST_1, HDR_SLAVE_BCAST_2, type, data, len);
}

/**
 * @brief UART
 * 
 * @param ticks_to_wait Max wait time in system ticks for transmission complete, if timeout return timeout error
 * @return esp_err_t ESP_OK means complete, ESP_ERR_TIMEOUT means timeout, other values mean other errors
 */
esp_err_t app_uart_wait_for_tx_done(TickType_t ticks_to_wait) {
    return uart_wait_tx_done(UART_SPP_NUM, ticks_to_wait);
}
