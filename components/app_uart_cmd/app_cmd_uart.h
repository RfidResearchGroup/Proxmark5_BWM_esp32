#ifndef APP_UART_H_
#define APP_UART_H_

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"

// ================= Hardware configuration =================
#define UART_SPP_NUM           (CONFIG_UART_SPP_NUM)
#define UART_TXD_PIN           (CONFIG_UART_SPP_TXP)
#define UART_RXD_PIN           (CONFIG_UART_SPP_RXP)
#define UART_BAUD_RATE_DEFAULT 460800   // Default communication baud rate, max supported depends on CONFIG_SOC_UART_BITRATE_MAX
#define UART_RX_TIMEOUT        200      // Receive timeout in milliseconds, needs tuning; too short may falsely timeout normal packets, too long may waste resources on incomplete packets
// The PM5 bounds its un-acked bytes in flight to this size minus slack
// (BWM_ESP_UART_RX_BUF / BWM_FC_BYTES in proxmark3 armsrc/bwm_forward.h) but sends
// on when acks stall past its 50 ms timeout, so only a radio stall longer than
// that on a full window can overflow the ring. Change one, change the other.
#define UART_RX_BUF_SIZE       12288    // Absorb device->host bursts so BLE can drain them before the UART overflows
// The rx task reads the ring in pieces of this size into a static buffer; a heap
// allocation sized by whatever is buffered can fail on the heap BLE leaves free.
#define UART_RX_CHUNK          2048
#define UART_TX_BUF_SIZE       0        // Important: with TX BUF=0, uart_write_bytes call blocks waiting
#define UART_EVENT_QUEUE_SZ    10       // UART event queue length, tune based on use; too small may lose events, too large wastes memory

// ================= Protocol constant definitions =================

// Packet type enum (must match Python side)
typedef enum {
    TYPE_HOST_CMD   = 0,
    TYPE_SLAVE_RESP = 1,
    TYPE_SLAVE_BCAST= 2,
    TYPE_UNKNOWN    = 3
} PacketType_t;

// Packet header magic number definitions
#define HDR_HOST_CMD_1      0x7C
#define HDR_HOST_CMD_2      0xC7
#define HDR_SLAVE_RESP_1    0x2D
#define HDR_SLAVE_RESP_2    0x3D
#define HDR_SLAVE_BCAST_1   0xD2
#define HDR_SLAVE_BCAST_2   0xD3

// Parse parameters
#define MAX_PAYLOAD_LEN     CONFIG_CMD_PAYLOAD_SIZE
#define CRC16_POLY          0x1021
#define CRC16_INIT          0xFFFF

// State machine state definitions
typedef enum {
    STATE_IDLE          = 0,
    STATE_HDR_2         = 1,
    STATE_CMD_LO        = 2,
    STATE_CMD_HI        = 3,
    STATE_LEN_LO        = 4,
    STATE_LEN_HI        = 5,
    STATE_PAYLOAD       = 6,
    STATE_CRC_LO        = 7,
    STATE_CRC_HI        = 8
} UartParseState_t;

// ================= Data structures =================

// Parse context
typedef struct {
    UartParseState_t state;
    PacketType_t pkt_type;
    uint16_t cmd_val;
    uint16_t len_val;
    uint16_t payload_idx;
    uint16_t crc_calc;
    uint16_t crc_recv;
     // +1 to ensure string data ends with \0, but this byte is reserved and user data shouldn't use it, so max user data length is still MAX_PAYLOAD_LEN
    uint8_t payload_buf[MAX_PAYLOAD_LEN + 1];
    uint8_t first_hdr_byte;
} UartParserCtx_t;

// Receive callback function definition
typedef void (*app_uart_command_callback_t)(PacketType_t type, uint16_t cmd, uint8_t *p_data, uint16_t length);
// Baud rate change callback function definition
typedef void (*app_uart_baudrate_change_callback_t)();

// ================= Interface functions =================

/**
 * @brief Initialize UART driver and receive task
 */
esp_err_t app_uart_init(void);

/**
 * @brief Set instruction/data receive callback
 * @param callback Callback function pointer
 */
esp_err_t app_uart_set_command_callback(app_uart_command_callback_t callback);

/**
 * @brief Set baud rate change callback function
 * @param callback Callback function pointer
 */
esp_err_t app_uart_set_baudrate_change_callback(app_uart_baudrate_change_callback_t callback);

/**
 * @brief Get current instruction communication UART baud rate
 */
uint32_t app_uart_get_baud_rate(void);

/**
 * @brief Check if target baud rate is available on current chip
 * @param baud_rate Target baud rate
 */
bool app_uart_is_baud_rate_supported(uint32_t baud_rate);

/**
 * @brief Dynamically set instruction communication UART baud rate
 * @param baud_rate Target baud rate
 */
esp_err_t app_uart_set_baud_rate(uint32_t baud_rate);

/**
 * @brief Send slave response packet (Type: SLAVE_RESP)
 * @param cmd Instruction code (corresponds to host's instruction)
 * @param data Payload data
 * @param len Payload length
 * @return ESP_OK success, other fail
 */
esp_err_t app_uart_send_response(uint16_t cmd, const uint8_t *data, uint16_t len);

/**
 * @brief Send slave broadcast packet (Type: SLAVE_BCAST)
 * @param type Broadcast type code
 * @param data Payload data
 * @param len Payload length
 * @return ESP_OK success, other fail
 */
esp_err_t app_uart_send_broadcast(uint16_t type, const uint8_t *data, uint16_t len);

/**
 * @brief Wait for UART transmission to complete to ensure previously sent data is fully sent
 * Avoid data loss in scenarios like baud rate switching that need to ensure transmission completes
 * 
 * @param ticks_to_wait Max wait time in system ticks for transmission complete, if timeout return timeout error
 * @return esp_err_t ESP_OK means complete, ESP_ERR_TIMEOUT means timeout, other values mean other errors
 */
esp_err_t app_uart_wait_for_tx_done(TickType_t ticks_to_wait);

#endif /* APP_UART_H_ */
