#ifndef TCP_SERVER_H_
#define TCP_SERVER_H_

#include <stdint.h>
#include "esp_err.h"

// Data receive callback
typedef void (*app_tcp_server_rx_callback_t)(uint8_t *data, uint16_t length);
// Client link callback: a client connected (true) or went away (false). Runs in the server task.
typedef void (*app_tcp_server_link_callback_t)(bool connected);

esp_err_t app_tcp_server_init(void);
esp_err_t app_tcp_server_deinit(void);
esp_err_t app_tcp_server_start(void);
esp_err_t app_tcp_server_stop(void);

esp_err_t app_tcp_server_send(uint8_t *data, size_t length);
esp_err_t app_tcp_server_set_rx_callback(app_tcp_server_rx_callback_t callback);
esp_err_t app_tcp_server_set_link_callback(app_tcp_server_link_callback_t callback);
esp_err_t app_tcp_server_get_state(uint8_t *state);

esp_err_t app_tcp_server_set_ip_mode(uint8_t mode);
esp_err_t app_tcp_server_set_port(uint16_t port);
esp_err_t app_tcp_server_set_so_linger(int so_linger);
esp_err_t app_tcp_server_set_tcp_nodelay(uint8_t enable);
esp_err_t app_tcp_server_set_so_sndtimeo(int sndtimeo);
esp_err_t app_tcp_server_set_keep_alive(uint8_t enable, int keep_idle, int keep_interval, int keep_count);

esp_err_t app_tcp_server_get_ip_mode(uint8_t *mode);
esp_err_t app_tcp_server_get_port(uint16_t *port);
esp_err_t app_tcp_server_get_so_linger(int *so_linger);
esp_err_t app_tcp_server_get_tcp_nodelay(uint8_t *enable);
esp_err_t app_tcp_server_get_so_sndtimeo(int *sndtimeo);
esp_err_t app_tcp_server_get_keep_alive(uint8_t *enable, int *keep_idle, int *keep_interval, int *keep_count);

#endif
