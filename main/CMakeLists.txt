# For now, only main.c and main_settings.c need to be registered; other implementations are in their respective components.
set(srcs 
    "main.c" 
    "main_settings.c"
)

# Component list. Add new components here if needed in the future.
set(components
    nvs_flash 
    app_common 
    app_nvs_rw
    app_uart_log 
    app_uart_cmd 
    app_ota_ops 
    app_ble_spp 
    app_power 
    app_mqtt_client 
    app_tcp_client 
    app_tcp_server 
    app_udp_client 
    app_udp_server 
    app_wifi_scanner 
    app_wifi_connect 
    app_wifi_sntp 
    app_wifi_netif_cfg
)

# Register component
idf_component_register(SRCS "${srcs}"
    PRIV_REQUIRES ${components}
    INCLUDE_DIRS ".")
