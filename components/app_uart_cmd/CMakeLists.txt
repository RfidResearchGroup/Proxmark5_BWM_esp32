# Register component
idf_component_register(SRCS "app_cmd_uart.c" INCLUDE_DIRS "." REQUIRES esp_driver_uart PRIV_REQUIRES esp_pm esp_timer)
