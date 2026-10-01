# Register component
idf_component_register(SRCS "app_wifi_mdns.c" INCLUDE_DIRS "." REQUIRES esp_wifi esp_netif)
