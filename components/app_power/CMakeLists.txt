# Register component
idf_component_register(SRCS "app_power.c" INCLUDE_DIRS "." PRIV_REQUIRES app_ble_spp esp_pm)
