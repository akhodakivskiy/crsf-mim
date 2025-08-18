idf_component_register(
    SRCS "libnet.c" 
    INCLUDE_DIRS "."
    REQUIRES 
        esp_driver_gpio
        esp_wifi
        esp_eth
        nvs_flash
)

#target_compile_definitions(${COMPONENT_LIB} PUBLIC "-DLOG_LOCAL_LEVEL=ESP_LOG_NONE")
