idf_component_register(
    SRCS "libnet.c" 
    INCLUDE_DIRS "."
    REQUIRES 
        esp_wifi
        esp_eth
        esp_driver_gpio
        nvs_flash
)
