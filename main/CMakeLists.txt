idf_component_register(
  SRCS 
    "mim_menu.c" 
    "mim_uart.c"
    "mim_skymap.c"
    "mim_settings.c"
    "main_rx.c" 
    "main_tx.c" 
  INCLUDE_DIRS "."
  REQUIRES 
    async_logging
    util
    libcrsf 
    libnet 
    esp_driver_gptimer 
    esp_driver_uart 
    esp_driver_gpio 
    skymap)
