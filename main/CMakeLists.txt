idf_component_register(
  SRCS 
    "mim_menu.c" 
    "mim_uart.c"
    "mim_settings.c"
    "mim_skymap.c"
    "mim_rc.c" 
    "main_rx.c" 
    "main_tx.c" 
  INCLUDE_DIRS "."
  REQUIRES 
    esp_driver_gptimer 
    esp_driver_uart 
    esp_driver_gpio 

    async_logging
    util
    libcrsf 
    libnet 
    nav
    skymap)
