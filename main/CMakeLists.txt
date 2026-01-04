idf_component_register(
  SRCS
    "main.c"
    "mim_menu.c"
    "mim_uart.c"
    "mim_telem.c"
    "mim_uart.c"

# Add panel sources if enabled
   INCLUDE_DIRS "."
   REQUIRES
     async_logging
     util
     libcrsf
     skymap
     mim_settings
     mim_conn
     mim_nav
     mim_rc
     mim_panel

     cjson
     esp_http_server
     spiffs
     esp_driver_gptimer
     esp_driver_uart
     esp_driver_gpio
   )
