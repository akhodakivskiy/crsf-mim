idf_component_register(
  SRCS "main_rx.cpp" "main_tx.cpp" "mim_menu.c"
  INCLUDE_DIRS "."
  REQUIRES libcrsf esp_driver_gptimer esp_driver_uart async_logging)
