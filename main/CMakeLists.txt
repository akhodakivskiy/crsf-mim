idf_component_register(
  SRCS "main_rx.cpp" "main_tx.cpp"
  INCLUDE_DIRS "."
  REQUIRES protocol esp_driver_gptimer esp_driver_uart)
