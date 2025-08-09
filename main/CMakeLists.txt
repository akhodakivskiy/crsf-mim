idf_component_register(
  SRCS "main_rx.cpp" "main_tx.cpp"
  INCLUDE_DIRS "."
  REQUIRES protocol esp_driver_gptimer esp_driver_uart esp_wifi esp_event esp_netif nvs_flash lwip proto)
