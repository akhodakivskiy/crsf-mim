idf_component_register(
    SRCS 
        "libcrsf_crc8.c" 
        "libcrsf_device.c" 
        "libcrsf_parser.c" 
        "libcrsf_payload.c" 
    INCLUDE_DIRS "."
    REQUIRES 
        esp_timer)
