idf_component_register(
    SRCS 
        "libcrsf.c" 
        "libcrsf_device.c" 
        "libcrsf_parser.c" 
        "libcrsf_payload.c" 
    INCLUDE_DIRS "."
    REQUIRES esp_app_format esp_timer tools)

idf_build_set_property(COMPILE_OPTIONS "-Wno-missing-field-initializers" APPEND)
