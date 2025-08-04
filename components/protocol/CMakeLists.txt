idf_component_register(
    SRCS 
        "protocol_crsf.cpp" 
        "protocol_crsf_device.cpp" 
        "protocol_crsf_device_param.cpp" 
    INCLUDE_DIRS "."
    REQUIRES esp_app_format esp_timer tools)

idf_build_set_property(COMPILE_OPTIONS "-Wno-missing-field-initializers" APPEND)
