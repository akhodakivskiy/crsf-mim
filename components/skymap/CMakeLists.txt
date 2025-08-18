idf_component_register(
    SRCS 
        "pb_common.c" 
        "pb_decode.c"
        "pb_encode.c"
        "skymap.pb.c"
        "skymap.c"
    INCLUDE_DIRS "."
    REQUIRES)

#target_compile_definitions(${COMPONENT_LIB} PUBLIC "-DLOG_LOCAL_LEVEL=ESP_LOG_WARN")
