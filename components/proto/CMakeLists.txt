# components/proto/CMakeLists.txt
idf_component_register(
    SRCS
        "generated/protocol.pb.c"
        "generated/google/protobuf/timestamp.pb.c"
    INCLUDE_DIRS
        "generated"                       # so #include "protocol.pb.h" works
        "generated"                       # root that contains google/protobuf/…
    REQUIRES
        nanopb                            # uses pb_encode/pb_decode
)
