#ifndef LIBCRSF_DEF_H
#define LIBCRSF_DEF_H

#include <sdkconfig.h>

#include <stdint.h>
#include <stdbool.h>

#define CRSF_MIN_FRAME_LEN 4 // sync+type+len+crc
#define CRSF_MAX_FRAME_LEN 64 // sync+type+len+[payload]+crc
#define CRSF_MAX_PAYLOAD_LEN (CRSF_MAX_FRAME_LEN - 4)
#define CRSF_SYNC_BYTE 0xC8
#define CRSF_SYNC_BYTE_EDGETX 0xEE

#ifdef CONFIG_LIBCRSF_PARAM_MAX_COUNT 
#define CRSF_DEVICE_PARAM_MAX_COUNT CONFIG_LIBCRSF_PARAM_MAX_COUNT
#else
#define CRSF_DEVICE_PARAM_MAX_COUNT 10
#endif

#ifdef CONFIG_LIBCRSF_PARAM_SELECT_OPTION_MAX_COUNT 
#define CRSF_DEVICE_PARAM_SELECT_OPTION_MAX_COUNT CONFIG_LIBCRSF_PARAM_SELECT_OPTION_MAX_COUNT 
#else
#define CRSF_DEVICE_PARAM_SELECT_OPTION_MAX_COUNT 10
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum : uint8_t {
    CRSF_ADDRESS_BROADCAST = 0x00,
    CRSF_ADDRESS_USB = 0x10,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
    CRSF_ADDRESS_RESERVED1 = 0x8A,
    CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
    CRSF_ADDRESS_GPS = 0xC2,
    CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
    CRSF_ADDRESS_RESERVED2 = 0xCA,
    CRSF_ADDRESS_RACE_TAG = 0xCC,
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
    CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
    CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE,
    CRSF_ADDRESS_ELRS_LUA = 0xEF,

    CRSF_ADDRESS_CRSF_MIM = 0xD0,
} crsf_address_t;

typedef enum : uint8_t {
    CRSF_FRAME_TYPE_NONE = 0x00,
    CRSF_FRAME_TYPE_GPS = 0x02,
    CRSF_FRAME_TYPE_VARIO = 0x07,
    CRSF_FRAME_TYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAME_TYPE_BARO_ALTITUDE = 0x09,
    CRSF_FRAME_TYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAME_TYPE_OPENTX_SYNC = 0x10,
    CRSF_FRAME_TYPE_HEARTBEAT = 0x0B,
    CRSF_FRAME_TYPE_RADIO_ID = 0x3A,
    CRSF_FRAME_TYPE_RC_CHANNELS_PACKED = 0x16,
    CRSF_FRAME_TYPE_ATTITUDE = 0x1E,
    CRSF_FRAME_TYPE_FLIGHT_MODE = 0x21,

    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAME_TYPE_DEVICE_PING = 0x28,
    CRSF_FRAME_TYPE_DEVICE_INFO = 0x29,
    CRSF_FRAME_TYPE_PARAM_ENTRY = 0x2B,
    CRSF_FRAME_TYPE_PARAM_READ = 0x2C,
    CRSF_FRAME_TYPE_PARAM_WRITE = 0x2D,
    CRSF_FRAME_TYPE_ELRS_STATUS = 0x2E,
    CRSF_FRAME_TYPE_COMMAND = 0x32,

    // MSP commands
    CRSF_FRAME_TYPE_MSP_REQ = 0x7A,    // response request using msp sequence as command
    CRSF_FRAME_TYPE_MSP_RESP = 0x7B,   // reply with 58 byte chunked binary
    CRSF_FRAME_TYPE_MSP_WRITE = 0x7C,  // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
    CRSF_FRAME_TYPE_ARDUPILOT_LEGACY = 0x7F,  // Legacy Ardupilot Frame
} crsf_frame_type_t;

typedef enum : uint8_t {
    CRSF_FRAME_SUBTYPE_TIMING_CORRECTION = 0x10,
} crsf_frame_subtype_t;

typedef struct {
    uint8_t sync;
    uint8_t length;
    crsf_frame_type_t type;
    uint8_t data[CRSF_MAX_PAYLOAD_LEN + 1]; // +1 for CRC

    bool is_extended;
    crsf_address_t source;
    crsf_address_t dest;
} crsf_frame_t;

#ifdef __cplusplus
}
#endif

#endif
