#ifndef PROTOCOL_CRSF_H
#define PROTOCOL_CRSF_H

#include <stdint.h>

#include "crc8.h"

#if !defined(PACKED)
#define PACKED __attribute__((packed))
#endif

#define CRSF_NUM_CHANNELS 16
#define CRSF_CHANNEL_VALUE_MIN 172  // 987us - actual CRSF min is 0 with E.Limits on
#define CRSF_CHANNEL_VALUE_1000 191
#define CRSF_CHANNEL_VALUE_MID 992
#define CRSF_CHANNEL_VALUE_2000 1792
#define CRSF_CHANNEL_VALUE_MAX 1811  // 2012us - actual CRSF max is 1984 with E.Limits on
#define CRSF_CHANNEL_VALUE_SPAN (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN)
#define CRSF_ELIMIT_US_MIN 891                           // microseconds for CRSF=0 (E.Limits=ON)
#define CRSF_ELIMIT_US_MAX 2119                          // microseconds for CRSF=1984
#define CRSF_MAX_PACKET_SIZE 64                          // max declared len is 62+DEST+LEN on top of that = 64
#define CRSF_MAX_PAYLOAD_LEN (CRSF_MAX_PACKET_SIZE - 4)  // Max size of payload in [dest] [len] [type] [payload] [crc8]

#define CRSF_CRC_POLY 0xD5

typedef enum {
    CRSF_PAYLOAD_LENGTH_LINK_STATISTICS = 12,
    CRSF_PAYLOAD_LENGTH_RC_CHANNELS_PACKED = 24, // 11 bits per channel * 16 channels = 22 bytes.
    CRSF_PAYLOAD_LENGTH_HEARTBEAT = 4, // 11 bits per channel * 16 channels = 22 bytes.
    CRSF_PAYLOAD_LENGTH_DEVICE_PING = 4,
    CRSF_PAYLOAD_LENGTH_PARAM_READ  = 6,
} crsf_payload_length_t;

typedef enum : uint8_t {
    CRSF_FRAME_TYPE_NONE = 0x00,
    CRSF_FRAME_TYPE_GPS = 0x02,
    CRSF_FRAME_TYPE_VARIO = 0x07,
    CRSF_FRAME_TYPE_BATTERY_SENSOR = 0x08,
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
    CRSF_FRAME_TYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAME_TYPE_PARAMETER_READ = 0x2C,
    CRSF_FRAME_TYPE_PARAMETER_WRITE = 0x2D,
    CRSF_FRAME_TYPE_ELRS_STATUS = 0x2E,
    CRSF_FRAME_TYPE_COMMAND = 0x32,
    // MSP commands
    CRSF_FRAME_TYPE_MSP_REQ = 0x7A,    // response request using msp sequence as command
    CRSF_FRAME_TYPE_MSP_RESP = 0x7B,   // reply with 58 byte chunked binary
    CRSF_FRAME_TYPE_MSP_WRITE = 0x7C,  // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
    CRSF_FRAME_TYPE_ARDUPILOT_LEGACY = 0x7F,  // Legacy Ardupilot Frame
} crsf_frame_type_t;

typedef enum {
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
} crsf_frame_address_t;

typedef enum : uint8_t {
    CRSF_PARAM_TYPE_UINT8 = 0,
    CRSF_PARAM_TYPE_INT8 = 1,
    CRSF_PARAM_TYPE_UINT16 = 2,
    CRSF_PARAM_TYPE_INT16 = 3,
    CRSF_PARAM_TYPE_UINT32 = 4,
    CRSF_PARAM_TYPE_INT32 = 5,
    CRSF_PARAM_TYPE_UINT64 = 6,
    CRSF_PARAM_TYPE_INT64 = 7,
    CRSF_PARAM_TYPE_FLOAT = 8,
    CRSF_PARAM_TYPE_SELECT = 9,
    CRSF_PARAM_TYPE_STRING = 10,
    CRSF_PARAM_TYPE_FOLDER = 11,
    CRSF_PARAM_TYPE_INFO = 12,
    CRSF_PARAM_TYPE_COMMAND = 13,
    CRSF_PARAM_TYPE_VTX = 15,
    CRSF_PARAM_TYPE_OUT_OF_RANGE = 127,
} crsf_device_param_type_t;

typedef enum {
    CRSF_COMMAND_STEP_IDLE,
    CRSF_COMMAND_STEP_CLICK,
    CRSF_COMMAND_STEP_EXECUTING,
    CRSF_COMMAND_STEP_ASKCONFIRM,
    CRSF_COMMAND_STEP_CONFIRMED,
    CRSF_COMMAND_STEP_CANCEL,
    CRSF_COMMAND_STEP_QUERY,
    CRSF_COMMAND_STEP__MAX,
} crsf_device_command_step_t;

template<typename T> 
struct crsf_device_param_value_basic_s {
    T value;
    T value_min;
    T value_max;
    const char *units;
};

typedef union {
    crsf_device_param_value_basic_s<int8_t> value_int8_t;
    crsf_device_param_value_basic_s<uint8_t> value_uint8_t;
    crsf_device_param_value_basic_s<int16_t> value_int16_t;
    crsf_device_param_value_basic_s<uint16_t> value_uint16_t;
    crsf_device_param_value_basic_s<int32_t> value_int32_t;
    crsf_device_param_value_basic_s<uint32_t> value_uint32_t;
    crsf_device_param_value_basic_s<int64_t> value_int64_t;
    crsf_device_param_value_basic_s<uint64_t> value_uint64_t;
    struct {
        int32_t value;
        int32_t value_min;
        int32_t value_max;
        uint8_t precision;
        uint32_t step;
        const char *units;
    } value_float;
    struct {
        uint8_t value;
        const char *options[10];
        uint16_t options_count;
        const char *units;
    } value_select;
    struct {
        const char *value;
        uint8_t length_max;
    } value_string;
    struct {
        const char *name;
    } value_folder;
    struct {
        const char *name;
    } value_info;
    struct {
        crsf_device_command_step_t step;
        uint8_t timeout;
        const char *status;
    } value_command;
} crsf_device_param_value_t;

typedef struct {
    uint8_t sync;
    uint8_t length; // type + payload + crc
    crsf_frame_type_t type;
    uint8_t data[CRSF_MAX_PAYLOAD_LEN + 1]; // + 1 for crc byte
} PACKED crsf_frame_t;

typedef enum {
    CRSF_FRAME_STATUS_FRAME_READY,
    CRSF_FRAME_STATUS_SYNC,
    CRSF_FRAME_STATUS_LENGTH,
    CRSF_FRAME_STATUS_TYPE,
    CRSF_FRAME_STATUS_PAYLOAD,
    CRSF_FRAME_STATUS_ERROR_SYNC,
    CRSF_FRAME_STATUS_ERROR_LENGTH,
    CRSF_FRAME_STATUS_ERROR_CRC,
} crsf_frame_status_t;

typedef struct {
    crsf_frame_t *frame;
    uint8_t cursor;
    crsf_frame_status_t status;
} crsf_parse_ctx_t;

typedef struct {
    uint16_t ch0 : 11;
    uint16_t ch1 : 11;
    uint16_t ch2 : 11;
    uint16_t ch3 : 11;
    uint16_t ch4 : 11;
    uint16_t ch5 : 11;
    uint16_t ch6 : 11;
    uint16_t ch7 : 11;
    uint16_t ch8 : 11;
    uint16_t ch9 : 11;
    uint16_t ch10 : 11;
    uint16_t ch11 : 11;
    uint16_t ch12 : 11;
    uint16_t ch13 : 11;
    uint16_t ch14 : 11;
    uint16_t ch15 : 11;
} PACKED crsf_payload_channels_t;

typedef struct {
    crsf_frame_address_t source;
} PACKED crsf_payload_heartbeat_t;

typedef struct {
    uint8_t uplink_rssi_1;
    uint8_t uplink_rssi_2;
    uint8_t uplink_link_quality;
    int8_t uplink_snr;
    uint8_t active_antenna;
    uint8_t rf_mode;
    uint8_t uplink_tx_power;
    uint8_t downlink_rssi;
    uint8_t downlink_link_quality;
    int8_t downlink_snr;
} PACKED crsf_payload_link_statistics_t;

typedef struct {
    crsf_frame_address_t dest;
    crsf_frame_address_t source;
} PACKED crsf_payload_device_ping_t;

typedef struct {
    crsf_frame_address_t dest;
    crsf_frame_address_t source;
    const char *name;
    uint32_t serial;
    uint8_t version_hw[4];
    uint8_t version_sw[4];
    uint8_t param_num;
    uint8_t param_version;
} crsf_payload_device_info_t;

typedef struct {
    crsf_frame_address_t dest;
    crsf_frame_address_t source;
    uint8_t param_idx;
    uint8_t chunk_idx;
} crsf_payload_parameter_read_t;

typedef uint32_t crsf_payload_parameter_value_t;

typedef struct {
    crsf_frame_address_t dest;
    crsf_frame_address_t source;
    uint8_t param_idx;
    crsf_payload_parameter_value_t value;
    uint8_t value_size;
} crsf_payload_parameter_write_t;

typedef struct {
    crsf_frame_address_t dest;
    crsf_frame_address_t source;
    uint8_t param_idx;
    uint8_t chunks_remaining;
    uint8_t parent_idx;
    crsf_device_param_type_t type;
    bool hidden;
    const char *label;
    crsf_device_param_value_t value;
} crsf_payload_parameter_entry_t;

typedef struct {
    crsf_frame_address_t dest;
    crsf_frame_address_t source;
    uint8_t packets_bad;
    uint16_t packets_good;
    uint8_t flags;
    const char *message;
} crsf_payload_elrs_status_t;

bool crsf_validate_frame(crsf_frame_t *f);
void crsf_frame_partial_parse(crsf_parse_ctx_t *frame, uint8_t b);

void crsf_parse_channels(const crsf_frame_t *frame, crsf_payload_channels_t *payload);
void crsf_parse_heartbeat(const crsf_frame_t *frame, crsf_payload_heartbeat_t *payload);
void crsf_parse_device_ping(const crsf_frame_t *frame, crsf_payload_device_ping_t *payload);
void crsf_parse_parameter_read(const crsf_frame_t *frame, crsf_payload_parameter_read_t *payload);
void crsf_parse_parameter_write(const crsf_frame_t *frame, crsf_payload_parameter_write_t *payload);

void crsf_serialize_channels_packed(const crsf_payload_channels_t *payload, crsf_frame_t *frame);
void crsf_serialize_link_statistics(const crsf_payload_link_statistics_t *payload, crsf_frame_t *frame);
void crsf_serialize_device_info(const crsf_payload_device_info_t *payload, crsf_frame_t *frame);
void crsf_serialize_parameter_settings_entry(const crsf_payload_parameter_entry_t *payload, crsf_frame_t *frame);
void crsf_serialize_elrs_status(const crsf_payload_elrs_status_t *payload, crsf_frame_t *frame);

#endif
