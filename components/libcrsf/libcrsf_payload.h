#ifndef LIBCRSF_PAYLOAD_H
#define LIBCRSF_PAYLOAD_H

#include <stdbool.h>

#include "libcrsf_def.h"
#include "libcrsf_device_param.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PACKED __attribute__((packed))

typedef enum : uint8_t {
    CRSF_PAYLOAD_LENGTH_DEVICE_PING = 4,
    CRSF_PAYLOAD_LENGTH_PARAM_READ = 6,
    CRSF_PAYLOAD_LENGTH_RADIO_ID = 13,
    CRSF_PAYLOAD_LENGTH_RC_CHANNELS = 24,
    CRSF_PAYLOAD_LENGTH_GPS = 17,
    CRSF_PAYLOAD_LENGTH_VARIO = 4,
    CRSF_PAYLOAD_LENGTH_AIRSPEED = 4,
    CRSF_PAYLOAD_LENGTH_BARO_ALTITUDE = 5,
} crsf_payload_length_t;

typedef struct {
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
    unsigned ch16 : 11;
} PACKED crsf_payload_rc_channels_t;

typedef struct {
    crsf_address_t source;
    crsf_address_t dest;
} crsf_payload_device_ping_t;

typedef struct {
    crsf_address_t source;
    crsf_address_t dest;
    const char *name;
    uint32_t serial;
    uint32_t hardware_version;
    uint32_t software_version;
    uint8_t param_count;
    uint8_t param_protocol_version;
} crsf_payload_device_info_t;

typedef struct {
    crsf_address_t source;
    crsf_address_t dest;
    uint8_t index;
    uint8_t chunk_index;
} crsf_payload_device_param_read_t;

typedef struct {
    crsf_address_t source;
    crsf_address_t dest;
    uint8_t index;
    crsf_device_param_type_t type;
    crsf_device_param_write_value_t value;
} crsf_payload_device_param_write_t;

typedef struct {
    crsf_address_t source;
    crsf_address_t dest;
    uint8_t index;
    uint8_t parent_index;
    crsf_device_param_type_t type;
    const char *name;
    crsf_device_param_read_value_t value;
} crsf_payload_device_param_entry_t;

typedef struct {
    crsf_address_t source;
    crsf_address_t dest;
    uint32_t interval_100ns;
    int32_t offset_100ns;
} crsf_payload_timing_correction_t;

typedef struct {
    int32_t latitude_10e7;
    int32_t longitude_10e7;
    uint16_t groundspeed_10kmh;
    uint16_t heading_cdeg;
    uint16_t altitude_m;
    uint8_t satellites;
} crsf_payload_gps_t;

typedef struct {
    uint16_t altitude_dm;
    int8_t vertical_speed_cm_s;
} crsf_payload_baro_altitude_t;

typedef struct {
    int16_t vspeed_cms;
} crsf_payload_vario_t;

typedef struct {
    uint16_t airspeed_100ms;
} crsf_payload_airspeed_t;

typedef enum : uint8_t {
    CRSF_PAYLOAD_ARDUPILOT_SUBTYPE_SINGLE = 0xF0,
    CRSF_PAYLOAD_ARDUPILOT_SUBTYPE_MULTI = 0xF2,
    CRSF_PAYLOAD_ARDUPILOT_SUBTYPE_TEXT = 0xF1,
} crsf_payload_ardupilot_subtype_t;

typedef struct {
    uint16_t appid;
    uint32_t data;
} crsf_payload_ardupilot_data_t;

typedef struct {
    crsf_payload_ardupilot_subtype_t subtype;
    union {
        crsf_payload_ardupilot_data_t single;
        struct {
            uint8_t size;
            crsf_payload_ardupilot_data_t values[9];
        } multi;
        struct {
            uint8_t severity;
            char text[50];
        } text;
    }; 
} crsf_payload_ardupilot_t;

bool crsf_payload__rc_channels_unpack(const crsf_frame_t *frame, crsf_payload_rc_channels_t *payload);
void crsf_payload__rc_channels_pack(crsf_frame_t *frame, const crsf_payload_rc_channels_t *payload);
uint16_t crsf_payload__rc_channels_get(const crsf_payload_rc_channels_t *payload, uint8_t channel);
void crsf_payload__rc_channels_set(crsf_payload_rc_channels_t *payload, uint8_t channel, uint16_t value);

bool crsf_payload_unpack__param_read(const crsf_frame_t *frame, crsf_payload_device_param_read_t *payload);
// param write is unpacked in two passes, first extract param index
// then look up the param type using the index
// and finally unpack the param value knowing the type
bool crsf_payload_unpack__param_write_header(const crsf_frame_t *frame, crsf_payload_device_param_write_t *payload);
bool crsf_payload_unpack__param_write_value(const crsf_frame_t *frame, crsf_payload_device_param_write_t *payload, crsf_device_param_type_t type);
bool crsf_payload_unpack__timing_correction(const crsf_frame_t *frame, crsf_payload_timing_correction_t *payload);
bool crsf_payload_unpack__gps(const crsf_frame_t *frame, crsf_payload_gps_t *payload);
bool crsf_payload_unpack__baro_altitude(const crsf_frame_t *frame, crsf_payload_baro_altitude_t *payload);
bool crsf_payload_unpack__vario(const crsf_frame_t *frame, crsf_payload_vario_t *payload);
bool crsf_payload_unpack__airspeed(const crsf_frame_t *frame, crsf_payload_airspeed_t *payload);
bool crsf_payload_unpack__ardupilot(const crsf_frame_t *frame, crsf_payload_ardupilot_t *payload);

// Pack payload structure into CRSF frame
void crsf_payload_pack__device_info(crsf_frame_t *frame, const crsf_payload_device_info_t *payload);
void crsf_payload_pack__param_entry(crsf_frame_t *frame, const crsf_payload_device_param_entry_t *payload);
void crsf_payload_pack__ardupilot(crsf_frame_t *frame, const crsf_payload_ardupilot_t *payload);

// Modify CRSF frame in place
bool crsf_payload_modify__timing_correction(crsf_frame_t *frame, uint32_t interval_100ns, int32_t offset_100ns);

#ifdef __cplusplus
}
#endif

#endif
