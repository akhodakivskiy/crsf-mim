#ifndef LIBCRSF_PAYLOAD_H
#define LIBCRSF_PAYLOAD_H

#include <stdbool.h>

#include "libcrsf_def.h"
#include "libcrsf_device_param.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CRSF_PAYLOAD_RC_CHANNELS_CENTER 992
#define CRSF_PAYLOAD_RC_CHANNELS_MIN 172
#define CRSF_PAYLOAD_RC_CHANNELS_MAX 1811
#define CRSF_PAYLOAD_RC_CHANNELS_TICKS_TO_US(x) ((x - 992) * 5 / 8 + 1500)
#define CRSF_PAYLOAD_RC_CHANNELS_US_TO_TICKS(x) ((x - 1500) * 8 / 5 + 992)

typedef enum : uint8_t {
    CRSF_PAYLOAD_LENGTH_DEVICE_PING = 4,
    CRSF_PAYLOAD_LENGTH_PARAM_READ = 6,
    CRSF_PAYLOAD_LENGTH_RADIO_ID = 13,
    CRSF_PAYLOAD_LENGTH_RC_CHANNELS = 24,
    CRSF_PAYLOAD_LENGTH_GPS = 17,
    CRSF_PAYLOAD_LENGTH_VARIO = 4,
    CRSF_PAYLOAD_LENGTH_BARO_ALTITUDE = 5,
} crsf_payload_length_t;

typedef struct {
    int16_t channels[16];
} crsf_payload_rc_channels_t;

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
    int32_t latitude;
    int32_t longitude;
    uint16_t groundspeed_kmh;
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

// Unpack CRSF frame into payload structure
bool crsf_payload_unpack__rc_channels(const crsf_frame_t *frame, crsf_payload_rc_channels_t *payload);
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

// Pack payload structure into CRSF frame
void crsf_payload_pack__rc_channels(crsf_frame_t *frame, const crsf_payload_rc_channels_t *payload);
void crsf_payload_pack__device_info(crsf_frame_t *frame, const crsf_payload_device_info_t *payload);
void crsf_payload_pack__param_entry(crsf_frame_t *frame, const crsf_payload_device_param_entry_t *payload);

// Modify CRSF frame in place
bool crsf_payload_modify__timing_correction(crsf_frame_t *frame, uint32_t interval_100ns, int32_t offset_100ns);
bool crsf_payload_modify__rc_channels(crsf_frame_t *frame, uint8_t channel, int16_t value);

#ifdef __cplusplus
}
#endif

#endif
