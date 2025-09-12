#include "libcrsf_payload.h"

#include "libcrsf_crc8.h"
#include "libcrsf_def.h"
#include "libcrsf_device_param.h"

#include <esp_log.h>
#include <string.h>
#include <endian.h>
#include <math.h>

static const char *TAG = "CRSF_PAYLOAD";

#define UNPACK_CHANNEL(ch_idx, bit_pos) \
    do { \
        uint32_t __byte_idx = (bit_pos) >> 3; \
        uint32_t __shift = (bit_pos) & 7; \
        payload->channels[ch_idx] = ((*(uint32_t*)(data + __byte_idx)) >> __shift) & 0x07FF; \
    } while(0)

#define PACK_CHANNEL(ch_idx, v, bit_pos) \
    do { \
        uint32_t __byte_idx = (bit_pos) >> 3; \
        uint32_t __shift = (bit_pos) & 7; \
        uint32_t __ch_v = v & 0x07FF; \
        *(uint32_t*)(data + __byte_idx) |= (__ch_v << __shift); \
    } while(0)

bool IRAM_ATTR crsf_payload_unpack__rc_channels(const crsf_frame_t *frame, crsf_payload_rc_channels_t *payload) {
    if ((frame->type == CRSF_FRAME_TYPE_RC_CHANNELS_PACKED) &&
        (frame->length == CRSF_PAYLOAD_LENGTH_RC_CHANNELS)) {

        const uint8_t *data = frame->data;

        UNPACK_CHANNEL(0,  0);
        UNPACK_CHANNEL(1,  11);
        UNPACK_CHANNEL(2,  22);
        UNPACK_CHANNEL(3,  33);
        UNPACK_CHANNEL(4,  44);
        UNPACK_CHANNEL(5,  55);
        UNPACK_CHANNEL(6,  66);
        UNPACK_CHANNEL(7,  77);
        UNPACK_CHANNEL(8,  88);
        UNPACK_CHANNEL(9,  99);
        UNPACK_CHANNEL(10, 110);
        UNPACK_CHANNEL(11, 121);
        UNPACK_CHANNEL(12, 132);
        UNPACK_CHANNEL(13, 143);
        UNPACK_CHANNEL(14, 154);
        UNPACK_CHANNEL(15, 165);

        return true;
    }

    return false;
}

bool IRAM_ATTR crsf_payload_unpack__param_read(const crsf_frame_t *frame, crsf_payload_device_param_read_t *payload) {
    if ((frame->type == CRSF_FRAME_TYPE_PARAM_READ) &&
        (frame->length == CRSF_PAYLOAD_LENGTH_PARAM_READ) &&
        (frame->is_extended)) {

        payload->dest = frame->dest;
        payload->source = frame->source;
        payload->index = frame->data[2];
        payload->chunk_index = frame->data[3];

        return true;
    }

    return false;
}

bool IRAM_ATTR crsf_payload_unpack__param_write_header(const crsf_frame_t *frame, crsf_payload_device_param_write_t *payload) {
    if (frame->type == CRSF_FRAME_TYPE_PARAM_WRITE) {

        payload->dest = frame->data[0];
        payload->source = frame->data[1];
        payload->index = frame->data[2];
        payload->type = CRSF_PARAM_TYPE_OUT_OF_RANGE;

        return true;
    }

    return false;
}

bool IRAM_ATTR crsf_payload_unpack__param_write_value(const crsf_frame_t *frame, crsf_payload_device_param_write_t *payload, crsf_device_param_type_t type) {
    payload->type = type;

    const uint8_t *ptr = frame->data + 3;
    switch (type) {
        case CRSF_PARAM_TYPE_UINT8:
            payload->value.u8 = *(uint8_t *)ptr;
            break;
        case CRSF_PARAM_TYPE_INT8:
            payload->value.i8 = *(int8_t *)ptr;
            break;
        case CRSF_PARAM_TYPE_UINT16:
            payload->value.u16 = be16toh(*((uint16_t *)ptr));
            break;
        case CRSF_PARAM_TYPE_INT16:
            payload->value.i16 = (int16_t)be16toh(*((uint16_t *)ptr));
            break;
        case CRSF_PARAM_TYPE_UINT32:
            payload->value.u32 = be32toh(*((uint32_t *)ptr));
            break;
        case CRSF_PARAM_TYPE_INT32:
            payload->value.i32 = (int32_t)be32toh(*((uint32_t *)ptr));
            break;
        case CRSF_PARAM_TYPE_FLOAT:
            payload->value.flt = (int32_t)be32toh(*((uint32_t *)ptr));
            break;
        case CRSF_PARAM_TYPE_SELECT:
            payload->value.select_index = *(uint8_t *)ptr;
            break;
        case CRSF_PARAM_TYPE_COMMAND:
            payload->value.command_action = (crsf_device_command_state_t)(*(uint8_t *)ptr);
            break;
        case CRSF_PARAM_TYPE_FOLDER:
            break;
        case CRSF_PARAM_TYPE_INFO:
            ESP_LOGE(TAG, "FOLDER and INFO are not editable");
            return false;
        case CRSF_PARAM_TYPE_STRING:
            ESP_LOGE(TAG, "expresslrs doesn't support editing strings");
            return false;
        case CRSF_PARAM_TYPE_VTX:
            ESP_LOGE(TAG, "VTX control not implemented");
            return false;
        case CRSF_PARAM_TYPE_UINT64:
        case CRSF_PARAM_TYPE_INT64:
        default:
            ESP_LOGE(TAG, "type=%u, expresslrs doesn't support 32 and 64 bit numbers", payload->type);
            return false;
    }

    return true;
}

bool IRAM_ATTR crsf_payload_unpack__timing_correction(const crsf_frame_t *frame, crsf_payload_timing_correction_t *payload) {
    if (frame->type == CRSF_FRAME_TYPE_RADIO_ID) {
        uint8_t subtype = frame->data[2];
        if ((subtype == CRSF_FRAME_SUBTYPE_TIMING_CORRECTION) && 
            frame->length == CRSF_PAYLOAD_LENGTH_RADIO_ID) {

            payload->dest = frame->data[0];
            payload->source = frame->data[1];
            payload->interval_100ns = be32toh(*(uint32_t *)(frame->data + 3));
            payload->offset_100ns = (int32_t)be32toh(*(uint32_t *)(frame->data + 7));

            return true;
        }
    }

    return false;
}

bool IRAM_ATTR crsf_payload_unpack__gps(const crsf_frame_t *frame, crsf_payload_gps_t *payload) {
    if (frame->type == CRSF_FRAME_TYPE_GPS && frame->length == CRSF_PAYLOAD_LENGTH_GPS) {
        payload->latitude = (int32_t)be32toh(*(int32_t *)(frame->data + 0));
        payload->longitude = (int32_t)be32toh(*(int32_t *)(frame->data + 4));
        payload->groundspeed_kmh = be16toh(*(uint16_t *)(frame->data + 8));
        payload->heading_cdeg = be16toh(*(uint16_t *)(frame->data + 10));
        payload->altitude_m = be16toh(*(uint16_t *)(frame->data + 12)) - 1000;
        payload->satellites = *(uint8_t *)(frame->data + 14);

        return true;
    }
    return false;
}

const float _CRSF_ALT_BARO_K_R = .026;
const int _CRSF_ALT_BARO_K_L = 100;

bool IRAM_ATTR crsf_payload_unpack__baro_altitude(const crsf_frame_t *frame, crsf_payload_baro_altitude_t *payload) {
    if (frame->type == CRSF_FRAME_TYPE_BARO_ALTITUDE && frame->length == CRSF_PAYLOAD_LENGTH_BARO_ALTITUDE) {
        ESP_LOG_BUFFER_HEX(TAG, frame->data, 2);
        uint8_t alt_packed_0 = frame->data[0];
        uint8_t alt_packed_1 = frame->data[1];
        payload->altitude_dm = (alt_packed_0 & 0x80) ? 
            ((((alt_packed_0 & 0x7f) << 8) + alt_packed_1) * 10) : 
            ((alt_packed_0 << 8) + alt_packed_1 - 10000);

        int8_t vspd_packed = *(int8_t *)(frame->data + 2);
        payload->vertical_speed_cm_s = exp(fabs(vspd_packed * _CRSF_ALT_BARO_K_R) - 1) * 
            _CRSF_ALT_BARO_K_L * (vspd_packed > 0 ? 1 : -1);

        return true;
    }
    return false;
}

bool IRAM_ATTR crsf_payload_unpack__vario(const crsf_frame_t *frame, crsf_payload_vario_t *payload) {
    if (frame->type == CRSF_FRAME_TYPE_VARIO && frame->length == CRSF_PAYLOAD_LENGTH_VARIO) {
        payload->vspeed_cms = (int16_t)be16toh(*(uint16_t *)frame->data);
        return true;
    }

    return false;
}

void IRAM_ATTR crsf_payload_pack__rc_channels(crsf_frame_t *frame, const crsf_payload_rc_channels_t *payload) {

    frame->type = CRSF_FRAME_TYPE_RC_CHANNELS_PACKED;
    frame->length = CRSF_PAYLOAD_LENGTH_RC_CHANNELS;

    uint8_t *data = frame->data;
    memset(data, 0, 22);

    PACK_CHANNEL(0, payload->channels[0], 0);
    PACK_CHANNEL(1, payload->channels[1], 11);
    PACK_CHANNEL(2, payload->channels[2], 22);
    PACK_CHANNEL(3, payload->channels[3],  33);
    PACK_CHANNEL(4, payload->channels[4],  44);
    PACK_CHANNEL(5, payload->channels[5],  55);
    PACK_CHANNEL(6, payload->channels[6],  66);
    PACK_CHANNEL(7, payload->channels[7],  77);
    PACK_CHANNEL(8, payload->channels[8],  88);
    PACK_CHANNEL(9, payload->channels[9],  99);
    PACK_CHANNEL(10, payload->channels[10], 110);
    PACK_CHANNEL(11, payload->channels[11], 121);
    PACK_CHANNEL(12, payload->channels[12], 132);
    PACK_CHANNEL(13, payload->channels[13], 143);
    PACK_CHANNEL(14, payload->channels[14], 154);
    PACK_CHANNEL(15, payload->channels[15], 165);

    frame->data[frame->length - 2] = crsf_calc_crc8(frame);
}

void IRAM_ATTR crsf_payload_pack__device_info(crsf_frame_t *frame, const crsf_payload_device_info_t *payload) {
    frame->sync = CRSF_SYNC_BYTE;
    frame->type = CRSF_FRAME_TYPE_DEVICE_INFO;

    uint8_t i = 0;
    frame->is_extended = true;
    frame->dest = (frame->data[i++] = payload->dest);
    frame->source = (frame->data[i++] = payload->source);

    size_t len = strlen(payload->name);
    memcpy(frame->data + i, payload->name, len + 1); i += len + 1;

    memcpy(frame->data + i, &payload->serial, 4); i += 4;
    memcpy(frame->data + i, &payload->hardware_version, 4); i += 4;
    memcpy(frame->data + i, &payload->software_version, 4); i += 4;
    frame->data[i++] = payload->param_count;
    frame->data[i++] = payload->param_protocol_version;

    frame->length = i + 2; // payload + type + crc
    frame->data[frame->length - 2] = crsf_calc_crc8(frame);
}

/* begin helper macro definitions */
#define _CRSF_PAYLOAD_WRITE_ENTRY_NUMBER(BUFFER, OFFSET, VALUE, SIZE) { \
    if (SIZE == 1) { \
        uint8_t value8_be = VALUE; \
        memcpy(BUFFER + OFFSET, &value8_be, SIZE); OFFSET += SIZE; \
    } else if (SIZE == 2) { \
        uint16_t value16_be = htobe16(VALUE); \
        memcpy(BUFFER + OFFSET, &value16_be, SIZE); OFFSET += SIZE; \
    } else if (SIZE == 4) { \
        uint32_t value32_be = htobe32(VALUE); \
        memcpy(BUFFER + OFFSET, &value32_be, SIZE); OFFSET += SIZE; \
    } else { \
        assert(false); \
    } \
} \

#define _CRSF_PAYLOAD_WRITE_ENTRY_VALUE(BUFFER, OFFSET, PAYLOAD, TYPE, SIZE) \
_CRSF_PAYLOAD_WRITE_ENTRY_NUMBER(BUFFER, OFFSET, (PAYLOAD -> value . TYPE . value), SIZE); \
_CRSF_PAYLOAD_WRITE_ENTRY_NUMBER(BUFFER, OFFSET, (PAYLOAD -> value . TYPE . value_min ), SIZE); \
_CRSF_PAYLOAD_WRITE_ENTRY_NUMBER(BUFFER, OFFSET, (PAYLOAD -> value . TYPE . value_max ), SIZE); \
/* last value is the 'default' value */ \
_CRSF_PAYLOAD_WRITE_ENTRY_NUMBER(BUFFER, OFFSET, (PAYLOAD -> value . TYPE . value), SIZE); \

#define _CRSF_PAYLOAD_WRITE_STRING(BUFFER, OFFSET, VALUE) { \
    size_t units_len = strlen(VALUE); \
    memcpy(BUFFER + OFFSET, VALUE, units_len + 1); OFFSET += units_len + 1; \
}

#define _CRSF_PAYLOAD_WRITE_UNITS(BUFFER, OFFSET, PAYLOAD, TYPE) \
_CRSF_PAYLOAD_WRITE_STRING(BUFFER, OFFSET, PAYLOAD -> value . TYPE . units) \
/* end helper macro definitions */

void IRAM_ATTR crsf_payload_pack__param_entry(crsf_frame_t *frame, 
                                    const crsf_payload_device_param_entry_t *payload) {

    frame->sync = CRSF_SYNC_BYTE;
    frame->type = CRSF_FRAME_TYPE_PARAM_ENTRY;

    uint8_t i = 0;
    frame->data[i++] = payload->dest;
    frame->data[i++] = payload->source;
    frame->data[i++] = payload->index;
    frame->data[i++] = 0; // chunks remaining, TODO: implement chunked replies
    frame->data[i++] = payload->parent_index;
    frame->data[i++] = payload->type;
    _CRSF_PAYLOAD_WRITE_STRING(frame->data, i, payload->name);

    switch (payload->type) {
        case CRSF_PARAM_TYPE_UINT8:
            _CRSF_PAYLOAD_WRITE_ENTRY_VALUE(frame->data, i, payload, u8, 1);
            _CRSF_PAYLOAD_WRITE_UNITS(frame->data, i, payload, u8);
            break;
        case CRSF_PARAM_TYPE_INT8:
            _CRSF_PAYLOAD_WRITE_ENTRY_VALUE(frame->data, i, payload, i8, 1);
            _CRSF_PAYLOAD_WRITE_UNITS(frame->data, i, payload, i8);
            break;
        case CRSF_PARAM_TYPE_UINT16:
            _CRSF_PAYLOAD_WRITE_ENTRY_VALUE(frame->data, i, payload, u16, 2);
            _CRSF_PAYLOAD_WRITE_UNITS(frame->data, i, payload, u16);
            break;
        case CRSF_PARAM_TYPE_INT16:
            _CRSF_PAYLOAD_WRITE_ENTRY_VALUE(frame->data, i, payload, i16, 2);
            _CRSF_PAYLOAD_WRITE_UNITS(frame->data, i, payload, i16);
            break;
        case CRSF_PARAM_TYPE_UINT32:
            _CRSF_PAYLOAD_WRITE_ENTRY_VALUE(frame->data, i, payload, u32, 4);
            _CRSF_PAYLOAD_WRITE_UNITS(frame->data, i, payload, u32);
            break;
        case CRSF_PARAM_TYPE_INT32:
            _CRSF_PAYLOAD_WRITE_ENTRY_VALUE(frame->data, i, payload, i32, 4);
            _CRSF_PAYLOAD_WRITE_UNITS(frame->data, i, payload, i32);
            break;
        case CRSF_PARAM_TYPE_FLOAT:
            _CRSF_PAYLOAD_WRITE_ENTRY_NUMBER(frame->data, i, payload->value.flt.value, 4);
            _CRSF_PAYLOAD_WRITE_ENTRY_NUMBER(frame->data, i, payload->value.flt.value_min, 4);
            _CRSF_PAYLOAD_WRITE_ENTRY_NUMBER(frame->data, i, payload->value.flt.value_max, 4);
            _CRSF_PAYLOAD_WRITE_ENTRY_NUMBER(frame->data, i, payload->value.flt.value, 4); // default value 
            _CRSF_PAYLOAD_WRITE_ENTRY_NUMBER(frame->data, i, payload->value.flt.decimal_places, 1);
            _CRSF_PAYLOAD_WRITE_ENTRY_NUMBER(frame->data, i, payload->value.flt.step, 4);
            _CRSF_PAYLOAD_WRITE_UNITS(frame->data, i, payload, flt);
            break;
        case CRSF_PARAM_TYPE_SELECT:
            for (int o = 0; o < payload->value.select.option_count; o++) {
                _CRSF_PAYLOAD_WRITE_STRING(frame->data, i, payload->value.select.options[o]);
                if (o != payload->value.select.option_count - 1) {
                    frame->data[i - 1] = ';';
                }
            }
            _CRSF_PAYLOAD_WRITE_ENTRY_NUMBER(frame->data, i, payload->value.select.index, 1);
            _CRSF_PAYLOAD_WRITE_ENTRY_NUMBER(frame->data, i, 0, 1);
            _CRSF_PAYLOAD_WRITE_ENTRY_NUMBER(frame->data, i, 0, 1);
            _CRSF_PAYLOAD_WRITE_ENTRY_NUMBER(frame->data, i, 0, 1); // default value 
            _CRSF_PAYLOAD_WRITE_UNITS(frame->data, i, payload, select);
            break;
        case CRSF_PARAM_TYPE_COMMAND:
            _CRSF_PAYLOAD_WRITE_ENTRY_NUMBER(frame->data, i, payload->value.command.state, 1);
            _CRSF_PAYLOAD_WRITE_ENTRY_NUMBER(frame->data, i, payload->value.command.timeout, 1);
            _CRSF_PAYLOAD_WRITE_STRING(frame->data, i, payload->value.command.status);
            break;
        case CRSF_PARAM_TYPE_FOLDER:
            _CRSF_PAYLOAD_WRITE_STRING(frame->data, i, payload->value.folder);
            break;
        case CRSF_PARAM_TYPE_INFO:
            _CRSF_PAYLOAD_WRITE_STRING(frame->data, i, payload->value.info);
            break;
        case CRSF_PARAM_TYPE_STRING:
            ESP_LOGE(TAG, "expresslrs doesn't support editing strings");
            assert(false);
        case CRSF_PARAM_TYPE_VTX:
            ESP_LOGE(TAG, "VTX control not implemented");
            assert(false);
        case CRSF_PARAM_TYPE_UINT64:
        case CRSF_PARAM_TYPE_INT64:
        default:
            ESP_LOGE(TAG, "expresslrs doesn't support 32 and 64 bit numbers");
            assert(false);
    }

    frame->length = i + 2; // payload + type + crc
    frame->data[frame->length - 2] = crsf_calc_crc8(frame);
}

bool IRAM_ATTR crsf_payload_modify__timing_correction(crsf_frame_t *frame, uint32_t interval_100ns, int32_t offset_100ns) {
    if ((frame->type == CRSF_FRAME_TYPE_RADIO_ID) &&
        (frame->data[2] == CRSF_FRAME_SUBTYPE_TIMING_CORRECTION) &&
        (frame->length == CRSF_PAYLOAD_LENGTH_RADIO_ID)) {
        *(uint32_t *)(frame->data + 3) = htobe32(interval_100ns);
        *(uint32_t *)(frame->data + 7) = htobe32((uint32_t)offset_100ns);

        frame->data[frame->length - 2] = crsf_calc_crc8(frame);

        return true;
    }
    return false;
}

bool IRAM_ATTR crsf_payload_modify__rc_channels(crsf_frame_t *frame, uint8_t channel, int16_t value) {
    if ((frame->type == CRSF_FRAME_TYPE_RC_CHANNELS_PACKED) &&
        (frame->length == CRSF_PAYLOAD_LENGTH_RC_CHANNELS)) {

        uint8_t *data = frame->data;
        PACK_CHANNEL(channel - 1, value, channel * 11);

        return true;
    }

    return false;
}
