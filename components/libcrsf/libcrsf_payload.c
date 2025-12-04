#include "libcrsf_payload.h"

#include "libcrsf_crc8.h"
#include "libcrsf_def.h"
#include "libcrsf_device_param.h"

#include <esp_log.h>
#include <string.h>
#include <endian.h>
#include <math.h>

static const char *TAG = "CRSF_PAYLOAD";

bool IRAM_ATTR crsf_payload__rc_channels_unpack(const crsf_frame_t *frame, crsf_payload_rc_channels_t *payload) {
    if ((frame->type != CRSF_FRAME_TYPE_RC_CHANNELS_PACKED) &&
        (frame->length != CRSF_PAYLOAD_LENGTH_RC_CHANNELS)) {
        return false;
    }

    memcpy(payload, frame->data, sizeof(crsf_payload_rc_channels_t));

    return true;
}

void IRAM_ATTR crsf_payload__rc_channels_pack(crsf_frame_t *frame, const crsf_payload_rc_channels_t *payload) {
    frame->is_extended = false;
    frame->type = CRSF_FRAME_TYPE_RC_CHANNELS_PACKED;
    frame->length = CRSF_PAYLOAD_LENGTH_RC_CHANNELS;

    memcpy(frame->data, payload, sizeof(crsf_payload_rc_channels_t));

    frame->data[frame->length - 2] = crsf_calc_crc8(frame);
}

uint16_t IRAM_ATTR crsf_payload__rc_channels_get(const crsf_payload_rc_channels_t *payload, uint8_t channel) {
    switch (channel) {
        case 0: return payload->ch1;
        case 1: return payload->ch2;
        case 2: return payload->ch3;
        case 3: return payload->ch4;
        case 4: return payload->ch5;
        case 5: return payload->ch6;
        case 6: return payload->ch7;
        case 7: return payload->ch8;
        case 8: return payload->ch9;
        case 9: return payload->ch10;
        case 10: return payload->ch11;
        case 11: return payload->ch12;
        case 12: return payload->ch13;
        case 13: return payload->ch14;
        case 14: return payload->ch15;
        case 15: return payload->ch16;
        default: assert(false);
    }
}
void IRAM_ATTR crsf_payload__rc_channels_set(crsf_payload_rc_channels_t *payload, uint8_t channel, uint16_t value) {
    switch (channel) {
        case 0: payload->ch1 = value; return;
        case 1: payload->ch2 = value; return;
        case 2: payload->ch3 = value; return;
        case 3: payload->ch4 = value; return;
        case 4: payload->ch5 = value; return;
        case 5: payload->ch6 = value; return;
        case 6: payload->ch7 = value; return;
        case 7: payload->ch8 = value; return;
        case 8: payload->ch9 = value; return;
        case 9: payload->ch10 = value; return;
        case 10: payload->ch11 = value; return;
        case 11: payload->ch12 = value; return;
        case 12: payload->ch13 = value; return;
        case 13: payload->ch14 = value; return;
        case 14: payload->ch15 = value; return;
        case 15: payload->ch16 = value; return;
        default: assert(false);
    }
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
        payload->latitude_10e7 = (int32_t)be32toh(*(int32_t *)(frame->data + 0));
        payload->longitude_10e7 = (int32_t)be32toh(*(int32_t *)(frame->data + 4));
        payload->groundspeed_10kmh = be16toh(*(uint16_t *)(frame->data + 8));
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
        uint8_t alt_packed_0 = frame->data[0];
        uint8_t alt_packed_1 = frame->data[1];
        payload->altitude_dm = (alt_packed_0 & 0x80) ? 
            ((((alt_packed_0 & 0x7f) << 8) + alt_packed_1) * 10) : 
            ((alt_packed_0 << 8) + alt_packed_1 - 10000);

        int8_t vspd_packed = *(int8_t *)(frame->data + 2);
        payload->vertical_speed_cm_s = expf(fabs(vspd_packed * _CRSF_ALT_BARO_K_R) - 1) * 
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

bool IRAM_ATTR crsf_payload_unpack__airspeed(const crsf_frame_t *frame, crsf_payload_airspeed_t *payload) {
    if (frame->type == CRSF_FRAME_TYPE_AIRSPEED && frame->length == CRSF_PAYLOAD_LENGTH_AIRSPEED) {
        payload->airspeed_100ms = be16toh(*(uint16_t *)frame->data);
        return true;
    }

    return false;
}

bool IRAM_ATTR crsf_payload_unpack__ardupilot(const crsf_frame_t *frame, crsf_payload_ardupilot_t *payload) {
    if (frame->type == CRSF_FRAME_TYPE_ARDUPILOT || frame->type == CRSF_FRAME_TYPE_ARDUPILOT_LEGACY) {
        uint8_t cursor = 0;
        payload->subtype = frame->data[(cursor++)];
        switch (payload->subtype) {
            case CRSF_PAYLOAD_ARDUPILOT_SUBTYPE_SINGLE:
                payload->single.appid = *(uint16_t *)(frame->data + cursor); cursor += 2;
                payload->single.data = *(uint32_t *)(frame->data + cursor); cursor += 4;
                break;
            case CRSF_PAYLOAD_ARDUPILOT_SUBTYPE_MULTI:
                payload->multi.size = *(uint8_t *)(frame->data + (cursor++));
                assert(payload->multi.size < 10);
                for (int i = 0; i < payload->multi.size; i++) {
                    payload->multi.values[i].appid = *(uint16_t *)(frame->data + cursor); cursor += 2;
                    payload->multi.values[i].data = *(uint32_t *)(frame->data + cursor); cursor += 4;
                }
                break;
            case CRSF_PAYLOAD_ARDUPILOT_SUBTYPE_TEXT:
                payload->text.severity = *(uint8_t *)(frame->data + (cursor++));
                strcpy((char *)payload->text.text, (char *)frame->data + cursor);
                break;
            default:
                ESP_LOGW(TAG, "unexpected ArduPilot frame subtype: %u", payload->subtype);
                return false;
        }
        return true;
    }

    return false;
}

void IRAM_ATTR crsf_payload_pack__device_info(crsf_frame_t *frame, const crsf_payload_device_info_t *payload) {
    frame->sync = CRSF_SYNC_BYTE;
    frame->type = CRSF_FRAME_TYPE_DEVICE_INFO;
    frame->is_extended = true;

    uint8_t i = 0;
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
#define _CRSF_PAYLOAD_WRITE_BYTE(BUFFER, OFFSET, VALUE) { \
    uint8_t value8_be = VALUE; \
    memcpy(BUFFER + OFFSET, &value8_be, 1); OFFSET += 1; \
} \

#define _CRSF_PAYLOAD_WRITE_WORD_BE(BUFFER, OFFSET, VALUE) { \
    uint16_t value16 = htobe16(VALUE); \
    memcpy(BUFFER + OFFSET, &value16, 2); OFFSET += 2; \
} \

#define _CRSF_PAYLOAD_WRITE_DOUBLE_WORD_BE(BUFFER, OFFSET, VALUE) { \
    uint32_t value32 = htobe32(VALUE); \
    memcpy(BUFFER + OFFSET, &value32, 4); OFFSET += 4; \
} \

#define _CRSF_PAYLOAD_WRITE_WORD_LE(BUFFER, OFFSET, VALUE) { \
    uint16_t value16 = htole16(VALUE); \
    memcpy(BUFFER + OFFSET, &value16, 2); OFFSET += 2; \
} \

#define _CRSF_PAYLOAD_WRITE_DOUBLE_WORD_LE(BUFFER, OFFSET, VALUE) { \
    uint32_t value32 = htole32(VALUE); \
    memcpy(BUFFER + OFFSET, &value32, 4); OFFSET += 4; \
} \

#define _CRSF_PAYLOAD_WRITE_ENTRY_BYTE(BUFFER, OFFSET, PAYLOAD, TYPE) \
_CRSF_PAYLOAD_WRITE_BYTE(BUFFER, OFFSET, (PAYLOAD -> value . TYPE . value)); \
_CRSF_PAYLOAD_WRITE_BYTE(BUFFER, OFFSET, (PAYLOAD -> value . TYPE . value_min )); \
_CRSF_PAYLOAD_WRITE_BYTE(BUFFER, OFFSET, (PAYLOAD -> value . TYPE . value_max )); \
_CRSF_PAYLOAD_WRITE_BYTE(BUFFER, OFFSET, (PAYLOAD -> value . TYPE . value)); \

#define _CRSF_PAYLOAD_WRITE_ENTRY_WORD_BE(BUFFER, OFFSET, PAYLOAD, TYPE) \
_CRSF_PAYLOAD_WRITE_WORD_BE(BUFFER, OFFSET, (PAYLOAD -> value . TYPE . value)); \
_CRSF_PAYLOAD_WRITE_WORD_BE(BUFFER, OFFSET, (PAYLOAD -> value . TYPE . value_min )); \
_CRSF_PAYLOAD_WRITE_WORD_BE(BUFFER, OFFSET, (PAYLOAD -> value . TYPE . value_max )); \
_CRSF_PAYLOAD_WRITE_WORD_BE(BUFFER, OFFSET, (PAYLOAD -> value . TYPE . value)); \

#define _CRSF_PAYLOAD_WRITE_ENTRY_DOUBLE_WORD_BE(BUFFER, OFFSET, PAYLOAD, TYPE) \
_CRSF_PAYLOAD_WRITE_DOUBLE_WORD_BE(BUFFER, OFFSET, (PAYLOAD -> value . TYPE . value)); \
_CRSF_PAYLOAD_WRITE_DOUBLE_WORD_BE(BUFFER, OFFSET, (PAYLOAD -> value . TYPE . value_min )); \
_CRSF_PAYLOAD_WRITE_DOUBLE_WORD_BE(BUFFER, OFFSET, (PAYLOAD -> value . TYPE . value_max )); \
_CRSF_PAYLOAD_WRITE_DOUBLE_WORD_BE(BUFFER, OFFSET, (PAYLOAD -> value . TYPE . value)); \

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
    frame->is_extended = true;

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
            _CRSF_PAYLOAD_WRITE_ENTRY_BYTE(frame->data, i, payload, u8);
            _CRSF_PAYLOAD_WRITE_UNITS(frame->data, i, payload, u8);
            break;
        case CRSF_PARAM_TYPE_INT8:
            _CRSF_PAYLOAD_WRITE_ENTRY_BYTE(frame->data, i, payload, i8);
            _CRSF_PAYLOAD_WRITE_UNITS(frame->data, i, payload, i8);
            break;
        case CRSF_PARAM_TYPE_UINT16:
            _CRSF_PAYLOAD_WRITE_ENTRY_WORD_BE(frame->data, i, payload, u16);
            _CRSF_PAYLOAD_WRITE_UNITS(frame->data, i, payload, u16);
            break;
        case CRSF_PARAM_TYPE_INT16:
            _CRSF_PAYLOAD_WRITE_ENTRY_WORD_BE(frame->data, i, payload, i16);
            _CRSF_PAYLOAD_WRITE_UNITS(frame->data, i, payload, i16);
            break;
        case CRSF_PARAM_TYPE_UINT32:
            _CRSF_PAYLOAD_WRITE_ENTRY_DOUBLE_WORD_BE(frame->data, i, payload, u32);
            _CRSF_PAYLOAD_WRITE_UNITS(frame->data, i, payload, u32);
            break;
        case CRSF_PARAM_TYPE_INT32:
            _CRSF_PAYLOAD_WRITE_ENTRY_DOUBLE_WORD_BE(frame->data, i, payload, i32);
            _CRSF_PAYLOAD_WRITE_UNITS(frame->data, i, payload, i32);
            break;
        case CRSF_PARAM_TYPE_FLOAT:
            _CRSF_PAYLOAD_WRITE_DOUBLE_WORD_BE(frame->data, i, payload->value.flt.value);
            _CRSF_PAYLOAD_WRITE_DOUBLE_WORD_BE(frame->data, i, payload->value.flt.value_min);
            _CRSF_PAYLOAD_WRITE_DOUBLE_WORD_BE(frame->data, i, payload->value.flt.value_max);
            _CRSF_PAYLOAD_WRITE_DOUBLE_WORD_BE(frame->data, i, payload->value.flt.value); // default value 
            _CRSF_PAYLOAD_WRITE_BYTE(frame->data, i, payload->value.flt.decimal_places);
            _CRSF_PAYLOAD_WRITE_DOUBLE_WORD_BE(frame->data, i, payload->value.flt.step);
            _CRSF_PAYLOAD_WRITE_UNITS(frame->data, i, payload, flt);
            break;
        case CRSF_PARAM_TYPE_SELECT:
            for (int o = 0; o < payload->value.select.option_count; o++) {
                _CRSF_PAYLOAD_WRITE_STRING(frame->data, i, payload->value.select.options[o]);
                if (o != payload->value.select.option_count - 1) {
                    frame->data[i - 1] = ';';
                }
            }
            _CRSF_PAYLOAD_WRITE_BYTE(frame->data, i, payload->value.select.index);
            _CRSF_PAYLOAD_WRITE_BYTE(frame->data, i, 0);
            _CRSF_PAYLOAD_WRITE_BYTE(frame->data, i, 0);
            _CRSF_PAYLOAD_WRITE_BYTE(frame->data, i, 0); // default value 
            _CRSF_PAYLOAD_WRITE_UNITS(frame->data, i, payload, select);
            break;
        case CRSF_PARAM_TYPE_COMMAND:
            _CRSF_PAYLOAD_WRITE_BYTE(frame->data, i, payload->value.command.state);
            _CRSF_PAYLOAD_WRITE_BYTE(frame->data, i, payload->value.command.timeout);
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

void IRAM_ATTR crsf_payload_pack__ardupilot(crsf_frame_t *frame, const crsf_payload_ardupilot_t *payload) {
    uint8_t i = 0;

    frame->sync = CRSF_SYNC_BYTE;
    frame->type = CRSF_FRAME_TYPE_ARDUPILOT;

    _CRSF_PAYLOAD_WRITE_BYTE(frame->data, i, payload->subtype);
    switch (payload->subtype) {
        case CRSF_PAYLOAD_ARDUPILOT_SUBTYPE_SINGLE:
            _CRSF_PAYLOAD_WRITE_WORD_LE(frame->data, i, payload->single.appid);
            _CRSF_PAYLOAD_WRITE_DOUBLE_WORD_LE(frame->data, i, payload->single.data);
            break;
        case CRSF_PAYLOAD_ARDUPILOT_SUBTYPE_MULTI:
            _CRSF_PAYLOAD_WRITE_BYTE(frame->data, i, payload->multi.size);
            for (int j = 0; j < 9; j++) {
                // we have to write all 9 values, otherwire some parsers will fail
                if (j < payload->multi.size) {
                    _CRSF_PAYLOAD_WRITE_WORD_LE(frame->data, i, payload->multi.values[j].appid);
                    _CRSF_PAYLOAD_WRITE_DOUBLE_WORD_LE(frame->data, i, payload->multi.values[j].data);
                } else {
                    _CRSF_PAYLOAD_WRITE_WORD_LE(frame->data, i, 0);
                    _CRSF_PAYLOAD_WRITE_DOUBLE_WORD_LE(frame->data, i, 0);
                }
            }
            break;
        case CRSF_PAYLOAD_ARDUPILOT_SUBTYPE_TEXT:
            _CRSF_PAYLOAD_WRITE_BYTE(frame->data, i, payload->text.severity);
            _CRSF_PAYLOAD_WRITE_STRING(frame->data, i, payload->text.text);
            break;
        default:
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
