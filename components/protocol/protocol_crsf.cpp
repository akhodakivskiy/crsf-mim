#include "protocol_crsf.h"

#include <esp_attr.h>
#include <assert.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <string.h>
#include <stdio.h>

static const char *TAG = "CRSF_PROTOCOL";


Crc8 _crc(CRSF_CRC_POLY);

inline uint8_t _crsf_calculate_crc(crsf_frame_t *frame) {
    return _crc.calc(reinterpret_cast<uint8_t *>(frame) + 2, frame->length - 1, 0);
}

inline void _crsf_append_crc_byte(crsf_frame_t *frame) {
    frame->data[frame->length - 2] = _crsf_calculate_crc(frame);
}

bool IRAM_ATTR crsf_validate_frame(crsf_frame_t *f) {
    return f->data[f->length - 2] == _crsf_calculate_crc(f);
}

void IRAM_ATTR crsf_frame_partial_parse(crsf_parse_ctx_t *p, uint8_t b) {
    if (p->frame->sync == 0) {
        p->frame->sync = b;
        p->frame->length = 0;
        p->frame->type = CRSF_FRAME_TYPE_NONE;
        p->cursor = 0;
        p->status = CRSF_FRAME_STATUS_SYNC;
    } else if (p->frame->length == 0) {
        if (b >= 2 && b <= CRSF_MAX_PAYLOAD_LEN + 2) { 
            // length of type + payload + crc, at least 2 bytes
            p->frame->length = b;
            p->status = CRSF_FRAME_STATUS_LENGTH;
        } else {
            p->frame->sync = 0; // bad frame
            p->status = CRSF_FRAME_STATUS_ERROR_LENGTH;
        }
    } else if (p->frame->type == CRSF_FRAME_TYPE_NONE) {
        p->frame->type = (crsf_frame_type_t)b;
        p->status = CRSF_FRAME_STATUS_TYPE;
    } else if (p->cursor < p->frame->length - 1) { // -1 for type
        p->frame->data[p->cursor++] = b;
        p->status = CRSF_FRAME_STATUS_PAYLOAD;
        if (p->cursor == p->frame->length - 1) {
            p->status = CRSF_FRAME_STATUS_FRAME_READY;
        }
    } else {
        // unexpected sync error
        p->status = CRSF_FRAME_STATUS_ERROR_SYNC;
        p->frame->sync = 0;
    }

    if (p->status == CRSF_FRAME_STATUS_FRAME_READY) {
        if (p->frame->type != 0 && p->frame->length > 0 && p->cursor == p->frame->length - 1) {
            uint8_t crc = p->frame->data[p->frame->length - 2];
            uint8_t crc_calc = _crsf_calculate_crc(p->frame);
            if (crc != crc_calc) {
                p->status = CRSF_FRAME_STATUS_ERROR_CRC;
            }
        } else {
            assert(false);
        }
    }
}

void IRAM_ATTR crsf_parse_channels(const crsf_frame_t *frame, crsf_payload_channels_t *payload) {
    assert(frame->type == CRSF_FRAME_TYPE_RC_CHANNELS_PACKED);
    assert(frame->length == CRSF_PAYLOAD_LENGTH_RC_CHANNELS_PACKED);

    memcpy(payload, frame->data, CRSF_PAYLOAD_LENGTH_RC_CHANNELS_PACKED - 2);
}

void crsf_parse_heartbeat(const crsf_frame_t *frame, crsf_payload_heartbeat_t *payload) {
    assert(frame->type == CRSF_FRAME_TYPE_HEARTBEAT);
    //assert(frame->length == CRSF_PAYLOAD_LENGTH_HEARTBEAT);

    payload->source = (crsf_frame_address_t)frame->data[0];
}

void IRAM_ATTR crsf_parse_device_ping(const crsf_frame_t *frame, crsf_payload_device_ping_t *payload) {
    assert(frame->type == CRSF_FRAME_TYPE_DEVICE_PING);
    assert(frame->length == CRSF_PAYLOAD_LENGTH_DEVICE_PING);

    payload->dest = (crsf_frame_address_t)frame->data[0];
    payload->source = (crsf_frame_address_t)frame->data[1];
}

void IRAM_ATTR crsf_parse_parameter_read(const crsf_frame_t *frame, crsf_payload_parameter_read_t *payload) {
    assert(frame->type == CRSF_FRAME_TYPE_PARAMETER_READ);
    assert(frame->length == CRSF_PAYLOAD_LENGTH_PARAM_READ);

    payload->dest = (crsf_frame_address_t)frame->data[0];
    payload->source = (crsf_frame_address_t)frame->data[1];
    payload->param_idx = frame->data[2];
    payload->chunk_idx = frame->data[3];
}

void IRAM_ATTR crsf_parse_parameter_write(const crsf_frame_t *frame, crsf_payload_parameter_write_t *payload) {
    assert(frame->type == CRSF_FRAME_TYPE_PARAMETER_WRITE);

    uint32_t value = 0;

    payload->dest = (crsf_frame_address_t)frame->data[0];
    payload->source = (crsf_frame_address_t)frame->data[1];
    payload->param_idx = frame->data[2];
    payload->value_size = frame->length - 5; // type, dest, source, param_idx, crc

    memset(&payload->value, 0, sizeof(crsf_payload_parameter_value_t));
    uint8_t *value_ptr = reinterpret_cast<uint8_t *>(&payload->value);
    for (int i = 0; i < payload->value_size; i++) {
        *(value_ptr + payload->value_size - 1 - i) = frame->data[3 + i];
    }

    //ESP_LOG_BUFFER_HEX(TAG, frame->data, frame->length - 1);
}

void IRAM_ATTR crsf_serialize_channels_packed(const crsf_payload_channels_t *payload, crsf_frame_t *frame) {
    assert(false);
    //frame->sync = CRSF_SYNC_BYTE;
    frame->length = CRSF_PAYLOAD_LENGTH_RC_CHANNELS_PACKED;
    frame->type = CRSF_FRAME_TYPE_RC_CHANNELS_PACKED;

    memcpy(frame->data, payload, CRSF_PAYLOAD_LENGTH_RC_CHANNELS_PACKED - 2);

    _crsf_append_crc_byte(frame);
}

void IRAM_ATTR crsf_serialize_link_statistics(const crsf_payload_link_statistics_t *payload, crsf_frame_t *frame) {
    assert(false);
    //frame->sync = CRSF_SYNC_BYTE;
    frame->length = CRSF_PAYLOAD_LENGTH_LINK_STATISTICS;
    frame->type = CRSF_FRAME_TYPE_LINK_STATISTICS;

    memcpy(frame->data, payload, CRSF_PAYLOAD_LENGTH_LINK_STATISTICS - 2);

    _crsf_append_crc_byte(frame);
}

void crsf_serialize_device_info(const crsf_payload_device_info_t *payload, crsf_frame_t *frame) {
    assert(false);
    //frame->sync = CRSF_SYNC_BYTE;
    frame->type = CRSF_FRAME_TYPE_DEVICE_INFO;

    uint8_t i = 0;
    frame->data[i++] = payload->source;
    frame->data[i++] = payload->dest;

    size_t tmp_len = strlen(payload->name);
    memcpy(frame->data + i, payload->name, tmp_len + 1); i += tmp_len + 1;

    memcpy(frame->data + i, "ELRS", 4); i += 4;
    memcpy(frame->data + i, payload->version_hw, 4); i += 4;
    memcpy(frame->data + i, payload->version_sw, 4); i += 4;
    frame->data[i++] = payload->param_num;
    frame->data[i++] = payload->param_version;

    frame->length = i + 2;

    _crsf_append_crc_byte(frame);
}

template<typename T>
inline uint8_t _serialize_param_basic_value(uint8_t *buffer, const T &value) {
    int size = sizeof(T);
    int offset = 0;
    const uint8_t *value_ptr = reinterpret_cast<const uint8_t *>(&value);
    for (int i = 0; i < size; i++) {
        uint8_t v = value_ptr[size - offset - 1];
        buffer[offset++] = v;
    }

    return offset;
}

template<typename T>
uint8_t _serialize_param_basic(uint8_t *buffer, const T &value, const T &value_min, const T &value_max, const T &value_default, const char *units) {
    int offset = 0;

    offset += _serialize_param_basic_value(buffer + offset, value);
    offset += _serialize_param_basic_value(buffer + offset, value_min);
    offset += _serialize_param_basic_value(buffer + offset, value_max);
    offset += _serialize_param_basic_value(buffer + offset, value_default);

    if (units != NULL) {
        size_t tmp_len = strlen(units);
        memcpy(buffer + offset, units, tmp_len + 1); offset += tmp_len + 1;
    }

    return offset;
}

template<typename T>
uint8_t _serialize_param_basic(uint8_t *buffer, const crsf_device_param_value_basic_s<T> &t) {
    return _serialize_param_basic<T>(buffer, t.value, t.value_min, t.value_max, 0, t.units);
}

uint8_t _serialize_param_string_opts(uint8_t *buffer, const char * const*options, uint8_t options_count) {
    uint8_t offset = 0;

    size_t tmp_len = 0;
    for (int i = 0; i < options_count; i++) {
        if (offset > 0) {
            buffer[offset] = ';';
            offset += 1;
        }
        tmp_len = strlen(options[i]);
        memcpy(buffer + offset, options[i], tmp_len); offset += tmp_len;
    }
    buffer[offset] = 0;

    return offset;
}

void crsf_serialize_parameter_settings_entry(const crsf_payload_parameter_entry_t *payload, crsf_frame_t *frame) {
    assert(false);
    //frame->sync = CRSF_SYNC_BYTE;
    frame->type = CRSF_FRAME_TYPE_PARAMETER_SETTINGS_ENTRY;

    uint8_t offset = 0;

    uint8_t type = payload->type | (payload->hidden ? 0x80 : 0x0);

    frame->data[offset++] = payload->source;
    frame->data[offset++] = payload->dest;
    frame->data[offset++] = payload->param_idx;
    frame->data[offset++] = payload->chunks_remaining;
    frame->data[offset++] = payload->parent_idx;
    frame->data[offset++] = type;

    size_t tmp_len = strlen(payload->label);
    memcpy(frame->data + offset, payload->label, tmp_len + 1); offset += tmp_len + 1;

    switch (payload->type) {
        case CRSF_PARAM_TYPE_UINT8:
            offset += _serialize_param_basic(frame->data + offset, payload->value.value_uint8_t);
            break;
        case CRSF_PARAM_TYPE_INT8:
            offset += _serialize_param_basic(frame->data + offset, payload->value.value_int8_t);
            break;
        case CRSF_PARAM_TYPE_UINT16:
            offset += _serialize_param_basic(frame->data + offset, payload->value.value_uint16_t);
            break;
        case CRSF_PARAM_TYPE_INT16:
            offset += _serialize_param_basic(frame->data + offset, payload->value.value_int16_t);
            break;
        case CRSF_PARAM_TYPE_UINT32:
        case CRSF_PARAM_TYPE_INT32:
        case CRSF_PARAM_TYPE_UINT64:
        case CRSF_PARAM_TYPE_INT64:
        case CRSF_PARAM_TYPE_FLOAT:
            assert(false);
        case CRSF_PARAM_TYPE_SELECT:
            offset += _serialize_param_string_opts(frame->data + offset, payload->value.value_select.options, payload->value.value_select.options_count) + 1;
            offset += _serialize_param_basic<uint8_t>(frame->data + offset, payload->value.value_select.value, 0, 0, 0, payload->value.value_select.units);
            break;
        case CRSF_PARAM_TYPE_STRING:
        case CRSF_PARAM_TYPE_FOLDER:
            tmp_len = strlen(payload->value.value_folder.name);
            memcpy(frame->data + offset, payload->value.value_folder.name, tmp_len + 1); offset += tmp_len + 1;
            break;
        case CRSF_PARAM_TYPE_INFO:
            tmp_len = strlen(payload->value.value_info.name);
            memcpy(frame->data + offset, payload->value.value_info.name, tmp_len + 1); offset += tmp_len + 1;
            break;
        case CRSF_PARAM_TYPE_COMMAND:
            frame->data[offset++] = payload->value.value_command.step;
            frame->data[offset++] = payload->value.value_command.timeout;
            tmp_len = strlen(payload->value.value_command.status);
            memcpy(frame->data + offset, payload->value.value_command.status, tmp_len + 1); offset += tmp_len + 1;
            break;
        case CRSF_PARAM_TYPE_VTX:
        default:
            assert(false);
    }

    frame->length = offset + 2;

    _crsf_append_crc_byte(frame);
}

void IRAM_ATTR crsf_serialize_elrs_status(const crsf_payload_elrs_status_t *payload, crsf_frame_t *frame) {
    assert(false);
    //frame->sync = CRSF_SYNC_BYTE;
    frame->type = CRSF_FRAME_TYPE_ELRS_STATUS;

    uint8_t offset = 0;

    frame->data[offset++] = payload->source;
    frame->data[offset++] = payload->dest;
    frame->data[offset++] = payload->packets_bad;
    frame->data[offset++] = payload->packets_good >> 8;
    frame->data[offset++] = (payload->packets_good & 0xFF);
    frame->data[offset++] = payload->flags;

    uint8_t tmp_len = strlen(payload->message);
    memcpy(frame->data + offset, payload->message, tmp_len + 1); offset += tmp_len + 1;

    frame->length = offset + 2;

    _crsf_append_crc_byte(frame);
}
