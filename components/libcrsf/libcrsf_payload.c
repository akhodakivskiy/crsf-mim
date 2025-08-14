#include "libcrsf_payload.h"

#include "libcrsf_crc8.h"

#include <esp_log.h>
#include <string.h>
#include <endian.h>

static const char *TAG = "CRSF_PAYLOAD";

void crsf_payload_unpack__param_read(const crsf_frame_t *frame, crsf_payload_device_param_read_t *payload) {
    assert(frame->type == CRSF_FRAME_TYPE_PARAM_READ);
    assert(frame->length == CRSF_PAYLOAD_LENGTH_PARAM_READ);
    assert(frame->is_extended);

    payload->dest = frame->dest;
    payload->source = frame->source;
    payload->index = frame->data[2];
    payload->chunk_index = frame->data[3];
}

void crsf_payload_unpack__param_write_header(const crsf_frame_t *frame, crsf_payload_device_param_write_t *payload) {
    assert(frame->type == CRSF_FRAME_TYPE_PARAM_WRITE);

    payload->dest = frame->data[0];
    payload->source = frame->data[1];
    payload->index = frame->data[2];
    payload->type = CRSF_PARAM_TYPE_OUT_OF_RANGE;
}

void crsf_payload_unpack__param_write_value(const crsf_frame_t *frame, crsf_payload_device_param_write_t *payload, crsf_device_param_type_t type) {
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
            ESP_LOGE(TAG, "COMMAND not implemented");
            assert(false);
        case CRSF_PARAM_TYPE_FOLDER:
            break;
        case CRSF_PARAM_TYPE_INFO:
            ESP_LOGE(TAG, "FOLDER and INFO are not editable");
            assert(false);
        case CRSF_PARAM_TYPE_STRING:
            ESP_LOGE(TAG, "expresslrs doesn't support editing strings");
            assert(false);
        case CRSF_PARAM_TYPE_VTX:
            ESP_LOGE(TAG, "VTX control not implemented");
            assert(false);
        case CRSF_PARAM_TYPE_UINT64:
        case CRSF_PARAM_TYPE_INT64:
        default:
            ESP_LOGE(TAG, "type=%u, expresslrs doesn't support 32 and 64 bit numbers", payload->type);
            assert(false);
    }
}

void crsf_payload_pack__device_info(crsf_frame_t *frame, const crsf_payload_device_info_t *payload) {
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

void crsf_payload_pack__param_entry(crsf_frame_t *frame, 
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
            ESP_LOGE(TAG, "COMMAND not implemented");
            assert(false);
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
