#ifndef LIBCRSF_DEVICE_PARAM_H
#define LIBCRSF_DEVICE_PARAM_H

#include "libcrsf_def.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// device parameter types
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
    CRSF_COMMAND_STATE_IDLE,
    CRSF_COMMAND_STATE_CLICK,
    CRSF_COMMAND_STATE_EXECUTING,
    CRSF_COMMAND_STATE_ASKCONFIRM,
    CRSF_COMMAND_STATE_CONFIRMED,
    CRSF_COMMAND_STATE_CANCEL,
    CRSF_COMMAND_STATE_QUERY,
    CRSF_COMMAND_STATE__MAX,
} crsf_device_command_state_t;

// device parameter value
#define CRSF_DEVICE_PARAM_VALUE_BASIC(TYPE, NAME) \
struct { \
    TYPE value; \
    TYPE value_min; \
    TYPE value_max; \
    const char *units; \
} NAME 

typedef union {
    CRSF_DEVICE_PARAM_VALUE_BASIC(int8_t, i8);
    CRSF_DEVICE_PARAM_VALUE_BASIC(uint8_t, u8);
    CRSF_DEVICE_PARAM_VALUE_BASIC(int16_t, i16);
    CRSF_DEVICE_PARAM_VALUE_BASIC(uint16_t, u16);
    CRSF_DEVICE_PARAM_VALUE_BASIC(int32_t, i32);
    CRSF_DEVICE_PARAM_VALUE_BASIC(uint32_t, u32);
    CRSF_DEVICE_PARAM_VALUE_BASIC(int64_t, i64);
    CRSF_DEVICE_PARAM_VALUE_BASIC(uint64_t, u64);
    struct {
        int32_t value; // actual falue is value / 10^decimal_places
        int32_t value_min;
        int32_t value_max;
        uint8_t decimal_places;
        uint32_t step;
        const char *units;
    } flt;
    struct {
        uint8_t index;
        uint8_t option_count;
        const char *options[CRSF_DEVICE_PARAM_SELECT_OPTION_MAX_COUNT];
        const char *units;
    } select;
    const char *info;
    const char *folder;
    struct {
        crsf_device_command_state_t state;
        uint8_t timeout;
        const char *status;
    } command;
} crsf_device_param_read_value_t;

typedef union {
    int8_t i8;
    uint8_t u8;
    int16_t i16;
    uint16_t u16;
    int32_t i32;
    uint32_t u32;
    int64_t i64;
    uint64_t u64;
    int32_t flt;
    uint8_t select_index;
    const char *info;
    const char *folder;
    crsf_device_command_state_t command_action; // click, confirm, cancel, etc.
} crsf_device_param_write_value_t;

#ifdef __cplusplus
}
#endif

#endif
