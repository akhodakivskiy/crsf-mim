#ifndef LIBCRSF_DEVICE_H
#define LIBCRSF_DEVICE_H

#include "libcrsf.h"
#include "libcrsf_payload.h"
#include "libcrsf_device_param.h"

#include <stdint.h>
#include "freertos/idf_additions.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CRSF_DEVICE_RESULT_OK = 0,
    CRSF_DEVICE_RESULT_INVALID_PARAM,
    CRSF_DEVICE_RESULT_INVALID_DEVICE,
    CRSF_DEVICE_RESULT_BUFFER_OVERFLOW,
    CRSF_DEVICE_RESULT_CRC_ERROR,
} crsf_device_result_t;

typedef void (crsf_device_param_get_value_func_t) 
(crsf_device_param_read_value_t *value);

typedef void (crsf_device_param_set_value_func_t) 
(const crsf_device_param_write_value_t *value);

typedef struct {
    uint8_t index;
    const char *name;
    crsf_device_param_type_t type;
    uint8_t parent_index;
    crsf_device_param_get_value_func_t *func_get;
    crsf_device_param_set_value_func_t *func_set;
} crsf_device_param_t;

typedef struct {
    crsf_address_t address;
    const char *name;
    uint32_t serial;
    uint8_t param_count;
    crsf_device_param_t params[CRSF_DEVICE_PARAM_MAX_COUNT];
} crsf_device_t;

void crsf_device_init(crsf_device_t *device, crsf_address_t address, 
                      const char *name, uint32_t serial);

void crsf_device_deinit(crsf_device_t *device);

crsf_device_param_t *crsf_device_add_param(crsf_device_t *device,
                                          const char *name,
                                          crsf_device_param_type_t type,
                                          crsf_device_param_t *parent,
                                          crsf_device_param_get_value_func_t *func_get,
                                          crsf_device_param_set_value_func_t *func_set);

crsf_device_param_t *crsf_device_get_param(crsf_device_t *device, uint8_t index);

crsf_device_result_t crsf_device_handle__ping(crsf_device_t *device, const crsf_frame_t *request, crsf_payload_device_info_t *response);
crsf_device_result_t crsf_device_handle__param_read(crsf_device_t *device, 
                                                   const crsf_payload_device_param_read_t *request, 
                                                   crsf_payload_device_param_entry_t *response);
crsf_device_result_t crsf_device_handle__param_write(crsf_device_t *device, 
                                                    const crsf_payload_device_param_write_t *request, 
                                                    crsf_payload_device_param_entry_t *response);

crsf_device_result_t crsf_device_client_handler(crsf_device_t *device, const crsf_frame_t *frame_in, crsf_frame_t *frame_out);

#ifdef __cplusplus
}
#endif

#endif
