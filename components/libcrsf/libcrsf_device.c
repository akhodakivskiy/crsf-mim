#include "libcrsf_device.h"

#include "esp_app_desc.h"
#include "libcrsf.h"
#include "libcrsf_payload.h"
#include "libcrsf_device_param.h"

#include "esp_log.h"
#include "esp_heap_caps.h"
#include <string.h>

static const char *TAG = "CRSF_DEVICE";

void _crsf_device_foot_get_value(crsf_device_param_read_value_t *value) {
    value->folder = "__root__";
}

void _crsf_device_foot_set_value(const crsf_device_param_write_value_t *value) {
}

void crsf_device_init(crsf_device_t *device, 
                      crsf_address_t address, const char *name, uint32_t serial) {
    device->address = address;
    device->name = name;
    device->serial = serial;
    device->param_count = 0;

    crsf_device_add_param(device, "__root__", CRSF_PARAM_TYPE_FOLDER, NULL, _crsf_device_foot_get_value, _crsf_device_foot_set_value);
}

crsf_device_param_t *crsf_device_add_param(crsf_device_t *device,
                                          const char *name,
                                          crsf_device_param_type_t type,
                                          crsf_device_param_t *parent,
                                          crsf_device_param_get_value_func_t *func_get,
                                          crsf_device_param_set_value_func_t *func_set) {
    assert(device->param_count < CRSF_DEVICE_PARAM_MAX_COUNT);

    crsf_device_param_t *p = &(device->params[device->param_count]);

    p->index = device->param_count;
    p->name = name;
    p->type = type;
    p->parent_index = parent != NULL ? parent->index : 0;
    p->func_get = func_get;
    p->func_set = func_set;

    device->param_count++;

    return p;
}

crsf_device_param_t *crsf_device_get_param(crsf_device_t *device, uint8_t index) {
    assert(device->param_count > index);

    return &device->params[index];
}

crsf_device_result_t crsf_device_handle__ping(crsf_device_t *device, const crsf_frame_t *request, crsf_payload_device_info_t *response) {
    assert(request->type == CRSF_FRAME_TYPE_DEVICE_PING);
    assert(request->length == CRSF_PAYLOAD_LENGTH_DEVICE_PING);
    assert(request->is_extended);

    crsf_device_result_t result = CRSF_DEVICE_RESULT_INVALID_DEVICE;

    if (request->dest == CRSF_ADDRESS_BROADCAST || request->dest == device->address) {
        response->source = device->address;
        response->dest = request->source;
        response->name = device->name;
        response->serial = device->serial;
        response->hardware_version = 0; // TODO
        response->software_version = 0; // TODO
        response->param_count = device->param_count - 1;
        response->param_protocol_version = 0; // TODO

        result = CRSF_DEVICE_RESULT_OK;
    }

    return result;
}

crsf_device_result_t crsf_device_handle__param_read(crsf_device_t *device, 
                                                   const crsf_payload_device_param_read_t *request, 
                                                   crsf_payload_device_param_entry_t *response) {
    crsf_device_result_t result = CRSF_DEVICE_RESULT_INVALID_PARAM;

    if (request->dest == device->address) {
        crsf_device_param_t *param = crsf_device_get_param(device, request->index);

        response->source = device->address;
        response->dest = request->source;
        response->index = param->index;
        response->parent_index = param->parent_index;
        response->type = param->type;
        response->name = param->name;

        if (param->func_get != NULL) {
            param->func_get(&response->value);
        }

        result = CRSF_DEVICE_RESULT_OK;
    }

    return result;
}

crsf_device_result_t crsf_device_handle__param_write(crsf_device_t *device, 
                                                    const crsf_payload_device_param_write_t *request, 
                                                    crsf_payload_device_param_entry_t *response) {
    crsf_device_result_t result = CRSF_DEVICE_RESULT_INVALID_PARAM;

    if (request->dest == device->address) {
        crsf_device_param_t *param = crsf_device_get_param(device, request->index);

        if (param->func_set != NULL) {
            param->func_set(&request->value);
        }

        response->source = device->address;
        response->dest = request->source;
        response->index = param->index;
        response->parent_index = param->parent_index;
        response->type = param->type;
        response->name = param->name;

        if (param->func_get != NULL) {
            param->func_get(&response->value);
        }

        result = CRSF_DEVICE_RESULT_OK;
    }

    return result;
}

crsf_device_result_t crsf_device_client_handler(crsf_device_t *device, const crsf_frame_t *frame_in, crsf_frame_t *frame_out) {
    crsf_device_result_t result = CRSF_DEVICE_RESULT_INVALID_DEVICE;

    switch (frame_in->type) {
        case CRSF_FRAME_TYPE_DEVICE_PING: {
            if (frame_in->is_extended && (frame_in->dest == device->address || frame_in->dest == CRSF_ADDRESS_BROADCAST)) {
                //ESP_LOGI(TAG, "ping> sync=%x, type=%x, length=%u, dest=%x, source=%x", frame_in->sync, frame_in->type, frame_in->length, frame_in->source, frame_in->dest);
                //ESP_LOG_BUFFER_HEX(TAG, frame_in->data, frame_in->length - 1);
                crsf_payload_device_info_t info;
                memset(&info, 0, sizeof(crsf_payload_device_info_t));
                result = crsf_device_handle__ping(device, frame_in, &info); 

                if (result == CRSF_DEVICE_RESULT_OK) {
                    crsf_payload_pack__device_info(frame_out, &info);
                    //ESP_LOGI(TAG, "<info sync=%x, type=%x, length=%u, dest=%x, source=%x", frame_out->sync, frame_out->type, frame_out->length, frame_out->source, frame_out->dest);
                    //ESP_LOG_BUFFER_HEX(TAG, frame_out->data, frame_out->length - 1);
                }

            }
            break;
        }
        case CRSF_FRAME_TYPE_PARAM_READ:
            if (frame_in->is_extended && frame_in->dest == device->address) {
                crsf_payload_device_param_read_t read;
                crsf_payload_unpack__param_read(frame_in, &read);

                crsf_payload_device_param_entry_t entry;
                memset(&entry, 0, sizeof(crsf_payload_device_param_entry_t));
                result = crsf_device_handle__param_read(device, &read, &entry);

                //ESP_LOGI(TAG, "read> dest=%x, source=%x, len=%u, index=%u, chunk=%u", frame_in->source, frame_in->dest, frame_in->length, read.index, read.chunk_index);
                //ESP_LOG_BUFFER_HEX(TAG, frame_in->data, frame_in->length - 1);

                if (result == CRSF_DEVICE_RESULT_OK) {
                    crsf_payload_pack__param_entry(frame_out, &entry);
                    //ESP_LOGI(TAG, "<entry dest=%x, source=%x, len=%u, name=%s, index=%u, parent=%u, type=%u", 
                             //frame_out->source, frame_out->dest, frame_out->length,
                             //entry.name, entry.index, entry.parent_index, entry.type);
                    //ESP_LOG_BUFFER_HEX(TAG, frame_out->data, frame_out->length - 1);
                }
            }
            break;
        case CRSF_FRAME_TYPE_PARAM_WRITE: {
            if (frame_in->is_extended && frame_in->dest == device->address) {
                crsf_payload_device_param_write_t write;
                // first unpack the param index
                crsf_payload_unpack__param_write_header(frame_in, &write);

                crsf_device_param_t *param = crsf_device_get_param(device, write.index);
                // now unpack parameter value
                crsf_payload_unpack__param_write_value(frame_in, &write, param->type);

                crsf_payload_device_param_entry_t entry;
                memset(&entry, 0, sizeof(crsf_payload_device_param_entry_t));
                result = crsf_device_handle__param_write(device, &write, &entry);

                //ESP_LOGI(TAG, "write> dest=%x, source=%x, len=%u, index=%u", frame_in->dest, frame_in->source, frame_in->length, write.index);
                //ESP_LOG_BUFFER_HEX(TAG, frame_in->data, frame_in->length - 1);

                if (result == CRSF_DEVICE_RESULT_OK) {
                    crsf_payload_pack__param_entry(frame_out, &entry);
                    //ESP_LOGI(TAG, "<entry dest=%x, source=%x, len=%u, name=%s, index=%u, parent=%u, type=%u", 
                             //frame_out->source, frame_out->dest, frame_in->length,
                             //entry.name, entry.index, entry.parent_index, entry.type);
                    //ESP_LOG_BUFFER_HEX(TAG, frame_out->data, frame_out->length - 1);
                }
            }
            break;
        }
        default:
            break;
    }

    return result;
}
