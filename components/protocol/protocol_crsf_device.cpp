#include "protocol_crsf_device.h"

#include <sdkconfig.h>

#include <esp_log.h>
#include <esp_app_desc.h>
#include <esp_timer.h>
#include <string.h>
#include <endian.h>

#define _PROTOCOL_CRSF_DEVICE_ACTIVITY_TIMEOUT_US 2000000

static const char *TAG = "CRSF_DEVICE";

CrsfDevice::CrsfDevice(CrsfDeviceFolder *root) : _frame_queue_cursor(0),
                                                 _is_active(false),
                                                 _last_activity_us(0),
                                                 _root(root) {
}

void CrsfDevice::init() {
}

void CrsfDevice::deinit() {
}

void CrsfDevice::devicePing(const crsf_payload_device_ping_t &request) {
    _last_activity_us = esp_timer_get_time();

    if (_frame_queue_cursor < CRSF_DEVICE_FRAME_QUEUE_SIZE) {
        if (request.source == CRSF_ADDRESS_RADIO_TRANSMITTER) {
            crsf_payload_device_info_t response;
            _compose_device_info(response);
            crsf_frame_t *frame = _frame_queue + _frame_queue_cursor;
            crsf_serialize_device_info(&response, frame);
            _frame_queue_cursor += 1;
        }
    } else {
        ESP_LOGE(TAG, "CRSF frame queue is full");
    }
}

void CrsfDevice::paramRead(const crsf_payload_parameter_read_t &request) {
    //ESP_LOGI(TAG, "param read, index=%u", request.param_idx);

    _last_activity_us = esp_timer_get_time();

    if (_frame_queue_cursor < CRSF_DEVICE_FRAME_QUEUE_SIZE) {
        crsf_frame_t *frame = _frame_queue + _frame_queue_cursor;
        frame->type = CRSF_FRAME_TYPE_NONE;

        if (request.param_idx == 0) {
            assert(false);
        } else if (request.param_idx <= CrsfDeviceParam::getParamCount()) {
            crsf_payload_parameter_entry_t response;
            _compose_param_entry(response, request.param_idx);
            crsf_serialize_parameter_settings_entry(&response, frame);
        } else {
            assert(false);
        }

        if (frame->type != CRSF_FRAME_TYPE_NONE) {
            _frame_queue_cursor += 1;
        }
    } else {
        ESP_LOGE(TAG, "CRSF frame queue is full");
    }
}

void CrsfDevice::paramWrite(const crsf_payload_parameter_write_t &request) {
    //ESP_LOGI(TAG, "param write, index=%u", request.param_idx);
    _last_activity_us = esp_timer_get_time();

    if (_frame_queue_cursor < CRSF_DEVICE_FRAME_QUEUE_SIZE) {
        crsf_frame_t *frame = _frame_queue + _frame_queue_cursor;
        frame->type = CRSF_FRAME_TYPE_NONE;

        if (request.param_idx == 0) {
            crsf_payload_elrs_status_t response;
            _compose_elrs_status(response);
            crsf_serialize_elrs_status(&response, frame);
        } else if (request.param_idx <= CrsfDeviceParam::getParamCount()) {
            //ESP_LOGI(TAG, "settings param, index=%u value_u=%lu", request.param_idx, request.value);

            _root->getParam(request.param_idx)->setValue(request.value);

            crsf_payload_parameter_entry_t response;
            _compose_param_entry(response, request.param_idx);
            crsf_serialize_parameter_settings_entry(&response, frame);
        }

        if (frame->type != CRSF_FRAME_TYPE_NONE) {
            _frame_queue_cursor += 1;
        }
    } else {
        ESP_LOGE(TAG, "CRSF frame queue is full");
    }
}

crsf_frame_t *CrsfDevice::dequeueFrame() {
    assert(_frame_queue_cursor <= CRSF_DEVICE_FRAME_QUEUE_SIZE);

    if (_frame_queue_cursor == 0) {
        return NULL;
    }

    _frame_queue_cursor -= 1;
    return _frame_queue + _frame_queue_cursor;
}

IRAM_ATTR bool CrsfDevice::isActive() const {
    return esp_timer_get_time() - _last_activity_us < _PROTOCOL_CRSF_DEVICE_ACTIVITY_TIMEOUT_US;
}

void CrsfDevice::_compose_device_info(crsf_payload_device_info_t &payload) {
    payload.dest = CRSF_ADDRESS_CRSF_TRANSMITTER;
    payload.source = CRSF_ADDRESS_RADIO_TRANSMITTER;
    payload.name = "quad tx";
    payload.serial = htobe32(0x454C5253);
    memset(payload.version_hw, 0, 4);
    memset(payload.version_sw, 0, 4);
    payload.param_num = (uint8_t)(CrsfDeviceParam::getParamCount() - 1);  // elrsV3.lua appends one extra menu item
    payload.param_version = 0;
}

void CrsfDevice::_compose_param_entry(crsf_payload_parameter_entry_t &payload, uint8_t index) {
    CrsfDeviceParam *param = CrsfDeviceParam::getParam(index);

    payload.dest = CRSF_ADDRESS_CRSF_TRANSMITTER;
    payload.source = CRSF_ADDRESS_RADIO_TRANSMITTER;
    payload.param_idx = index;
    payload.chunks_remaining = 0;
    payload.parent_idx = param->getParentIndex();
    payload.hidden = false;

    param->render(payload);
}

void CrsfDevice::_compose_elrs_status(crsf_payload_elrs_status_t &payload) {
    payload.dest = CRSF_ADDRESS_CRSF_TRANSMITTER;
    payload.source = CRSF_ADDRESS_RADIO_TRANSMITTER;
    payload.packets_bad = 0;
    payload.packets_good = 0;
    payload.flags = 0;
}
