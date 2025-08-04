#include "protocol_crsf_device_param.h"

#include <esp_log.h>

uint8_t CrsfDeviceParam::getParentIndex() const { 
    return (_folder != NULL) ? _folder->getIndex() : 0;
}

CrsfDeviceParam *CrsfDeviceParam::getParam(uint8_t index) {
    return _index_param[index];
}

uint8_t CrsfDeviceParam::getParamCount() {
    return _index_counter;
}

CrsfDeviceParam *CrsfDeviceParam::_index_param[] = { NULL };

uint8_t CrsfDeviceParam::_index_counter = 0;
