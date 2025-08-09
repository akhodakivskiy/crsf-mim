#include "mim_menu.h"

#include "libcrsf.h"
#include "libcrsf_device.h"
#include "libcrsf_device_param.h"
#include "libcrsf_payload.h"

#include <esp_log.h>
#include <string.h>

#define MIM_CRSF_DEVICE_SERIAL 0x04423448

static const char *TAG = "MENU";

uint8_t u8 = 1;
int8_t i8 = -2;
uint16_t u16 = 300;
int16_t i16 = -10;
uint32_t u32 = 300;
int32_t i32 = -10;
int32_t flt = -500;
const char *info = "info-value";

uint8_t select_index = 0;

void _param_folder_get(crsf_device_param_read_value_t *value) {
    value->folder = "folder";
}

void _param_u8_get(crsf_device_param_read_value_t *value) {
    value->u8.value = u8;
    value->u8.value_min = 0;
    value->u8.value_max = 10;
    value->u8.units = "u";
}

void _param_u8_set(const crsf_device_param_write_value_t *value) {
    u8 = value->u8;
}

void _param_i8_get(crsf_device_param_read_value_t *value) {
    value->i8.value = i8;
    value->i8.value_min = -10;
    value->i8.value_max = 10;
    value->i8.units = "i";
}

void _param_i8_set(const crsf_device_param_write_value_t *value) {
    i8 = value->i8;
}

void _param_u16_get(crsf_device_param_read_value_t *value) {
    value->u16.value = u16;
    value->u16.value_min = 250;
    value->u16.value_max = 300;
    value->u16.units = "u";
}

void _param_u16_set(const crsf_device_param_write_value_t *value) {
    u16 = value->u16;
}

void _param_i16_get(crsf_device_param_read_value_t *value) {
    value->i16.value = i16;
    value->i16.value_min = -20;
    value->i16.value_max = 10;
    value->i16.units = "i";
}

void _param_i16_set(const crsf_device_param_write_value_t *value) {
    i16 = value->i16;
}

void _param_flt_get(crsf_device_param_read_value_t *value) {
    value->flt.value = flt;
    value->flt.value_min = -1000;
    value->flt.value_max = 1000;
    value->flt.decimal_places = 2;
    value->flt.step = 5;
    value->flt.units = "f";
}

void _param_flt_set(const crsf_device_param_write_value_t *value) {
    flt = value->flt;
}

void _param_select_get(crsf_device_param_read_value_t *value) {
    value->select.index = select_index;
    value->select.option_count = 4;
    value->select.options[0] = "1";
    value->select.options[1] = "2";
    value->select.options[2] = "3";
    value->select.options[3] = "4";
    value->select.units = "ch";
}

void _param_select_set(const crsf_device_param_write_value_t *value) {
    select_index = value->select_index;
}

void _param_info_get(crsf_device_param_read_value_t *value) {
    value->info = info;
}

void mim_menu_init(crsf_device_t *device) {
    crsf_device_init(device, CRSF_ADDRESS_CRSF_MIM, "crsf-mim", MIM_CRSF_DEVICE_SERIAL);

    crsf_device_param_t *folder = crsf_device_add_param(device, "folder", CRSF_PARAM_TYPE_FOLDER, NULL, 
                                                        _param_folder_get, NULL);
    crsf_device_add_param(device, "u8", CRSF_PARAM_TYPE_UINT8, folder, 
                          _param_u8_get, _param_u8_set);
    crsf_device_add_param(device, "i8", CRSF_PARAM_TYPE_INT8, folder, 
                          _param_i8_get, _param_i8_set);
    crsf_device_add_param(device, "u16", CRSF_PARAM_TYPE_UINT16, folder, 
                          _param_u16_get, _param_u16_set);
    crsf_device_add_param(device, "i16", CRSF_PARAM_TYPE_INT16, folder, 
                          _param_i16_get, _param_i16_set);
    crsf_device_add_param(device, "flt", CRSF_PARAM_TYPE_FLOAT, NULL, 
                          _param_flt_get, _param_flt_set);
    crsf_device_add_param(device, "select", CRSF_PARAM_TYPE_SELECT, NULL, 
                          _param_select_get, _param_select_set);
    crsf_device_add_param(device, "info", CRSF_PARAM_TYPE_INFO, NULL, _param_info_get, NULL);
}

