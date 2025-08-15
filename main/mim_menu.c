#include "mim_menu.h"

#include "libnet.h"

#include "mim_settings.h"

#include "util.h"

#include "libcrsf.h"
#include "libcrsf_device.h"
#include "libcrsf_device_param.h"
#include "libcrsf_payload.h"


#include <stdint.h>
#include <esp_log.h>
#include <string.h>
#include <lwip/ip4_addr.h>
#include <esp_netif_ip_addr.h>

#define MIM_CRSF_DEVICE_SERIAL 0x04423448

static const char *TAG = "MENU";

void _param_select_mode_get(crsf_device_param_read_value_t *value) {
    value->select.index = mim_settings_get()->mode;
    value->select.option_count = 2;
    value->select.options[0] = "wifi";
    value->select.options[1] = "ethernet";
    value->select.units = "";
}

void _param_select_mode_set(const crsf_device_param_write_value_t *value) {
    assert(value->select_index == 0 || value->select_index == 1);

    mim_settings_mode_t mode = (mim_settings_mode_t)value->select_index;
    if (mim_settings_get()->mode != mode) {
        mim_settings_set_mode(mode);
        mim_settings_save();

        esp_restart();
    }
}

void _param_info_link_get(crsf_device_param_read_value_t *value) {
    value->info = libnet_is_connected() ? "up" : "down";
}

char _ip_address[16];
void _param_info_ip_address(crsf_device_param_read_value_t *value) {
    ip4_addr_t addr;
    libnet_get_ip_address(&addr);
    snprintf(_ip_address, 16, IPSTR, IP2STR(&addr));
    value->info = _ip_address;
}

void _param_u16_skymap_udp_port_get(crsf_device_param_read_value_t *value) {
    value->u16.value = mim_settings_get()->skymap_udp_port;
    value->u16.value_min = 0;
    value->u16.value_max = UINT16_MAX;
    value->u16.units = "";
}

void _param_u16_skymap_udp_port_set(const crsf_device_param_write_value_t *value) {
    mim_settings_set_skymap_udp_port(value->u16);
    mim_settings_save();
}

void mim_menu_init(crsf_device_t *device) {
    crsf_device_init(device, CRSF_ADDRESS_CRSF_MIM, "crsf-mim", MIM_CRSF_DEVICE_SERIAL);

    crsf_device_add_param(device, "mode", CRSF_PARAM_TYPE_SELECT, NULL, 
                          _param_select_mode_get, _param_select_mode_set);
    crsf_device_add_param(device, "link", CRSF_PARAM_TYPE_INFO, NULL, 
                          _param_info_link_get, NULL);
    crsf_device_add_param(device, "ip address", CRSF_PARAM_TYPE_INFO, NULL, 
                          _param_info_ip_address, NULL);
    crsf_device_add_param(device, "Skymap port", CRSF_PARAM_TYPE_UINT16, NULL, 
                          _param_u16_skymap_udp_port_get, _param_u16_skymap_udp_port_set);
}

