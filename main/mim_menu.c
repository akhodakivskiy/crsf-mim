#include "mim_menu.h"

#include "libnet.h"

#include "mim_settings.h"

#include "skymap.h"
#include "util.h"

#include "libcrsf.h"
#include "libcrsf_device.h"
#include "libcrsf_device_param.h"

#include "mim_rc.h"

#include <stdint.h>
#include <esp_log.h>
#include <string.h>
#include <lwip/ip4_addr.h>
#include <esp_netif_ip_addr.h>

#define MIM_CRSF_DEVICE_SERIAL 0x04423448

typedef enum {
    _MIM_MENU_TEST_RC_IDLE,
    _MIM_MENU_TEST_RC_START,
    _MIM_MENU_TEST_RC_ROLL_LEFT,
    _MIM_MENU_TEST_RC_ROLL_RIGHT,
    _MIM_MENU_TEST_RC_PITCH_UP,
    _MIM_MENU_TEST_RC_PITCH_DOWN,
    _MIM_MENU_TEST_RC_FINISH,
} _mim_menu_test_rc_state_t;

static const char *TAG = "MENU";

static skymap_t *_skymap = NULL;

static _mim_menu_test_rc_state_t _test_rc_state = _MIM_MENU_TEST_RC_IDLE;

void _param_select_engage_channel_get(crsf_device_param_read_value_t *value) {
    uint8_t channel = mim_settings_get()->engage_channel;
    assert(channel >= 5 && channel <= 16);

    value->select.index = channel - 5;
    value->select.option_count = 12;
    value->select.options[0] = "5";
    value->select.options[1] = "6";
    value->select.options[2] = "7";
    value->select.options[3] = "8";
    value->select.options[4] = "9";
    value->select.options[5] = "10";
    value->select.options[6] = "11";
    value->select.options[7] = "12";
    value->select.options[8] = "13";
    value->select.options[9] = "14";
    value->select.options[10] = "15";
    value->select.options[11] = "16";
    value->select.units = "RC chan";
}

void _param_select_engage_channel_set(const crsf_device_param_write_value_t *value) {
    assert(value->select_index < 12);

    mim_settings_set_engage_channel(value->select_index + 5);
}

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

void _param_u8_engage_channel_get(crsf_device_param_read_value_t *value) {
    value->u8.value = mim_settings_get()->engage_channel;
    value->u8.value_min = 1;
    value->u8.value_max = 16;
    value->u8.units = "";
}

void _param_u8_engage_channel_set(const crsf_device_param_write_value_t *value) {
    mim_settings_set_engage_channel(value->u8);
    mim_settings_save();
}

void _param_folder_guidance_get(crsf_device_param_read_value_t *value) {
    value->folder = "guidance";
}

void _param_float_guidance_N_get(crsf_device_param_read_value_t *value) {
    value->flt.value = mim_settings_get()->guidance.N * 10;
    value->flt.value_min = 10;
    value->flt.value_max = 100;
    value->flt.decimal_places = 1;
    value->flt.step = 1;
    value->flt.units = "";
}

void _param_float_guidance_N_set(const crsf_device_param_write_value_t *value) {
    mim_settings_set_guidance_N(((float)value->flt) / 10.0);
    mim_settings_save();
}

void _param_u8_guidance_max_roll_deg_get(crsf_device_param_read_value_t *value) {
    value->u8.value = mim_settings_get()->guidance.max_roll_deg;
    value->u8.value_min = 0;
    value->u8.value_max = 90;
    value->u8.units = "deg";
}

void _param_u8_guidance_max_roll_deg_set(const crsf_device_param_write_value_t *value) {
    mim_settings_set_guidance_max_roll_deg(value->u8);
    mim_settings_save();
}

void _param_u8_guidance_max_pitch_deg_get(crsf_device_param_read_value_t *value) {
    value->u8.value = mim_settings_get()->guidance.max_pitch_deg;
    value->u8.value_min = 0;
    value->u8.value_max = 90;
    value->u8.units = "deg";
}

void _param_u8_guidance_max_pitch_deg_set(const crsf_device_param_write_value_t *value) {
    mim_settings_set_guidance_max_pitch_deg(value->u8);
    mim_settings_save();
}

void _param_select_guidance_pitch_invert_get(crsf_device_param_read_value_t *value) {
    value->select.index = mim_settings_get()->guidance.pitch_invert ? 0 : 1;
    value->select.option_count = 2;
    value->select.options[0] = "yes";
    value->select.options[1] = "no";
    value->select.units = "";
}

void _param_select_guidance_pitch_invert_set(const crsf_device_param_write_value_t *value) {
    assert(value->select_index == 0 || value->select_index == 1);

    mim_settings_set_guidance_pitch_invert(value->select_index == 0);
    mim_settings_save();
}

void _param_command_test_rc_get(crsf_device_param_read_value_t *value) {
    switch (_test_rc_state) {
        case _MIM_MENU_TEST_RC_IDLE:
            value->command.state = CRSF_COMMAND_STATE_READY;
            value->command.status = "";
            value->command.timeout = 1;
            break;
        case _MIM_MENU_TEST_RC_START:
            value->command.state = CRSF_COMMAND_STATE_PROGRESS;
            value->command.status = "starting...";
            value->command.timeout = 100;
            break;
        case _MIM_MENU_TEST_RC_ROLL_LEFT:
            value->command.state = CRSF_COMMAND_STATE_PROGRESS;
            value->command.status = "roll left";
            value->command.timeout = 200;
            break;
        case _MIM_MENU_TEST_RC_ROLL_RIGHT:
            value->command.state = CRSF_COMMAND_STATE_PROGRESS;
            value->command.status = "roll right";
            value->command.timeout = 200;
            break;
        case _MIM_MENU_TEST_RC_PITCH_UP:
            value->command.state = CRSF_COMMAND_STATE_PROGRESS;
            value->command.status = "pitch up";
            value->command.timeout = 200;
            break;
        case _MIM_MENU_TEST_RC_PITCH_DOWN:
            value->command.state = CRSF_COMMAND_STATE_PROGRESS;
            value->command.status = "pitch down";
            value->command.timeout = 200;
            break;
        case _MIM_MENU_TEST_RC_FINISH:
            assert(false);
        default:
            break;
    }
}

void _param_command_test_rc_set(const crsf_device_param_write_value_t *value) {
    switch (value->command_action) {
        case CRSF_COMMAND_STATE_START:
            _test_rc_state = _MIM_MENU_TEST_RC_START;
            break;
        case CRSF_COMMAND_STATE_POLL:
            _test_rc_state += 1;
            if (_test_rc_state == _MIM_MENU_TEST_RC_FINISH) {
                _test_rc_state = _MIM_MENU_TEST_RC_IDLE;
            }
            mim_rc_clear_overrides();
            switch (_test_rc_state) {
                case _MIM_MENU_TEST_RC_ROLL_LEFT:
                    mim_rc_override_channel(MIM_RC_CHANNEL_ROLL, -1, MIM_RC_OVERRIDE_LEVEL_TEST);
                    break;
                case _MIM_MENU_TEST_RC_ROLL_RIGHT:
                    mim_rc_override_channel(MIM_RC_CHANNEL_ROLL, 1, MIM_RC_OVERRIDE_LEVEL_TEST);
                    break;
                case _MIM_MENU_TEST_RC_PITCH_UP:
                    mim_rc_override_channel(MIM_RC_CHANNEL_PITCH, 1, MIM_RC_OVERRIDE_LEVEL_TEST);
                    break;
                case _MIM_MENU_TEST_RC_PITCH_DOWN:
                    mim_rc_override_channel(MIM_RC_CHANNEL_PITCH, -1, MIM_RC_OVERRIDE_LEVEL_TEST);
                    break;
                default:
                    break;
            }
        default:
            break;
    }
}

void mim_menu_init(crsf_device_t *device) {
    crsf_device_init(device, CRSF_ADDRESS_CRSF_MIM, "crsf-mim", MIM_CRSF_DEVICE_SERIAL);

    crsf_device_param_t *guidance = crsf_device_add_param(device, "guidance", CRSF_PARAM_TYPE_FOLDER, NULL, 
                          _param_folder_guidance_get, NULL);

    crsf_device_add_param(device, "N factor", CRSF_PARAM_TYPE_FLOAT, guidance, 
                          _param_float_guidance_N_get, 
                          _param_float_guidance_N_set);
    crsf_device_add_param(device, "max roll", CRSF_PARAM_TYPE_UINT8, guidance, 
                          _param_u8_guidance_max_roll_deg_get, 
                          _param_u8_guidance_max_roll_deg_set);
    crsf_device_add_param(device, "max pitch", CRSF_PARAM_TYPE_UINT8, guidance, 
                          _param_u8_guidance_max_pitch_deg_get, 
                          _param_u8_guidance_max_pitch_deg_set);

    crsf_device_add_param(device, "pitch invert", CRSF_PARAM_TYPE_SELECT, guidance, 
                          _param_select_guidance_pitch_invert_get, 
                          _param_select_guidance_pitch_invert_set);

    crsf_device_add_param(device, "test RC", CRSF_PARAM_TYPE_COMMAND, NULL,
                          _param_command_test_rc_get,
                          _param_command_test_rc_set);

    crsf_device_add_param(device, "mode", CRSF_PARAM_TYPE_SELECT, NULL, 
                          _param_select_mode_get, 
                          _param_select_mode_set);
    crsf_device_add_param(device, "link", CRSF_PARAM_TYPE_INFO, NULL, 
                          _param_info_link_get, NULL);
    crsf_device_add_param(device, "ip address", CRSF_PARAM_TYPE_INFO, NULL, 
                          _param_info_ip_address, NULL);
    crsf_device_add_param(device, "skymap port", CRSF_PARAM_TYPE_UINT16, NULL, 
                          _param_u16_skymap_udp_port_get, 
                          _param_u16_skymap_udp_port_set);

    crsf_device_add_param(device, "engage chan", CRSF_PARAM_TYPE_UINT8, NULL, 
                          _param_u8_engage_channel_get, 
                          _param_u8_engage_channel_set);

}

void mim_menu_set_skymap(skymap_t *sm) {
    _skymap = sm;
}
