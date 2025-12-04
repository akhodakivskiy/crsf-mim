#include "mim_menu.h"

#include "esp_timer.h"
#include "mim_settings.h"
#include "libcrsf.h"
#include "libcrsf_device.h"
#include "libcrsf_device_param.h"

#include "mim_rc.h"
#include "mim_nav.h"
#include "nav.h"
#include "util.h"

#include <stdint.h>
#include <esp_log.h>
#include <string.h>
#include <esp_netif_ip_addr.h>

#define _MIM_MENU_DEVICE_SERIAL 0x04423448
#define _MIM_MENU_MAX_COMMAND_STATUS_LENGTH CRSF_MAX_PAYLOAD_LEN - 2
#define _MIM_MENU_TEST_RC_TIMEOUT 3000000

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

static mim_nav_handle_t _mim_menu_nav = NULL;

static char _mim_menu_command_status[_MIM_MENU_MAX_COMMAND_STATUS_LENGTH];

static _mim_menu_test_rc_state_t _mim_menu_test_rc_state = _MIM_MENU_TEST_RC_IDLE;
static uint64_t _mim_menu_test_rc_last_time = 0;

static char _mim_menu_ip_address_str[16];

static void _param_select_engage_channel_get(crsf_device_param_read_value_t *value) {
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

static void _param_select_engage_channel_set(const crsf_device_param_write_value_t *value) {
    assert(value->select_index < 12);

    mim_settings_set_engage_channel(value->select_index + 5);
}

static void _param_select_mode_get(crsf_device_param_read_value_t *value) {
    value->select.index = mim_settings_get()->mode;
    value->select.option_count = 2;
    value->select.options[0] = "wifi";
    value->select.options[1] = "ethernet";
    value->select.units = "";
}

static void _param_select_mode_set(const crsf_device_param_write_value_t *value) {
    assert(value->select_index == 0 || value->select_index == 1);

    mim_settings_mode_t mode = (mim_settings_mode_t)value->select_index;
    if (mim_settings_get()->mode != mode) {
        mim_settings_set_mode(mode);
        mim_settings_save();

        esp_restart();
    }
}

static void _param_info_ip_address(crsf_device_param_read_value_t *value) {
    ip4_addr_t addr = mim_nav_get_ip_address(_mim_menu_nav);
    snprintf(_mim_menu_ip_address_str, 16, IPSTR, IP2STR(&addr));
    value->info = _mim_menu_ip_address_str;
}

static void _param_info_skymap_ip_address(crsf_device_param_read_value_t *value) {
    ip4_addr_t addr = mim_nav_get_skymap_ip_address(_mim_menu_nav);
    snprintf(_mim_menu_ip_address_str, 16, IPSTR, IP2STR(&addr));
    value->info = _mim_menu_ip_address_str;
}

static void _param_u16_skymap_udp_port_get(crsf_device_param_read_value_t *value) {
    value->u16.value = mim_settings_get()->skymap_udp_port;
    value->u16.value_min = 0;
    value->u16.value_max = UINT16_MAX;
    value->u16.units = "";
}

static void _param_u16_skymap_udp_port_set(const crsf_device_param_write_value_t *value) {
    mim_settings_set_skymap_udp_port(value->u16);
    mim_settings_save();
}

static void _param_u8_engage_channel_get(crsf_device_param_read_value_t *value) {
    value->u8.value = mim_settings_get()->engage_channel + 1;
    value->u8.value_min = 1;
    value->u8.value_max = 16;
    value->u8.units = "";
}

static void _param_u8_engage_channel_set(const crsf_device_param_write_value_t *value) {
    assert(value->u8 > 0 && value->u8 <= 16);
    mim_settings_set_engage_channel(value->u8 - 1);
    mim_settings_save();
}

static void _param_folder_guidance_get(crsf_device_param_read_value_t *value) {
    value->folder = "guidance";
}

static void _param_float_nav_N_get(crsf_device_param_read_value_t *value) {
    value->flt.value = mim_settings_get()->nav.N * 10;
    value->flt.value_min = 10;
    value->flt.value_max = 100;
    value->flt.decimal_places = 1;
    value->flt.step = 1;
    value->flt.units = "";
}

static void _param_float_nav_N_set(const crsf_device_param_write_value_t *value) {
    mim_settings_set_nav_N(((float)value->flt) / 10.0);
    mim_settings_save();
}

static void _param_u8_nav_max_roll_deg_get(crsf_device_param_read_value_t *value) {
    value->u8.value = mim_settings_get()->nav.max_roll_deg;
    value->u8.value_min = 0;
    value->u8.value_max = 90;
    value->u8.units = "deg";
}

static void _param_u8_nav_max_roll_deg_set(const crsf_device_param_write_value_t *value) {
    mim_settings_set_nav_max_roll_deg(value->u8);
    mim_settings_save();
}

static void _param_folder_nav_attack_get(crsf_device_param_read_value_t *value) {
    value->folder = "attack angle";
}

static void _param_u8_nav_attack_angle_get(crsf_device_param_read_value_t *value) {
    value->u8.value = mim_settings_get()->nav.attack_angle_deg;
    value->u8.value_min = 0;
    value->u8.value_max = 90;
    value->u8.units = "deg";
}

static void _param_u8_nav_attack_angle_set(const crsf_device_param_write_value_t *value) {
    mim_settings_set_nav_attack_angle_deg(value->u8);
    mim_settings_save();
}

static void _param_float_nav_attack_factor_get(crsf_device_param_read_value_t *value) {
    value->flt.value = mim_settings_get()->nav.attack_factor * 10;
    value->flt.value_min = 0;
    value->flt.value_max = 100;
    value->flt.decimal_places = 1;
    value->flt.step = 1;
    value->flt.units = "";
}

static void _param_float_nav_attack_factor_set(const crsf_device_param_write_value_t *value) {
    mim_settings_set_nav_attack_factor(((float)value->flt) / 10.0);
    mim_settings_save();
}

static void _param_folder_nav_pitcher_get(crsf_device_param_read_value_t *value) {
    value->folder = "pitch control";
}

static void _param_float_nav_pitcher_p_gain_get(crsf_device_param_read_value_t *value) {
    value->flt.value = mim_settings_get()->pitcher.kp * 100;
    value->flt.value_min = 0;
    value->flt.value_max = 100;
    value->flt.decimal_places = 2;
    value->flt.step = 1;
    value->flt.units = "";
}

static void _param_float_nav_pitcher_p_gain_set(const crsf_device_param_write_value_t *value) {
    mim_settings_set_nav_pitcher_p_gain(((float)value->flt) / 100.0);
    mim_settings_save();
}

static void _param_float_nav_pitcher_i_gain_get(crsf_device_param_read_value_t *value) {
    value->flt.value = mim_settings_get()->pitcher.ki * 100;
    value->flt.value_min = 0;
    value->flt.value_max = 100;
    value->flt.decimal_places = 2;
    value->flt.step = 1;
    value->flt.units = "";
}

static void _param_float_nav_pitcher_i_gain_set(const crsf_device_param_write_value_t *value) {
    mim_settings_set_nav_pitcher_i_gain(((float)value->flt) / 100.0);
    mim_settings_save();
}

static void _param_float_nav_pitcher_d_gain_get(crsf_device_param_read_value_t *value) {
    value->flt.value = mim_settings_get()->pitcher.kd * 100;
    value->flt.value_min = 0;
    value->flt.value_max = 100;
    value->flt.decimal_places = 2;
    value->flt.step = 1;
    value->flt.units = "";
}

static void _param_float_nav_pitcher_d_gain_set(const crsf_device_param_write_value_t *value) {
    mim_settings_set_nav_pitcher_d_gain(((float)value->flt) / 100.0);
    mim_settings_save();
}

static void _param_float_nav_pitcher_max_rate_get(crsf_device_param_read_value_t *value) {
    value->flt.value = mim_settings_get()->pitcher.max_rate * 100;
    value->flt.value_min = 0;
    value->flt.value_max = 100;
    value->flt.decimal_places = 2;
    value->flt.step = 1;
    value->flt.units = "";
}

static void _param_float_nav_pitcher_max_rate_set(const crsf_device_param_write_value_t *value) {
    mim_settings_set_nav_pitcher_max_rate(((float)value->flt) / 100.0);
    mim_settings_save();
}

static void _param_float_nav_pitcher_integral_limit_get(crsf_device_param_read_value_t *value) {
    value->flt.value = mim_settings_get()->pitcher.integral_limit * 100;
    value->flt.value_min = 0;
    value->flt.value_max = 100;
    value->flt.decimal_places = 2;
    value->flt.step = 1;
    value->flt.units = "";
}

static void _param_float_nav_pitcher_integral_limit_set(const crsf_device_param_write_value_t *value) {
    mim_settings_set_nav_pitcher_integral_limit(((float)value->flt) / 100.0);
    mim_settings_save();
}

static void _param_float_nav_pitcher_alpha_get(crsf_device_param_read_value_t *value) {
    value->flt.value = mim_settings_get()->pitcher.alpha * 100;
    value->flt.value_min = 0;
    value->flt.value_max = 100;
    value->flt.decimal_places = 2;
    value->flt.step = 1;
    value->flt.units = "";
}

static void _param_float_nav_pitcher_alpha_set(const crsf_device_param_write_value_t *value) {
    mim_settings_set_nav_pitcher_alpha(((float)value->flt) / 100.0);
    mim_settings_save();
}

static void _param_select_nav_pitcher_inverted_get(crsf_device_param_read_value_t *value) {
    value->select.index = mim_settings_get()->pitcher.inverted ? 0 : 1;
    value->select.option_count = 2;
    value->select.options[0] = "yes";
    value->select.options[1] = "no";
    value->select.units = "";
}

static void _param_select_nav_pitcher_inverted_set(const crsf_device_param_write_value_t *value) {
    assert(value->select_index == 0 || value->select_index == 1);

    mim_settings_set_nav_pitcher_inverted(value->select_index == 0);
    mim_settings_save();
}

static void _param_command_test_rc_get(crsf_device_param_read_value_t *value) {
    value->command.status = _mim_menu_command_status;
    value->command.timeout = 20;
    value->command.state = CRSF_COMMAND_STATE_PROGRESS;

    char *label = "";
    switch (_mim_menu_test_rc_state) {
        case _MIM_MENU_TEST_RC_IDLE:
            value->command.state = CRSF_COMMAND_STATE_READY;
            value->command.status = "";
            break;
        case _MIM_MENU_TEST_RC_START:
            label = "starting...";
            break;
        case _MIM_MENU_TEST_RC_ROLL_LEFT:
            label = "roll left";
            break;
        case _MIM_MENU_TEST_RC_ROLL_RIGHT:
            label = "roll right";
            break;
        case _MIM_MENU_TEST_RC_PITCH_UP:
            label = "pitch up";
            break;
        case _MIM_MENU_TEST_RC_PITCH_DOWN:
            label = "pitch down";
            break;
        case _MIM_MENU_TEST_RC_FINISH:
            assert(false);
        default:
            break;
    }

    if (value->command.state == CRSF_COMMAND_STATE_PROGRESS) {
        uint16_t roll = CRSF_RC_CHANNELS_TICKS_TO_US(mim_rc_get_channel_with_override(MIM_RC_CHANNEL_ROLL));
        uint16_t pitch = CRSF_RC_CHANNELS_TICKS_TO_US(mim_rc_get_channel_with_override(MIM_RC_CHANNEL_PITCH));

        snprintf(_mim_menu_command_status, _MIM_MENU_MAX_COMMAND_STATUS_LENGTH, 
                 "%s\x1E" 
                 "roll %u\x1E" 
                 "pitch %u", 
                 label, roll, pitch);
    }
}

static void _param_command_test_rc_set(const crsf_device_param_write_value_t *value) {
    int64_t time = esp_timer_get_time();

    switch (value->command_action) {
        case CRSF_COMMAND_STATE_START:
            ESP_LOGI(TAG, "test rc start");
            _mim_menu_test_rc_state = _MIM_MENU_TEST_RC_START;
            _mim_menu_test_rc_last_time = esp_timer_get_time();
            break;
        case CRSF_COMMAND_STATE_POLL:
            if (time - _mim_menu_test_rc_last_time > _MIM_MENU_TEST_RC_TIMEOUT) {
                _mim_menu_test_rc_last_time = esp_timer_get_time();
                _mim_menu_test_rc_state += 1;
                if (_mim_menu_test_rc_state == _MIM_MENU_TEST_RC_FINISH) {
                    _mim_menu_test_rc_state = _MIM_MENU_TEST_RC_IDLE;
                }
            }

            bool pitch_inverted = mim_settings_get()->pitcher.inverted;

            mim_rc_reset_overrides(MIM_RC_OVERRIDE_LEVEL_TEST);
            switch (_mim_menu_test_rc_state) {
                case _MIM_MENU_TEST_RC_ROLL_LEFT:
                    mim_rc_set_override(MIM_RC_CHANNEL_ROLL, CRSF_RC_CHANNELS_MIN, MIM_RC_OVERRIDE_LEVEL_TEST);
                    break;
                case _MIM_MENU_TEST_RC_ROLL_RIGHT:
                    mim_rc_set_override(MIM_RC_CHANNEL_ROLL, CRSF_RC_CHANNELS_MAX, MIM_RC_OVERRIDE_LEVEL_TEST);
                    break;
                case _MIM_MENU_TEST_RC_PITCH_UP:
                    mim_rc_set_override(MIM_RC_CHANNEL_PITCH, pitch_inverted ? CRSF_RC_CHANNELS_MIN : CRSF_RC_CHANNELS_MAX, MIM_RC_OVERRIDE_LEVEL_TEST);
                    break;
                case _MIM_MENU_TEST_RC_PITCH_DOWN:
                    mim_rc_set_override(MIM_RC_CHANNEL_PITCH, pitch_inverted ? CRSF_RC_CHANNELS_MAX : CRSF_RC_CHANNELS_MIN, MIM_RC_OVERRIDE_LEVEL_TEST);
                    break;
                default:
                    break;
            }
            break;
        case CRSF_COMMAND_STATE_CANCEL:
            ESP_LOGI(TAG, "test rc cancelling");
            mim_rc_reset_overrides(MIM_RC_OVERRIDE_LEVEL_TEST);
            _mim_menu_test_rc_state = _MIM_MENU_TEST_RC_IDLE;
            _mim_menu_test_rc_last_time = 0;
            break;
        default:
            assert(false);
    }
}

static const char *_param_command_estimage_status_label(mim_nav_estimate_status_t status) {
    switch (status) {
        case MIM_NAV_ESTIMATE_STATUS_SKYMAP:
            return "s";
        case MIM_NAV_ESTIMATE_STATUS_CRSF:
            return "c";
        case MIM_NAV_ESTIMATE_STATUS_NONE:
            return "-";
    }
    assert(false);
}

static const char *_param_command_nav_cmd_type_label(nav_type_t type) {
    switch (type) {
        case NAV_NONE:
            return "none";
        case NAV_PRONAV:
            return "pronav";
        case NAV_PURSUIT:
            return "pursuit";
    }
    assert(false);
}

static bool _param_track_enabled = false;

static void _param_command_track_get(crsf_device_param_read_value_t *value) {
    if (_param_track_enabled) {
        const nav_command_t *cmd = mim_nav_get_last_command(_mim_menu_nav);
        const char *cmd_type_label = _param_command_nav_cmd_type_label(cmd->type);

        const char *status_target = _param_command_estimage_status_label(mim_nav_target_status(_mim_menu_nav));
        const char *status_interceptor = _param_command_estimage_status_label(mim_nav_interceptor_status(_mim_menu_nav));

        snprintf(_mim_menu_command_status, _MIM_MENU_MAX_COMMAND_STATUS_LENGTH, 
                 "[t%s i%s %s]\x1E"
                 "r=%.1f dh=f%.f\x1E"
                 "ah=%.1f av=%.1f ",
                 //"tgo=%.0f 0em=%.0f ",
                 status_target, status_interceptor, cmd_type_label, 
                 cmd->range, cmd->range_ver, cmd->accel_lat, cmd->accel_ver /*cmd->time_to_go_s, cmd->zero_effort_miss_m*/);

        value->command.state = CRSF_COMMAND_STATE_PROGRESS;
        value->command.status = _mim_menu_command_status;
        value->command.timeout = 20;
    } else {
        value->command.state = CRSF_COMMAND_STATE_READY;
        value->command.status = "";
        value->command.timeout = 20;
    }
}

static void _param_command_track_set(const crsf_device_param_write_value_t *value) {
    switch (value->command_action) {
        case CRSF_COMMAND_STATE_START:
            _param_track_enabled = true;
            break;
        case CRSF_COMMAND_STATE_POLL:
            break;
        case CRSF_COMMAND_STATE_CANCEL:
            _param_track_enabled = false;
            break;
        default:
            assert(false);
    }
}

static bool _param_state_is_enabled = false;
static bool _param_state_is_target = false;

static void _param_command_state_get(crsf_device_param_read_value_t *value) {
    if (_param_state_is_enabled) {
        const nav_state_t *s = NULL;
        const char *label = NULL;
        if (_param_state_is_target) {
            s = mim_nav_get_target(_mim_menu_nav);
            label = "target";
        } else {
            s = mim_nav_get_interceptor(_mim_menu_nav);
            label = "interceptor";
        }

        float vh = la_sqrt(la_pow(s->vel_east, 2) + la_pow(s->vel_north, 2));

        snprintf(_mim_menu_command_status, _MIM_MENU_MAX_COMMAND_STATUS_LENGTH, 
                 "%s\x1E"
                 "alt=%.1f\x1E"
                 "vh=%.1f vz=%.1f ",
                 label, s->alt, vh, s->vel_up);
        value->command.state = CRSF_COMMAND_STATE_PROGRESS;
        value->command.status = _mim_menu_command_status;
        value->command.timeout = 20;
    } else {
        value->command.state = CRSF_COMMAND_STATE_READY;
        value->command.status = "";
        value->command.timeout = 20;
    }
}

static void _param_command_state_set(const crsf_device_param_write_value_t *value) {
    switch (value->command_action) {
        case CRSF_COMMAND_STATE_START:
            _param_state_is_enabled = true;
            _param_state_is_target = !_param_state_is_target;
            break;
        case CRSF_COMMAND_STATE_POLL:
            break;
        case CRSF_COMMAND_STATE_CANCEL:
            _param_state_is_enabled = false;
            break;
        default:
            assert(false);
    }
}

static bool _param_reset_pending = false;

static void _param_command_reset_defaults_get(crsf_device_param_read_value_t *value) {
    if (_param_reset_pending) {
        value->command.state = CRSF_COMMAND_STATE_CONFIRMATION_NEEDED;
        value->command.status = "reset to defaults?";
        value->command.timeout = 20;
    } else {
        value->command.state = CRSF_COMMAND_STATE_READY;
        value->command.status = "";
        value->command.timeout = 1;
    }
}

static void _param_command_reset_defaults_set(const crsf_device_param_write_value_t *value) {
    switch (value->command_action) {
        case CRSF_COMMAND_STATE_START:
            _param_reset_pending = true;
            break;
        case CRSF_COMMAND_STATE_CONFIRM:
            mim_settings_reset_to_defaults();
            mim_settings_save();
            util_reboot_with_delay(10);
            _param_reset_pending = false;
            break;
        case CRSF_COMMAND_STATE_CANCEL:
            _param_reset_pending = false;
            break;
        default:
            assert(false);
    }
}

void mim_menu_init(crsf_device_t *device, mim_nav_handle_t nav) {
    _mim_menu_nav = nav;

    crsf_device_init(device, CRSF_ADDRESS_CRSF_MIM, "crsf-mim", _MIM_MENU_DEVICE_SERIAL);

    crsf_device_param_t *config = crsf_device_add_param(device, "config", CRSF_PARAM_TYPE_FOLDER, NULL, 
                          _param_folder_guidance_get, NULL);

    crsf_device_add_param(device, "N factor", CRSF_PARAM_TYPE_FLOAT, config, 
                          _param_float_nav_N_get, 
                          _param_float_nav_N_set);
    crsf_device_add_param(device, "max roll", CRSF_PARAM_TYPE_UINT8, config, 
                          _param_u8_nav_max_roll_deg_get, 
                          _param_u8_nav_max_roll_deg_set);
    crsf_device_add_param(device, "engage chan", CRSF_PARAM_TYPE_UINT8, config, 
                          _param_u8_engage_channel_get, 
                          _param_u8_engage_channel_set);

    crsf_device_param_t *guidance_attack = crsf_device_add_param(device, "attack angle", CRSF_PARAM_TYPE_FOLDER, config,
                          _param_folder_nav_attack_get, NULL);

    crsf_device_add_param(device, "angle", CRSF_PARAM_TYPE_UINT8, guidance_attack, 
                          _param_u8_nav_attack_angle_get, 
                          _param_u8_nav_attack_angle_set);
    crsf_device_add_param(device, "factor", CRSF_PARAM_TYPE_FLOAT, guidance_attack, 
                          _param_float_nav_attack_factor_get, 
                          _param_float_nav_attack_factor_set);

    crsf_device_param_t *guidance_pitcher = crsf_device_add_param(device, "pitch", CRSF_PARAM_TYPE_FOLDER, config,
                          _param_folder_nav_pitcher_get, NULL);

    crsf_device_add_param(device, "P gain", CRSF_PARAM_TYPE_FLOAT, guidance_pitcher, 
                          _param_float_nav_pitcher_p_gain_get, 
                          _param_float_nav_pitcher_p_gain_set);
    crsf_device_add_param(device, "I gain", CRSF_PARAM_TYPE_FLOAT, guidance_pitcher, 
                          _param_float_nav_pitcher_i_gain_get, 
                          _param_float_nav_pitcher_i_gain_set);
    crsf_device_add_param(device, "D gain", CRSF_PARAM_TYPE_FLOAT, guidance_pitcher, 
                          _param_float_nav_pitcher_d_gain_get, 
                          _param_float_nav_pitcher_d_gain_set);
    crsf_device_add_param(device, "max rate", CRSF_PARAM_TYPE_FLOAT, guidance_pitcher, 
                          _param_float_nav_pitcher_max_rate_get, 
                          _param_float_nav_pitcher_max_rate_set);
    crsf_device_add_param(device, "integral_limit", CRSF_PARAM_TYPE_FLOAT, guidance_pitcher, 
                          _param_float_nav_pitcher_integral_limit_get, 
                          _param_float_nav_pitcher_integral_limit_set);
    crsf_device_add_param(device, "alpha", CRSF_PARAM_TYPE_FLOAT, guidance_pitcher, 
                          _param_float_nav_pitcher_alpha_get, 
                          _param_float_nav_pitcher_alpha_set);

    crsf_device_add_param(device, "pitch invert", CRSF_PARAM_TYPE_SELECT, guidance_pitcher, 
                          _param_select_nav_pitcher_inverted_get, 
                          _param_select_nav_pitcher_inverted_set);

    crsf_device_add_param(device, "test RC", CRSF_PARAM_TYPE_COMMAND, NULL,
                          _param_command_test_rc_get,
                          _param_command_test_rc_set);

    crsf_device_add_param(device, "track", CRSF_PARAM_TYPE_COMMAND, NULL,
                          _param_command_track_get,
                          _param_command_track_set);

    crsf_device_add_param(device, "state", CRSF_PARAM_TYPE_COMMAND, NULL,
                          _param_command_state_get,
                          _param_command_state_set);

    crsf_device_add_param(device, "mode", CRSF_PARAM_TYPE_SELECT, NULL, 
                          _param_select_mode_get, 
                          _param_select_mode_set);

    crsf_device_add_param(device, "address", CRSF_PARAM_TYPE_INFO, NULL, 
                          _param_info_ip_address, NULL);

    crsf_device_add_param(device, "skymap addr", CRSF_PARAM_TYPE_INFO, NULL, 
                          _param_info_skymap_ip_address, NULL);

    crsf_device_add_param(device, "skymap port", CRSF_PARAM_TYPE_UINT16, NULL, 
                          _param_u16_skymap_udp_port_get, 
                          _param_u16_skymap_udp_port_set);

    crsf_device_add_param(device, "reset defaults", CRSF_PARAM_TYPE_COMMAND, NULL,
                          _param_command_reset_defaults_get,
                          _param_command_reset_defaults_set);

}
