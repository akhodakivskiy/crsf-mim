#ifndef MIM_SETTINGS_H
#define MIM_SETTINGS_H

#include <esp_err.h>
#include <stdint.h>
#include <stdbool.h>

#include "nav.h"
#include "nav_pitcher.h"
#include "mim_rc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MIM_SETTINGS_MODE_WIFI = 0,
    MIM_SETTINGS_MODE_ETHERNET = 1,
} mim_settings_mode_t;

typedef struct {
    mim_settings_mode_t mode;
    char wifi_ssid[64];
    char wifi_password[64];
    uint16_t skymap_udp_port;
    mim_rc_channel_t engage_channel;
    nav_config_t nav;
    nav_pitcher_config_t pitcher;
} mim_settings_t;

esp_err_t mim_settings_init(void);

esp_err_t mim_settings_load(void);

esp_err_t mim_settings_save(void);

const mim_settings_t* mim_settings_get(void);

esp_err_t mim_settings_reset_to_defaults(void);

esp_err_t mim_settings_set_mode(mim_settings_mode_t mode);

esp_err_t mim_settings_set_wifi(const char *ssid, const char *password);

esp_err_t mim_settings_set_skymap_udp_port(uint16_t port);

esp_err_t mim_settings_set_engage_channel(mim_rc_channel_t channel);

esp_err_t mim_settings_set_nav_N(float N);

esp_err_t mim_settings_set_nav_max_roll_deg(uint8_t max_roll_deg);

esp_err_t mim_settings_set_nav_attack_angle_deg(uint8_t angle);

esp_err_t mim_settings_set_nav_attack_factor(float factor);

esp_err_t mim_settings_set_nav_pitcher_p_gain(float gain);

esp_err_t mim_settings_set_nav_pitcher_i_gain(float gain);

esp_err_t mim_settings_set_nav_pitcher_d_gain(float gain);

esp_err_t mim_settings_set_nav_pitcher_max_rate(float max_rate);

esp_err_t mim_settings_set_nav_pitcher_integral_limit(float limit);

esp_err_t mim_settings_set_nav_pitcher_alpha(float alpha);

esp_err_t mim_settings_set_nav_pitcher_inverted(bool inverted);

#ifdef __cplusplus
}
#endif

#endif
