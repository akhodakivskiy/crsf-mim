#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MIM_SETTINGS_MODE_WIFI = 0,
    MIM_SETTINGS_MODE_ETHERNET = 1,
} mim_settings_mode_t;

typedef struct {
    float N;
    uint8_t max_roll_deg;
    uint8_t max_pitch_deg;
    bool pitch_invert;
} mim_settings_guidance_t;

typedef struct {
    mim_settings_mode_t mode;
    char wifi_ssid[64];
    char wifi_password[64];
    uint16_t skymap_udp_port;
    uint8_t engage_channel;
    mim_settings_guidance_t guidance;
} mim_settings_t;

esp_err_t mim_settings_init(void);

esp_err_t mim_settings_load(void);

esp_err_t mim_settings_save(void);

const mim_settings_t* mim_settings_get(void);

esp_err_t mim_settings_set_mode(mim_settings_mode_t mode);

esp_err_t mim_settings_set_wifi(const char *ssid, const char *password);

esp_err_t mim_settings_set_skymap_udp_port(uint16_t port);

esp_err_t mim_settings_set_engage_channel(uint8_t channel);

esp_err_t mim_settings_set_guidance_N(float N);

esp_err_t mim_settings_set_guidance_max_roll_deg(uint8_t max_roll_deg);

esp_err_t mim_settings_set_guidance_max_pitch_deg(uint8_t max_pitch_deg);

esp_err_t mim_settings_set_guidance_pitch_invert(bool invert);

esp_err_t mim_settings_reset_to_defaults(void);

#ifdef __cplusplus
}
#endif
