#include "mim_rc.h"
#include "util.h"

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "libcrsf_def.h"
#include "util.h"

const char *TAG = "MIM_RC";

typedef struct {
    uint16_t value;
    float override;
    mim_rc_override_level_t override_level;
} mim_rc_override_t;

mim_rc_override_t _channels[CRSF_NUM_CHANNELS] = {0};

void mim_rc_reset() {
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) {
        _channels[i].value = CRSF_RC_CHANNELS_CENTER;
        _channels[i].override = 0;
        _channels[i].override_level = MIM_RC_OVERRIDE_LEVEL_NONE;
    }
    _channels[MIM_RC_CHANNEL_THROTTLE].value = CRSF_RC_CHANNELS_MIN;
}

void IRAM_ATTR mim_rc_reset_overrides(mim_rc_override_level_t level) {
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) {
        if (_channels[i].override_level <= level) {
            _channels[i].override = 0;
            _channels[i].override_level = MIM_RC_OVERRIDE_LEVEL_NONE;
        }
    }
}

void mim_rc_set_crsf_channels(const crsf_payload_rc_channels_t *channels) {
    _channels[0].value = channels->ch1;
    _channels[1].value = channels->ch2;
    _channels[2].value = channels->ch3;
    _channels[3].value = channels->ch4;
    _channels[4].value = channels->ch5;
    _channels[5].value = channels->ch6;
    _channels[6].value = channels->ch7;
    _channels[7].value = channels->ch8;
    _channels[8].value = channels->ch9;
    _channels[9].value = channels->ch10;
    _channels[10].value = channels->ch11;
    _channels[11].value = channels->ch12;
    _channels[12].value = channels->ch13;
    _channels[13].value = channels->ch14;
    _channels[14].value = channels->ch15;
    _channels[15].value = channels->ch16;
}

void mim_rc_set_channel(mim_rc_channel_t channel, uint16_t value) {
    assert(channel < CRSF_NUM_CHANNELS);
    _channels[channel].value = value;
}

void IRAM_ATTR mim_rc_set_override(mim_rc_channel_t channel, uint16_t value, mim_rc_override_level_t level) {
    assert(channel < CRSF_NUM_CHANNELS);

    if (_channels[channel].override_level <= level) {
        _channels[channel].override = value;
        _channels[channel].override_level = level;
    }
}

uint16_t mim_rc_get_channel(mim_rc_channel_t channel) {
    assert(channel < CRSF_NUM_CHANNELS);
    return _channels[channel].value;
}

uint16_t IRAM_ATTR mim_rc_get_channel_with_override(mim_rc_channel_t channel) {
    uint16_t result = 0;
    if (_channels[channel].override_level > MIM_RC_OVERRIDE_LEVEL_NONE) {
        uint16_t value = _channels[channel].value;
        uint16_t override = _channels[channel].override;

        // channel value scaled to [-1, 1]
        float value_factor = UTIL_CLAMP((float)(value - CRSF_RC_CHANNELS_MIN) / ((float)CRSF_RC_CHANNELS_RANGE / 2.0f), 0.0f, 1.0f) - 1.0f;

        if (value_factor > 0) {
            result = override + (CRSF_RC_CHANNELS_MAX - override) * value_factor;
        } else {
            result = override + (override - CRSF_RC_CHANNELS_MIN) * value_factor;
        }

        return UTIL_CLAMP(result, CRSF_RC_CHANNELS_MIN, CRSF_RC_CHANNELS_MAX);
    } else {
        return _channels[channel].value;
    }
}
