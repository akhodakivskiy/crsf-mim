#include "mim_rc.h"
#include "util.h"

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "libcrsf_def.h"
#include "la.h"
#include "util.h"

const char *TAG = "MIM_RC";

typedef struct {
    float value;
    float rate;
    int64_t last_rate_time;
    mim_rc_override_level_t level;
} mim_rc_override_t;

mim_rc_override_t _overrides[16] = {0};

void IRAM_ATTR mim_rc_clear_overrides() {
    memset(_overrides, 0, sizeof(_overrides));
}

void IRAM_ATTR mim_rc_override_channel(uint8_t channel, uint16_t value, mim_rc_override_level_t level) {
    if (_overrides[channel - 1].level <= level) {
        _overrides[channel - 1].value = value;
        _overrides[channel - 1].rate = 0.0;
        _overrides[channel - 1].last_rate_time = 0.0;
        _overrides[channel - 1].level = level;
    }
}

void mim_rc_override_channel_rate(uint8_t channel, float rate, mim_rc_override_level_t level, int64_t time) {
    if (_overrides[channel - 1].level <= level) {
        _overrides[channel - 1].rate = rate;
        _overrides[channel - 1].last_rate_time = time;
        if (_overrides[channel - 1].value == 0) {
            _overrides[channel - 1].value = CRSF_RC_CHANNELS_CENTER;
        }
        _overrides[channel - 1].level = level;
    }
}

uint16_t IRAM_ATTR mim_rc_apply_override(uint8_t channel, uint16_t value) {
    uint16_t result = 0;
    if (_overrides[channel - 1].level > MIM_RC_OVERRIDE_LEVEL_NONE) {
        uint16_t override = _overrides[channel - 1].value;

        float value_range = CRSF_RC_CHANNELS_TICKS_TO_RANGE(value);

        if (value_range > 0) {
            result = override + (CRSF_RC_CHANNELS_MAX - override) * value_range;
        } else {
            result = override + (override - CRSF_RC_CHANNELS_MIN) * value_range;
        }

        return la_clamp(result, CRSF_RC_CHANNELS_MIN, CRSF_RC_CHANNELS_MAX);
    } else {
        return value;
    }
}

#ifdef CONFIG_CRSF_MIM_CHANNELS_RESPONSE

uint16_t _channels[4];

void mim_rc_set_4_channels(const uint16_t *channels, uint8_t channel_count) {
    assert(channel_count == 4);
    for (int i = 0; i < channel_count; i++) {
        _channels[i] = channels[i];
    }
}

void mim_rc_get_4_channels(uint16_t *channels, uint8_t channel_count) {
    assert(channel_count == 4);
    for (int i = 0; i < channel_count; i++) {
        channels[i] = _channels[i];
    }
}

#endif
