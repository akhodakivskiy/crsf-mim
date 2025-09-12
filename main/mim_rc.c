#include "mim_rc.h"
#include "util.h"

#include <string.h>
#include <freertos/FreeRTOS.h>

#include "libcrsf_def.h"
#include "la.h"

#define CRSF_RC_CHANNELS_TICKS_TO_RANGE(x) \
    ((x - CRSF_RC_CHANNELS_CENTER) / CRSF_RC_CHANNELS_RANGE * 2)
#define CRSF_RC_CHANNELS_RANGE_TO_TICKS(x) \
    (x * CRSF_RC_CHANNELS_RANGE / 2 + CRSF_RC_CHANNELS_CENTER)

typedef struct {
    float value;
    mim_rc_override_level_t level;
} mim_rc_override_t;

mim_rc_override_t _overrides[16] = {0};

void IRAM_ATTR mim_rc_clear_overrides() {
    memset(_overrides, 0, sizeof(_overrides));
}

void IRAM_ATTR mim_rc_override_channel(uint8_t channel, float value, mim_rc_override_level_t level) {
    if (_overrides[channel - 1].level <= level) {
        _overrides[channel - 1].value = value;
        _overrides[channel - 1].level = level;
    }
}

uint16_t IRAM_ATTR mim_rc_apply_override(uint8_t channel, uint16_t value) {
    uint16_t result = 0;
    if (_overrides[channel - 1].level > MIM_RC_OVERRIDE_LEVEL_NONE) {
        float g = _overrides[channel - 1].value;
        if (g > 0) {
            result = value + (CRSF_RC_CHANNELS_MAX - value) * g;
        } else {
            result = value + (value - CRSF_RC_CHANNELS_MIN) * g;
        }
        return la_clamp(result, CRSF_RC_CHANNELS_MIN, CRSF_RC_CHANNELS_MAX);
    } else {
        return value;
    }
}
