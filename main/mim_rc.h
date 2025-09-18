#ifndef MIM_RC_H
#define MIM_RC_H

#include <sdkconfig.h>
#include <stdint.h>

typedef enum {
    MIM_RC_OVERRIDE_LEVEL_NONE,
    MIM_RC_OVERRIDE_LEVEL_GUIDANCE,
    MIM_RC_OVERRIDE_LEVEL_TEST,
} mim_rc_override_level_t;

typedef enum {
    MIM_RC_CHANNEL_ROLL = 1,
    MIM_RC_CHANNEL_PITCH = 2,
    MIM_RC_CHANNEL_THROTTLE = 3,
    MIM_RC_CHANNEL_YAW = 4,
} mim_rc_channel_t;

// clear overrides from all channels
void mim_rc_clear_overrides();

// override value must be in [-1.0, 1.0] range
void mim_rc_override_channel(uint8_t channel, uint16_t value, mim_rc_override_level_t level);

uint16_t mim_rc_apply_override(uint8_t channel, uint16_t value);

#ifdef CONFIG_CRSF_MIM_CHANNELS_RESPONSE
void mim_rc_set_4_channels(const uint16_t *channels, uint8_t channel_count);
void mim_rc_get_4_channels(uint16_t *channels, uint8_t channel_count);
#endif

#endif
