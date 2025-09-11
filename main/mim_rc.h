#ifndef MIM_RC_H
#define MIM_RC_H

#include <stdint.h>

typedef enum {
    MIM_RC_OVERRIDE_LEVEL_NONE,
    MIM_RC_OVERRIDE_LEVEL_GUIDANCE,
    MIM_RC_OVERRIDE_LEVEL_TEST,
} mim_rc_override_level_t;

typedef enum {
    MIM_RC_CHANNEL_ROLL = 1,
    MIM_RC_CHANNEL_PITCH = 2,
} mim_rc_channel_t;

// clear overrides from all channels
void mim_rc_clear_overrides();

// override value must be in [-1.0, 1.0] range
void mim_rc_override_channel(uint8_t channel, float value, mim_rc_override_level_t level);

uint16_t mim_rc_apply_override(uint8_t channel, uint16_t value);

#endif
