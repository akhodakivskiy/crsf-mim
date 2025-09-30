#ifndef MIM_RC_H
#define MIM_RC_H

#include <sdkconfig.h>
#include <stdint.h>

#include "libcrsf_payload.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MIM_RC_OVERRIDE_LEVEL_NONE,
    MIM_RC_OVERRIDE_LEVEL_GUIDANCE,
    MIM_RC_OVERRIDE_LEVEL_TEST,
} mim_rc_override_level_t;

typedef enum : uint8_t {
    MIM_RC_CHANNEL_ROLL = 0,
    MIM_RC_CHANNEL_PITCH = 1,
    MIM_RC_CHANNEL_THROTTLE = 2,
    MIM_RC_CHANNEL_YAW = 3,
    MIM_RC_CHANNEL_5 = 4,
    MIM_RC_CHANNEL_6 = 5,
    MIM_RC_CHANNEL_7 = 6,
    MIM_RC_CHANNEL_8 = 7,
    MIM_RC_CHANNEL_9 = 8,
    MIM_RC_CHANNEL_10 = 9,
    MIM_RC_CHANNEL_11 = 10,
    MIM_RC_CHANNEL_12 = 11,
    MIM_RC_CHANNEL_13 = 12,
    MIM_RC_CHANNEL_14 = 13,
    MIM_RC_CHANNEL_15 = 14,
    MIM_RC_CHANNEL_16 = 15,
    MIM_RC_CHANNEL__MAX = 16,
} mim_rc_channel_t;

void mim_rc_reset();
void mim_rc_reset_overrides();

void mim_rc_set_crsf_channels(const crsf_payload_rc_channels_t *channels);
void mim_rc_set_channel(mim_rc_channel_t channel, uint16_t value);
void mim_rc_set_override(mim_rc_channel_t channel, uint16_t value, mim_rc_override_level_t level);

uint16_t mim_rc_get_channel(mim_rc_channel_t channel);
uint16_t mim_rc_get_channel_with_override(mim_rc_channel_t channel);

#ifdef __cplusplus
}
#endif

#endif
