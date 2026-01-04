#ifndef NAV_PITCHER_H
#define NAV_PITCHER_H

#include "la.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NAV_PITCHER_CONFIG_DEFAULT() \
{ \
    .kp = 0.1, \
    .ki = 0.005, \
    .kd = 0.02, \
    .max_rate = 0.5, \
    .integral_limit = 0.5, \
    .alpha = 0.3 \
}

typedef struct {
    la_float kp;
    la_float kd;
    la_float ki;
    la_float max_rate;
    la_float integral_limit;
    la_float alpha;
    bool inverted;
} nav_pitcher_config_t;

typedef struct {
    la_float prev_vel_ms;
    la_float a_cmd_filtered;
    la_float a_integral;
    la_float last_pitch;
} nav_pitcher_state_t;

void nav_pitcher_init(nav_pitcher_state_t *state);

la_float nav_pitcher_update(const nav_pitcher_config_t *cfg, 
                            nav_pitcher_state_t *state,
                            la_float dt_s,
                            la_float vel_ms,
                            la_float a_cmd);

#define nav_pitcher_reset nav_pitcher_init

#ifdef __cplusplus
}
#endif

#endif
