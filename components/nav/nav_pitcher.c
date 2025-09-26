#include "nav_pitcher.h"
#include "la.h"

void nav_pitcher_init(nav_pitcher_state_t *state) {
    state->prev_vel_ms = 0;
    state->a_cmd_filtered = 0;
    state->a_integral = 0;
    state->last_pitch = 0;
}

la_float nav_pitcher_update(const nav_pitcher_config_t *cfg, 
                            nav_pitcher_state_t *state,
                            la_float dt_s,
                            la_float vel_ms,
                            la_float a_cmd) {
    if (dt_s <= 0) {
        return 0;
    }

    state->a_cmd_filtered = (1 - cfg->alpha) * state->a_cmd_filtered + 
        cfg->alpha * a_cmd;

    la_float gain_p = cfg->kp * state->a_cmd_filtered * dt_s;
    la_float gain_d = -cfg->kd * (vel_ms - state->prev_vel_ms) * dt_s;

    state->prev_vel_ms = vel_ms;

    if (la_abs(a_cmd) > 0.1) {
        state->a_integral += a_cmd * dt_s;
        state->a_integral = la_clamp2(state->a_integral, cfg->integral_limit);
    } else {
        state->a_integral *= (1.0 - 0.05 * dt_s);
    }

    la_float gain_i = cfg->ki * state->a_integral * dt_s;

    la_float gain = la_clamp2(gain_p + gain_d + gain_i, cfg->max_rate * dt_s);

    int sign = cfg->inverted ? -1 : 1;

    la_float pitch = state->last_pitch + (gain * sign);

    if (la_abs(pitch) > 1) {
        state->a_integral *= (1.0 - 0.1 * dt_s);
    }

    state->last_pitch = la_clamp2(pitch, 1.0);

    return state->last_pitch;
}
