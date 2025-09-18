#include "nav_guidance.h"

#include "la.h"

#include <esp_log.h>

static const char *TAG = "NAV";

#define NAV_PI 3.14159265358979323846
#define NAV_DEG_TO_RAD (NAV_PI / 180.0)
#define NAV_R_EARTH 6371000.0
#define NAV_DEG_TO_LEN (NAV_DEG_TO_RAD * NAV_R_EARTH)
#define NAV_G 9.81

#define _NAV_RAD_TO_DEG(X) (X * 180 / NAV_PI)

void nav_state_to_ned(const nav_guidance_state_t *state_ref, 
                      const nav_guidance_state_t *state, 
                      la_float *ned) {
    double dlat = state->lat - state_ref->lat;
    double dlon = state->lon - state_ref->lon;
    double dalt = state->alt - state_ref->alt;

    ned[0] = dlat * NAV_DEG_TO_LEN;
    ned[1] = dlon * NAV_DEG_TO_LEN * la_cos(state_ref->lat * NAV_DEG_TO_RAD);
    ned[2] = -dalt;
}

bool nav_guidance_pursuit(
    la_float N,
    const la_float *range,
    const la_float *vel_i,
    la_float *accel) {

    la_zero(accel, 3);

    // range vector and its unit
    la_float range_unit[3] = {0};
    la_float range_norm = la_vec_unit(range, range_unit, 3);
    if (range_norm < 1.0) {
        ESP_LOGW(TAG, "PURSUIT: low range");
        return false;
    }


    // interceptor velocity unit vector
    la_float vel_i_unit[3] = {0};
    la_float vel_i_norm = la_vec_unit(vel_i, vel_i_unit, 3);

    if (vel_i_norm == 0.0) {
        ESP_LOGW(TAG, "PURSUIT: low interceptor velocity");
        return false;
    }

    // component of LOS along velocity
    la_float r_along_vel_i[3] = {0};
    la_scale(vel_i_unit, la_vec_dot(range_unit, vel_i_unit, 3), r_along_vel_i, 3);

    // component of LOS perpendicular to velocity
    la_float r_perp_vel_i[3] = {0};
    la_sub(range_unit, r_along_vel_i, r_perp_vel_i, 3);

    la_float r_perp_vel_i_unit[3];
    la_float r_perp_vel_i_norm = la_vec_unit(r_perp_vel_i, r_perp_vel_i_unit, 3);

    if (r_perp_vel_i_norm > DBL_EPSILON) {
        la_float angular_error = la_asin(la_clamp2(r_perp_vel_i_norm, 1.0));

        la_float accel_norm = vel_i_norm * angular_error;

        la_scale(r_perp_vel_i_unit, accel_norm, accel, 3);
    }

    return true;
}

bool nav_guidance_pronav(
    la_float N,
    const la_float *range,
    const la_float *vel_i, 
    const la_float *vel_t,
    la_float *accel) {

    la_zero(accel, 3);

    // interceptor velocity norm
    la_float vel_i_unit[3];
    la_float vel_i_norm = la_vec_unit(vel_i, vel_i_unit, 3);
    if (vel_i_norm < LA_EPSILON) {
        ESP_LOGW(TAG, "PRONAV: low speed");
        return false;
    }

    la_float range_unit[3];
    la_float range_norm = la_vec_unit(range, range_unit, 3);
    la_float range_norm_sq = range_norm * range_norm;

    if (range_norm < LA_EPSILON) {
        ESP_LOGW(TAG, "PRONAV: low range");
        return false;
    }

    // relative speed
    la_float v[3];
    la_sub(vel_t, vel_i, v, 3);
    la_float v_norm = la_vec_norm(v, 3);

    if (v_norm < DBL_EPSILON) {
        ESP_LOGW(TAG, "PRONAV: low relative speed");
        return false;
    }

    // LOS angular speed
    la_float omega[3];
    la_vec_cross_3(range, v, omega);
    la_scale(omega, 1.0 / range_norm_sq, omega, 3);

    // ProNav calculation

    /* pure pronav */
    la_vec_cross_3(vel_i_unit, omega, accel);

    /* true pronav */
    // la_vec_cross_3(range_unit, omega, accel);

    la_scale(accel, -N * v_norm, accel, 3);

    return true;
}

nav_guidance_type_t nav_guidance_compute_accel(
    const nav_guidance_t *g,
    const la_float *range,
    const la_float *vel_i, 
    const la_float *vel_t,
    la_float *accel) {

    la_float range_unit[3] = {0};
    la_float range_norm = la_vec_unit(range, range_unit, 3);

    la_float v[3] = {0};
    la_sub(vel_t, vel_i, v, 3);
    la_float closing_speed = -la_vec_dot(v, range_unit, 3);

    ESP_LOGI(TAG, "closing_speed=%f, range=%f", closing_speed, range_norm);

    if (closing_speed < 0 && range_norm > 100) {
        if (nav_guidance_pursuit(g->N, range, vel_i, accel)) {
            return NAV_GUIDANCE_PURSUIT;
        }
    } else {
        if (nav_guidance_pronav(g->N, range, vel_i, vel_t, accel)) {
            return NAV_GUIDANCE_PRONAV;
        }
    }

    return NAV_GUIDANCE_NONE;
}

void nav_guidance_compute_command(
    const nav_guidance_t *g, 
    const nav_guidance_state_t *i, 
    const nav_guidance_state_t *t,
    nav_guidance_command_t *command) {
    // zero command
    memset(command, 0, sizeof(nav_guidance_command_t));

    // target NED coordinates relatively to the interceptor
    la_float range[3] = {0};
    nav_state_to_ned(i, t, range);

    // interceptor and target velocities in NED (down is positive)
    la_float vel_i[3] = { i->vel_north, i->vel_east, -i->vel_up, };
    la_float vel_t[3] = { t->vel_north, t->vel_east, -t->vel_up, };

    // compute commanded acceleration perpendicular to the interceptor speed vector
    command->type = nav_guidance_compute_accel(g, range, vel_i, vel_t, command->accel);

    // compute lateral and vertical acceleration components
    la_float vel_horizontal[3] = { vel_i[0], vel_i[1], 0 };
    la_float down_unit[3] = { 0, 0, 1 };

    la_float lateral_unit[3];
    la_vec_cross_3(down_unit, vel_horizontal, lateral_unit);
    float lateral_unit_norm = la_vec_norm(lateral_unit, 3);
    if (lateral_unit_norm < LA_EPSILON) {
        return;
    }
    la_scale(lateral_unit, 1.0 / lateral_unit_norm, lateral_unit, 3);

    command->accel_lateral = la_vec_dot(command->accel, lateral_unit, 3);
    command->accel_vertical = la_vec_dot(command->accel, down_unit, 3);
    command->roll_deg = la_clamp2(_NAV_RAD_TO_DEG(la_atan2(command->accel_lateral, NAV_G)), g->max_roll_deg);
    command->pitch_deg = la_clamp2(_NAV_RAD_TO_DEG(-la_asin(la_clamp2(command->accel_vertical / NAV_G, 1.0))), g->max_pitch_deg);
    command->roll_cmd = la_clamp2(command->roll_deg / g->max_roll_deg, 1.0);
    command->pitch_cmd = la_clamp2(command->pitch_deg / g->max_pitch_deg, 1.0);
}
