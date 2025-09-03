#include "nav_guidance.h"

#include "nav_vector.h"

#include <esp_log.h>
#include <float.h>

static const char *TAG = "NAV";

#define NAV_PI 3.14159265358979323846
#define NAV_DEG_TO_RAD (NAV_PI / 180.0)
#define NAV_R_EARTH 6371000.0
#define NAV_DEG_TO_LEN (NAV_DEG_TO_RAD * NAV_R_EARTH)
#define NAV_G 9.81

#define NAV_GUIDANCE_COMMAND_SET_ZERO(C) \
   C->accel_lateral = 0; \
   C->accel_vertical = 0; \
   C->type = NAV_GUIDANCE_NONE;

#define _NAV_CLAMP(X, MIN, MAX) ((X < MIN) ? MIN : ((X > MAX) ? MAX : X))
#define _NAV_CLAMP2(X, Y) _NAV_CLAMP(X, -Y, Y)
#define _NAV_RAD_TO_DEG(X) (X * 180 / NAV_PI)

void nav_state_to_ned(const nav_guidance_state_t *state_ref, 
                      const nav_guidance_state_t *state, 
                      nav_vector_t *ned) {
    double dlat = state->lat - state_ref->lat;
    double dlon = state->lon - state_ref->lon;
    double dalt = state->alt - state_ref->alt;

    ned->x = dlat * NAV_DEG_TO_LEN;
    ned->y = dlon * NAV_DEG_TO_LEN * cos(state_ref->lat * NAV_DEG_TO_RAD);
    ned->z = -dalt;
}

bool nav_guidance_pursuit(
    double N,
    const nav_vector_t *range,
    const nav_vector_t *vel_i,
    nav_vector_t *accel) {

    nav_vector_set_zero(accel);

    // range vector and its unit
    double range_norm = 0.0;
    nav_vector_t range_unit = nav_vector_unit(range, &range_norm);

    // interceptor velocity unit vector
    double vel_i_norm = 0.0;
    nav_vector_t vel_i_unit = nav_vector_unit(vel_i, &vel_i_norm);

    if (range_norm < 1 || vel_i_norm < DBL_EPSILON) {
        ESP_LOGI(TAG, "low range");
        return false;
    }

    // component of LOS along velocity
    nav_vector_t r_along_vel_i = nav_vector_scale(
        &vel_i_unit, 
        nav_vector_dot(&range_unit, &vel_i_unit));

    // component of LOS perpendicular to velocity
    nav_vector_t r_perp_vel_i = nav_vector_subtract(&range_unit, &r_along_vel_i);

    double r_perp_vel_i_norm = 0.0;
    nav_vector_t r_perp_vel_i_unit = nav_vector_unit(&r_perp_vel_i, &r_perp_vel_i_norm);

    if (r_perp_vel_i_norm > DBL_EPSILON) {
        double angular_error = asin(_NAV_CLAMP(r_perp_vel_i_norm, -1.0, 1.0));

        double accel_norm = N * vel_i_norm * angular_error;

        *accel = nav_vector_scale(&r_perp_vel_i_unit, accel_norm);
    }

    return true;
}

bool nav_guidance_pronav_true(
    double N,
    const nav_vector_t *range,
    const nav_vector_t *vel_i, const nav_vector_t *vel_t,
    nav_vector_t *accel) {

    nav_vector_set_zero(accel);

    // interceptor velocity norm
    double vel_i_norm = 0.0;
    nav_vector_t vel_i_unit = nav_vector_unit(vel_i, &vel_i_norm);
    if (vel_i_norm < DBL_EPSILON) {
        ESP_LOGI(TAG, "vel_i_norm=%lf", vel_i_norm);
        return false;
    }

    double range_norm_sq = nav_vector_dot(range, range);

    // relative speed
    nav_vector_t v = nav_vector_subtract(vel_t, vel_i);
    double v_norm = nav_vector_norm(&v);

    if (range_norm_sq < 1 || v_norm < DBL_EPSILON) {
        ESP_LOGI(TAG, "range_norm_sq=%lf, v_norm=%lf", range_norm_sq, v_norm);
        return false;
    }

    // LOS angular speed
    nav_vector_t omega = nav_vector_cross(range, &v);
    nav_vector_scale_self(&omega, 1.0 / range_norm_sq);

    // ProNav calculation
    *accel = nav_vector_cross(&vel_i_unit, &omega);
    nav_vector_scale_self(accel, -N * v_norm);

    return true;
}

nav_guidance_type_t nav_guidance_compute_accel(
    const nav_guidance_t *g,
    const nav_vector_t *range,
    const nav_vector_t *vel_i, const nav_vector_t *vel_t,
    nav_vector_t *accel) {

    double range_norm = 0.0;
    nav_vector_t range_unit = nav_vector_unit(range, &range_norm);
    nav_vector_t v = nav_vector_subtract(vel_t, vel_i);
    double closing_speed = -nav_vector_dot(&v, &range_unit);

    if (closing_speed < 0 && range_norm > 100) {
        if (nav_guidance_pursuit(g->N, range, vel_i, accel)) {
            return NAV_GUIDANCE_PURSUIT;
        }
    } else {
        if (nav_guidance_pronav_true(g->N, range, vel_i, vel_t, accel)) {
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
    NAV_GUIDANCE_COMMAND_SET_ZERO(command)

    // target NED coordinates relatively to the interceptor
    nav_vector_t range = {0};
    nav_state_to_ned(i, t, &range);

    // interceptor and target velocities in NED (down is positive)
    nav_vector_t vel_i = { .x = i->vel_north, .y = i->vel_east, .z = -i->vel_up, };
    nav_vector_t vel_t = { .x = t->vel_north, .y = t->vel_east, .z = -t->vel_up, };

    // compute commanded acceleration perpendicular to the interceptor speed vector
    command->type = nav_guidance_compute_accel(g, &range, &vel_i, &vel_t, &command->accel);

    // compute lateral and vertical acceleration components
    nav_vector_t vel_horizontal = { .x = vel_i.x, .y = vel_i.y, .z = 0.0 };
    nav_vector_t down_unit = { .x = 0, .y = 0, .z = 1.0 };

    nav_vector_t lateral_unit = nav_vector_cross(&down_unit, &vel_horizontal);
    double lateral_unit_norm = nav_vector_norm(&lateral_unit);
    if (lateral_unit_norm < DBL_EPSILON) {
        return;
    }
    nav_vector_scale_self(&lateral_unit, 1.0 / lateral_unit_norm);

    command->accel_lateral = nav_vector_dot(&command->accel, &lateral_unit);
    command->accel_vertical = nav_vector_dot(&command->accel, &down_unit);
    command->roll_deg = _NAV_RAD_TO_DEG(atan2(command->accel_lateral, NAV_G));
    command->pitch_deg = _NAV_RAD_TO_DEG(-asin(_NAV_CLAMP(command->accel_vertical / NAV_G, -1.0, 1.0)));
    command->roll_cmd = _NAV_CLAMP2(command->roll_deg / g->max_roll_deg, 1.0);
    command->pitch_cmd = _NAV_CLAMP2(command->pitch_deg / g->max_pitch_deg, 1.0);
}
