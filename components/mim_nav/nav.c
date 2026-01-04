#include "nav.h"

#include "la.h"

#include <esp_log.h>
#include <time.h>

static const char *TAG = "NAV";

bool nav_pursuit(la_float N,
                 const nav_ctx_t *ctx,
                 la_float *accel) {
    la_zero(accel, 3);

    // component of LOS along velocity
    la_float r_lon[3] = {0};
    la_float r_lon_norm = la_vec_dot(ctx->range_unit, ctx->vel_i_unit, 3);
    la_scale(ctx->vel_i_unit, r_lon_norm, r_lon, 3);

    // component of LOS perpendicular to velocity
    la_float r_lat[3] = {0};
    la_sub(ctx->range_unit, r_lon, r_lat, 3);

    la_float r_lat_unit[3];
    la_float r_lat_norm = la_vec_unit(r_lat, r_lat_unit, 3);

    if (r_lat_norm == 0.0f) {
        return false;
    } else {
        la_float angular_error = la_asin(la_clamp2(r_lat_norm, 1.0));

        la_float accel_norm = N * ctx->vel_i_norm * angular_error / NAV_G;

        la_scale(r_lat_unit, accel_norm, accel, 3);
    }

    return true;
}

bool nav_pronav(la_float N,
                         const nav_ctx_t *ctx,
                         la_float *accel) {
    la_zero(accel, 3);

    if (ctx->vel_i_norm < 1) {
        ESP_LOGW(TAG, "low speed");
        return false;
    }

    // LOS angular speed
    la_float omega[3];
    la_vec_cross_3(ctx->range_unit, ctx->vel_rel, omega);
    la_scale(omega, 1.0 / ctx->range_norm, omega, 3);

    /* pure pronav */
    la_vec_cross_3(ctx->vel_i_unit, omega, accel);

    /* true pronav */
    // la_vec_cross_3(ctx->range_unit, omega, accel);

    la_scale(accel, -N * ctx->vel_rel_norm, accel, 3);

    return true;
}

bool nav_compute_tail_bias(
    la_float angle_rad, 
    la_float factor,
    const nav_ctx_t *ctx, 
    la_float *accel) {

    la_zero(accel, 3);

    la_float r_lon_unit = la_vec_dot(ctx->range_unit, ctx->vel_t_unit, 3);
    la_float tail_angle = la_acos(la_clamp2(r_lon_unit, 1));

    if (tail_angle <= angle_rad) {
        // already in tail cone
        return false;
    }

    la_float rot_unit[3] = {0};
    la_vec_cross_3(ctx->range_unit, ctx->vel_t_unit, rot_unit);
    la_float rot_unit_norm = la_vec_unit(rot_unit, rot_unit, 3);

    if (rot_unit_norm == 0.0f) {
        return false;
    }

    la_float tail_angle_delta = tail_angle - angle_rad;

    la_float v[3] = {0};
    la_scale(ctx->range_unit, -1, v, 3);

    la_float range_unit_rot[3] = {0};
    la_vec_rotate_rodrigues(ctx->range_unit, rot_unit, tail_angle_delta, 
                            range_unit_rot);

    la_float range_unit_error[3] = {0}; 
    la_sub(range_unit_rot, ctx->range_unit, range_unit_error, 3);

    la_scale(range_unit_error, - factor * ctx->vel_i_norm, accel, 3);

    return true;
}

nav_type_t nav_compute_accel(
    const nav_config_t *g,
    const nav_ctx_t *ctx,
    la_float *accel) {

    nav_type_t type = NAV_NONE;
    if (ctx->vel_closing < 0 && ctx->range_norm > 100) {
        if (nav_pursuit(g->N, ctx, accel)) {
            type = NAV_PURSUIT;
        }
    } else {
        if (nav_pronav(g->N, ctx, accel)) {
            type = NAV_PRONAV;

            la_float a_bias[3];
            la_float approach_angle_rad = NAV_DEG_TO_RAD(g->attack_angle_deg);
            nav_compute_tail_bias(approach_angle_rad, g->attack_factor, ctx, a_bias);

            la_add(accel, a_bias, accel, 3);
        }
    }

    return type;
}

bool nav_compute_tgo_zem(
    const nav_ctx_t *ctx,
    la_float *tgo,
    la_float *zem) {

    bool result = false;

    if (la_abs(ctx->range_norm) < 1 || 
        ctx->vel_rel_norm == 0.0f || 
        ctx->vel_closing < 0.0) {
        *tgo = 0.0;
    } else if (ctx->vel_closing == 0.0f) {
        *tgo = la_vec_dot(ctx->range, ctx->vel_rel, 3) / la_pow(ctx->vel_rel_norm, 2);
    } else {
        *tgo = ctx->range_norm / ctx->vel_closing;
    }

    if (*tgo > 0) {
        la_float zem_vec[3] = {0};
        la_scale(ctx->vel_rel, *tgo, zem_vec, 3);
        la_add(zem_vec, ctx->range, zem_vec, 3);
        *zem= la_vec_norm(zem_vec, 3);

        result = true;
    }

    return result;
}

void nav_state_to_ned(const nav_state_t *state_ref, 
                      const nav_state_t *state, 
                      la_float *ned) {
    double dlat = state->lat - state_ref->lat;
    double dlon = state->lon - state_ref->lon;
    double dalt = state->alt - state_ref->alt;

    double lat_avg = (state_ref->lat + state->lat) / 2;
    double radius = NAV_R_EARTH + (state_ref->alt + state->alt) / 2;

    ned[0] = NAV_DEG_TO_RAD(dlat) * radius;
    ned[1] = NAV_DEG_TO_RAD(dlon) * radius * la_cos(NAV_DEG_TO_RAD(lat_avg));
    ned[2] = -dalt;
}

void nav_state_advance(nav_state_t *state, int64_t timestamp_us) {
    if (state->timestamp_us > 0 && timestamp_us > state->timestamp_us) {
        float dt_s = (float)(timestamp_us - state->timestamp_us) / 1000000.0f;

        la_float radius = NAV_R_EARTH + state->alt;

        la_float dlat_rad = state->vel_north * dt_s / radius;
        la_float dlon_rad = state->vel_east * dt_s / (radius * la_cos(state->lat));

        state->lat += NAV_RAD_TO_DEG(dlat_rad);
        state->lon += NAV_RAD_TO_DEG(dlon_rad);
        state->alt += state->vel_up * dt_s;
    }
}

void nav_state_to_ctx(const nav_state_t *state_i, 
                      const nav_state_t *state_t, 
                      nav_ctx_t *ctx) {
    memset(ctx, 0, sizeof(nav_ctx_t));
    // interceptor and target velocities in NED (down is positive)
    nav_state_to_ned(state_i, state_t, ctx->range);

    ctx->vel_i[0] = state_i->vel_north;
    ctx->vel_i[1] = state_i->vel_east;
    ctx->vel_i[2] = -state_i->vel_up;
    ctx->vel_t[0] = state_t->vel_north;
    ctx->vel_t[1] = state_t->vel_east;
    ctx->vel_t[2] = -state_t->vel_up;

    la_sub(ctx->vel_t, ctx->vel_i, ctx->vel_rel, 3);

    ctx->range_norm = la_vec_unit(ctx->range, ctx->range_unit, 3);
    ctx->vel_i_norm = la_vec_unit(ctx->vel_i, ctx->vel_i_unit, 3);
    ctx->vel_t_norm = la_vec_unit(ctx->vel_t, ctx->vel_t_unit, 3);
    ctx->vel_rel_norm = la_vec_unit(ctx->vel_rel, ctx->vel_rel_unit, 3);

    ctx->vel_closing = -la_vec_dot(ctx->vel_rel, ctx->range_unit, 3);
}

void nav_compute_command(
    const nav_config_t *g, 
    const nav_state_t *state_i, 
    const nav_state_t *state_t,
    nav_command_t *command) {
    // zero command
    memset(command, 0, sizeof(nav_command_t));
    command->type = NAV_NONE;

    nav_ctx_t ctx;
    nav_state_to_ctx(state_i, state_t, &ctx);

    if (ctx.range_norm < 1) {
        ESP_LOGW(TAG, "low range");
        return;
    }

    // compute commanded acceleration
    la_float accel[3];
    nav_type_t type = nav_compute_accel(g, &ctx, accel);

    // compute longitudinal, lateral and vertical acceleration components
    la_float unit_down[3] = { 0, 0, 1 };
    la_float unit_lat[3] = {0};
    la_float unit_lon[3] = {0};

    la_copy(ctx.vel_i_unit, unit_lon, 3);

    la_vec_cross_3(unit_down, unit_lon, unit_lat);
    la_float unit_lat_norm = la_vec_unit(unit_lat, unit_lat, 3);
    if (unit_lat_norm == 0.0f) {
        ESP_LOGW(TAG, "low lat acc");
    }

    command->type = type;
    command->range = ctx.range_norm;
    command->range_hor = la_sqrt(ctx.range[0] * ctx.range[0] + ctx.range[1] * ctx.range[1]);
    command->range_ver = ctx.range[2];
    command->accel_lon = la_vec_dot(accel, unit_lon, 3), 
    command->accel_lat = la_vec_dot(accel, unit_lat, 3), 
    command->accel_ver = -la_vec_dot(accel, unit_down, 3);

    la_float tgo, zem;
    if (nav_compute_tgo_zem(&ctx, &tgo, &zem)) {
        command->time_to_go_s = tgo;
        command->zero_effort_miss_m = zem;
    } else {
        command->time_to_go_s = 0;
        command->zero_effort_miss_m = 0;
    }
}
