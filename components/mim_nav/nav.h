#ifndef NAV_H
#define NAV_H

#include "la.h"

#include <stdbool.h>

#define NAV_PI 3.14159265358979323846
#define NAV_RAD_TO_DEG(x) (x * 180.0 / NAV_PI)
#define NAV_DEG_TO_RAD(x) (x * NAV_PI / 180.0)
#define NAV_R_EARTH 6371000.0
#define NAV_G 9.81

#define NAV_CONFIG_DEFAULT() { \
    .N = 3, \
    .attack_angle_deg = 20, \
    .attack_factor = 1, \
}

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float N;
    float max_roll_deg;
    float attack_angle_deg;
    float attack_factor;
} nav_config_t;

typedef enum {
    NAV_NONE,
    NAV_PURSUIT,
    NAV_PRONAV,
} nav_type_t;

typedef struct {
    nav_type_t type;
    la_float range;
    la_float range_hor;
    la_float range_ver;
    la_float accel_lon;
    la_float accel_lat;
    la_float accel_ver;
    la_float time_to_go_s;
    la_float zero_effort_miss_m;
} nav_command_t;

typedef struct {
    int64_t timestamp_us;

    double lat;
    double lon;
    la_float alt;
    la_float vel_north;
    la_float vel_east;
    la_float vel_up;
} nav_state_t;

typedef struct {
    la_float range[3];
    la_float range_unit[3];
    la_float range_norm;

    la_float vel_i[3];
    la_float vel_i_unit[3];
    la_float vel_i_norm;

    la_float vel_t[3];
    la_float vel_t_unit[3];
    la_float vel_t_norm;

    la_float vel_rel[3];
    la_float vel_rel_unit[3];
    la_float vel_rel_norm;

    la_float vel_closing;
} nav_ctx_t;

bool nav_pursuit(
    la_float N,
    const nav_ctx_t *ctx,
    la_float *accel);

bool nav_pronav(
    la_float N,
    const nav_ctx_t *ctx,
    la_float *accel);

bool nav_compute_tail_bias(
    la_float angle_rad,
    la_float factor,
    const nav_ctx_t *ctx, 
    la_float *accel);

nav_type_t nav_compute_accel(
    const nav_config_t *g,
    const nav_ctx_t *ctx,
    la_float *accel);

bool nav_compute_tgo_zem(
    const nav_ctx_t *ctx,
    la_float *tgo,
    la_float *zem);

void nav_state_to_ned(
    const nav_state_t *state_ref, 
    const nav_state_t *state, 
    la_float *ned);

void nav_state_to_ctx(
    const nav_state_t *state_i, 
    const nav_state_t *state_t, 
    nav_ctx_t *ctx);

void nav_state_advance(nav_state_t *state, int64_t timestamp_us);

void nav_compute_command(
    const nav_config_t *g, 
    const nav_state_t *state_i, 
    const nav_state_t *state_t,
    nav_command_t *command);

#ifdef __cplusplus
}
#endif

#endif
