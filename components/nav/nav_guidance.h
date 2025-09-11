#ifndef NAV_GUIDANCE_H
#define NAV_GUIDANCE_H

#include "la.h"

#include <stdbool.h>

typedef struct {
    float N;
    float max_roll_deg;
    float max_pitch_deg;
} nav_guidance_t;

typedef enum {
    NAV_GUIDANCE_NONE,
    NAV_GUIDANCE_PURSUIT,
    NAV_GUIDANCE_PRONAV,
} nav_guidance_type_t;

typedef struct {
    nav_guidance_type_t type;
    la_float accel[3];
    la_float accel_lateral;
    la_float accel_vertical;
    la_float roll_deg;
    la_float pitch_deg;
    la_float roll_cmd;
    la_float pitch_cmd;
} nav_guidance_command_t;

typedef struct {
    double lat;
    double lon;
    la_float alt;
    la_float vel_north;
    la_float vel_east;
    la_float vel_up;
} nav_guidance_state_t;

void nav_state_to_ned(const nav_guidance_state_t *state_ref, 
                      const nav_guidance_state_t *state, 
                      la_float *ned);

bool nav_guidance_pursuit(
    la_float N,
    const la_float *range,
    const la_float *vel_i,
    la_float *accel);

bool nav_guidance_pronav_true(
    la_float N,
    const la_float *range,
    const la_float *vel_i, 
    const la_float *vel_t,
    la_float *accel);

nav_guidance_type_t nav_guidance_compute_accel(
    const nav_guidance_t *g,
    const la_float *range,
    const la_float *vel_i, 
    const la_float *vel_t,
    la_float *accel);

void nav_guidance_compute_command(
    const nav_guidance_t *g, 
    const nav_guidance_state_t *interceptor, 
    const nav_guidance_state_t *target,
    nav_guidance_command_t *command);

#endif
