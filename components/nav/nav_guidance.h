#ifndef NAV_GUIDANCE_H
#define NAV_GUIDANCE_H

#include "nav_vector.h"

#include <stdbool.h>

typedef struct {
    double N;
    double max_roll_deg;
    double max_pitch_deg;
} nav_guidance_t;

typedef enum {
    NAV_GUIDANCE_NONE,
    NAV_GUIDANCE_PURSUIT,
    NAV_GUIDANCE_PRONAV,
} nav_guidance_type_t;

typedef struct {
    nav_guidance_type_t type;
    nav_vector_t accel;
    double accel_lateral;
    double accel_vertical;
    double roll_deg;
    double pitch_deg;
    double roll_cmd;
    double pitch_cmd;
} nav_guidance_command_t;

typedef struct {
    double lat;
    double lon;
    double alt;
    double vel_north;
    double vel_east;
    double vel_up;
} nav_guidance_state_t;

void nav_state_to_ned(const nav_guidance_state_t *state_ref, 
                      const nav_guidance_state_t *state, 
                      nav_vector_t *ned);

bool nav_guidance_pursuit(
    double N,
    const nav_vector_t *range,
    const nav_vector_t *vel_i,
    nav_vector_t *accel);

bool nav_guidance_pronav_true(
    double N,
    const nav_vector_t *range,
    const nav_vector_t *vel_i, const nav_vector_t *vel_t,
    nav_vector_t *accel);

nav_guidance_type_t nav_guidance_compute_accel(
    const nav_guidance_t *g,
    const nav_vector_t *range,
    const nav_vector_t *vel_i, const nav_vector_t *vel_t,
    nav_vector_t *accel);

void nav_guidance_compute_command(
    const nav_guidance_t *g, 
    const nav_guidance_state_t *interceptor, 
    const nav_guidance_state_t *target,
    nav_guidance_command_t *command);

#endif
