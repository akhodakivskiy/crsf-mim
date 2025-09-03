#ifndef MIM_SKYMAP_H
#define MIM_SKYMAP_H

#include "skymap.h"

typedef struct {
    double accel_lateral;
    double accel_vertical;
} mim_skymap_command_t;

void mim_skymap_init();

void mim_skymap_get_command(mim_skymap_command_t *command);

#endif
