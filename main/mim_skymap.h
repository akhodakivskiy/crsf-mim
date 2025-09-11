#ifndef MIM_SKYMAP_H
#define MIM_SKYMAP_H

#include "skymap.h"

typedef struct {
    bool is_valid;
    double roll_cmd;
    double pitch_cmd;
} mim_skymap_command_t;

void mim_skymap_init();

bool mim_skymap_guidance_is_enabled();

void mim_skymap_guidance_enable(bool enable);

void mim_skymap_get_command(mim_skymap_command_t *command);

#endif
