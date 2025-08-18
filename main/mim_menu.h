#ifndef MIM_MENU_H
#define MIM_MENU_H

#include "libcrsf_device.h"
#include "skymap.h"

#ifdef __cplusplus
extern "C" {
#endif

void mim_menu_init(crsf_device_t *device);

void mim_menu_set_skymap(skymap_t *sm);

#ifdef __cplusplus
}
#endif

#endif // MIM_MENU_H
