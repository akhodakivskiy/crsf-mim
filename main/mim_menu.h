#ifndef MIM_MENU_H
#define MIM_MENU_H

#include "libcrsf_device.h"
#include "mim_nav.h"
#include <lwip/ip4_addr.h>

#ifdef __cplusplus
extern "C" {
#endif

void mim_menu_init(crsf_device_t *device, mim_nav_handle_t nav);

#ifdef __cplusplus
}
#endif

#endif // MIM_MENU_H
