#ifndef MIM_MENU_H
#define MIM_MENU_H

#include "libcrsf_device.h"
#include <lwip/ip4_addr.h>

#ifdef __cplusplus
extern "C" {
#endif

void mim_menu_init(crsf_device_t *device);

void mim_menu_set_ip_address(const ip4_addr_t *addr);

void mim_menu_set_connected(bool value);

#ifdef __cplusplus
}
#endif

#endif // MIM_MENU_H
