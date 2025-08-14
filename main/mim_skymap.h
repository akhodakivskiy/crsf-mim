#ifndef SKYMAP_H
#define SKYMAP_H

#include "lwip/ip_addr.h"
#include "skymap.pb.h"

typedef struct {
    ip_addr_t skymap_ip;
    uint16_t skymap_port;
    int64_t last_ping_time;

    ai_skyfortress_guidance_ServerMessage skymap_msg;
} mim_skymap_t;

void mim_skymap_init(mim_skymap_t *skymap);

void mim_skymap_handle_packet(mim_skymap_t *skymap, uint8_t *data, uint16_t len);

#endif
