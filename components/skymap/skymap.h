#ifndef SKYMAP_H
#define SKYMAP_H

#include <stdint.h>
#include <lwip/ip4_addr.h>
#include <skymap.pb.h>

typedef enum {
    SKYMAP_OK,
    SKYMAP_ERR_HEADER_MISSING,
    SKYMAP_ERR_HEADER_INVALID,
    SKYMAP_ERR_MESSAGE_INVALID,
} skymap_err_t;

skymap_err_t skymap_read_server_message(
    ai_skyfortress_guidance_ServerMessage *sm, 
    const uint8_t *data, uint16_t len);

skymap_err_t skymap_write_client_message(
    const ai_skyfortress_guidance_ClientMessage *sm, 
    uint8_t *data, uint16_t len, uint16_t *len_written);

#endif
