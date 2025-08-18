#ifndef SKYMAP_H
#define SKYMAP_H

#include <stdint.h>
#include <lwip/ip4_addr.h>

#define SKYMAP_CONNECTION_TIMEOUT_US 10000000 // 10 seconds
#define SKYMAP_STATUS_TIMEOUT_US 5000000 // 5 seconds

typedef enum {
    SKYMAP_OK,
    SKYMAP_ERR_HEADER_INVALID,
    SKYMAP_ERR_MESSAGE_INVALID,
} skymap_err_t;

typedef struct {
    bool is_connected;
    bool is_ready_to_engage;
    bool is_client_message_ready;

    int64_t last_time_client_message;
    int64_t last_time_server_message;
    int64_t last_time_ping;
    int64_t last_time_interceptor;
    int64_t last_time_target;
} skymap_t;

void skymap_reset(skymap_t *sm);

void skymap_update(skymap_t *sm, int64_t time);

skymap_err_t skymap_read_server_message(skymap_t *sm, int64_t time, uint8_t *data, uint16_t len);

skymap_err_t skymap_write_client_message(skymap_t *sm, int64_t time, uint8_t *data, uint16_t len, uint16_t *len_written);

#endif
