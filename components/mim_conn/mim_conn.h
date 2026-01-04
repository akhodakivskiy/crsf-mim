#ifndef MIM_CONN_H
#define MIM_CONN_H

#include <freertos/FreeRTOS.h>
#include <lwip/ip4_addr.h>

#include "libcrsf_payload.h"
#include "portmacro.h"
#include "skymap.pb.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mim_conn_s *mim_conn_handle_t;

typedef enum {
    MIM_CONN_CRSF_SUBSET_TYPE_NONE,
    MIM_CONN_CRSF_SUBSET_TYPE_GPS,
    MIM_CONN_CRSF_SUBSET_TYPE_BAROALT,
    MIM_CONN_CRSF_SUBSET_TYPE_VARIO,
    MIM_CONN_CRSF_SUBSET_TYPE_AIRSPEED,
} mim_conn_crsf_subset_type_t;

typedef struct {
    mim_conn_crsf_subset_type_t type;
    union {
        crsf_payload_gps_t gps;
        crsf_payload_baro_altitude_t alt;
        crsf_payload_vario_t vario;
        crsf_payload_airspeed_t airspeed;
    } message;
} mim_conn_crsf_subset_t;

typedef enum {
    MIM_CONN_MSG_TYPE_TICK,
    MIM_CONN_MSG_TYPE_SERVER,
    MIM_CONN_MSG_TYPE_CRSF,
} mim_conn_msg_type_t;

typedef struct {
    mim_conn_msg_type_t type;
    union {
        ai_skyfortress_guidance_ServerMessage server;
        mim_conn_crsf_subset_t crsf;
    } message;
} mim_conn_msg_t;

typedef void (*mim_conn_callback)(mim_conn_handle_t conn,
                                  int64_t timestamp_us,
                                  const mim_conn_msg_t *msg,
                                  void *user_ctx);

typedef struct {
    BaseType_t core;
    UBaseType_t priority;
    mim_conn_callback callback;
    void *callback_arg;

    // mim_settings_mode_t netmode;

    uint16_t mim_port;
} mim_conn_config_t;

esp_err_t mim_conn_init(const mim_conn_config_t *config, mim_conn_handle_t *handle);
esp_err_t mim_conn_deinit(mim_conn_handle_t handle);

bool mim_conn_is_connected(const mim_conn_handle_t handle);

ip4_addr_t mim_conn_get_mim_addr(const mim_conn_handle_t handle);
ip4_addr_t mim_conn_get_skymap_addr(const mim_conn_handle_t handle);

void mim_conn_send_skymap_message(mim_conn_handle_t h, const ai_skyfortress_guidance_ClientMessage *msg);

void mim_conn_passthrough(mim_conn_handle_t handle, const mim_conn_msg_t *msg);

#ifdef __cplusplus
}
#endif

#endif
