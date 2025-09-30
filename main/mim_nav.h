#ifndef MIM_NAV_H
#define MIM_NAV_H

#include <freertos/FreeRTOS.h>

#include "libcrsf_payload.h"
#include "skymap.pb.h"
#include "nav.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MIM_NAV_STATUS_READY_TO_ENGAGE,
    MIM_NAV_STATUS_ERROR_CONNECTION,
    MIM_NAV_STATUS_ERROR_TARGET_POSITION,
    MIM_NAV_STATUS_ERROR_INTERCEPTOR_POSITION,
} mim_nav_status_t;

typedef enum {
    MIM_NAV_CRSF_SUBSET_TYPE_GPS,
    MIM_NAV_CRSF_SUBSET_TYPE_ALT,
    MIM_NAV_CRSF_SUBSET_TYPE_VARIO,
    MIM_NAV_CRSF_SUBSET_TYPE_AIRSPEED,
} mim_nav_crsf_subset_type_t;

typedef struct {
    mim_nav_crsf_subset_type_t type;
    union {
        crsf_payload_gps_t gps;
        crsf_payload_baro_altitude_t alt;
        crsf_payload_vario_t vario;
        crsf_payload_airspeed_t airspeed;
    } message;
} mim_nav_crsf_subset_t;

typedef enum {
    MIM_NAV_MSG_TYPE_SERVER,
    MIM_NAV_MSG_TYPE_CLIENT,
    MIM_NAV_MSG_TYPE_CRSF,
} mim_nav_msg_type_t;

typedef struct {
    mim_nav_msg_type_t type;
    union {
        ai_skyfortress_guidance_ServerMessage server;
        ai_skyfortress_guidance_ClientMessage client;
        mim_nav_crsf_subset_t crsf;
    } message;
} mim_nav_msg_t;

typedef struct mim_nav_ctx_s *mim_nav_handle_t;

esp_err_t mim_nav_init(BaseType_t priority, mim_nav_handle_t *handle);

esp_err_t mim_nav_deinit(mim_nav_handle_t handle);

void mim_nav_enqueue(mim_nav_handle_t handle, const mim_nav_msg_t *msg);

mim_nav_status_t mim_nav_get_status(const mim_nav_handle_t handle);
bool mim_nav_is_target_ready(const mim_nav_handle_t handle);
bool mim_nav_is_interceptor_ready(const mim_nav_handle_t handle);

bool mim_nav_is_engaging(const mim_nav_handle_t handle);
void mim_nav_set_engaging(mim_nav_handle_t handle, bool enable);

const nav_command_t *mim_nav_get_last_command(mim_nav_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif
