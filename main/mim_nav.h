#ifndef MIM_NAV_H
#define MIM_NAV_H

#include <freertos/FreeRTOS.h>
#include <lwip/ip4_addr.h>

#include "mim_conn.h"
#include "nav.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MIM_NAV_STATUS_READY_TO_ENGAGE,
    MIM_NAV_STATUS_ERROR_TARGET_POSITION,
    MIM_NAV_STATUS_ERROR_INTERCEPTOR_POSITION,
} mim_nav_status_t;

typedef enum {
    MIM_NAV_ESTIMATE_STATUS_NONE,
    MIM_NAV_ESTIMATE_STATUS_SKYMAP,
    MIM_NAV_ESTIMATE_STATUS_CRSF,
} mim_nav_estimate_status_t;

typedef enum {
    MIM_NAV_CRSF_ARDUPILOT_PAYLOAD_APPID_STATUS = 0x6000,
    MIM_NAV_CRSF_ARDUPILOT_PAYLOAD_APPID_ACCEL_LAT = 0x6001,
    MIM_NAV_CRSF_ARDUPILOT_PAYLOAD_APPID_ACCEL_VER = 0x6002,
    MIM_NAV_CRSF_ARDUPILOT_PAYLOAD_APPID_DIST_HOR = 0x6003,
    MIM_NAV_CRSF_ARDUPILOT_PAYLOAD_APPID_DIST_VER = 0x6004,
    MIM_NAV_CRSF_ARDUPILOT_PAYLOAD_APPID_ZEM = 0x6005,
    MIM_NAV_CRSF_ARDUPILOT_PAYLOAD_APPID_TTGO = 0x6006,
} mim_nav_crsf_ardupilot_payload_appid_t;

typedef struct mim_nav_ctx_s *mim_nav_handle_t;

esp_err_t mim_nav_init(mim_nav_handle_t *handle);
esp_err_t mim_nav_deinit(mim_nav_handle_t handle);

int64_t mim_nav_last_message_time(mim_nav_handle_t handle);

void mim_nav_on_message(mim_conn_handle_t conn, int64_t timestamp_us, const mim_conn_msg_t *msg, void *arg);

mim_nav_estimate_status_t mim_nav_target_status(const mim_nav_handle_t handle);
mim_nav_estimate_status_t mim_nav_interceptor_status(const mim_nav_handle_t handle);

const nav_state_t *mim_nav_get_interceptor(const mim_nav_handle_t handle);
const nav_state_t *mim_nav_get_target(const mim_nav_handle_t handle);

bool mim_nav_is_engaging(const mim_nav_handle_t handle);
void mim_nav_set_engaging(mim_nav_handle_t handle, bool enable);

const nav_command_t *mim_nav_get_last_command(mim_nav_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif
