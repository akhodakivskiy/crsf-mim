#ifndef MIM_TELEM_H
#define MIM_TELEM_H

#include "libcrsf_def.h"
#include "libcrsf_payload.h"
#include "mim_conn.h"
#include "mim_nav.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {

    MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID_STATUS = 0x6000,
    MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID_ACCEL_LAT,
    MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID_ACCEL_VER,
    MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID_DIST_HOR,
    MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID_DIST_VER,
    MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID_ZEM,
    MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID_TTGO,
    MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID_ADDR_MIM,
    MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID_ADDR_SKYMAP,

    MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID__FIRST = MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID_STATUS,
    MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID__LAST = MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID_ADDR_SKYMAP,
} mim_nav_crsf_ardupilot_payload_appid_t;

#define MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID__COUNT                                                                  \
    MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID__LAST - MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID__FIRST + 1

void mim_telem_write_payload(mim_nav_handle_t nav, mim_conn_handle_t conn, crsf_payload_ardupilot_t *payload);
void mim_telem_write_frame(mim_nav_handle_t nav, mim_conn_handle_t conn, crsf_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif
