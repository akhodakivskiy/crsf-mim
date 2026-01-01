#include "mim_telem.h"
#include "mim_conn.h"

#include <endian.h>
#include <esp_log.h>

static const char *TAG = "MIM_TELEM";

void mim_telem_write_payload(mim_nav_handle_t nav, mim_conn_handle_t conn, crsf_payload_ardupilot_t *payload) {
    bool is_connected = mim_conn_is_connected(conn);
    bool is_engaging = mim_nav_is_engaging(nav);
    bool is_target_ready = mim_nav_target_status(nav) != MIM_NAV_ESTIMATE_STATUS_NONE;
    bool is_interceptor_skymap_ready = mim_nav_interceptor_status(nav) == MIM_NAV_ESTIMATE_STATUS_SKYMAP;
    bool is_interceptor_crsf_ready = mim_nav_interceptor_status(nav) == MIM_NAV_ESTIMATE_STATUS_CRSF;
    const nav_command_t *cmd = mim_nav_get_last_command(nav);
    ip4_addr_t addr_mim = mim_conn_get_mim_addr(conn);
    ip4_addr_t addr_skymap = mim_conn_get_skymap_addr(conn);

    payload->subtype = CRSF_PAYLOAD_ARDUPILOT_SUBTYPE_MULTI;
    payload->multi.size = MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID__COUNT;
    payload->multi.values[0].appid = MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID_STATUS;
    payload->multi.values[0].data = htole32((is_connected) |                     //
                                            (is_engaging << 1) |                 //
                                            (is_target_ready << 2) |             //
                                            (is_interceptor_skymap_ready << 3) | //
                                            (is_interceptor_crsf_ready << 4) |   //
                                            (cmd->type << 5));

    payload->multi.values[1].appid = MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID_ACCEL_LAT;
    payload->multi.values[1].data = htole32((uint32_t)(cmd->accel_lat * 1000));

    payload->multi.values[2].appid = MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID_ACCEL_VER;
    payload->multi.values[2].data = htole32((uint32_t)(cmd->accel_ver * 1000));

    payload->multi.values[3].appid = MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID_DIST_HOR;
    payload->multi.values[3].data = htole32((uint32_t)(cmd->range_hor));

    payload->multi.values[4].appid = MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID_DIST_VER;
    payload->multi.values[4].data = htole32((int32_t)(cmd->range_ver));

    payload->multi.values[5].appid = MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID_ZEM;
    payload->multi.values[5].data = htole32((uint32_t)cmd->zero_effort_miss_m);

    payload->multi.values[6].appid = MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID_TTGO;
    payload->multi.values[6].data = htole32((uint32_t)cmd->time_to_go_s);

    payload->multi.values[7].appid = MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID_ADDR_MIM;
    payload->multi.values[7].data = htole32(addr_mim.addr);

    payload->multi.values[8].appid = MIM_TELEM_CRSF_ARDUPILOT_PAYLOAD_APPID_ADDR_SKYMAP;
    payload->multi.values[8].data = htole32(addr_skymap.addr);
}

void mim_telem_write_frame(mim_nav_handle_t nav, mim_conn_handle_t conn, crsf_frame_t *frame) {
    crsf_payload_ardupilot_t payload;

    mim_telem_write_payload(nav, conn, &payload);

    crsf_payload_pack__ardupilot(frame, &payload);
}
