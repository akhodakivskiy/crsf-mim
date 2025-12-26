#include "mim_nav.h"

#include "libcrsf_payload.h"
#include "mim_conn.h"
#include "mim_rc.h"
#include "mim_settings.h"
#include "nav.h"
#include "nav_pitcher.h"

#include <esp_log.h>
#include <esp_netif_ip_addr.h>
#include <float.h>
#include <string.h>
#include <time.h>

#define _MIM_NAV_TASK_STOP_BIT 1 << 0
#define _MIM_NAV_QUEUE_SIZE 10

#define _MIM_NAV_CONNECTION_TIMEOUT_US 5000000 // 5 seconds
#define _MIM_NAV_STATUS_TIMEOUT_US 4000000     // 4 seconds

static const char *TAG = "MIM_NAV";

struct mim_nav_ctx_s {
    int64_t time_last_message;
    int64_t time_last_status_message;
    int64_t time_last_update;
    int64_t time_last_command;
    int64_t time_last_telemetry;

    nav_state_t state_interceptor_crsf;
    nav_state_t state_interceptor;
    nav_state_t state_target;
    nav_pitcher_state_t state_pitcher;

    nav_command_t command;

    bool is_engaging;
};

static bool _is_nav_state_ready(int64_t timestamp_us, const nav_state_t *state);
mim_nav_status_t _get_status(const mim_nav_handle_t h);

static void _skymap_enqueue_status_message_if_necessary(mim_conn_handle_t conn, mim_nav_handle_t h);
static void _skymap_handle_tick_message(mim_nav_handle_t h);
static void _skymap_handle_server_message(mim_nav_handle_t h, const ai_skyfortress_guidance_ServerMessage *msg);
static void _skymap_handle_crsf_message(mim_nav_handle_t h, const mim_conn_crsf_subset_t *msg);

static void
_skymap_estimate_to_nav_state(int64_t timestamp_us, const ai_skyfortress_guidance_TargetEstimate *e, nav_state_t *s);

static void _nav_update(mim_nav_handle_t h);

esp_err_t mim_nav_init(mim_nav_handle_t *handle) {
    mim_nav_handle_t h
        = (mim_nav_handle_t)heap_caps_calloc(1, sizeof(struct mim_nav_ctx_s), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    memset(h, 0, sizeof(struct mim_nav_ctx_s));

    *handle = h;

    return ESP_OK;
}

esp_err_t mim_nav_deinit(mim_nav_handle_t h) {
    heap_caps_free(h);

    return ESP_OK;
}

int64_t mim_nav_last_message_time(mim_nav_handle_t h) { return h->time_last_message; }

void mim_nav_on_message(mim_conn_handle_t conn, int64_t timestamp_us, const mim_conn_msg_t *msg, void *arg) {
    mim_nav_handle_t h = (mim_nav_handle_t)arg;

    h->time_last_message = timestamp_us;

    switch (msg->type) {
    case MIM_CONN_MSG_TYPE_TICK:
        _skymap_handle_tick_message(h);
        break;
    case MIM_CONN_MSG_TYPE_SERVER:
        _skymap_handle_server_message(h, &msg->message.server);
        break;
    case MIM_CONN_MSG_TYPE_CRSF:
        _skymap_handle_crsf_message(h, &msg->message.crsf);
        break;
    }

    _skymap_enqueue_status_message_if_necessary(conn, h);
}

mim_nav_estimate_status_t mim_nav_target_status(const mim_nav_handle_t h) {
    if (_is_nav_state_ready(h->time_last_message, &h->state_target)) {
        return MIM_NAV_ESTIMATE_STATUS_SKYMAP;
    } else {
        return MIM_NAV_ESTIMATE_STATUS_NONE;
    }
}

mim_nav_estimate_status_t mim_nav_interceptor_status(const mim_nav_handle_t h) {
    if (_is_nav_state_ready(h->time_last_message, &h->state_interceptor)) {
        return MIM_NAV_ESTIMATE_STATUS_SKYMAP;
    } else if (_is_nav_state_ready(h->time_last_message, &h->state_interceptor_crsf)) {
        return MIM_NAV_ESTIMATE_STATUS_CRSF;
    } else {
        return MIM_NAV_ESTIMATE_STATUS_NONE;
    }
}

const nav_state_t *mim_nav_get_interceptor(const mim_nav_handle_t h) {
    if (_is_nav_state_ready(h->time_last_message, &h->state_interceptor)) {
        return &h->state_interceptor;
    } else {
        return &h->state_interceptor_crsf;
    }
}

const nav_state_t *mim_nav_get_target(const mim_nav_handle_t h) { return &h->state_target; }

bool mim_nav_is_engaging(const mim_nav_handle_t handle) { return handle->is_engaging; }

void mim_nav_set_engaging(mim_nav_handle_t handle, bool enable) { handle->is_engaging = enable; }

mim_nav_status_t _get_status(const mim_nav_handle_t h) {
    if (!_is_nav_state_ready(h->time_last_message, &h->state_target)) {
        return MIM_NAV_STATUS_ERROR_TARGET_POSITION;
    } else if (!_is_nav_state_ready(h->time_last_message, &h->state_interceptor)
               && !_is_nav_state_ready(h->time_last_message, &h->state_interceptor_crsf)) {
        return MIM_NAV_STATUS_ERROR_INTERCEPTOR_POSITION;
    }

    return MIM_NAV_STATUS_READY_TO_ENGAGE;
}

static bool _is_nav_state_ready(int64_t timestamp_us, const nav_state_t *s) {
    return s->timestamp_us > 0 && timestamp_us >= s->timestamp_us
           && (timestamp_us - s->timestamp_us) <= _MIM_NAV_CONNECTION_TIMEOUT_US;
}

const nav_command_t *mim_nav_get_last_command(mim_nav_handle_t handle) { return &handle->command; }

static void _skymap_enqueue_status_message_if_necessary(mim_conn_handle_t conn, mim_nav_handle_t h) {
    if (h->time_last_message - h->time_last_status_message > _MIM_NAV_STATUS_TIMEOUT_US) {
        ai_skyfortress_guidance_ClientMessage msg;
        memset(&msg, 0, sizeof(ai_skyfortress_guidance_ClientMessage));
        msg.which_message = ai_skyfortress_guidance_ClientMessage_status_tag;
        msg.message.status.state = ai_skyfortress_guidance_ClientState_Ready;
        msg.message.status.has_position = false;

        mim_conn_send_skymap_message(conn, &msg);

        h->time_last_status_message = h->time_last_message;
    }
}

static void _skymap_handle_tick_message(mim_nav_handle_t h) {}

static void _skymap_handle_server_message(mim_nav_handle_t h, const ai_skyfortress_guidance_ServerMessage *msg) {

    switch (msg->which_message) {
    case ai_skyfortress_guidance_ServerMessage_interceptor_estimate_tag:
        _skymap_estimate_to_nav_state(h->time_last_message, &msg->message.interceptor_estimate, &h->state_interceptor);
        break;
    case ai_skyfortress_guidance_ServerMessage_target_estimate_tag:
        _skymap_estimate_to_nav_state(h->time_last_message, &msg->message.target_estimate, &h->state_target);
        break;
    default:
        return;
    }

    _nav_update(h);
}

static void _skymap_handle_crsf_message(mim_nav_handle_t h, const mim_conn_crsf_subset_t *msg) {
    switch (msg->type) {
    case MIM_CONN_CRSF_SUBSET_TYPE_GPS: {
        h->state_interceptor_crsf.timestamp_us = h->time_last_message;

        float heading_rad = NAV_DEG_TO_RAD(msg->message.gps.heading_cdeg / 100.0);
        float speed_ms = msg->message.gps.groundspeed_10kmh / 36.0;

        h->state_interceptor_crsf.lat = msg->message.gps.latitude_10e7 / 10000000.0;
        h->state_interceptor_crsf.lon = msg->message.gps.longitude_10e7 / 10000000.0;
        h->state_interceptor_crsf.alt = msg->message.gps.altitude_m;
        h->state_interceptor_crsf.vel_north = speed_ms * cosf(heading_rad);
        h->state_interceptor_crsf.vel_east = speed_ms * sinf(heading_rad);
        break;
    }
    case MIM_CONN_CRSF_SUBSET_TYPE_VARIO:
        h->state_interceptor_crsf.vel_up = msg->message.vario.vspeed_cms / 100.0f;
        break;
    case MIM_CONN_CRSF_SUBSET_TYPE_BAROALT:
        // h->state_interceptor_crsf.vel_up = msg->message.vario.vspeed_cms /
        // 100.0f;
        break;
    default:
        break;
    }

    _nav_update(h);
}

static void
_skymap_estimate_to_nav_state(int64_t timestamp_us, const ai_skyfortress_guidance_TargetEstimate *e, nav_state_t *s) {
    if (e->has_position && e->has_velocity) {
        s->timestamp_us = timestamp_us;
        s->lat = e->position.latitude_deg;
        s->lon = e->position.longitude_deg;
        s->alt = e->position.altitude_msl_m;
        s->vel_north = e->velocity.north_ms;
        s->vel_east = e->velocity.east_ms;
        s->vel_up = e->velocity.up_ms;
    }
}

static void _nav_update(mim_nav_handle_t h) {
    memset(&h->command, 0, sizeof(nav_command_t));
    h->command.type = NAV_NONE;

    nav_state_t *state_interceptor = NULL;

    // calculate nav command
    mim_nav_status_t status = _get_status(h);
    if (status != MIM_NAV_STATUS_READY_TO_ENGAGE) {
        h->time_last_update = 0;
        nav_pitcher_reset(&h->state_pitcher);
    } else if (h->time_last_update > 0) {

        if (_is_nav_state_ready(h->time_last_message, &h->state_interceptor)) {
            state_interceptor = &h->state_interceptor;
        } else if (_is_nav_state_ready(h->time_last_message, &h->state_interceptor_crsf)) {
            state_interceptor = &h->state_interceptor_crsf;
        }

        // advance interceptor and target from their last known positions
        nav_state_advance(state_interceptor, h->time_last_message);
        nav_state_advance(&h->state_target, h->time_last_message);

        nav_compute_command(&mim_settings_get()->nav, state_interceptor, &h->state_target, &h->command);
        h->time_last_command = h->time_last_message;
    }

    // apply overrides
    if (h->command.type != NAV_NONE && h->is_engaging) {
        float dt_s = (float)(h->time_last_message - h->time_last_update) / 1000000.0;

        la_float max_roll_rad = NAV_DEG_TO_RAD(mim_settings_get()->nav.max_roll_deg);

        la_float roll_cmd = la_clamp2(la_atan2(h->command.accel_lat, NAV_G) / max_roll_rad, 1);
        la_float pitch_cmd = nav_pitcher_update(
            &mim_settings_get()->pitcher, &h->state_pitcher, dt_s, state_interceptor->vel_up, h->command.accel_ver);

        uint16_t roll_ticks = CRSF_RC_CHANNELS_CENTER + roll_cmd * CRSF_RC_CHANNELS_RANGE / 2;
        uint16_t pitch_ticks = CRSF_RC_CHANNELS_CENTER + pitch_cmd * CRSF_RC_CHANNELS_RANGE / 2;

        mim_rc_set_override(MIM_RC_CHANNEL_ROLL, roll_ticks, MIM_RC_OVERRIDE_LEVEL_GUIDANCE);
        mim_rc_set_override(MIM_RC_CHANNEL_PITCH, pitch_ticks, MIM_RC_OVERRIDE_LEVEL_GUIDANCE);
    } else {
        mim_rc_reset_overrides(MIM_RC_OVERRIDE_LEVEL_GUIDANCE);
    }

    h->time_last_update = h->time_last_message;
}
