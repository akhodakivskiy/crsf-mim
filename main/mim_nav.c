#include "mim_nav.h"

#include "async_logging.h"
#include "libnet.h"
#include "mim_menu.h"
#include "mim_rc.h"
#include "libnet.h"
#include "skymap.h"
#include "mim_settings.h"
#include "nav.h"
#include "nav_pitcher.h"

#include <lwip/ip4_addr.h>
#include <esp_netif_ip_addr.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <time.h>

#define _MIM_NAV_TASK_STOP_BIT 1 << 0
#define _MIM_NAV_QUEUE_SIZE 10

#define _MIM_NAV_CONNECTION_TIMEOUT_US 5000000 // 5 seconds
#define _MIM_NAV_STATUS_TIMEOUT_US 4000000 // 4 seconds

static const char *TAG = "MIM_NAV";

struct mim_nav_ctx_s {
    libnet_handle_t libnet;

    ip4_addr_t skymap_addr;
    uint16_t skymap_port;

    int64_t time_last_status_message;
    int64_t time_last_update;
    int64_t time_last_command;

    nav_state_t state_interceptor_crsf;
    nav_state_t state_interceptor;
    nav_state_t state_target;
    nav_pitcher_state_t state_pitcher;

    nav_command_t command;

    QueueHandle_t queue;
    TaskHandle_t task;

    bool is_engaging;
    bool is_connected;
    bool is_initialized;
};

static bool _is_nav_state_ready(int64_t timestamp_us, const nav_state_t *state);
mim_nav_status_t _get_status(int64_t timestamp_us, const mim_nav_handle_t h);

static void _libnet_init(mim_nav_handle_t handle);
static void _task_skymap_impl(void *arg);

static void _on_connected(void *user_ctx, ip4_addr_t *addr);
static void _on_disconnected(void *user_ctx);
static void _on_packet(void *user_ctx, uint8_t *data, uint16_t len, ip4_addr_t *addr_from, uint16_t port_from);

static void _skymap_enqueue_status_message_if_necessary(int64_t timestamp_us, mim_nav_handle_t h, mim_nav_msg_t *msg);
static void _skymap_send_client_message(int64_t timestamp_us, mim_nav_handle_t h, const ai_skyfortress_guidance_ClientMessage *msg);
static void _skymap_handle_server_message(int64_t timestamp_us,mim_nav_handle_t h, const ai_skyfortress_guidance_ServerMessage *msg);
static void _skymap_estimate_to_nav_state(int64_t timestamp_us, const ai_skyfortress_guidance_TargetEstimate *e, nav_state_t *s);
static void _skymap_handle_crsf_message(int64_t timestsamp_us, mim_nav_handle_t h, const mim_nav_crsf_subset_t *msg);

static void _nav_update(int64_t timestamp_us, mim_nav_handle_t h);

TaskHandle_t _task_skymap = NULL;

esp_err_t mim_nav_init(BaseType_t priority, mim_nav_handle_t *handle) {
    mim_nav_handle_t h = (mim_nav_handle_t) heap_caps_calloc(1, sizeof(struct mim_nav_ctx_s), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    memset(h, 0, sizeof(struct mim_nav_ctx_s));

    assert(_task_skymap == NULL);

    ip4_addr_set_any(&h->skymap_addr);
    h->skymap_port = 0;
    h->queue = xQueueCreate(_MIM_NAV_QUEUE_SIZE, sizeof(mim_nav_msg_t));

    _libnet_init(h);
    assert(xTaskCreatePinnedToCore(_task_skymap_impl, "skymap", 4096, h, priority, &h->task, APP_CPU_NUM) == pdPASS);

    h->is_initialized = true;

    *handle = h;

    return ESP_OK;
}

esp_err_t mim_nav_deinit(mim_nav_handle_t h) {
    if (!h->is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xTaskNotify(h->task, _MIM_NAV_TASK_STOP_BIT, eSetBits);
    while(h->task != NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_ERROR_CHECK(libnet_deinit(h->libnet));

    h->is_initialized = false;

    heap_caps_free(h);

    return ESP_OK;
}

void mim_nav_enqueue(mim_nav_handle_t handle, const mim_nav_msg_t *msg) {
    assert(xQueueSend(handle->queue, msg, 0) == pdTRUE);
}

mim_nav_status_t mim_nav_get_status(const mim_nav_handle_t h) {
    int64_t time = esp_timer_get_time();
    return _get_status(time, h);
}

bool mim_nav_is_target_ready(const mim_nav_handle_t h) {
    return _is_nav_state_ready(esp_timer_get_time(), &h->state_target);
}

bool mim_nav_is_interceptor_ready(const mim_nav_handle_t h) {
    return _is_nav_state_ready(esp_timer_get_time(), &h->state_interceptor) ||
           _is_nav_state_ready(esp_timer_get_time(), &h->state_interceptor_crsf);
}

bool mim_nav_is_engaging(const mim_nav_handle_t handle) {
    return handle->is_engaging;
}

void mim_nav_set_engaging(mim_nav_handle_t handle, bool enable) {
    handle->is_engaging = enable;
}

mim_nav_status_t _get_status(int64_t timestamp_us, const mim_nav_handle_t h) {
    if (!h->is_connected) {
        return MIM_NAV_STATUS_ERROR_CONNECTION;
    } else if (!_is_nav_state_ready(timestamp_us, &h->state_target)) {
        return MIM_NAV_STATUS_ERROR_TARGET_POSITION;
    } else if (!_is_nav_state_ready(timestamp_us, &h->state_interceptor) && !_is_nav_state_ready(timestamp_us, &h->state_interceptor_crsf)) {
        return MIM_NAV_STATUS_ERROR_INTERCEPTOR_POSITION;
    }

    return MIM_NAV_STATUS_READY_TO_ENGAGE;
}

static bool _is_nav_state_ready(int64_t timestamp_us, const nav_state_t *s) {
    return s->timestamp_us > 0 && 
        timestamp_us >= s->timestamp_us &&
        (timestamp_us - s->timestamp_us) <= _MIM_NAV_CONNECTION_TIMEOUT_US;
}

const nav_command_t *mim_nav_get_last_command(mim_nav_handle_t handle) {
    return &handle->command;
}

static void _libnet_init(mim_nav_handle_t h) {
    libnet_config_t libnet_cfg = {0};

    libnet_cfg.priority = uxTaskPriorityGet(xTaskGetCurrentTaskHandle());
    libnet_cfg.core_id = PRO_CPU_NUM;
    libnet_cfg.callbacks.connected = _on_connected;
    libnet_cfg.callbacks.disconnected = _on_disconnected;
    libnet_cfg.callbacks.packet = _on_packet;
    libnet_cfg.user_ctx = h;

    switch (mim_settings_get()->mode) {
        case MIM_SETTINGS_MODE_ETHERNET:
            libnet_cfg.interface = LIBNET_INTERFACE_ETHERNET;
            break;
        case MIM_SETTINGS_MODE_WIFI:
            libnet_cfg.interface = LIBNET_INTERFACE_WIFI;
            strncpy(libnet_cfg.net.wifi.ssid, mim_settings_get()->wifi_ssid, sizeof(libnet_cfg.net.wifi.ssid));
            strncpy(libnet_cfg.net.wifi.password, mim_settings_get()->wifi_password, sizeof(libnet_cfg.net.wifi.password));
            break;
    }

    ESP_ERROR_CHECK(libnet_init(&libnet_cfg, &h->libnet));
    ESP_ERROR_CHECK(libnet_udp_server_start(h->libnet, 8888));
}

static IRAM_ATTR void _task_skymap_impl(void *arg) {
    mim_nav_handle_t h = (mim_nav_handle_t)arg;

    uint32_t notification_value;
    mim_nav_msg_t msg;

    while (true) {
        if (xTaskNotifyWait(0, _MIM_NAV_TASK_STOP_BIT, &notification_value, 0) == pdTRUE) {
            if (notification_value & _MIM_NAV_TASK_STOP_BIT) {
                break;  // Exit loop
            }
        }

        if (xQueueReceive(h->queue, &msg, pdMS_TO_TICKS(100)) == pdTRUE) {

            int64_t timestamp_us = esp_timer_get_time();

            switch (msg.type) {
                case MIM_NAV_MSG_TYPE_CLIENT:
                    _skymap_send_client_message(timestamp_us, h, &msg.message.client);
                    break;
                case MIM_NAV_MSG_TYPE_SERVER:
                    _skymap_handle_server_message(timestamp_us, h, &msg.message.server);
                    break;
                case MIM_NAV_MSG_TYPE_CRSF:
                    _skymap_handle_crsf_message(timestamp_us, h, &msg.message.crsf);
                    break;
            }
        }

        _skymap_enqueue_status_message_if_necessary(esp_timer_get_time(), h, &msg);
    }

    ESP_LOGI(TAG, "skymap task exit");

    h->task = NULL;
    vTaskDelete(NULL);
}

static void _on_connected(void *user_ctx, ip4_addr_t *addr) {
    mim_nav_handle_t h = (mim_nav_handle_t)(user_ctx);

    h->is_connected = true;

    mim_menu_set_ip_address(addr);
    mim_menu_set_connected(true);

    ESP_LOGI(TAG, "connected, IP=" IPSTR, IP2STR(addr));
}

static void _on_disconnected(void *user_ctx) {
    mim_nav_handle_t h = (mim_nav_handle_t)(user_ctx);

    h->is_connected = false;

    ip4_addr_t addr;
    ip4_addr_set_any(&addr);
    mim_menu_set_ip_address(&addr);
    mim_menu_set_connected(false);

    ESP_LOGI(TAG, "disconnected");
}

static IRAM_ATTR void _on_packet(void *user_ctx, uint8_t *data, uint16_t len, ip4_addr_t *addr_from, uint16_t port_from) {
    mim_nav_handle_t h = (mim_nav_handle_t)(user_ctx);

    assert(h->is_initialized);

    mim_nav_msg_t msg;
    msg.type = MIM_NAV_MSG_TYPE_SERVER;

    skymap_err_t err = skymap_read_server_message(&msg.message.server, data, len);

    if (err == SKYMAP_OK) {
assert(xQueueSend(h->queue, &msg, 0) == pdTRUE);
        h->skymap_addr = *addr_from;
        h->skymap_port = port_from;
    } else {
        ESP_LOGE(TAG, "failed to parse Skymap packet from " IPSTR ", len=%u", IP2STR(addr_from), len);
    }
}

static void _skymap_enqueue_status_message_if_necessary(int64_t timestamp_us, mim_nav_handle_t h, mim_nav_msg_t *msg) {
    if (timestamp_us - h->time_last_status_message > _MIM_NAV_STATUS_TIMEOUT_US) {
        memset(msg, 0, sizeof(mim_nav_msg_t));
        msg->type = MIM_NAV_MSG_TYPE_CLIENT;
        msg->message.client.which_message = ai_skyfortress_guidance_ClientMessage_status_tag;
        msg->message.client.message.status.state = ai_skyfortress_guidance_ClientState_Ready;
        msg->message.client.message.status.has_position = false;

        assert(xQueueSend(h->queue, msg, 0) == pdTRUE);

        h->time_last_status_message = timestamp_us;
    }
}

static void _skymap_send_client_message(int64_t timestamp_us, mim_nav_handle_t h, const ai_skyfortress_guidance_ClientMessage *msg) {
    if (!ip4_addr_isany(&h->skymap_addr) && h->skymap_port > 0) {
        uint8_t data[128];
        uint16_t len_written = 0;

        assert(skymap_write_client_message(msg, data, 128, &len_written) == SKYMAP_OK);

        libnet_udp_send(h->libnet, &h->skymap_addr, h->skymap_port, data, len_written);
    }
}

static void _skymap_handle_server_message(int64_t timestamp_us, mim_nav_handle_t h, const ai_skyfortress_guidance_ServerMessage *msg) {

    switch (msg->which_message) {
        case ai_skyfortress_guidance_ServerMessage_interceptor_estimate_tag:
            _skymap_estimate_to_nav_state(timestamp_us, &msg->message.interceptor_estimate, &h->state_interceptor);
            break;
        case ai_skyfortress_guidance_ServerMessage_target_estimate_tag:
            _skymap_estimate_to_nav_state(timestamp_us, &msg->message.target_estimate, &h->state_target);
            break;
        default:
            break;
    }

    _nav_update(timestamp_us, h);
}

static void _skymap_estimate_to_nav_state(int64_t timestamp_us, const ai_skyfortress_guidance_TargetEstimate *e, nav_state_t *s) {
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

static void _skymap_handle_crsf_message(int64_t timestamp_us, mim_nav_handle_t h, const mim_nav_crsf_subset_t *msg) {
    switch (msg->type) {
        case MIM_NAV_CRSF_SUBSET_TYPE_GPS: {
            h->state_interceptor_crsf.timestamp_us = timestamp_us;

            float heading_rad = NAV_DEG_TO_RAD(msg->message.gps.heading_cdeg / 100.0);
            float speed_ms = msg->message.gps.groundspeed_10kmh / 36.0;

            h->state_interceptor_crsf.lat = msg->message.gps.latitude_10e7 / 10000000.0;
            h->state_interceptor_crsf.lon = msg->message.gps.longitude_10e7 / 10000000.0;
            h->state_interceptor_crsf.alt = msg->message.gps.altitude_m;
            h->state_interceptor_crsf.vel_north = speed_ms * cosf(heading_rad);
            h->state_interceptor_crsf.vel_east = speed_ms * sinf(heading_rad);
            break;
        }
        case MIM_NAV_CRSF_SUBSET_TYPE_VARIO:
            h->state_interceptor_crsf.vel_up = msg->message.vario.vspeed_cms / 100.0f;
            break;
        default:
            break;
    }

    _nav_update(timestamp_us, h);
}

static void _nav_update(int64_t timestamp_us, mim_nav_handle_t h) {
    memset(&h->command, 0, sizeof(nav_command_t));
    h->command.type = NAV_NONE;

    nav_state_t *state_interceptor = NULL;

    // calculate nav command
    mim_nav_status_t status = _get_status(timestamp_us, h);
    if (status != MIM_NAV_STATUS_READY_TO_ENGAGE) {
        h->time_last_update = 0;
        nav_pitcher_reset(&h->state_pitcher);
    } else if (h->time_last_update > 0) {

        if (_is_nav_state_ready(timestamp_us, &h->state_interceptor)) {
            state_interceptor = &h->state_interceptor;
        } else if (_is_nav_state_ready(timestamp_us, &h->state_interceptor_crsf)) {
            state_interceptor = &h->state_interceptor_crsf;
        }

        // advance interceptor and target from their last known positions
        nav_state_advance(state_interceptor, timestamp_us);
        nav_state_advance(&h->state_target, timestamp_us);

        nav_compute_command(&mim_settings_get()->nav, state_interceptor, &h->state_target, &h->command);
        h->time_last_command = timestamp_us;
    }

    // apply overrides
    if (h->command.type != NAV_NONE && h->is_engaging) {
        float dt_s = (float)(timestamp_us - h->time_last_update) / 1000000.0;

        la_float max_roll_rad = NAV_DEG_TO_RAD(mim_settings_get()->nav.max_roll_deg);

        la_float roll_cmd = la_clamp2(la_atan2(h->command.accel_lat, NAV_G) / max_roll_rad, 1);
        la_float pitch_cmd = nav_pitcher_update(&mim_settings_get()->pitcher, &h->state_pitcher, dt_s, state_interceptor->vel_up, h->command.accel_ver);

        uint16_t roll_ticks = CRSF_RC_CHANNELS_CENTER + roll_cmd * CRSF_RC_CHANNELS_RANGE / 2;
        uint16_t pitch_ticks = CRSF_RC_CHANNELS_CENTER + pitch_cmd * CRSF_RC_CHANNELS_RANGE / 2;

        mim_rc_set_override(MIM_RC_CHANNEL_ROLL, roll_ticks, MIM_RC_OVERRIDE_LEVEL_GUIDANCE);
        mim_rc_set_override(MIM_RC_CHANNEL_PITCH, pitch_ticks, MIM_RC_OVERRIDE_LEVEL_GUIDANCE);
    } else {
        mim_rc_reset_overrides();
    }

    h->time_last_update = timestamp_us;
}
