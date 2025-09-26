#include "mim_skymap.h"

#include "freertos/idf_additions.h"
#include "mim_menu.h"
#include "portmacro.h"
#include "skymap.h"
#include "skymap.pb.h"
#include "libnet.h"
#include "mim_settings.h"
#include "mim_rc.h"
#include "nav.h"
#include "nav_pitcher.h"

#include <freertos/FreeRTOS.h>
#include <lwip/ip4_addr.h>
#include <esp_netif_ip_addr.h>
#include <esp_timer.h>
#include <esp_log.h>


#define _MIM_SKYMAP_TASK_STOP_BIT 1 << 0
#define _MIM_SKYMAP_QUEUE_SIZE 10

#define _MIM_SKYMAP_CONNECTION_TIMEOUT_US 5000000 // 5 seconds
#define _MIM_SKYMAP_STATUS_TIMEOUT_US 4000000 // 4 seconds


static const char *TAG = "MIM_SKYMAP";

struct mim_skymap_ctx_s {
    libnet_handle_t libnet;

    ip4_addr_t skymap_addr;
    uint16_t skymap_port;

    int64_t time_last_status_message;
    int64_t time_last_update;

    nav_state_t state_interceptor;
    nav_state_t state_target;
    nav_pitcher_state_t state_pitcher;

    QueueHandle_t queue;
    TaskHandle_t task;

    bool is_engaging;
    bool is_connected;
    bool is_initialized;
};

typedef struct {
    bool is_client;
    union {
        ai_skyfortress_guidance_ServerMessage server;
        ai_skyfortress_guidance_ClientMessage client;
    } message;
} mim_skymap_msg_t;

static void _libnet_init(mim_skymap_handle_t handle);
static void _task_skymap_impl(void *arg);

static void _on_connected(void *user_ctx, ip4_addr_t *addr);
static void _on_disconnected(void *user_ctx);
static void _on_packet(void *user_ctx, uint8_t *data, uint16_t len, ip4_addr_t *addr_from, uint16_t port_from);

static void _skymap_enqueue_status_message_if_necessary(mim_skymap_handle_t h, mim_skymap_msg_t *msg);
static void _skymap_send_client_message(mim_skymap_handle_t h, const ai_skyfortress_guidance_ClientMessage *msg);
static void _skymap_handle_server_message(mim_skymap_handle_t h, const ai_skyfortress_guidance_ServerMessage *msg);
static void _skymap_estimate_to_nav_state(int64_t time, const ai_skyfortress_guidance_TargetEstimate *e, nav_state_t *s);

static void _nav_update(mim_skymap_handle_t h, int64_t dt_us);
static void _nav_reset(mim_skymap_handle_t h);

TaskHandle_t _task_skymap = NULL;

esp_err_t mim_skymap_init(BaseType_t priority, mim_skymap_handle_t *handle) {
    mim_skymap_handle_t h = (mim_skymap_handle_t) heap_caps_calloc(1, sizeof(struct mim_skymap_ctx_s), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    memset(h, 0, sizeof(struct mim_skymap_ctx_s));

    assert(_task_skymap == NULL);

    ip4_addr_set_any(&h->skymap_addr);
    h->skymap_port = 0;
    h->queue = xQueueCreate(_MIM_SKYMAP_QUEUE_SIZE, sizeof(mim_skymap_msg_t));

    _libnet_init(h);
    assert(xTaskCreatePinnedToCore(_task_skymap_impl, "skymap", 4096, h, priority, &h->task, APP_CPU_NUM) == pdPASS);

    h->is_initialized = true;

    *handle = h;

    return ESP_OK;
}

esp_err_t mim_skymap_deinit(mim_skymap_handle_t h) {
    if (!h->is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xTaskNotify(h->task, _MIM_SKYMAP_TASK_STOP_BIT, eSetBits);
    while(h->task != NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_ERROR_CHECK(libnet_deinit(h->libnet));

    h->is_initialized = false;

    heap_caps_free(h);

    return ESP_OK;
}

bool mim_skymap_is_ready(const mim_skymap_handle_t h) {
    int64_t time = esp_timer_get_time();

    return h->is_initialized && h->is_connected &&
        h->state_interceptor.timestamp_us > 0 &&
        (time - h->state_interceptor.timestamp_us) < _MIM_SKYMAP_CONNECTION_TIMEOUT_US &&
        h->state_target.timestamp_us > 0 &&
        (time - h->state_target.timestamp_us) < _MIM_SKYMAP_CONNECTION_TIMEOUT_US;
}

bool mim_skymap_is_engaging(const mim_skymap_handle_t handle) {
    return handle->is_engaging;
}

void mim_skymap_set_engaging(mim_skymap_handle_t handle, bool enable) {
    handle->is_engaging = enable;
}

static void _libnet_init(mim_skymap_handle_t h) {
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
    mim_skymap_handle_t h = (mim_skymap_handle_t)arg;

    uint32_t notification_value;
    mim_skymap_msg_t msg;

    while (true) {
        if (xTaskNotifyWait(0, _MIM_SKYMAP_TASK_STOP_BIT, &notification_value, 0) == pdTRUE) {
            if (notification_value & _MIM_SKYMAP_TASK_STOP_BIT) {
                break;  // Exit loop
            }
        }

        if (xQueueReceive(h->queue, &msg, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (msg.is_client) {
                _skymap_send_client_message(h, &msg.message.client);
            } else {
                _skymap_handle_server_message(h, &msg.message.server);
            }

        }

        _skymap_enqueue_status_message_if_necessary(h, &msg);
    }

    ESP_LOGI(TAG, "skymap task exit");

    h->task = NULL;
    vTaskDelete(NULL);
}

static void _on_connected(void *user_ctx, ip4_addr_t *addr) {
    mim_skymap_handle_t h = (mim_skymap_handle_t)(user_ctx);

    h->is_connected = true;

    mim_menu_set_ip_address(addr);
    mim_menu_set_connected(true);

    ESP_LOGI(TAG, "connected, IP=" IPSTR, IP2STR(addr));
}

static void _on_disconnected(void *user_ctx) {
    mim_skymap_handle_t h = (mim_skymap_handle_t)(user_ctx);

    h->is_connected = false;

    ip4_addr_t addr;
    ip4_addr_set_any(&addr);
    mim_menu_set_ip_address(&addr);
    mim_menu_set_connected(false);

    ESP_LOGI(TAG, "disconnected");
}

static IRAM_ATTR void _on_packet(void *user_ctx, uint8_t *data, uint16_t len, ip4_addr_t *addr_from, uint16_t port_from) {
    mim_skymap_handle_t h = (mim_skymap_handle_t)(user_ctx);

    assert(h->is_initialized);

    mim_skymap_msg_t msg;
    msg.is_client = false;

    skymap_err_t err = skymap_read_server_message(&msg.message.server, data, len);

    if (err == SKYMAP_OK) {
assert(xQueueSend(h->queue, &msg, 0) == pdTRUE);
        h->skymap_addr = *addr_from;
        h->skymap_port = port_from;
    } else {
        ESP_LOGE(TAG, "failed to parse Skymap packet from " IPSTR ", len=%u", IP2STR(addr_from), len);
    }
}

static void _skymap_enqueue_status_message_if_necessary(mim_skymap_handle_t h, mim_skymap_msg_t *msg) {
    int64_t time = esp_timer_get_time();

    if (time - h->time_last_status_message > _MIM_SKYMAP_STATUS_TIMEOUT_US) {
        memset(msg, 0, sizeof(mim_skymap_msg_t));
        msg->is_client = true;
        msg->message.client.which_message = ai_skyfortress_guidance_ClientMessage_status_tag;
        msg->message.client.message.status.state = ai_skyfortress_guidance_ClientState_Ready;
        msg->message.client.message.status.has_position = false;

        assert(xQueueSend(h->queue, msg, 0) == pdTRUE);

        h->time_last_status_message = time;
    }
}

static void _skymap_send_client_message(mim_skymap_handle_t h, const ai_skyfortress_guidance_ClientMessage *msg) {
    if (!ip4_addr_isany(&h->skymap_addr) && h->skymap_port > 0) {
        uint8_t data[128];
        uint16_t len_written = 0;

        assert(skymap_write_client_message(msg, data, 128, &len_written) == SKYMAP_OK);

        libnet_udp_send(h->libnet, &h->skymap_addr, h->skymap_port, data, len_written);
    }
}

static void _skymap_handle_server_message(mim_skymap_handle_t h, const ai_skyfortress_guidance_ServerMessage *msg) {
    int64_t time = esp_timer_get_time();

    switch (msg->which_message) {
        case ai_skyfortress_guidance_ServerMessage_interceptor_estimate_tag:
            _skymap_estimate_to_nav_state(time, &msg->message.interceptor_estimate, &h->state_interceptor);
            break;
        case ai_skyfortress_guidance_ServerMessage_target_estimate_tag:
            _skymap_estimate_to_nav_state(time, &msg->message.target_estimate, &h->state_target);
            break;
        default:
            break;
    }

    _nav_update(h, time);
}

static void _skymap_estimate_to_nav_state(int64_t time, const ai_skyfortress_guidance_TargetEstimate *e, nav_state_t *s) {
    if (e->has_position && e->has_velocity) {
        s->timestamp_us = time; 
        s->lat = e->position.latitude_deg; 
        s->lon = e->position.longitude_deg; 
        s->alt = e->position.altitude_msl_m; 
        s->vel_north = e->velocity.north_ms; 
        s->vel_east = e->velocity.east_ms; 
        s->vel_up = e->velocity.up_ms; 
    }
}

static void _nav_update(mim_skymap_handle_t h, int64_t time) {
    nav_command_t cmd = {0};
    cmd.type = NAV_NONE;

    if (!mim_skymap_is_ready(h)) {
        h->time_last_update = 0;
        nav_pitcher_reset(&h->state_pitcher);
    } else if (h->time_last_update > 0) {
        // advance interceptor and target from their last known positions
        nav_state_advance(&h->state_interceptor, time);
        nav_state_advance(&h->state_target, time);

        nav_compute_command(&mim_settings_get()->nav, &h->state_interceptor, &h->state_target, &cmd);
    }

    // reset overrides
    if (cmd.type != NAV_NONE) {
        float dt_s = (float)(time - h->time_last_update) / 1000000.0;

        la_float max_roll_rad = NAV_DEG_TO_RAD(mim_settings_get()->nav.max_roll_deg);

        la_float roll_cmd = la_clamp2(la_atan2(cmd.accel_lat, NAV_G) / max_roll_rad, 1);
        la_float pitch_cmd = nav_pitcher_update(&mim_settings_get()->pitcher, &h->state_pitcher, dt_s, h->state_interceptor.vel_up, cmd.accel_ver);

        uint16_t roll_ticks = CRSF_RC_CHANNELS_CENTER + roll_cmd * CRSF_RC_CHANNELS_RANGE / 2;
        uint16_t pitch_ticks = CRSF_RC_CHANNELS_CENTER + pitch_cmd * CRSF_RC_CHANNELS_RANGE / 2;

        mim_rc_set_override(MIM_RC_CHANNEL_ROLL, roll_ticks, MIM_RC_OVERRIDE_LEVEL_GUIDANCE);
        mim_rc_set_override(MIM_RC_CHANNEL_PITCH, pitch_ticks, MIM_RC_OVERRIDE_LEVEL_GUIDANCE);
    } else {
        mim_rc_reset_overrides();
    }

    h->time_last_update = time;
}
