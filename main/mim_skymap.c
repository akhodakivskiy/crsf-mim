#include "mim_skymap.h"

#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "portmacro.h"
#include "skymap.h"
#include "skymap.pb.h"
#include "libnet.h"
#include "mim_menu.h"
#include "mim_settings.h"
#include "mim_rc.h"
#include "nav_guidance.h"

#include <esp_netif_ip_addr.h>
#include <esp_timer.h>
#include <esp_log.h>

static const char *TAG = "MIM_SKYMAP";

typedef struct {
    skymap_t skymap;
    ip4_addr_t skymap_addr;
    uint16_t skymap_port;
    SemaphoreHandle_t skymap_sem;

    bool guidance_enabled;
    nav_guidance_t guidance_config;
    nav_guidance_command_t guidance_command;
} skymap_ctx_t;

static void _udp_on_connected(void *user_ctx, ip4_addr_t *addr);
static void _udp_on_disconnected(void *user_ctx);
static void _udp_on_packet(void *user_ctx, uint8_t *data, uint16_t len, ip4_addr_t *addr_from, uint16_t port_from);
static void _task_skymap_impl(void *arg);
static void _skymap_update();

TaskHandle_t _task_skymap = NULL;
skymap_ctx_t _skymap_ctx;

void mim_skymap_init() {
    assert(_task_skymap == NULL);

    skymap_init(&_skymap_ctx.skymap);
    mim_menu_set_skymap(&_skymap_ctx.skymap);

    _skymap_ctx.skymap_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(_skymap_ctx.skymap_sem);

    _skymap_ctx.skymap_port = 0;
    ip4_addr_set_any(&_skymap_ctx.skymap_addr);

    memset(&_skymap_ctx.guidance_command, 0, sizeof(nav_guidance_command_t));

    _skymap_ctx.guidance_enabled = false;
    _skymap_ctx.guidance_config.N = mim_settings_get()->guidance.N;
    _skymap_ctx.guidance_config.max_roll_deg = mim_settings_get()->guidance.max_roll_deg;
    _skymap_ctx.guidance_config.max_pitch_deg = mim_settings_get()->guidance.max_pitch_deg;

    assert(xTaskCreatePinnedToCore(_task_skymap_impl, "skymap", 4096, NULL, configMAX_PRIORITIES - 3, &_task_skymap, APP_CPU_NUM) == pdPASS);
}

bool mim_skymap_guidance_is_enabled() {
    return _skymap_ctx.guidance_enabled;
}

void mim_skymap_guidance_enable(bool enable) {
    _skymap_ctx.guidance_enabled = enable;
}

void mim_skymap_get_command(mim_skymap_command_t *command) {
    xSemaphoreTake(_skymap_ctx.skymap_sem, portMAX_DELAY);
    command->is_valid = _skymap_ctx.skymap.is_ready_to_engage && (_skymap_ctx.guidance_command.type != NAV_GUIDANCE_NONE);
    if (_skymap_ctx.guidance_enabled && command->is_valid) {
        command->roll_cmd = _skymap_ctx.guidance_command.roll_cmd;
        command->pitch_cmd = _skymap_ctx.guidance_command.pitch_cmd;
    } else {
        command->roll_cmd = 0.0;
        command->pitch_cmd = 0.0;
    }
    xSemaphoreGive(_skymap_ctx.skymap_sem);
}

static void _udp_on_connected(void *user_ctx, ip4_addr_t *addr) {
    skymap_ctx_t *ctx = (skymap_ctx_t*)(user_ctx);

    ESP_LOGI(TAG, "connected, IP=" IPSTR, IP2STR(addr));
    xSemaphoreTake(_skymap_ctx.skymap_sem, portMAX_DELAY);
    skymap_init(&ctx->skymap);
    xSemaphoreGive(_skymap_ctx.skymap_sem);
}

static void _udp_on_disconnected(void *user_ctx) {
    skymap_ctx_t *ctx = (skymap_ctx_t*)(user_ctx);

    ESP_LOGI(TAG, "disconnected");
    xSemaphoreTake(_skymap_ctx.skymap_sem, portMAX_DELAY);
    skymap_init(&ctx->skymap);
    xSemaphoreGive(_skymap_ctx.skymap_sem);
}

static void _udp_on_packet(void *user_ctx, uint8_t *data, uint16_t len, ip4_addr_t *addr_from, uint16_t port_from) {
    skymap_ctx_t *ctx = (skymap_ctx_t*)(user_ctx);

    ctx->skymap_addr = *addr_from;
    ctx->skymap_port = port_from;

    xSemaphoreTake(_skymap_ctx.skymap_sem, portMAX_DELAY);
    skymap_err_t err = skymap_read_server_message(&ctx->skymap, esp_timer_get_time(), data, len);

    if (err == SKYMAP_OK) {
        _skymap_update();
    } else {
        ESP_LOGE(TAG, "failed to parse Skymap packet from " IPSTR ", len=%u", IP2STR(addr_from), len);
    }
    xSemaphoreGive(_skymap_ctx.skymap_sem);
}

static void _task_skymap_impl(void *arg) {
    libnet_config_t libnet_cfg = {0};

    libnet_cfg.core_id = PRO_CPU_NUM;
    libnet_cfg.callbacks.connected = _udp_on_connected;
    libnet_cfg.callbacks.disconnected = _udp_on_disconnected;
    libnet_cfg.callbacks.packet = _udp_on_packet;
    libnet_cfg.user_ctx = &_skymap_ctx;

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

    libnet_init(&libnet_cfg);
    libnet_udp_server_start(8888);

    while (true) {
        xSemaphoreTake(_skymap_ctx.skymap_sem, portMAX_DELAY);
        if (_skymap_ctx.skymap.is_client_message_ready) {
            uint8_t data[ai_skyfortress_guidance_ClientMessage_size + 4];

            uint16_t len_written = 0;

            if (skymap_write_client_message(&_skymap_ctx.skymap, esp_timer_get_time(), data, sizeof(data), &len_written) == SKYMAP_OK) {
                ESP_ERROR_CHECK(libnet_udp_send(&_skymap_ctx.skymap_addr, _skymap_ctx.skymap_port, data, len_written));
            }
        } else {
            skymap_update(&_skymap_ctx.skymap, esp_timer_get_time());
        }
        xSemaphoreGive(_skymap_ctx.skymap_sem);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void _skymap_update() {
    if (_skymap_ctx.guidance_enabled && _skymap_ctx.skymap.is_ready_to_engage) {
        skymap_t *sm = &_skymap_ctx.skymap;

        nav_guidance_state_t state_interceptor = {
            .lat = sm->interceptor.position.latitude_deg,
            .lon = sm->interceptor.position.longitude_deg,
            .alt = sm->interceptor.position.altitude_msl_m,
            .vel_north = sm->interceptor.velocity.north_ms,
            .vel_east = sm->interceptor.velocity.east_ms,
            .vel_up = sm->interceptor.velocity.up_ms
        };

        nav_guidance_state_t state_target = {
            .lat = sm->target.position.latitude_deg,
            .lon = sm->target.position.longitude_deg,
            .alt = sm->target.position.altitude_msl_m,
            .vel_north = sm->target.velocity.north_ms,
            .vel_east = sm->target.velocity.east_ms,
            .vel_up = sm->target.velocity.up_ms
        };

        nav_guidance_compute_command(
            &_skymap_ctx.guidance_config,
            &state_interceptor,
            &state_target,
            &_skymap_ctx.guidance_command
        );

        if (_skymap_ctx.guidance_command.type == NAV_GUIDANCE_NONE) {
            mim_rc_clear_overrides();
        } else {
            mim_rc_override_channel(MIM_RC_CHANNEL_ROLL, _skymap_ctx.guidance_command.roll_cmd, MIM_RC_OVERRIDE_LEVEL_GUIDANCE);
            mim_rc_override_channel(MIM_RC_CHANNEL_PITCH, _skymap_ctx.guidance_command.pitch_cmd, MIM_RC_OVERRIDE_LEVEL_GUIDANCE);
        }
    }
}
