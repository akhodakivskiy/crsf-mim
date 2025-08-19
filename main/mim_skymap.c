#include "mim_skymap.h"

#include "freertos/idf_additions.h"
#include "skymap.pb.h"
#include "libnet.h"
#include "mim_menu.h"
#include "mim_settings.h"

#include <esp_netif_ip_addr.h>
#include <esp_timer.h>
#include <esp_log.h>

static const char *TAG = "MIM_SKYMAP";

typedef struct {
    skymap_t skymap;
    ip4_addr_t skymap_addr;
    uint16_t skymap_port;
} skymap_ctx_t;

static void _udp_on_connected(void *user_ctx, ip4_addr_t *addr);
static void _udp_on_disconnected(void *user_ctx);
static void _udp_on_packet(void *user_ctx, uint8_t *data, uint16_t len, ip4_addr_t *addr_from, uint16_t port_from);
static void _task_skymap_impl(void *arg);

TaskHandle_t _task_skymap = NULL;
skymap_ctx_t _skymap_ctx;

void mim_skymap_init() {
    assert(_task_skymap == NULL);

    skymap_reset(&_skymap_ctx.skymap);
    mim_menu_set_skymap(&_skymap_ctx.skymap);

    assert(xTaskCreatePinnedToCore(_task_skymap_impl, "skymap", 4096, NULL, configMAX_PRIORITIES - 3, &_task_skymap, APP_CPU_NUM) == pdPASS);
}

static void _udp_on_connected(void *user_ctx, ip4_addr_t *addr) {
    skymap_ctx_t *ctx = (skymap_ctx_t*)(user_ctx);

    ESP_LOGI(TAG, "connected, IP=" IPSTR, IP2STR(addr));
    skymap_reset(&ctx->skymap);
}

static void _udp_on_disconnected(void *user_ctx) {
    skymap_ctx_t *ctx = (skymap_ctx_t*)(user_ctx);

    ESP_LOGI(TAG, "disconnected");
    skymap_reset(&ctx->skymap);
}

static void _udp_on_packet(void *user_ctx, uint8_t *data, uint16_t len, ip4_addr_t *addr_from, uint16_t port_from) {
    skymap_ctx_t *ctx = (skymap_ctx_t*)(user_ctx);

    ctx->skymap_addr = *addr_from;
    ctx->skymap_port = port_from;

    if (skymap_read_server_message(&ctx->skymap, esp_timer_get_time(), data, len) != SKYMAP_OK) {
        ESP_LOGE(TAG, "failed to parse Skymap packet from " IPSTR ", len=%u", IP2STR(addr_from), len);
    }
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

    while(true) {
        if (_skymap_ctx.skymap.is_client_message_ready) {
            uint8_t data[ai_skyfortress_guidance_ClientMessage_size + 4];

            uint16_t len_written = 0;
            if (skymap_write_client_message(&_skymap_ctx.skymap, esp_timer_get_time(), data, sizeof(data), &len_written) == SKYMAP_OK) {
                ESP_ERROR_CHECK(libnet_udp_send(&_skymap_ctx.skymap_addr, _skymap_ctx.skymap_port, data, len_written));
                ESP_LOGI(TAG, "sending client status message");
            }
        } else {
            skymap_update(&_skymap_ctx.skymap, esp_timer_get_time());
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

