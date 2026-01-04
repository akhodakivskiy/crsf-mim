#include "mim_conn.h"
#include "libnet.h"
// #include "mim_settings.h"
#include "skymap.h"

#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_netif_ip_addr.h>
#include <esp_timer.h>
#include <freertos/idf_additions.h>
#include <stdbool.h>

#define _MIM_CONN_TASK_STOP_BIT 1 << 0
#define _MIM_CONN_QUEUE_SIZE 10
#define _MIM_CONN_TASK_TIMEOUT_MS 100

static const char *TAG = "MIM_CONN";

struct mim_conn_s {
    mim_conn_config_t config;

    QueueHandle_t queue;
    TaskHandle_t task;
    libnet_handle_t libnet;

    ip4_addr_t mim_addr;
    ip4_addr_t skymap_addr;
    uint16_t skymap_port;

    bool is_connected;
};

static void _on_connected(void *user_ctx, ip4_addr_t *addr);
static void _on_disconnected(void *user_ctx);
static void _on_packet(void *user_ctx, uint8_t *data, uint16_t len, ip4_addr_t *addr_from, uint16_t port_from);
static void _mim_conn_libnet_init(mim_conn_handle_t h);
static void _mim_conn_task(void *arg);

esp_err_t mim_conn_init(const mim_conn_config_t *config, mim_conn_handle_t *handle) {
    mim_conn_handle_t h
        = (mim_conn_handle_t)heap_caps_calloc(1, sizeof(struct mim_conn_s), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);

    memset(h, 0, sizeof(struct mim_conn_s));
    memcpy(&h->config, config, sizeof(mim_conn_config_t));

    h->queue = xQueueCreate(_MIM_CONN_QUEUE_SIZE, sizeof(mim_conn_msg_t));

    assert(pdPASS
           == xTaskCreatePinnedToCore(_mim_conn_task, "conn", 4096, h, config->priority, &h->task, config->core));

    *handle = h;

    return ESP_OK;
}

esp_err_t mim_conn_deinit(mim_conn_handle_t h) {
    assert(h != NULL);

    xTaskNotify(h->task, _MIM_CONN_TASK_STOP_BIT, eSetBits);
    while (h->task != NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    heap_caps_free(h);

    return ESP_OK;
}

bool mim_conn_is_connected(const mim_conn_handle_t h) { return h->is_connected; }
ip4_addr_t mim_conn_get_mim_addr(const mim_conn_handle_t h) { return h->mim_addr; }
ip4_addr_t mim_conn_get_skymap_addr(const mim_conn_handle_t h) { return h->skymap_addr; };

void mim_conn_send_skymap_message(mim_conn_handle_t h, const ai_skyfortress_guidance_ClientMessage *msg) {
    if (!ip4_addr_isany(&h->skymap_addr) && h->skymap_port > 0) {
        uint8_t data[128];
        uint16_t len_written = 0;

        assert(skymap_write_client_message(msg, data, 128, &len_written) == SKYMAP_OK);

        libnet_udp_send(h->libnet, &h->skymap_addr, h->skymap_port, data, len_written);
    }
}

void mim_conn_passthrough(mim_conn_handle_t h, const mim_conn_msg_t *msg) {
    assert(xQueueSend(h->queue, msg, 0) == pdTRUE);
}

static void _on_connected(void *user_ctx, ip4_addr_t *addr) {
    mim_conn_handle_t h = (mim_conn_handle_t)(user_ctx);

    h->is_connected = true;
    h->mim_addr = *addr;

    ESP_LOGI(TAG, "connected, IP=" IPSTR, IP2STR(addr));
}

static void _on_disconnected(void *user_ctx) {
    mim_conn_handle_t h = (mim_conn_handle_t)(user_ctx);

    h->is_connected = false;
    ip4_addr_set_any(&h->mim_addr);
    ip4_addr_set_any(&h->skymap_addr);
    h->skymap_port = 0;

    ESP_LOGI(TAG, "disconnected");
}

static void _on_packet(void *user_ctx, uint8_t *data, uint16_t len, ip4_addr_t *addr_from, uint16_t port_from) {
    mim_conn_handle_t h = (mim_conn_handle_t)(user_ctx);

    mim_conn_msg_t msg;
    msg.type = MIM_CONN_MSG_TYPE_SERVER;

    skymap_err_t err = skymap_read_server_message(&msg.message.server, data, len);

    if (err == SKYMAP_OK) {
        assert(xQueueSend(h->queue, &msg, 0) == pdTRUE);
        h->skymap_addr = *addr_from;
        h->skymap_port = port_from;
    } else {
        ESP_LOGE(TAG, "failed to parse Skymap packet from " IPSTR ", len=%u", IP2STR(addr_from), len);
    }
}

static void _mim_conn_libnet_init(mim_conn_handle_t h) {
    libnet_config_t cfg = {0};

    cfg.core_id = xTaskGetCoreID(NULL);
    cfg.priority = uxTaskPriorityGet(NULL);
    cfg.callbacks.connected = _on_connected;
    cfg.callbacks.disconnected = _on_disconnected;
    cfg.callbacks.packet = _on_packet;
    cfg.user_ctx = h;
    cfg.interface = LIBNET_INTERFACE_ETHERNET;

    /*
    switch (h->config.netmode) {
    case MIM_SETTINGS_MODE_ETHERNET:
        cfg.interface = LIBNET_INTERFACE_ETHERNET;
        break;
    case MIM_SETTINGS_MODE_WIFI:
        cfg.interface = LIBNET_INTERFACE_WIFI;
        strncpy(cfg.net.wifi.ssid, mim_settings_get()->wifi_ssid, sizeof(cfg.net.wifi.ssid));
        strncpy(cfg.net.wifi.password, mim_settings_get()->wifi_password, sizeof(cfg.net.wifi.password));
        break;
    }
    */

    ESP_ERROR_CHECK(libnet_init(&cfg, &h->libnet));
    ESP_ERROR_CHECK(libnet_udp_server_start(h->libnet, h->config.mim_port));
}

static void _mim_conn_task(void *arg) {
    mim_conn_handle_t h = (mim_conn_handle_t)arg;

    ESP_LOGI(TAG, "conn task started");

    _mim_conn_libnet_init(h);

    uint32_t notification_value;
    mim_conn_msg_t msg;

    while (true) {
        if (xTaskNotifyWait(0, _MIM_CONN_TASK_STOP_BIT, &notification_value, 0) == pdTRUE) {
            if (notification_value & _MIM_CONN_TASK_STOP_BIT) {
                break;
            }
        }

        if (xQueueReceive(h->queue, &msg, pdMS_TO_TICKS(_MIM_CONN_TASK_TIMEOUT_MS)) != pdTRUE) {
            // send a tick message if nothing from Skymap
            msg.type = MIM_CONN_MSG_TYPE_TICK;
        }

        int64_t timestamp_us = esp_timer_get_time();
        h->config.callback(h, timestamp_us, &msg, h->config.callback_arg);
    }

    ESP_LOGI(TAG, "conn task exit");

    h->task = NULL;
    vTaskDelete(NULL);
}
