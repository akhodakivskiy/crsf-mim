#include <unity.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>

#include <sdkconfig.h>

#include "libnet.h"

static const char *TAG = "TEST_LIBNET";

void _cb_connected(void *user_ctx, ip4_addr_t *addr) {
    SemaphoreHandle_t sem = (SemaphoreHandle_t)user_ctx;
    xSemaphoreGive(sem);
}

void _cb_disconnected(void *user_ctx) {
    SemaphoreHandle_t sem = (SemaphoreHandle_t)user_ctx;
    xSemaphoreGive(sem);
}

void _cb_packet(void *user_ctx, uint8_t* data, uint16_t len, ip4_addr_t *addr_from, uint16_t port) {
}

TEST_CASE("test ethernet", "la") {
    for (int i = 0; i < CONFIG_CRSF_MIM_TEST_NETIF_REPEATS; i++) {
        SemaphoreHandle_t sem = xSemaphoreCreateBinary();

        libnet_config_t config = {
            .priority = configMAX_PRIORITIES - 5,
            .core_id = PRO_CPU_NUM,
            .callbacks = {
                .connected = _cb_connected,
                .disconnected = _cb_disconnected,
                .packet = _cb_packet,
            },
            .interface = LIBNET_INTERFACE_ETHERNET,
            .net = {
                .eth = {}
            },
            .user_ctx = sem
        };

        libnet_handle_t handle;

        ESP_LOGI(TAG, "connecting %i...", i);

        TEST_ASSERT_EQUAL(ESP_OK, libnet_init(&config, &handle));
        TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(sem, pdMS_TO_TICKS(10000)));

        ESP_LOGI(TAG, "disconnecting %i...", i);

        TEST_ASSERT_EQUAL(ESP_OK, libnet_deinit(handle));
        TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(sem, pdMS_TO_TICKS(10000)));

        vSemaphoreDelete(sem);
    }
}


TEST_CASE("test wifi", "la") {
    for (int i = 0; i < CONFIG_CRSF_MIM_TEST_NETIF_REPEATS; i++) {
        SemaphoreHandle_t sem = xSemaphoreCreateBinary();

        libnet_config_t config = {
            .priority = configMAX_PRIORITIES - 5,
            .core_id = PRO_CPU_NUM,
            .callbacks = {
                .connected = _cb_connected,
                .disconnected = _cb_disconnected,
                .packet = _cb_packet,
            },
            .interface = LIBNET_INTERFACE_WIFI,
            .net = {
                .wifi = {
                    .ssid = CONFIG_CRSF_MIM_TEST_WIFI_SSID,
                    .password = CONFIG_CRSF_MIM_TEST_WIFI_PASSWORD
                }
            },
            .user_ctx = sem
        };

        libnet_handle_t handle;

        ESP_LOGI(TAG, "connecting %i...", i);

        TEST_ASSERT_EQUAL(ESP_OK, libnet_init(&config, &handle));
        TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(sem, pdMS_TO_TICKS(10000)));

        ESP_LOGI(TAG, "disconnecting %i...", i);

        TEST_ASSERT_EQUAL(ESP_OK, libnet_deinit(handle));
        TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(sem, pdMS_TO_TICKS(10000)));

        vSemaphoreDelete(sem);
    }
}
