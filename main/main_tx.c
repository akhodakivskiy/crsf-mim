#include "freertos/projdefs.h"
#include "skymap.pb.h"
#include <sdkconfig.h>

#ifdef CONFIG_CRSF_MIM_FIRMWARE_TX

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <driver/uart.h>
#include <driver/gptimer.h>
#include <hal/uart_types.h>
#include <lwip/ip4_addr.h>
#include <esp_netif_ip_addr.h>

#include <stdint.h>
#include <string.h>

#include "async_logging.h"
#include "libcrsf_parser.h"
#include "mim_menu.h"
#include "mim_uart.h"
#include "mim_settings.h"
#include "mim_skymap.h"

static const char *TAG= "MAIN_TX";
static const char *TAG_CONTROLLER = "MAIN_TX_CONTROLLER";
static const char *TAG_MODULE = "MAIN_TX_MODULE";
static const char *TAG_SKYMAP = "MAIN_TX_SKYMAP";


#define UART_PORT_CONTROLLER UART_NUM_1
#define UART_PORT_MODULE UART_NUM_2

#define UART_PIN_CONTROLLER GPIO_NUM_46
#define UART_PIN_MODULE GPIO_NUM_45

typedef struct {
    const char *name;
    uint32_t frames;
    uint32_t frames_by_type[UINT8_MAX];
} frame_counter_t;

static void _frame_handler_controller(crsf_frame_t *frame);
static void _frame_handler_module(crsf_frame_t *frame);
static void _count_frames(const crsf_frame_t *frame, frame_counter_t *counter);
static void _skymap_handler(crsf_frame_t *frame);

crsf_device_t crsf_device;

frame_counter_t counter_controller;
frame_counter_t counter_module;

void app_main(void) {
    memset(&counter_controller, 0, sizeof(frame_counter_t));
    counter_controller.name = "CONTROLLER";
    memset(&counter_module, 0, sizeof(frame_counter_t));
    counter_module.name = "MODULE";

    async_logging_init(PRO_CPU_NUM);

    // init and load settings
    mim_settings_init();
    mim_settings_load();

    // init UART tasks and set frame handler
    mim_uart_init(UART_PORT_CONTROLLER, UART_PIN_CONTROLLER, UART_PORT_MODULE, UART_PIN_MODULE);
    mim_uart_set_controller_handler(_frame_handler_controller);
    mim_uart_set_module_handler(_frame_handler_module);

    // init CRSF device menu
    mim_menu_init(&crsf_device);

    //mim_skymap_init();
}

static void _frame_handler_controller(crsf_frame_t *frame) {
    _count_frames(frame, &counter_controller);

    crsf_frame_t frame_out;
    crsf_device_result_t err = crsf_device_client_handler(&crsf_device, frame, &frame_out);

    if (err == CRSF_DEVICE_RESULT_OK) {
        // first handle menu frame
        mim_uart_enqueue_module_frame(&frame_out);
    } else {
        // then run skymap handler
        _skymap_handler(frame);
    }
}

static void _frame_handler_module(crsf_frame_t *frame) {
    _count_frames(frame, &counter_module);
}

static void _count_frames(const crsf_frame_t *frame, frame_counter_t *counter) {

    counter->frames += 1;
    counter->frames_by_type[frame->type] = counter->frames_by_type[frame->type] + 1;

    if (counter->frames % 100 == 0) {
        char log[256];
        size_t log_cursor = 0;
        for (int i = 0; i < UINT8_MAX; i++) {
            if (counter->frames_by_type[i] > 0) {
                log_cursor += snprintf(log + log_cursor, 256 - log_cursor, "0x%x=%lu, ", i, counter->frames_by_type[i]);
            }
        }

        ESP_LOGI(TAG, "%s: %s", counter->name, log);
    }
}

static void _skymap_handler(crsf_frame_t *frame) {
}

#endif
