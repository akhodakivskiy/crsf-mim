#include "freertos/projdefs.h"
#include "libcrsf_payload.h"
#include "skymap.pb.h"
#include <sdkconfig.h>

#ifdef CONFIG_CRSF_MIM_FIRMWARE_TX

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
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
#include "mim_rc.h"

static const char *TAG= "MAIN_TX";
static const char *TAG_CONTROLLER = "MAIN_TX_CONTROLLER";
static const char *TAG_MODULE = "MAIN_TX_MODULE";
static const char *TAG_SKYMAP = "MAIN_TX_SKYMAP";

#define PRIORITY_TASK_ASYNC_LOGGING (configMAX_PRIORITIES - 5)
#define PRIORITY_TASK_SKYMAP (configMAX_PRIORITIES - 3)
#define PRIORITY_TASK_CONTROLLER (configMAX_PRIORITIES - 2)
#define PRIORITY_TASK_MODULE (configMAX_PRIORITIES - 1)

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

frame_counter_t _counter_controller;
frame_counter_t _counter_module;

void app_main(void) {
    memset(&_counter_controller, 0, sizeof(frame_counter_t));
    _counter_controller.name = "CONTROLLER";
    memset(&_counter_module, 0, sizeof(frame_counter_t));
    _counter_module.name = "MODULE";

    async_logging_init(PRIORITY_TASK_ASYNC_LOGGING, PRO_CPU_NUM);

    // init and load settings
    mim_settings_init();
    mim_settings_load();

    // init UART tasks and set frame handler
    mim_uart_init(PRIORITY_TASK_CONTROLLER, UART_PORT_CONTROLLER, UART_PIN_CONTROLLER, 
                  PRIORITY_TASK_MODULE, UART_PORT_MODULE, UART_PIN_MODULE);
    mim_uart_set_controller_handler(_frame_handler_controller);
    mim_uart_set_module_handler(_frame_handler_module);

    // init CRSF device menu
    mim_menu_init(&crsf_device);

    mim_skymap_init(PRIORITY_TASK_SKYMAP);
}

static IRAM_ATTR void _frame_handler_controller(crsf_frame_t *frame) {
    _count_frames(frame, &_counter_controller);

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

static IRAM_ATTR void _frame_handler_module(crsf_frame_t *frame) {
    _count_frames(frame, &_counter_module);

    /*
    union {
        crsf_payload_gps_t gps;
        crsf_payload_baro_altitude_t alt;
        crsf_payload_vario_t vario;
    } f;

    if (crsf_payload_unpack__gps(frame, &f.gps)) {
        ESP_LOGI(TAG, "GPS lat=%li, lon=%li, gs=%u, heading=%.2f, alt=%u, sats=%u",
                 f.gps.latitude, f.gps.longitude, f.gps.groundspeed_kmh, f.gps.heading_cdeg / 100.0, f.gps.altitude_m, f.gps.satellites);
    } else if (crsf_payload_unpack__baro_altitude(frame, &f.alt)) {
        ESP_LOGI(TAG, "BARO-ALT %.4fm, %.4f", f.alt.altitude_dm / 10.0, f.alt.vertical_speed_cm_s / 100.0);
    } else if (crsf_payload_unpack__vario(frame, &f.vario)) {
        ESP_LOGI(TAG, "VARIO %.4f", f.vario.vspeed_cms / 100.0);
    }
    */
}

static IRAM_ATTR void _count_frames(const crsf_frame_t *frame, frame_counter_t *counter) {

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
        ESP_LOGI(TAG, "%s - %s", counter->name, log);
    }
}

static void IRAM_ATTR _skymap_handler(crsf_frame_t *frame) {
    crsf_payload_rc_channels_t c;
    if (crsf_payload_unpack__rc_channels(frame, &c)) {
        // enable skymap based on the engage channel
        uint8_t ch_engage = mim_settings_get()->engage_channel - 1;
        if (ch_engage >= 0 && ch_engage < 16) {
            bool guidance_enabled = c.channels[ch_engage] > CRSF_RC_CHANNELS_CENTER;
            mim_skymap_guidance_enable(guidance_enabled);
        } else if (mim_skymap_guidance_is_enabled()) {
            mim_skymap_guidance_enable(false);
            mim_rc_clear_overrides();
        }

        // override channels
        uint16_t roll = mim_rc_apply_override(MIM_RC_CHANNEL_ROLL, c.channels[MIM_RC_CHANNEL_ROLL - 1]);
        uint16_t pitch = mim_rc_apply_override(MIM_RC_CHANNEL_PITCH, c.channels[MIM_RC_CHANNEL_PITCH - 1]);

        c.channels[MIM_RC_CHANNEL_ROLL] = roll;
        c.channels[MIM_RC_CHANNEL_PITCH] = pitch;

        crsf_payload_pack__rc_channels(frame, &c);

        static int64_t last_log_time = 0;
        if (esp_timer_get_time() > last_log_time + 1000000) {
            last_log_time = esp_timer_get_time();
            /*
            ESP_LOGI(TAG, "%u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u",
                     c.channels[0], c.channels[1], c.channels[2], c.channels[3],
                     c.channels[4], c.channels[5], c.channels[6], c.channels[7],
                     c.channels[8], c.channels[9], c.channels[10], c.channels[11],
                     c.channels[12], c.channels[13], c.channels[14], c.channels[15]);
            ESP_LOGI(TAG, "channel: %u[%u], roll: %u, pitch: %u",
                     ch_engage, c.channels[ch_engage], roll, pitch);
            */
        }
    }
}

#endif
