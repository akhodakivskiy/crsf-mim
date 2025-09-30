#include "freertos/projdefs.h"
#include "libcrsf_payload.h"
#include "nav.h"
#include "skymap.pb.h"
#include "soc/gpio_num.h"
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
#include "mim_nav.h"
#include "mim_rc.h"

static const char *TAG= "MAIN_TX";
static const char *TAG_CONTROLLER = "MAIN_TX_CONTROLLER";
static const char *TAG_MODULE = "MAIN_TX_MODULE";
static const char *TAG_SKYMAP = "MAIN_TX_SKYMAP";

#define PRIORITY_TASK_ASYNC_LOGGING (configMAX_PRIORITIES - 3)
#define PRIORITY_TASK_SKYMAP (configMAX_PRIORITIES - 2)
#define PRIORITY_TASK_CRSF (configMAX_PRIORITIES - 1)

#define UART_PORT_CONTROLLER UART_NUM_1
#define UART_PORT_MODULE UART_NUM_2

#ifndef CONFIG_CRSF_MIM_PIN_CONTROLLER
#define CONFIG_CRSF_MIM_PIN_CONTROLLER GPIO_NUM_45
#endif

#ifndef CONFIG_CRSF_MIM_PIN_MODULE
#define CONFIG_CRSF_MIM_PIN_MODULE GPIO_NUM_46
#endif

static void _frame_handler_module(crsf_frame_t *frame);
static void _frame_handler_controller(crsf_frame_t *frame);
static void _frame_handler_controller_rc(crsf_frame_t *frame);

crsf_device_t crsf_device;
mim_nav_handle_t skymap;

void app_main(void) {
    async_logging_init(PRIORITY_TASK_ASYNC_LOGGING, PRO_CPU_NUM);

    // init and load settings
    ESP_LOGI(TAG, "initializing and loading settings");
    mim_settings_init();
    mim_settings_load();

    ESP_LOGI(TAG, "initializing navigation");
    ESP_ERROR_CHECK(mim_nav_init(PRIORITY_TASK_SKYMAP, &skymap));

    // init CRSF device menu
    ESP_LOGI(TAG, "initializing CRSF");
    mim_menu_init(&crsf_device, skymap);

    ESP_LOGI(TAG, "initializing UART");
    mim_uart_set_module_handler(_frame_handler_module);
    mim_uart_set_controller_handler(_frame_handler_controller);

    // init UART tasks and set frame handler
    mim_uart_init(PRIORITY_TASK_CRSF, 
                  UART_PORT_CONTROLLER, (gpio_num_t)CONFIG_CRSF_MIM_PIN_CONTROLLER, 
                  UART_PORT_MODULE, (gpio_num_t)CONFIG_CRSF_MIM_PIN_MODULE);
}

static IRAM_ATTR void _frame_handler_module(crsf_frame_t *frame) {
    mim_nav_msg_t msg;
    msg.type = MIM_NAV_MSG_TYPE_CRSF;

    if (crsf_payload_unpack__gps(frame, &msg.message.crsf.message.gps)) {
        msg.message.crsf.type = MIM_NAV_CRSF_SUBSET_TYPE_GPS;
    } else if (crsf_payload_unpack__baro_altitude(frame, &msg.message.crsf.message.alt)) {
        msg.message.crsf.type = MIM_NAV_CRSF_SUBSET_TYPE_ALT;
    } else if (crsf_payload_unpack__vario(frame, &msg.message.crsf.message.vario)) {
        msg.message.crsf.type = MIM_NAV_CRSF_SUBSET_TYPE_VARIO;
    } else if (crsf_payload_unpack__airspeed(frame, &msg.message.crsf.message.airspeed)) {
        msg.message.crsf.type = MIM_NAV_CRSF_SUBSET_TYPE_AIRSPEED;
    } else {
        return;
    }

    mim_nav_enqueue(skymap, &msg);
}

static IRAM_ATTR void _frame_handler_controller(crsf_frame_t *frame) {
    crsf_frame_t frame_out;
    crsf_device_result_t err = crsf_device_client_handler(&crsf_device, frame, &frame_out);

    if (err == CRSF_DEVICE_RESULT_OK) {
        // first handle menu frame
        mim_uart_enqueue_module_frame(&frame_out);
    } else {
        // then run skymap handler
        _frame_handler_controller_rc(frame);
    }
}

static void IRAM_ATTR _frame_handler_controller_rc(crsf_frame_t *frame) {
    crsf_payload_rc_channels_t rc;
    if (crsf_payload__rc_channels_unpack(frame, &rc)) {
        // save channel values
        mim_rc_set_crsf_channels(&rc);

        // enable skymap based on the engage channel value
        mim_rc_channel_t engage_channel = mim_settings_get()->engage_channel;
        bool is_engaging = (engage_channel < MIM_RC_CHANNEL__MAX) &&
            (mim_rc_get_channel(engage_channel) > CRSF_RC_CHANNELS_CENTER);

        mim_nav_set_engaging(skymap, is_engaging);

        if (is_engaging) {
            // override channels
            uint16_t roll = mim_rc_get_channel_with_override(MIM_RC_CHANNEL_ROLL);
            uint16_t pitch = mim_rc_get_channel_with_override(MIM_RC_CHANNEL_PITCH);

            crsf_payload__rc_channels_set(&rc, MIM_RC_CHANNEL_ROLL, roll);
            crsf_payload__rc_channels_set(&rc, MIM_RC_CHANNEL_PITCH, pitch);

            crsf_payload__rc_channels_pack(frame, &rc);


            static int64_t last_log_time = 0;
            if (esp_timer_get_time() > last_log_time + 1000000) {
                last_log_time = esp_timer_get_time();
                /*
            ESP_LOGI(TAG, "%u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u",
                     rc.ch1, rc.ch2,  rc.ch3,  rc.ch4,  rc.ch5,  rc.ch6,  rc.ch7,  rc.ch8, 
                     rc.ch9, rc.ch10, rc.ch11, rc.ch12, rc.ch13, rc.ch14, rc.ch15, rc.ch16);
            ESP_LOGI(TAG, "engage[channel=%u, value=%u], roll: %u, pitch: %u",
                     engage_channel, is_engaging, roll, pitch);
        */
                const nav_command_t *cmd = mim_nav_get_last_command(skymap);
                ESP_LOGI(TAG, "type=%u, range=%.2f, ah=%.2f, av=%.2f, roll=%u, pitch=%u", 
                         cmd->type, cmd->range, cmd->accel_lat, cmd->accel_ver, roll, pitch);
            }
        }
    }
}

#endif
