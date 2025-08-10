#include <sdkconfig.h>

#ifdef CONFIG_CRSF_FIRMWARE_RX

#include <driver/uart.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "soc/gpio_num.h"
#include <cstdint>

#include "driver/uart.h"
#include "hal/uart_types.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <soc/soc.h>
#include <hal/uart_ll.h>

#include "protocol_crsf.h"

#include <math.h>

static const char *TAG = "MAIN_RX";


#define CRSF_BAUDRATE 420000
#define CRSF_UART_PORT UART_NUM_1
#define CRSF_UART_PIN_TX GPIO_NUM_14
#define CRSF_UART_PIN_RX GPIO_NUM_13

#ifndef CRSF_FRAMETYPE_RC_CHANNELS_PACKED
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#endif

#define US_MIN   988
#define US_MAX   2012
#define US_SPAN  (US_MAX - US_MIN)   // 1024
#define US_MID   1500
#define US_HALF  (US_SPAN/2)         // 512

#ifndef GUID_H_MAX_M
#define GUID_H_MAX_M 500.0
#endif

#define MODE_SWITCH_CH_IDX 4
#define MODE_ON_TH   1550
#define MODE_OFF_TH  1450

TaskHandle_t task_crsf = NULL;
QueueHandle_t queue_crsf = NULL;

static inline const uint8_t* crsf_payload_ptr_ro(const crsf_frame_t* f) {
    return ((const uint8_t*)&f->type) + 1;
}

static inline uint16_t crsf_raw_to_us(uint16_t raw) {
    if (raw < 172) raw = 172; else if (raw > 1811) raw = 1811;
    return (uint16_t)(US_MIN + ((uint32_t)(raw - 172) * 1024u) / 1639u);
}

static inline bool crsf_unpack_rc_channels_us(const crsf_frame_t *f, uint16_t ch_us[16]) {
    if (f->type != CRSF_FRAMETYPE_RC_CHANNELS_PACKED) return false;

    const uint8_t *p = crsf_payload_ptr_ro(f); // 22 bytes
    for (int i = 0; i < 16; ++i) {
        uint32_t bitpos = (uint32_t)i * 11u;
        uint32_t byteix = bitpos >> 3;
        uint32_t bitoff = bitpos & 7u;

        uint32_t b0 = p[byteix];
        uint32_t b1 = (byteix + 1 < 22) ? p[byteix + 1] : 0;
        uint32_t b2 = (byteix + 2 < 22) ? p[byteix + 2] : 0;
        uint32_t w  = b0 | (b1 << 8) | (b2 << 16);

        uint16_t raw = (w >> bitoff) & 0x7FFu;
        ch_us[i] = crsf_raw_to_us(raw);
    }
    return true;
}

static inline double us_to_azimuth_deg(uint16_t us){
    double d = (double)(us - US_MIN) * 360.0 / (double)US_SPAN;
    if (d < 0) d = 0; else if (d > 360) d = 360;
    return d;
}
static inline double us_to_height_m(uint16_t us){
    double h = ((double)us - (double)US_MID) * (GUID_H_MAX_M / (double)US_HALF);
    if (h < -GUID_H_MAX_M) h = -GUID_H_MAX_M;
    if (h >  GUID_H_MAX_M) h =  GUID_H_MAX_M;
    return h;
}

void _reset_crsf_parse_ctx(crsf_parse_ctx_t *ctx) {
    crsf_frame_t *frame = ctx->frame;
    memset(ctx, 0, sizeof(crsf_parse_ctx_t));
    memset(frame, 0, sizeof(crsf_frame_t));

    ctx->frame = frame;
}

static void _task_crsf(void *arg) {
    uint8_t buff[256];
    uart_event_t event;

    crsf_frame_t frame;
    crsf_parse_ctx_t ctx;
    ctx.frame = &frame;

    static bool s_guidance = false;
    static int64_t s_last_log_us = 0;

    while (true) {
        if (xQueueReceive(queue_crsf, &event, portMAX_DELAY) == pdTRUE) {
            if (event.type == UART_DATA) {
                size_t len_buffer = 0;
                ESP_ERROR_CHECK(uart_get_buffered_data_len(CRSF_UART_PORT, &len_buffer));
                assert(len_buffer < sizeof(buff));

                size_t len_read = uart_read_bytes(CRSF_UART_PORT, buff, len_buffer, portMAX_DELAY);

                // ESP_LOG_BUFFER_HEX(TAG, buff, len_read);

                _reset_crsf_parse_ctx(&ctx);
                for (int i = 0; i < (int)len_read; i++) {
                    crsf_frame_partial_parse(&ctx, buff[i]);
                    switch (ctx.status) {
                        case CRSF_FRAME_STATUS_FRAME_READY: {
                            const crsf_frame_t* f = ctx.frame;
                            // ESP_LOGI("MAIN_RX", "CRSF frame type=0x%02X len=%u", f->type, f->length);
                            
                            if (f->type == 0x16 && f->length == 24) {
                                uint16_t ch_us[16];
                                if (crsf_unpack_rc_channels_us(f, ch_us)) {
                                    int64_t now = esp_timer_get_time();
                                    if (now - s_last_log_us >= 100000) {
                                    s_last_log_us = now;
                                    double az_rel_deg   = us_to_azimuth_deg(ch_us[0]); // CH1
                                    double rel_height_m = us_to_height_m(ch_us[2]);    // CH3
                                    ESP_LOGI("MAIN_RX",
                    "[RC] CH1=%u µs (az=%.1f°) | CH3=%u µs (h=%.1f m) | CH5=%u µs", ch_us[0], az_rel_deg, ch_us[2], rel_height_m, ch_us[4]);
                    }
                    }
                    }
                    _reset_crsf_parse_ctx(&ctx);
                    break;
}
                        case CRSF_FRAME_STATUS_ERROR_SYNC:
                        case CRSF_FRAME_STATUS_ERROR_LENGTH:
                        case CRSF_FRAME_STATUS_ERROR_CRC:
                            ESP_LOGI(TAG, "frame error: %u", ctx.status);
                            _reset_crsf_parse_ctx(&ctx);
                            break;
                        default:
                            break;
                    }
                }
            } 
            else if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
                ESP_LOGW(TAG, "UART overflow/full → flush & reset");
                uart_flush_input(CRSF_UART_PORT);
                xQueueReset(queue_crsf);
                _reset_crsf_parse_ctx(&ctx);
                }
            else {
                ESP_LOGI(TAG, "uart event=%u, size=%u, flag=%u", event.type, event.size, event.timeout_flag);
                xQueueReset(queue_crsf);
            }
        }
    }
}

void _setup_uart(uart_port_t port, gpio_num_t pin_tx, gpio_num_t pin_rx, 
                 QueueHandle_t *queue) {

    uart_config_t cfg = {
        .baud_rate = CRSF_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {
            .allow_pd = 0,
            .backup_before_sleep = 0,
        },
    };

    uint32_t uart_intr_mask = UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_OVF | UART_INTR_RXFIFO_TOUT;

    uart_intr_config_t intr_cfg {
        .intr_enable_mask = uart_intr_mask,
        .rx_timeout_thresh = 2,
        .txfifo_empty_intr_thresh = 0,
        .rxfifo_full_thresh = 120,
    };

    ESP_ERROR_CHECK(uart_param_config(port, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(port, pin_tx, pin_rx, -1, -1));
    ESP_ERROR_CHECK(uart_driver_install(port, 4096, 4096, 64, queue, 0));
    ESP_ERROR_CHECK(uart_intr_config(port, &intr_cfg));
    ESP_ERROR_CHECK(uart_enable_rx_intr(port));
}

extern "C" void app_main() {
    _setup_uart(CRSF_UART_PORT, CRSF_UART_PIN_TX, CRSF_UART_PIN_RX, &queue_crsf);

    assert(xTaskCreatePinnedToCore(_task_crsf, "read-crsf", 4096, NULL, configMAX_PRIORITIES - 2, &task_crsf, APP_CPU_NUM) == pdPASS);
}

#endif