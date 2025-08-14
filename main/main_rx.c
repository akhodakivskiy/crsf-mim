#include <sdkconfig.h>

#ifdef CONFIG_CRSF_MIM_FIRMWARE_RX

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

#include "libcrsf_parser.h"

static const char *TAG = "MAIN_RX";


#define CRSF_BAUDRATE 420000
#define CRSF_UART_PORT UART_NUM_1
#define CRSF_UART_PIN_TX GPIO_NUM_13
#define CRSF_UART_PIN_RX GPIO_NUM_14

TaskHandle_t task_crsf = NULL;
QueueHandle_t queue_crsf = NULL;

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

    _reset_crsf_parse_ctx(&ctx);

    while (true) {
        if (xQueueReceive(queue_crsf, &event, portMAX_DELAY) == pdTRUE) {
            if (event.type == UART_DATA) {
                size_t len_buffer = 0;
                ESP_ERROR_CHECK(uart_get_buffered_data_len(CRSF_UART_PORT, &len_buffer));
                assert(len_buffer < sizeof(buff));

                size_t len_read = uart_read_bytes(CRSF_UART_PORT, buff, len_buffer, portMAX_DELAY);
                ESP_LOG_BUFFER_HEX(TAG, buff, len_read);
                _reset_crsf_parse_ctx(&ctx);
                for (int i = 0; i < len_read; i++) {
                    crsf_frame_partial_parse(&ctx, buff[i]);
                    switch (ctx.status) {
                        case CRSF_FRAME_STATUS_FRAME_READY:
                            //ESP_LOGI(TAG, "frame %u, sync=%u, len=%u", ctx.frame->type, ctx.frame->sync, ctx.frame->length);
                            break;
                        case CRSF_FRAME_STATUS_ERROR_SYNC:
                        case CRSF_FRAME_STATUS_ERROR_LENGTH:
                        case CRSF_FRAME_STATUS_ERROR_CRC:
                            ESP_LOGI(TAG, "frame error: %u", ctx.status);
                            break;
                        default:
                            break;
                    }
                }
            } else {
                ESP_LOGI(TAG, "uart event=%u, size=%u, flag=%u", event.type, event.size, event.timeout_flag);
                xQueueReset(queue_crsf);
            }
            uart_flush_input(CRSF_UART_PORT);
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
        .rxfifo_full_thresh = 64,
    };

    ESP_ERROR_CHECK(uart_param_config(port, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(port, pin_tx, pin_rx, -1, -1));
    ESP_ERROR_CHECK(uart_driver_install(port, 256, 256, 512, queue, 0));
    ESP_ERROR_CHECK(uart_intr_config(port, &intr_cfg));
    ESP_ERROR_CHECK(uart_enable_rx_intr(port));
}

extern "C" void app_main() {
    _setup_uart(CRSF_UART_PORT, CRSF_UART_PIN_TX, CRSF_UART_PIN_RX, &queue_crsf);

    assert(xTaskCreatePinnedToCore(_task_crsf, "read-crsf", 4096, NULL, configMAX_PRIORITIES - 2, &task_crsf, APP_CPU_NUM) == pdPASS);
}

#endif
