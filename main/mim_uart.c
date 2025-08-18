#include "mim_uart.h"

#include "driver/uart.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "hal/uart_types.h"
#include "libcrsf.h"
#include "libcrsf_def.h"
#include "libcrsf_def.h"
#include "portmacro.h"
#include "soc/gpio_num.h"
#include "util_lqcalc.h"

#include <hal/uart_ll.h>
#include <soc/uart_periph.h>
#include <esp_log.h>
#include <esp_timer.h>

#define CRSF_BAUDRATE 400000

static const char *TAG= "MIM_UART";
static const char *TAG_CONTROLLER = "MIM_UART_CONTROLLER";
static const char *TAG_MODULE = "MIM_UART_MODULE";

bool _is_init = false;

gpio_num_t _pin_controller = GPIO_NUM_NC;
uart_port_t _port_controller = UART_NUM_MAX;
QueueHandle_t _queue_uart_controller = NULL;
QueueHandle_t _queue_crsf_controller = NULL;
util_lqcalc_t _lq = {0};
TaskHandle_t _task_controller = NULL;

mim_uart_handler _handler_controller = NULL;
mim_uart_handler _handler_module = NULL;

gpio_num_t _pin_module = GPIO_NUM_NC;
uart_port_t _port_module = UART_NUM_MAX;
QueueHandle_t _queue_uart_module = NULL;
QueueHandle_t _queue_crsf_module = NULL;
TaskHandle_t _task_module = NULL;

static void _uart_init_port(uart_port_t port, gpio_num_t pin, QueueHandle_t *queue);
static void _uart_half_duplex_set_tx(uart_port_t port, gpio_num_t pin);
static void _uart_half_duplex_set_rx(uart_port_t port, gpio_num_t pin);
static bool _uart_read_crsf_frame(const char *tag, uart_port_t port, crsf_frame_t *frame);
static void _uart_write_crsf_frame(uart_port_t port, crsf_frame_t *frame);
static void _task_controller_impl(void *arg);
static void _task_module_impl(void *arg);

void mim_uart_init(uart_port_t port_controller, gpio_num_t pin_controller,
                    uart_port_t port_module, gpio_num_t pin_module) {
    assert(!_is_init);

    _port_controller = port_controller;
    _pin_controller = pin_controller;
    // we don't want to queue controller messages as it will introduce latency
    // instead just drop the stored message
    _queue_crsf_controller = xQueueCreate(1, sizeof(crsf_frame_t));

    _port_module = port_module;
    _pin_module = pin_module;
    _queue_crsf_module = xQueueCreate(10, sizeof(crsf_frame_t));

    assert(xTaskCreatePinnedToCore(_task_controller_impl, "controller", 4096, NULL, configMAX_PRIORITIES - 2, &_task_controller, APP_CPU_NUM) == pdPASS);

    assert(xTaskCreatePinnedToCore(_task_module_impl, "module", 4096, NULL, configMAX_PRIORITIES - 3, &_task_module, APP_CPU_NUM) == pdPASS);

    _is_init = true;
}

void mim_uart_set_controller_handler(mim_uart_handler handler) {
    _handler_controller = handler;
}

void mim_uart_set_module_handler(mim_uart_handler handler) {
    _handler_module = handler;
}

void mim_uart_enqueue_module_frame(const crsf_frame_t *frame) {
    assert(_is_init);
    assert(xQueueSend(_queue_crsf_module, frame, 0) == pdPASS);
}

static void _uart_init_port(uart_port_t port, gpio_num_t pin, QueueHandle_t *queue) {
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

    uart_intr_config_t intr_cfg = {
        .intr_enable_mask = uart_intr_mask,
        .rx_timeout_thresh = 1,
        .txfifo_empty_intr_thresh = 0,
        .rxfifo_full_thresh = 64,
    };

    ESP_ERROR_CHECK(uart_param_config(port, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(port, pin, pin, -1, -1));
    ESP_ERROR_CHECK(uart_driver_install(port, 256, 256, 10, queue, 0));
    ESP_ERROR_CHECK(uart_intr_config(port, &intr_cfg));
    ESP_ERROR_CHECK(uart_enable_rx_intr(port));
}

static void _uart_half_duplex_set_tx(uart_port_t port, gpio_num_t pin) {
    ESP_ERROR_CHECK(uart_flush_input(port));

    ESP_ERROR_CHECK(gpio_set_pull_mode(pin, GPIO_FLOATING));
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
    ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_OUTPUT));
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT, UART_PERIPH_SIGNAL(port, SOC_UART_RX_PIN_IDX), false);
    esp_rom_gpio_connect_out_signal(pin, UART_PERIPH_SIGNAL(port, SOC_UART_TX_PIN_IDX), true, false);
}

static void _uart_half_duplex_set_rx(uart_port_t port, gpio_num_t pin) {
    ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_INPUT));
    //gpio_matrix_in(pin, UART_PERIPH_SIGNAL(port, SOC_UART_RX_PIN_IDX), true);
    esp_rom_gpio_connect_in_signal(pin, UART_PERIPH_SIGNAL(port, SOC_UART_RX_PIN_IDX), true);
    ESP_ERROR_CHECK(gpio_set_pull_mode(pin, GPIO_PULLDOWN_ONLY));

    // flush input and queue before receiving
    ESP_ERROR_CHECK(uart_flush_input(port));
}

static bool _uart_read_crsf_frame(const char *tag, uart_port_t port, crsf_frame_t *frame) {
    bool result = false;
    uint8_t buff[256];

    size_t len_buffer = 0;

    ESP_ERROR_CHECK(uart_get_buffered_data_len(port, &len_buffer));
    if (len_buffer > 0) {
        size_t len_read = uart_read_bytes(port, buff, len_buffer, portMAX_DELAY);

        crsf_parser_t p;
        crsf_parser_init(&p, frame);

        for (int i = 0; i < len_read; i++) {
            crsf_parse_result_t err = crsf_parser_parse_byte(&p, buff[i]);

            if (err == CRSF_PARSE_RESULT_FRAME_COMPLETE) {
                result = true;
                break;
            } else if (err != CRSF_PARSE_RESULT_NEED_MORE_DATA) {
                ESP_LOG_BUFFER_HEX(tag, (uint8_t *)buff, len_read);
                ESP_LOGE(tag, "crsf [parser error=%u, state=%u], [buffer len=%u, read=%u]", err, p.state, len_buffer, len_read);
            }
        }
    }

    // flush input and reset queue for the next round
    uart_flush_input(port);

    return result;
}


void _uart_write_crsf_frame(uart_port_t port,  crsf_frame_t *frame) {
    uint8_t header[3] = { frame->sync, frame->length, frame->type };
    assert(uart_write_bytes(port, header, 3) == 3);
    if (frame->length > 1) {
        assert(uart_write_bytes(port, frame->data, frame->length - 1) == (frame->length - 1));
    }
    ESP_ERROR_CHECK(uart_wait_tx_done(port, portMAX_DELAY));
}

static void _task_controller_impl(void *arg) {
    _uart_init_port(_port_controller, _pin_controller, &_queue_uart_controller);

    _uart_half_duplex_set_rx(_port_controller, _pin_controller);

    bool is_frame_ready = false;
    crsf_frame_t frame;
    uart_event_t event;

    ESP_LOGI(TAG_CONTROLLER, "controller task started");

    while (true) {
        if (xQueueReceive(_queue_uart_controller, &event, portMAX_DELAY) == pdPASS) {
            if (event.type == UART_DATA) {
                is_frame_ready = _uart_read_crsf_frame(TAG_CONTROLLER, _port_controller, &frame);

                if (is_frame_ready) {
                    if (_handler_controller != NULL) {
                        _handler_controller(&frame);
                    }

                    assert(xQueueOverwrite(_queue_crsf_controller, &frame) == pdPASS);

                    if (xQueueReceive(_queue_crsf_module, &frame, 0) == pdPASS) {
                        if (_handler_module != NULL) {
                            _handler_module(&frame);
                        }

                        _uart_half_duplex_set_tx(_port_controller, _pin_controller);
                        _uart_write_crsf_frame(_port_controller, &frame);
                        _uart_half_duplex_set_rx(_port_controller, _pin_controller);
                        xQueueReset(_queue_uart_controller);
                    }
                } else {
                    ESP_LOGE(TAG_CONTROLLER, "failed to parse controller frame");
                }
            } else {
                ESP_LOGE(TAG_CONTROLLER, "unhandled uart event=0x%x, port=%u, size=%u, flag=%u", event.type, _port_controller, event.size, event.timeout_flag);
            }
        }
    }
}

static void _task_module_impl(void *arg) {
    _uart_init_port(_port_module, _pin_module, &_queue_uart_module);

    _uart_half_duplex_set_rx(_port_module, _pin_module);

    bool is_frame_ready = false;
    crsf_frame_t frame;
    uart_event_t event;

    ESP_LOGI(TAG, "module task started");

    while (true) {
        if (xQueueReceive(_queue_crsf_controller, &frame, portMAX_DELAY) == pdPASS) {
            _uart_half_duplex_set_tx(_port_module, _pin_module);
            _uart_write_crsf_frame(_port_module, &frame);
            _uart_half_duplex_set_rx(_port_module, _pin_module);
            xQueueReset(_queue_uart_module);

            if (xQueueReceive(_queue_uart_module, &event, pdMS_TO_TICKS(4)) == pdPASS) {
                if (event.type == UART_DATA) {
                    is_frame_ready = _uart_read_crsf_frame(TAG_MODULE, _port_module, &frame);

                    if (is_frame_ready) {
                        assert(xQueueSend(_queue_crsf_module, &frame, 0) == pdPASS);
                    }
                }
            }
        }
    }
}
