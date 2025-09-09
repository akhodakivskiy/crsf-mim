#include "mim_uart.h"

#include "driver/uart.h"
#include "freertos/projdefs.h"
#include "hal/uart_types.h"
#include "libcrsf.h"
#include "libcrsf_def.h"
#include "libcrsf_def.h"
#include "libcrsf_payload.h"
#include "soc/gpio_num.h"
#include "util.h"
#include "util_lqcalc.h"

#include <hal/uart_ll.h>
#include <soc/uart_periph.h>
#include <esp_log.h>
#include <esp_timer.h>

static const char *TAG= "MIM_UART";
static const char *TAG_CONTROLLER = "MIM_UART_CONTROLLER";
static const char *TAG_MODULE = "MIM_UART_MODULE";

#define MIM_UART_WDT_TIMEOUT 1000000 // 1 second
#define MIM_UART_AUTOBAUD_TIMEOUT 500000 // 500 milliseconds
static const int32_t MIM_UART_BAUD_RATES[] = {400000, 115200, 5250000, 3750000, 1870000, 921600, 2250000};

bool _is_init = false;

gpio_num_t _pin_controller = GPIO_NUM_NC;
uart_port_t _port_controller = UART_NUM_MAX;
QueueHandle_t _queue_uart_controller = NULL;
QueueHandle_t _queue_crsf_controller = NULL;
TaskHandle_t _task_controller = NULL;

mim_uart_handler _handler_controller = NULL;
mim_uart_handler _handler_module = NULL;

gpio_num_t _pin_module = GPIO_NUM_NC;
uart_port_t _port_module = UART_NUM_MAX;
QueueHandle_t _queue_uart_module = NULL;
QueueHandle_t _queue_crsf_module = NULL;
TaskHandle_t _task_module = NULL;

util_lqcalc_t _lq = {0};
int64_t _last_uart_wdt_time = 0;
int64_t _autobaud_start_time = 0;
uint8_t _autobaud_current_idx = 0;

static void _uart_init_port(uart_port_t port, gpio_num_t pin, QueueHandle_t *queue);
static void _uart_half_duplex_set_tx(uart_port_t port, gpio_num_t pin);
static void _uart_half_duplex_set_rx(uart_port_t port, gpio_num_t pin);
static bool _uart_read_crsf_frame(const char *tag, uart_port_t port, crsf_frame_t *frame);
static void _uart_write_crsf_frame(uart_port_t port, crsf_frame_t *frame);
static void _task_controller_impl(void *arg);
static void _task_module_impl(void *arg);
static void _uart_wdt();
static uint32_t _autobaud();

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

    assert(xTaskCreatePinnedToCore(_task_module_impl, "module", 4096, NULL, configMAX_PRIORITIES - 2, &_task_module, APP_CPU_NUM) == pdPASS);

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
        .baud_rate = MIM_UART_BAUD_RATES[_autobaud_current_idx],
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
    ESP_ERROR_CHECK(uart_driver_install(port, 256, 256, 10, queue, ESP_INTR_FLAG_LEVEL3));
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
        int size_written = uart_write_bytes(port, frame->data, frame->length - 1);
        assert(size_written == (frame->length - 1));
    }
    ESP_ERROR_CHECK(uart_wait_tx_done(port, portMAX_DELAY));
}

static void _task_controller_impl(void *arg) {
    _uart_init_port(_port_controller, _pin_controller, &_queue_uart_controller);

    _uart_half_duplex_set_rx(_port_controller, _pin_controller);

    util_lqcalc_init(&_lq, 100);
    util_lqcalc_reset_100(&_lq);

    bool is_frame_ready = false;
    crsf_frame_t frame;
    uart_event_t event;

    ESP_LOGI(TAG_CONTROLLER, "controller task started");

    while (true) {
        _uart_wdt();

        util_lqcalc_prepare(&_lq);

        if (xQueueReceive(_queue_uart_controller, &event, pdMS_TO_TICKS(20)) == pdTRUE) {
            if (event.type == UART_DATA) {
                is_frame_ready = _uart_read_crsf_frame(TAG_CONTROLLER, _port_controller, &frame);

                if (is_frame_ready) {
                    util_lqcalc_receive(&_lq);

                    if (_handler_controller != NULL) {
                        _handler_controller(&frame);
                    }

                    assert(xQueueOverwrite(_queue_crsf_controller, &frame) == pdTRUE);

                    if (xQueueReceive(_queue_crsf_module, &frame, 0) == pdTRUE) {
                        if (_handler_module != NULL) {
                            _handler_module(&frame);
                        }

                        _uart_half_duplex_set_tx(_port_controller, _pin_controller);
                        _uart_write_crsf_frame(_port_controller, &frame);
                        _uart_half_duplex_set_rx(_port_controller, _pin_controller);
                        xQueueReset(_queue_uart_controller);
                    }
                } else {
                    //ESP_LOGE(TAG_CONTROLLER, "failed to parse controller frame");
                }
            } else {
                //ESP_LOGE(TAG_CONTROLLER, "unhandled uart event=0x%x, port=%u, size=%u, flag=%u", event.type, _port_controller, event.size, event.timeout_flag);
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

    uint32_t interval_us = (1000 / 250) * 1000; // default rate for 400k baud is 250HZ

    ESP_LOGI(TAG, "module task started");

    while (true) {
        if (xQueueReceive(_queue_crsf_controller, &frame, portMAX_DELAY) == pdTRUE) {
            int64_t time_start = esp_timer_get_time();

            _uart_half_duplex_set_tx(_port_module, _pin_module);
            _uart_write_crsf_frame(_port_module, &frame);
            _uart_half_duplex_set_rx(_port_module, _pin_module);
            xQueueReset(_queue_uart_module);

            int64_t time_rx = esp_timer_get_time();
            int64_t period_remaining_ms = (interval_us - (time_rx - time_start)) / 1000;

            //ESP_LOGI(TAG_MODULE, "period_remaining=%lli, ticks=%lu", period_remaining_ms, pdMS_TO_TICKS(period_remaining_ms));

            if (xQueueReceive(_queue_uart_module, &event, pdMS_TO_TICKS(period_remaining_ms > 0 ? period_remaining_ms : 1)) == pdTRUE) {
                if (event.type == UART_DATA) {
                    is_frame_ready = _uart_read_crsf_frame(TAG_MODULE, _port_module, &frame);

                    if (is_frame_ready) {
                        crsf_payload_timing_correction_t payload;
                        if ((frame.type == CRSF_FRAME_TYPE_RADIO_ID) &&
                            (crsf_payload_unpack__timing_correction(&frame, &payload))) {
                            interval_us = payload.interval_100ns / 10;
                            //ESP_LOGI(TAG_MODULE, "tining connection, period=%lu, offset=%li", payload.interval_100ns / 10, payload.offset_100ns / 10);
                        }
                        assert(xQueueSend(_queue_crsf_module, &frame, 0) == pdTRUE);
                    }
                } else {
                    //ESP_LOGI(TAG_MODULE, "unhandled uart event=%u", event.type);
                }
            }
        }
    }
}

static void _uart_wdt() {
    int64_t time = esp_timer_get_time();
    uint8_t lq = util_lqcalc_get_lq(&_lq);

    //ESP_LOGI(TAG, "uart wdt on port=%u, pin=%u, lq=%u, time=%llu, last_wdt_time=%llu", port, pin, util_lqcalc_get_lq(&_lq), time, _last_uart_wdt_time);
    if (time - _last_uart_wdt_time < MIM_UART_WDT_TIMEOUT) {
        return;
    } 

    if (lq < 50) {
        uint32_t baud = _autobaud();
        if (baud > 0) {
            uint32_t best_baud = baud;
            uint32_t best_diff = UINT32_MAX;

            for (int i = 0; i < ARRAY_LENGTH(MIM_UART_BAUD_RATES); i++) {
                uint32_t diff = abs((int32_t)MIM_UART_BAUD_RATES[i] - (int32_t)baud);
                if (diff < best_diff) {
                    best_baud = MIM_UART_BAUD_RATES[i];
                    best_diff = diff;
                }
            }

            ESP_LOGI(TAG, "baud rate detected at %lu, nearest %lu", baud, best_baud);

            baud = best_baud;

            uart_set_baudrate(_port_controller, baud);
            _uart_half_duplex_set_rx(_port_controller, _pin_controller);
            uart_set_baudrate(_port_module, baud);
            _uart_half_duplex_set_rx(_port_module, _pin_module);

            util_lqcalc_reset_100(&_lq);

        }

        // no autobaud pending
        if (_autobaud_start_time == 0) {
            _last_uart_wdt_time = time;
        }
    } else {
        _last_uart_wdt_time = time;
    }

}

static uint32_t _autobaud() {
    uint32_t result = 0;
#ifdef CONFIG_IDF_TARGET_ESP32S3
    uart_dev_t *u = UART_LL_GET_HW(_port_controller);

    int64_t time = esp_timer_get_time();

    if (_autobaud_start_time > 0) {
        if (time - _autobaud_start_time > MIM_UART_AUTOBAUD_TIMEOUT) {
            ESP_LOGW(TAG, "baud rate detection timeout after %uus, autobaud en: %u, edges detected: %u", MIM_UART_AUTOBAUD_TIMEOUT, u->conf0.autobaud_en, u->rxd_cnt.rxd_edge_cnt);
            _autobaud_start_time = 0;

            u->conf0.autobaud_en = 0;
            u->rx_filt.glitch_filt_en = 0;

        } else if (u->conf0.autobaud_en && u->rxd_cnt.rxd_edge_cnt >= 100) {
            uint32_t low_period = u->lowpulse.lowpulse_min_cnt;
            uint32_t high_period = u->highpulse.highpulse_min_cnt;
            uint32_t baud_auto = UART_CLK_FREQ / ((low_period + high_period + 2) / 2);

            ESP_LOGI(TAG, "baud rate detection done, baud=%lu, time=%lli", baud_auto, time - _autobaud_start_time);

            _autobaud_start_time = 0;

            u->conf0.autobaud_en = 0;
            u->rx_filt.glitch_filt_en = 0;

            result = baud_auto;
        }

    } else {
        ESP_LOGI(TAG, "baud rate detection started");
        // start baud rate detection
        _autobaud_start_time = time;

        assert(u->clk_conf.sclk_sel == 1);  // assume APB clock source

        if (!u->conf0.autobaud_en) {
            u->rx_filt.glitch_filt = 8;
            u->rx_filt.glitch_filt_en = 1;
            u->lowpulse.lowpulse_min_cnt = 4095;
            u->highpulse.highpulse_min_cnt = 4095;
            u->conf0.autobaud_en = 1;
        }
    }

#else
    _autobaud_current_idx += 1;
    _autobaud_current_idx = _autobaud_current_idx % ARRAY_LENGTH(MIM_UART_BAUD_RATES);
    result = MIM_UART_BAUD_RATES[_autobaud_current_idx];
#endif
    return result;
}

