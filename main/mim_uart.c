#include "mim_uart.h"

#include <freertos/FreeRTOS.h>
#include <driver/uart.h>
#include <hal/uart_ll.h>
#include <soc/uart_periph.h>
#include <esp_intr_alloc.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_cpu.h>
#include <xtensa/core-macros.h>
#include <driver/gptimer.h>

#include "driver/gptimer_types.h"
#include "freertos/idf_additions.h"
#include "libcrsf.h"
#include "libcrsf_def.h"
#include "libcrsf_def.h"
#include "libcrsf_payload.h"
#include "portmacro.h"
#include "util.h"
#include "util_lqcalc.h"

static const char *TAG= "MIM_UART";

#define MIM_UART_EVENT_TIMER UART_EVENT_MAX + 1

#define MIM_UART_WDT_TIMEOUT 1000000 // 1 second
#define MIM_UART_AUTOBAUD_TIMEOUT 500000 // 500 milliseconds
static const int32_t MIM_UART_BAUD_RATES[] = {400000, 115200, 5250000, 3750000, 1870000, 921600, 2250000};
static const int32_t MIM_UART_PACKET_RATES[] = {250, 62, 500, 500, 500, 500, 250};

#define MIM_GPIO_CONTROLLER_TX GPIO_NUM_34
#define MIM_GPIO_MODULE_TX GPIO_NUM_33
#define MIM_GPIO_MODULE_DATA_INTR GPIO_NUM_35
#define MIM_GPIO_CONTROLLER_DATA_INTR GPIO_NUM_36

static bool _is_init = false;

typedef struct {
    gpio_num_t pin;
    uart_port_t port;
    QueueHandle_t queue_out;
    mim_uart_handler handler;
    TaskHandle_t task;
} _mim_uart_t;

typedef struct {
    int64_t last_correction_time;
    uint32_t period_us;
    uint32_t correction_pending;
} _mim_timing_t;

typedef struct {
    int64_t last_uart_wdt_time;
    int64_t start_time;
    uint8_t idx;
} _mim_uart_autobaud_t;

static _mim_uart_t _controller_s = {0};
static _mim_uart_t _module_s = {0};
static _mim_timing_t _timing_s = {0};
static _mim_uart_autobaud_t _autobaud_s = {0};

static void _uart_init_port(uart_port_t port, gpio_num_t pin, QueueHandle_t *queue);
static void _uart_half_duplex_set_tx(uart_port_t port, gpio_num_t pin);
static void _uart_half_duplex_set_rx(uart_port_t port, gpio_num_t pin);
static bool _uart_read_crsf_frame(uart_port_t port, crsf_frame_t *frame);
static void _uart_write_crsf_frame(uart_port_t port, crsf_frame_t *frame);
//static void _task_crsf_controller_impl(void *arg);
//static void _task_crsf_module_impl(void *arg);
static void _task_crsf_impl(void *arg);
static void _uart_wdt(util_lqcalc_t *lq_calc);
static uint32_t _autobaud();

/*
static bool _crsf_handle_timing_correction(crsf_frame_t *frame, uint32_t *period_us, int32_t *offset_us);
static uint32_t _crsf_get_frame_time_on_wire_us(const crsf_frame_t *frame);

static void _timer_init(gptimer_handle_t *timer, uint64_t period_us, QueueHandle_t queue);
static void _timer_set_period(gptimer_handle_t timer, uint32_t period_us, int32_t offset_us);
static bool _timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);
*/

void mim_uart_init(BaseType_t core, UBaseType_t priority, 
                   uart_port_t port_controller, gpio_num_t pin_controller,
                   uart_port_t port_module, gpio_num_t pin_module) {
    assert(!_is_init);

    _autobaud_s.last_uart_wdt_time = 0;
    _autobaud_s.start_time = 0;
    _autobaud_s.idx = 0;

    _controller_s.port = port_controller;
    _controller_s.pin = pin_controller;
    _controller_s.queue_out = xQueueCreate(1, sizeof(crsf_frame_t));

    _module_s.port = port_module;
    _module_s.pin = pin_module;
    _module_s.queue_out = xQueueCreate(10, sizeof(crsf_frame_t));

    _timing_s.last_correction_time = 0;
    _timing_s.correction_pending = false;
    _timing_s.period_us = 1000000 / MIM_UART_PACKET_RATES[_autobaud_s.idx]; 

    gpio_set_direction(MIM_GPIO_CONTROLLER_TX, GPIO_MODE_OUTPUT);
    gpio_set_direction(MIM_GPIO_MODULE_TX, GPIO_MODE_OUTPUT);
    gpio_set_direction(MIM_GPIO_MODULE_DATA_INTR, GPIO_MODE_OUTPUT);
    gpio_set_direction(MIM_GPIO_CONTROLLER_DATA_INTR, GPIO_MODE_OUTPUT);
    //gpio_set_direction(GPIO_NUM_35, GPIO_MODE_OUTPUT);
    //gpio_set_direction(GPIO_NUM_36, GPIO_MODE_OUTPUT);

    //assert(xTaskCreatePinnedToCore(_task_crsf_controller_impl, "task_crsf_controller", 4096, NULL, priority, &_controller_s.task, core) == pdPASS);
    //assert(xTaskCreatePinnedToCore(_task_crsf_module_impl, "task_crsf_module", 4096, NULL, priority, &_module_s.task, core) == pdPASS);
    assert(xTaskCreatePinnedToCore(_task_crsf_impl, "task_crsf", 4096, NULL, priority, NULL, core) == pdPASS);

    _is_init = true;
}

void mim_uart_set_controller_handler(mim_uart_handler handler) {
    _controller_s.handler = handler;
}

void mim_uart_set_module_handler(mim_uart_handler handler) {
    _module_s.handler = handler;
}

void IRAM_ATTR mim_uart_enqueue_module_frame(const crsf_frame_t *frame) {
    assert(_is_init);
    if (xQueueSend(_module_s.queue_out, frame, 0) != pdPASS) {
        ESP_LOGE(TAG, "failed to enqueue module frame, type=%x, length=%u", frame->type, frame->length);
    }
}

static void _uart_init_port(uart_port_t port, gpio_num_t pin, QueueHandle_t *queue) {
    uart_config_t cfg = {
        .baud_rate = MIM_UART_BAUD_RATES[_autobaud_s.idx],
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

    ESP_ERROR_CHECK(uart_param_config(port, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(port, pin, pin, -1, -1));
    ESP_ERROR_CHECK(uart_driver_install(port, 256, 256, 10, queue, ESP_INTR_FLAG_LEVEL3));

    if (queue != NULL) {
        uint32_t uart_intr_mask = 
            UART_INTR_RXFIFO_FULL | 
            UART_INTR_RXFIFO_TOUT;

        uart_intr_config_t intr_cfg = {
            .intr_enable_mask = uart_intr_mask,
            .rx_timeout_thresh = 1,
            .txfifo_empty_intr_thresh = 0,
            .rxfifo_full_thresh = CRSF_MAX_FRAME_LEN,
        };
        ESP_ERROR_CHECK(uart_intr_config(port, &intr_cfg));
        ESP_ERROR_CHECK(uart_enable_rx_intr(port));
    }
}

static IRAM_ATTR void _uart_half_duplex_set_tx(uart_port_t port, gpio_num_t pin) {
    ESP_ERROR_CHECK(gpio_set_pull_mode(pin, GPIO_FLOATING));
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
    ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_OUTPUT));
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT, UART_PERIPH_SIGNAL(port, SOC_UART_RX_PIN_IDX), false);
    esp_rom_gpio_connect_out_signal(pin, UART_PERIPH_SIGNAL(port, SOC_UART_TX_PIN_IDX), true, false);
}

static IRAM_ATTR void _uart_half_duplex_set_rx(uart_port_t port, gpio_num_t pin) {
    ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_INPUT));
    //gpio_matrix_in(pin, UART_PERIPH_SIGNAL(port, SOC_UART_RX_PIN_IDX), true);
    esp_rom_gpio_connect_in_signal(pin, UART_PERIPH_SIGNAL(port, SOC_UART_RX_PIN_IDX), true);
    ESP_ERROR_CHECK(gpio_set_pull_mode(pin, GPIO_PULLDOWN_ONLY));

    // flush input and queue before receiving
    ESP_ERROR_CHECK(uart_flush_input(port));
}

static IRAM_ATTR bool _uart_read_crsf_frame(uart_port_t port, crsf_frame_t *frame) {
    bool result = false;
    uint8_t buff[256];

    size_t len_buffer = 0;

    frame->type = CRSF_FRAME_TYPE_NONE;

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
                ESP_LOG_BUFFER_HEX(TAG, (uint8_t *)buff, len_read);
                ESP_LOGE(TAG, "crsf port=%u [parser error=%u, state=%u], [buffer len=%u, read=%u]", port, err, p.state, len_buffer, len_read);
            }
        }
    }

    // flush input and reset queue for the next round
    uart_flush_input(port);

    return result;
}


void IRAM_ATTR _uart_write_crsf_frame(uart_port_t port,  crsf_frame_t *frame) {
    uint8_t header[3] = { frame->sync, frame->length, frame->type };
    assert(uart_write_bytes(port, header, 3) == 3);
    if (frame->length > 1) {
        int size_written = uart_write_bytes(port, frame->data, frame->length - 1);
        assert(size_written == (frame->length - 1));
    }
}

/*
static void IRAM_ATTR _task_crsf_controller_impl(void *arg) {
    ESP_LOGI(TAG, "crsf controller task started");

    QueueHandle_t queue_uart;
    util_lqcalc_t lq_calc = {0};
    uart_event_t event = {0};
    crsf_frame_t frame = {0};
    bool tx_pending = false;

    util_lqcalc_init(&lq_calc, 100);
    util_lqcalc_reset_100(&lq_calc);

    _uart_init_port(_controller_s.port, _controller_s.pin, &queue_uart);
    _uart_half_duplex_set_rx(_controller_s.port, _controller_s.pin);

    while (true) {
        _uart_wdt(&lq_calc);

        if (xQueueReceive(queue_uart, &event, pdMS_TO_TICKS(20)) == pdTRUE) {
            gpio_set_level(MIM_GPIO_CONTROLLER_DATA_INTR, 1);
            util_lqcalc_prepare(&lq_calc);
            if (event.type == UART_DATA) {
                // Send (N-1) response to the controller
                if (xQueueReceive(_module_s.queue_out, &frame, 0)) {
                    gpio_set_level(MIM_GPIO_CONTROLLER_TX, 1);
                    _uart_half_duplex_set_tx(_controller_s.port, _controller_s.pin);
                    _uart_write_crsf_frame(_controller_s.port, &frame);
                    tx_pending = true;
                }

                // Read and handle (N) request from the controller
                if (_uart_read_crsf_frame(_controller_s.port, &frame)) {
                    util_lqcalc_receive(&lq_calc);

                    if (_controller_s.handler != NULL) {
                        _controller_s.handler(&frame);
                    }

                    assert(xQueueOverwrite(_controller_s.queue_out, &frame) == pdTRUE);
                }

                // Wait for (N-1) response to be fully written
                if (tx_pending) {
                    ESP_ERROR_CHECK(uart_wait_tx_done(_controller_s.port, portMAX_DELAY));
                    _uart_half_duplex_set_rx(_controller_s.port, _controller_s.pin);
                    gpio_set_level(MIM_GPIO_CONTROLLER_TX, 0);
                    tx_pending = false;
                }
            }
            gpio_set_level(MIM_GPIO_CONTROLLER_DATA_INTR, 0);
        }
    }
}


static void IRAM_ATTR _task_crsf_module_impl(void *arg) {
    ESP_LOGI(TAG, "crsf module task started");

    crsf_frame_t frame = {0};

    QueueHandle_t queue_uart = NULL;
    uart_event_t event = {0};

    _uart_init_port(_module_s.port, _module_s.pin, &queue_uart);
    _uart_half_duplex_set_rx(_module_s.port, _module_s.pin);

    gptimer_handle_t timer;

    _timer_init(&timer, _timing_s.period_us, queue_uart);

    while (true) {
        if (xQueueReceive(queue_uart, &event, portMAX_DELAY)) {
            if (event.type == MIM_UART_EVENT_TIMER) {

                //ESP_LOGI(TAG, "timer %lli", esp_timer_get_time());

                // Forward request to the module
                if (xQueueReceive(_controller_s.queue_out, &frame, 0)) {

                    gpio_set_level(MIM_GPIO_MODULE_TX, 1);
                    _uart_half_duplex_set_tx(_module_s.port, _module_s.pin);
                    _uart_write_crsf_frame(_module_s.port, &frame);
                    ESP_ERROR_CHECK(uart_wait_tx_done(_module_s.port, portMAX_DELAY));
                    _uart_half_duplex_set_rx(_module_s.port, _module_s.pin);
                    gpio_set_level(MIM_GPIO_MODULE_TX, 0);
                }
            } else if (event.type == UART_DATA) {
                gpio_set_level(MIM_GPIO_MODULE_DATA_INTR, 1);
                // Read response from module
                if (_uart_read_crsf_frame(_module_s.port, &frame)) {
                    if (esp_timer_get_time() - _timing_s.last_correction_time > 200000) {
                        int32_t offset_us = 0;
                        if (_crsf_handle_timing_correction(&frame, &_timing_s.period_us, &offset_us)) {
                            _timing_s.last_correction_time = esp_timer_get_time();
                            _timer_set_period(timer, _timing_s.period_us, offset_us);
                        }
                    }

                    if (_module_s.handler != NULL) {
                        _module_s.handler(&frame);
                    }

                    assert(xQueueSend(_module_s.queue_out, &frame, 0) == pdTRUE);
                }

                gpio_set_level(MIM_GPIO_MODULE_DATA_INTR, 0);
            }
        }
    }
}
*/

static void IRAM_ATTR _task_crsf_impl(void *arg) {
    ESP_LOGI(TAG, "crsf task started");

    QueueHandle_t queue_uart_controller;
    util_lqcalc_t lq_calc = {0};

    _uart_init_port(_controller_s.port, _controller_s.pin, &queue_uart_controller);
    _uart_init_port(_module_s.port, _module_s.pin, NULL);

    util_lqcalc_init(&lq_calc, 100);
    util_lqcalc_reset_100(&lq_calc);

    crsf_frame_t frame_from_controller;
    crsf_frame_t frame_from_module;
    crsf_frame_t frame_to_controller;
    crsf_frame_t frame_to_module;
    uart_event_t event;

    bool module_tx_pending = false;
    bool controller_tx_pending = false;
    bool has_frame_from_module = false;
    bool has_frame_from_controller = false;

    _uart_half_duplex_set_rx(_controller_s.port, _controller_s.pin);
    _uart_half_duplex_set_rx(_module_s.port, _module_s.pin);

    while (true) {
        _uart_wdt(&lq_calc);

        if (xQueueReceive(queue_uart_controller, &event, pdMS_TO_TICKS(20)) == pdTRUE) {
            util_lqcalc_prepare(&lq_calc);
            if (event.type == UART_DATA) {

                // Forward (N) request to the module
                if (xQueueReceive(_controller_s.queue_out, &frame_to_module, 0)) {
                    gpio_set_level(MIM_GPIO_MODULE_TX, 1);
                    _uart_half_duplex_set_tx(_module_s.port, _module_s.pin);
                    _uart_write_crsf_frame(_module_s.port, &frame_to_module);
                    module_tx_pending = true;
                }

                // receive previous module frame (N-1)
                has_frame_from_module = _uart_read_crsf_frame(_module_s.port, &frame_from_module);

                // Send (N-1) response to the controller

                if (xQueueReceive(_module_s.queue_out, &frame_to_controller, 0)) {
                    gpio_set_level(MIM_GPIO_CONTROLLER_TX, 1);
                    _uart_half_duplex_set_tx(_controller_s.port, _controller_s.pin);
                    _uart_write_crsf_frame(_controller_s.port, &frame_to_controller);
                    controller_tx_pending = true;
                }

                // receive current frame from controller (N)
                has_frame_from_controller = _uart_read_crsf_frame(_controller_s.port, &frame_from_controller);

                if (module_tx_pending) {
                    // Wait for TX done on module
                    ESP_ERROR_CHECK(uart_wait_tx_done(_module_s.port, portMAX_DELAY));
                    _uart_half_duplex_set_rx(_module_s.port, _module_s.pin);
                    gpio_set_level(MIM_GPIO_MODULE_TX, 0);
                    module_tx_pending = false;
                }

                // Wait for TX done on controller
                if (controller_tx_pending) {
                    ESP_ERROR_CHECK(uart_wait_tx_done(_controller_s.port, portMAX_DELAY));
                    _uart_half_duplex_set_rx(_controller_s.port, _controller_s.pin);
                    gpio_set_level(MIM_GPIO_CONTROLLER_TX, 0);
                    controller_tx_pending = false;
                }

                if (has_frame_from_controller) {
                    util_lqcalc_receive(&lq_calc);

                    if (_controller_s.handler != NULL) {
                        _controller_s.handler(&frame_from_controller);
                    }

                    assert(xQueueOverwrite(_controller_s.queue_out, &frame_from_controller) == pdTRUE);

                    has_frame_from_controller = false;
                }

                if (has_frame_from_module) {
                    if (_module_s.handler != NULL) {
                        _module_s.handler(&frame_from_module);
                    }
                    assert(xQueueSend(_module_s.queue_out, &frame_from_module, 0) == pdTRUE);

                    has_frame_from_module = false;
                }
            } else {
                //ESP_LOGE(TAG, "unhandled uart event=0x%x, port=%u, size=%u, flag=%u", event.type, _port_controller, event.size, event.timeout_flag);
            }
        }
    }
}

static void IRAM_ATTR _uart_wdt(util_lqcalc_t *lq_calc) {
    int64_t time = esp_timer_get_time();
    uint8_t lq = util_lqcalc_get_lq(lq_calc);

    //ESP_LOGI(TAG, "uart wdt on port=%u, pin=%u, lq=%u, time=%llu, last_wdt_time=%llu", port, pin, util_lqcalc_get_lq(&_lq), time, _last_uart_wdt_time);
    if (time - _autobaud_s.last_uart_wdt_time < MIM_UART_WDT_TIMEOUT) {
        return;
    } 

    if (lq < 50) {
        uint32_t baud = _autobaud();
        if (baud > 0) {
            uint32_t best_diff = UINT32_MAX;

            for (int i = 0; i < UTIL_ARRAY_LENGTH(MIM_UART_BAUD_RATES); i++) {
                uint32_t diff = abs((int32_t)MIM_UART_BAUD_RATES[i] - (int32_t)baud);
                if (diff < best_diff) {
                    _autobaud_s.idx = i;
                    best_diff = diff;
                }
            }

            uint32_t autobaud = MIM_UART_BAUD_RATES[_autobaud_s.idx];
            uint32_t rate = MIM_UART_PACKET_RATES[_autobaud_s.idx];

            _timing_s.period_us = 1000000 / rate;

            ESP_LOGI(TAG, "baud rate detected baud=%lu, nearest=%lu, packet_rate=%lu, period=%lu", baud, autobaud, rate, _timing_s.period_us);

            uart_set_baudrate(_controller_s.port, baud);
            _uart_half_duplex_set_rx(_controller_s.port, _controller_s.pin);
            uart_set_baudrate(_module_s.port, baud);
            _uart_half_duplex_set_rx(_module_s.port, _module_s.pin);


            util_lqcalc_reset_100(lq_calc);
        }

        // no autobaud pending
        if (_autobaud_s.start_time == 0) {
            _autobaud_s.last_uart_wdt_time = time;
        }
    } else {
        _autobaud_s.last_uart_wdt_time = time;
    }

}

static uint32_t IRAM_ATTR _autobaud() {
    uint32_t result = 0;
#ifdef CONFIG_IDF_TARGET_ESP32S3
    uart_dev_t *u = UART_LL_GET_HW(_controller_s.port);

    int64_t time = esp_timer_get_time();

    if (_autobaud_s.start_time > 0) {
        if (time - _autobaud_s.start_time > MIM_UART_AUTOBAUD_TIMEOUT) {
            ESP_LOGW(TAG, "baud rate detection timeout after %uus, autobaud en: %u, edges detected: %u", MIM_UART_AUTOBAUD_TIMEOUT, u->conf0.autobaud_en, u->rxd_cnt.rxd_edge_cnt);
            _autobaud_s.start_time = 0;

            u->conf0.autobaud_en = 0;
            u->rx_filt.glitch_filt_en = 0;

        } else if (u->conf0.autobaud_en && u->rxd_cnt.rxd_edge_cnt >= 100) {
            uint32_t low_period = u->lowpulse.lowpulse_min_cnt;
            uint32_t high_period = u->highpulse.highpulse_min_cnt;
            uint32_t baud_auto = UART_CLK_FREQ / ((low_period + high_period + 2) / 2);

            ESP_LOGI(TAG, "baud rate detection done, baud=%lu, time=%lli", baud_auto, time - _autobaud_s.start_time);

            _autobaud_s.start_time = 0;

            u->conf0.autobaud_en = 0;
            u->rx_filt.glitch_filt_en = 0;

            result = baud_auto;
        }

    } else {
        ESP_LOGI(TAG, "baud rate detection started");
        // start baud rate detection
        _autobaud_s.start_time = time;

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

/*

static bool _crsf_handle_timing_correction(crsf_frame_t *frame, uint32_t *period_us, int32_t *offset_us) {
    crsf_payload_timing_correction_t p;
    if (crsf_payload_unpack__timing_correction(frame, &p)) {
        *offset_us = (p.offset_100ns / 10);
        *period_us = (p.interval_100ns / 10);

        //crsf_payload_modify__timing_correction(frame, p.interval_100ns, 0);

        return true;
    }
    return false;
}

static uint32_t _crsf_get_frame_time_on_wire_us(const crsf_frame_t *frame) {
    return ((frame->length + 2) * 10000000) / MIM_UART_BAUD_RATES[_autobaud_s.idx];
}

static void _timer_init(gptimer_handle_t *timer, uint64_t period_us, QueueHandle_t queue) {
    gptimer_config_t cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1us
    };

    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = period_us,
        .reload_count = 0,
        .flags = { .auto_reload_on_alarm = true }
    };

    gptimer_event_callbacks_t callback = {
        .on_alarm = _timer_isr
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&cfg, timer));

    ESP_ERROR_CHECK(gptimer_set_alarm_action(*timer, &alarm_cfg));

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(*timer, &callback, queue));

    ESP_ERROR_CHECK(gptimer_enable(*timer));

    ESP_ERROR_CHECK(gptimer_start(*timer));
}

static void _timer_set_period(gptimer_handle_t timer, uint32_t period_us, int32_t offset_us) {
    uint32_t count_alarm = period_us + offset_us;

    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = count_alarm,
        .reload_count = 0,
        .flags = { .auto_reload_on_alarm = true }
    };

    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_cfg));

    if (abs(offset_us) > 50) {
        ESP_LOGI(TAG, "timing correction period=%lu, offset=%li, next_alarm=%lu", period_us, offset_us, count_alarm);
    }
}

static bool _timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    QueueHandle_t queue = (QueueHandle_t)user_ctx;

    if (edata->alarm_value != _timing_s.period_us) {
        _timer_set_period(timer, _timing_s.period_us, 0);
    }

    uart_event_t e = {
        .type = MIM_UART_EVENT_TIMER,
        .size = 0,
        .timeout_flag = false
    };

    BaseType_t result = pdFALSE;
    xQueueSendFromISR(queue, &e, &result);
    return result;
}

*/
