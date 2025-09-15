#include "mim_uart.h"

#include <freertos/FreeRTOS.h>
#include <driver/uart.h>
#include <hal/uart_ll.h>
#include <soc/uart_periph.h>
#include <esp_intr_alloc.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "libcrsf.h"
#include "libcrsf_def.h"
#include "libcrsf_def.h"
#include "libcrsf_payload.h"
#include "util.h"
#include "util_lqcalc.h"

static const char *TAG= "MIM_UART";

#define MIM_UART_WDT_TIMEOUT 1000000 // 1 second
#define MIM_UART_AUTOBAUD_TIMEOUT 500000 // 500 milliseconds
static const int32_t MIM_UART_BAUD_RATES[] = {400000, 115200, 5250000, 3750000, 1870000, 921600, 2250000};
static const int32_t MIM_UART_PACKET_RATES[] = {250, 62, 500, 500, 500, 500, 250};

bool _is_init = false;

gpio_num_t _pin_controller = GPIO_NUM_NC;
uart_port_t _port_controller = UART_NUM_MAX;
QueueHandle_t _queue_crsf_controller = NULL;
mim_uart_handler _handler_controller_frame = NULL;

gpio_num_t _pin_module = GPIO_NUM_NC;
uart_port_t _port_module = UART_NUM_MAX;
QueueHandle_t _queue_crsf_module = NULL;
mim_uart_handler _handler_module_frame = NULL;

TaskHandle_t _task_crsf = NULL;

util_lqcalc_t _lq = {0};
int64_t _last_uart_wdt_time = 0;
int64_t _autobaud_start_time = 0;
uint8_t _autobaud_current_idx = 0;

static void _uart_init_port(uart_port_t port, gpio_num_t pin, QueueHandle_t *queue);
static void _uart_half_duplex_set_tx(uart_port_t port, gpio_num_t pin);
static void _uart_half_duplex_set_rx(uart_port_t port, gpio_num_t pin);
static bool _uart_read_crsf_frame(uart_port_t port, crsf_frame_t *frame);
static void _uart_write_crsf_frame(uart_port_t port, crsf_frame_t *frame);
static void _task_crsf_impl(void *arg);
static void _uart_wdt();
static uint32_t _autobaud();

void mim_uart_init(UBaseType_t priority, 
                   uart_port_t port_controller, gpio_num_t pin_controller,
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

    assert(xTaskCreatePinnedToCore(_task_crsf_impl, "task_crsf", 4096, NULL, priority, &_task_crsf, APP_CPU_NUM) == pdPASS);

    _is_init = true;
}

void mim_uart_set_controller_handler(mim_uart_handler handler) {
    _handler_controller_frame = handler;
}

void mim_uart_set_module_handler(mim_uart_handler handler) {
    _handler_module_frame = handler;
}

void IRAM_ATTR mim_uart_enqueue_module_frame(const crsf_frame_t *frame) {
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

static void IRAM_ATTR _task_crsf_impl(void *arg) {
    ESP_LOGI(TAG, "crsf task started");

    QueueHandle_t queue_uart_controller;

    _uart_init_port(_port_controller, _pin_controller, &queue_uart_controller);
    _uart_init_port(_port_module, _pin_module, NULL);

    util_lqcalc_init(&_lq, 100);
    util_lqcalc_reset_100(&_lq);

    crsf_frame_t frame_from_controller;
    crsf_frame_t frame_from_module;
    crsf_frame_t frame_to_controller;
    crsf_frame_t frame_to_module;
    uart_event_t event;

    bool module_tx_pending = false;
    bool controller_tx_pending = false;
    bool has_frame_from_module = false;
    bool has_frame_from_controller = false;

    _uart_half_duplex_set_rx(_port_controller, _pin_controller);
    _uart_half_duplex_set_rx(_port_module, _pin_module);

    while (true) {
        _uart_wdt();

        if (xQueueReceive(queue_uart_controller, &event, pdMS_TO_TICKS(20)) == pdTRUE) {
            util_lqcalc_prepare(&_lq);
            if (event.type == UART_DATA) {

                // Forward (N) request to the module
                if (xQueueReceive(_queue_crsf_controller, &frame_to_module, 0)) {
                    _uart_half_duplex_set_tx(_port_module, _pin_module);
                    _uart_write_crsf_frame(_port_module, &frame_to_module);
                    module_tx_pending = true;
                }

                // receive previous module frame (N-1)
                has_frame_from_module = _uart_read_crsf_frame(_port_module, &frame_from_module);

                // Send (N-1) response to the controller
                if (xQueueReceive(_queue_crsf_module, &frame_to_controller, 0)) {
                    _uart_half_duplex_set_tx(_port_controller, _pin_controller);
                    _uart_write_crsf_frame(_port_controller, &frame_to_controller);
                    controller_tx_pending = true;
                }

                // receive current frame from controller (N)
                has_frame_from_controller = _uart_read_crsf_frame(_port_controller, &frame_from_controller);

                if (module_tx_pending) {
                    // Wait for TX done on module
                    ESP_ERROR_CHECK(uart_wait_tx_done(_port_module, portMAX_DELAY));
                    _uart_half_duplex_set_rx(_port_module, _pin_module);
                    module_tx_pending = false;
                }

                // Wait for TX done on controller
                if (controller_tx_pending) {
                    ESP_ERROR_CHECK(uart_wait_tx_done(_port_controller, portMAX_DELAY));
                    _uart_half_duplex_set_rx(_port_controller, _pin_controller);
                    controller_tx_pending = false;
                }

                if (has_frame_from_controller) {
                    util_lqcalc_receive(&_lq);

                    if (_handler_controller_frame != NULL) {
                        _handler_controller_frame(&frame_from_controller);
                    }

                    assert(xQueueOverwrite(_queue_crsf_controller, &frame_from_controller) == pdTRUE);

                    has_frame_from_controller = false;
                }

                if (has_frame_from_module) {
                    if (_handler_module_frame != NULL) {
                        _handler_module_frame(&frame_from_module);
                    }
                    assert(xQueueSend(_queue_crsf_module, &frame_from_module, 0) == pdTRUE);

                    has_frame_from_module = false;
                }
            } else {
                //ESP_LOGE(TAG, "unhandled uart event=0x%x, port=%u, size=%u, flag=%u", event.type, _port_controller, event.size, event.timeout_flag);
            }
        }
    }
}

static void IRAM_ATTR _uart_wdt() {
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
            uint32_t best_packet_rate = 100;
            uint32_t best_diff = UINT32_MAX;

            for (int i = 0; i < ARRAY_LENGTH(MIM_UART_BAUD_RATES); i++) {
                uint32_t diff = abs((int32_t)MIM_UART_BAUD_RATES[i] - (int32_t)baud);
                if (diff < best_diff) {
                    best_baud = MIM_UART_BAUD_RATES[i];
                    best_packet_rate = MIM_UART_PACKET_RATES[i];
                    best_diff = diff;
                }
            }

            ESP_LOGI(TAG, "baud rate detected at %lu, packet_rate=%lu, nearest %lu", baud, best_packet_rate, best_baud);

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

static uint32_t IRAM_ATTR _autobaud() {
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

