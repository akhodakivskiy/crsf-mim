#include "esp_log_buffer.h"
#include "libcrsf.h"
#include "libcrsf_device.h"
#include "libcrsf_device_param.h"
#include <sdkconfig.h>

#ifdef CONFIG_CRSF_FIRMWARE_TX

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include <stdint.h>
#include <string.h>
static const char *TAG= "MAIN_TX";
static const char *TAG_CONTROLLER = "MAIN_TX_CONTROLLER";
static const char *TAG_MODULE = "MAIN_TX_MODULE";

#include <driver/uart.h>
#include <driver/gptimer.h>
#include <hal/uart_types.h>

#include <esp_log.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <soc/soc.h>
#include <soc/uart_periph.h>
#include <hal/uart_ll.h>
#include <rom/gpio.h>

#include "async_logging.h"
#include "libcrsf_parser.h"
#include "mim_menu.h"


#define UART_PORT_CONTROLLER UART_NUM_1
#define UART_PORT_MODULE UART_NUM_2

#define UART_PIN_CONTROLLER GPIO_NUM_13
#define UART_PIN_MODULE GPIO_NUM_14

#define CRSF_RATE_HZ 62
#define CRSF_BAUDRATE 115200
#define CRSF_INVERTED false

TaskHandle_t task_controller = NULL;
TaskHandle_t task_module = NULL;
QueueHandle_t queue_uart_controller = NULL;
QueueHandle_t queue_uart_module = NULL;

QueueHandle_t queue_crsf_controller = NULL;
QueueHandle_t queue_crsf_module = NULL;

uint64_t _last_crsf_period_us;

crsf_device_t crsf_device;

/*
static bool _timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *event, void *user_ctx) {
    ESP_ERROR_CHECK(gptimer_stop(timer));

    QueueHandle_t queue = reinterpret_cast<QueueHandle_t>(user_ctx);
    uart_event_t e { 
        .type = UART_BREAK,
        .size = 0,
        .timeout_flag = true,
    };

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(queue, &e, &xHigherPriorityTaskWoken);
    return xHigherPriorityTaskWoken;
}

void _setup_timer(gptimer_handle_t *timer, QueueHandle_t task) {
    gptimer_config_t timer_cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
        .intr_priority = 0,
        .flags = {
            .intr_shared = 0,
            .allow_pd = 0,
            .backup_before_sleep = 0,
        }
    };

    gptimer_event_callbacks_t timer_cbs = {
        .on_alarm = _timer_isr,
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_cfg, timer));
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(*timer, &timer_cbs, task));
    ESP_ERROR_CHECK(gptimer_enable(*timer));
    ESP_ERROR_CHECK(gptimer_start(*timer));
}

void _timer_set_alarm(gptimer_handle_t timer, uint64_t alarm_us) {
    gptimer_alarm_config_t timer_alarm_cfg = {
        .alarm_count = alarm_us,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = false,
        }
    };

    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &timer_alarm_cfg));
    ESP_ERROR_CHECK(gptimer_set_raw_count(timer, 0));
    ESP_ERROR_CHECK(gptimer_start(timer));
}
*/

void _handle_crsf_device(const crsf_frame_t *frame_in) {
    crsf_frame_t frame_out;
    crsf_device_result_t err = crsf_device_client_handler(&crsf_device, frame_in, &frame_out);

    if (err == CRSF_DEVICE_RESULT_OK) {
        assert(xQueueSend(queue_crsf_module, &frame_out, 0) == pdPASS);
    }

}

void IRAM_ATTR _half_duplex_set_rx(uart_port_t port, gpio_num_t pin) {
    ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_INPUT));
    //gpio_matrix_in(pin, UART_PERIPH_SIGNAL(port, SOC_UART_RX_PIN_IDX), true);
    esp_rom_gpio_connect_in_signal(pin, UART_PERIPH_SIGNAL(port, SOC_UART_RX_PIN_IDX), true);
    ESP_ERROR_CHECK(gpio_set_pull_mode(pin, GPIO_PULLDOWN_ONLY));
}

void IRAM_ATTR _half_duplex_set_tx(uart_port_t port, gpio_num_t pin) {
    ESP_ERROR_CHECK(gpio_set_pull_mode(pin, GPIO_FLOATING));
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
    ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_OUTPUT));
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT, UART_PERIPH_SIGNAL(port, SOC_UART_RX_PIN_IDX), false);
    esp_rom_gpio_connect_out_signal(pin, UART_PERIPH_SIGNAL(port, SOC_UART_TX_PIN_IDX), true, false);

    //constexpr uint8_t MATRIX_DETACH_IN_LOW = 0x30; // routes 0 to matrix slot
    //gpio_matrix_in(MATRIX_DETACH_IN_LOW, UART_PERIPH_SIGNAL(port, SOC_UART_RX_PIN_IDX), false); // Disconnect RX from all pads
    //gpio_matrix_out(pin, UART_PERIPH_SIGNAL(port, SOC_UART_TX_PIN_IDX), true, false);
}

void _setup_uart(uart_port_t port, gpio_num_t pin, QueueHandle_t *queue) {
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
    ESP_ERROR_CHECK(uart_set_pin(port, pin, pin, -1, -1));
    ESP_ERROR_CHECK(uart_driver_install(port, 256, 256, 512, queue, 0));
    ESP_ERROR_CHECK(uart_intr_config(port, &intr_cfg));
    ESP_ERROR_CHECK(uart_enable_rx_intr(port));
}

bool _uart_read_crsf_frame(const char *tag, uart_port_t port, QueueHandle_t queue_uart, 
                           crsf_frame_t *frame, TickType_t timeout) {
    bool result = false;
    uint8_t buff[256];

    uart_event_t event;
    size_t len_buffer = 0, len_read = 0;

    if (xQueueReceive(queue_uart, &event, timeout) == pdPASS) {
        ESP_ERROR_CHECK(uart_get_buffered_data_len(port, &len_buffer));

        if (event.type == UART_DATA || event.type == UART_BREAK) {
            len_read = uart_read_bytes(port, buff, len_buffer, portMAX_DELAY);

            crsf_parser_t p;
            crsf_parser_init(&p, frame);

            for (int i = 0; i < len_read; i++) {
                crsf_parse_result_t err = crsf_parser_parse_byte(&p, buff[i]);

                if (err == CRSF_PARSE_RESULT_FRAME_COMPLETE) {
                    result = true;
                    break;
                } else if (err != CRSF_PARSE_RESULT_NEED_MORE_DATA) {
                    ESP_LOG_BUFFER_HEX(tag, reinterpret_cast<uint8_t *>(buff), len_read);
                    ESP_LOGE(tag, "crsf [parser error=%u, state=%u], event[%u, size=%u, flag=%u, len_buffer=%u]", err, p.state, event.type, event.size, event.timeout_flag, len_buffer);
                }
            }
        } else if (event.type == UART_FIFO_OVF) {
            ESP_LOGI(tag, "event=overflow, size=%u, flag=%u, len_buffer=%u", event.size, event.timeout_flag, len_buffer);
            uart_flush_input(port);
            xQueueReset(queue_uart);
        } else if (event.type == UART_BUFFER_FULL) {
            ESP_LOGI(tag, "event=full, size=%u, flag=%u, len_buffer=%u", event.size, event.timeout_flag, len_buffer);
            uart_flush_input(port);
            xQueueReset(queue_uart);
        } else {
            ESP_LOGI(tag, "event=0x%x, size=%u, flag=%u, len_buffer=%u", event.type, event.size, event.timeout_flag, len_buffer);
        }
    }

    uart_flush_input(port);

    return result;
}


void _swap_crsf_frame(crsf_frame_t *frame) {
    static uint32_t _frame_type_counter[UINT8_MAX] = {0};
    _frame_type_counter[frame->type] = _frame_type_counter[frame->type] + 1;

    static uint64_t _last_stats_time = 0;
    if (_last_stats_time + 1000000 < esp_timer_get_time()) {
        _last_stats_time = esp_timer_get_time();

        char log[256];
        size_t log_cursor = 0;
        for (int i = 0; i < UINT8_MAX; i++) {
            if (_frame_type_counter[i] > 0) {
                log_cursor += snprintf(log + log_cursor, 256 - log_cursor, "0x%x=%lu, ", i, _frame_type_counter[i]);
            }
        }
        ESP_LOGI(TAG, "%s", log);
    }
}

static void _task_controller(void *arg) {
    _setup_uart(UART_PORT_CONTROLLER, UART_PIN_CONTROLLER, &queue_uart_controller);
    _half_duplex_set_rx(UART_PORT_CONTROLLER, UART_PIN_CONTROLLER);
    mim_menu_init(&crsf_device);

    static uint32_t frames_rcvd = 0, frames_sent = 0;

    crsf_frame_t frame;

    while (true) {
        bool is_frame_ready = _uart_read_crsf_frame(TAG_CONTROLLER, UART_PORT_CONTROLLER, queue_uart_controller, &frame, portMAX_DELAY);

        if (is_frame_ready) {
            frames_rcvd += 1;

            _swap_crsf_frame(&frame);
            _handle_crsf_device(&frame);

            assert(xQueueSend(queue_crsf_controller, &frame, 0) == pdPASS);

            if (xQueueReceive(queue_crsf_module, &frame, 0) == pdPASS) {
                frames_sent += 1;

                _half_duplex_set_tx(UART_PORT_CONTROLLER, UART_PIN_CONTROLLER);

                uint8_t header[3] = { frame.sync, frame.length, frame.type };
                assert(uart_write_bytes(UART_PORT_CONTROLLER, header, 3) == 3);
                assert(uart_write_bytes(UART_PORT_CONTROLLER, frame.data, frame.length - 1) == (frame.length - 1));
                ESP_ERROR_CHECK(uart_wait_tx_done(UART_PORT_CONTROLLER, portMAX_DELAY));

                _half_duplex_set_rx(UART_PORT_CONTROLLER, UART_PIN_CONTROLLER);
            }

            if (frames_rcvd % CRSF_RATE_HZ == 0) {
                //ESP_LOGI(TAG_CONTROLLER, "rcvd: %lu, sent: %lu, current frame sync=0x%x, type=0x%x, length=%u", frames_rcvd, frames_sent, frame.sync, frame.type, frame.length);
            }
        }
    }
}

static void _task_module(void *arg) {
    _setup_uart(UART_PORT_MODULE, UART_PIN_MODULE, &queue_uart_module);

    _half_duplex_set_rx(UART_PORT_MODULE, UART_PIN_MODULE);

    crsf_frame_t frame;

    uint32_t frames_sent = 0, frames_rcvd = 0;

    while (true) {
        if (xQueueReceive(queue_crsf_controller, &frame, portMAX_DELAY) == pdPASS) {
            frames_rcvd += 1;
            _half_duplex_set_tx(UART_PORT_MODULE, UART_PIN_MODULE);

            uint8_t header[3] = { frame.sync, frame.length, frame.type };
            assert(uart_write_bytes(UART_PORT_MODULE, header, 3) == 3);
            assert(uart_write_bytes(UART_PORT_MODULE, frame.data, frame.length - 1) == (frame.length - 1));
            ESP_ERROR_CHECK(uart_wait_tx_done(UART_PORT_MODULE, portMAX_DELAY));

            _half_duplex_set_rx(UART_PORT_MODULE, UART_PIN_MODULE);

            bool is_frame_ready = _uart_read_crsf_frame(TAG_MODULE, UART_PORT_MODULE, queue_uart_module, &frame, pdMS_TO_TICKS(1000/CRSF_RATE_HZ));

            if (is_frame_ready) {
                frames_sent += 1;
                assert(xQueueSend(queue_crsf_module, &frame, 0) == pdPASS);
            }
        }
    }
}

extern "C" void app_main(void) {
    async_logging_init(PRO_CPU_NUM);

    queue_crsf_controller = xQueueCreate(10, sizeof(crsf_frame_t));
    queue_crsf_module = xQueueCreate(10, sizeof(crsf_frame_t));

    assert(xTaskCreatePinnedToCore(_task_module, "module", 4096, NULL, configMAX_PRIORITIES - 2, &task_module, PRO_CPU_NUM) == pdPASS);
    vTaskDelay(10);
    assert(xTaskCreatePinnedToCore(_task_controller, "controller", 4096, NULL, configMAX_PRIORITIES - 2, &task_controller, PRO_CPU_NUM) == pdPASS);
}

#endif
