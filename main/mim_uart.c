#include "mim_uart.h"

#include <hal/uart_ll.h>
#include <soc/uart_periph.h>

#define CRSF_BAUDRATE 400000

void mim_uart_setup(uart_port_t port, gpio_num_t pin, QueueHandle_t *queue) {
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

void mim_uart_half_duplex_set_tx(uart_port_t port, gpio_num_t pin) {
    ESP_ERROR_CHECK(gpio_set_pull_mode(pin, GPIO_FLOATING));
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
    ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_OUTPUT));
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT, UART_PERIPH_SIGNAL(port, SOC_UART_RX_PIN_IDX), false);
    esp_rom_gpio_connect_out_signal(pin, UART_PERIPH_SIGNAL(port, SOC_UART_TX_PIN_IDX), true, false);
}

void mim_uart_half_duplex_set_rx(uart_port_t port, gpio_num_t pin) {
    ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_INPUT));
    //gpio_matrix_in(pin, UART_PERIPH_SIGNAL(port, SOC_UART_RX_PIN_IDX), true);
    esp_rom_gpio_connect_in_signal(pin, UART_PERIPH_SIGNAL(port, SOC_UART_RX_PIN_IDX), true);
    ESP_ERROR_CHECK(gpio_set_pull_mode(pin, GPIO_PULLDOWN_ONLY));
}
