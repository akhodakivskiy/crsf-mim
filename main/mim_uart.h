#ifndef MIM_UART_H
#define MIM_UART_H

#include <driver/uart.h>
#include <driver/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

void mim_uart_setup(uart_port_t port, gpio_num_t pin, QueueHandle_t *queue);

void mim_uart_half_duplex_set_tx(uart_port_t port, gpio_num_t pin);

void mim_uart_half_duplex_set_rx(uart_port_t port, gpio_num_t pin);

#ifdef __cplusplus
}
#endif

#endif
