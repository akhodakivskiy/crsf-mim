#ifndef MIM_UART_H
#define MIM_UART_H

#include "libcrsf_def.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <driver/uart.h>
#include <driver/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*mim_uart_handler)(crsf_frame_t *frame);

void mim_uart_init(BaseType_t core, UBaseType_t priority, 
                   uart_port_t port_controller, gpio_num_t pin_controller,
                   uart_port_t port_module, gpio_num_t pin_module);

void mim_uart_set_controller_handler(mim_uart_handler handler);
void mim_uart_set_module_handler(mim_uart_handler handler);

void mim_uart_enqueue_module_frame(const crsf_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif
