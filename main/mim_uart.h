#ifndef MIM_UART_H
#define MIM_UART_H

#include "esp_err.h"
#include "hal/uart_types.h"
#include "libcrsf_def.h"
#include "soc/gpio_num.h"
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*mim_uart_handler)(crsf_frame_t *frame);

void mim_uart_init(uart_port_t port_controller, gpio_num_t pin_controller,
		    uart_port_t port_module, gpio_num_t pin_module);

void mim_uart_set_controller_handler(mim_uart_handler handler);
void mim_uart_set_module_handler(mim_uart_handler handler);

void mim_uart_enqueue_module_frame(const crsf_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif
