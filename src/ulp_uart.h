#ifndef ULP_UART_H
#define ULP_UART_H

#include <esp_attr.h>
#include <soc/gpio_num.h>
#include <stdbool.h>
#include <stdint.h>

#define ULP_UART_RECEIVE_BUFFER_SIZE 246

extern uint8_t ulp_uart_rx_buffer[ULP_UART_RECEIVE_BUFFER_SIZE];

void ulp_uart_setup(gpio_num_t rx_pin);
bool ulp_uart_is_running(void);

#endif
