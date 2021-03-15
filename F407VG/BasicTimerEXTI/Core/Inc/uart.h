#ifndef __UART_H__
#define __UART_H__

#include "stm32f4xx_hal.h"

#define MAX_UART_RX_BUFFER      512
#define MAX_UART_COMMAND_LEN    64

int drv_uart_init(void);
int drv_uart_rx_buffer(uint8_t *buf, uint16_t size);
int drv_uart_tx_buffer(uint8_t *buf, uint16_t size);

#endif