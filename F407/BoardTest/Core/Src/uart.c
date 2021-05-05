#include <stdio.h>
#include <string.h>

#include "uart.h"

static UART_HandleTypeDef huart2;

int __io_putchar(int ch)
{
    if(HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10) == HAL_OK)
        return ch;
    return -1;
}

int drv_uart_init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
        return -1;

    return 0;
}

int drv_uart_rx_buffer(uint8_t *buf, uint16_t size)
{
    uint8_t ch;
    uint16_t length = 0;

    do
    {
        uint8_t len = 1;
        uint8_t str[4] = {0};

        if(length >= MAX_UART_COMMAND_LEN)
        {
            char message[64];
            
            sprintf(message, "\r\ncommand buffer is full\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), 100);
            HAL_UART_Transmit(&huart2, buf, length, 100);
            length = 0;
            break;
        }

        if(HAL_UART_Receive(&huart2, &ch, 1, 10) != HAL_OK)
            continue;

        switch(ch)
        {
            case '\r':
            case '\n':
                len = 2;
                sprintf((char *)str, "\r\n");
                break;

            case '\b':
            case 0x7F:
                if(length == 0)
                {
                    len = 0;
                    break;
                }

                len = 3;
                sprintf((char *)str, "\b \b");
                buf[length--] = '\0';
                break;

            default:
                str[0] = ch;
                buf[length++] = ch;
                break;
        }

        if(len > 0) HAL_UART_Transmit(&huart2, str, len, 100);
    } while(ch != '\r' && ch != '\n');

    return length;
}

int drv_uart_tx_buffer(uint8_t *buf, uint16_t size)
{
    if(HAL_UART_Transmit(&huart2, buf, size, 100) != HAL_OK)
        return -1;

    return 0;
}
