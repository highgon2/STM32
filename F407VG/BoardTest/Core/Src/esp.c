#include <stdio.h>
#include <string.h>

#include "esp.h"
#include "uart.h"

typedef struct _cb_data_t
{
    uint8_t buf[MAX_UART_RX_BUFFER+1];
    uint16_t length;
}cb_data_t;

static uint8_t data;
static cb_data_t cb_data;
UART_HandleTypeDef huart3;

static int esp_at_command(uint8_t *cmd, uint8_t *resp, uint16_t *length, int16_t time_out)
{
    int ret = 0;

    memset(&cb_data, 0x00, sizeof(cb_data_t));
    if(HAL_UART_Transmit(&huart3, cmd, strlen((char *)cmd), 100) != HAL_OK)
        return -1;

    while(time_out > 0)
    {
        if(strstr((char *)cb_data.buf, "OK") != NULL)
        {
            memcpy(resp, cb_data.buf, cb_data.length);
            *length = cb_data.length;
            break;
        }
        else if(strstr((char *)cb_data.buf, "ERROR") != NULL)
        {
            ret = -2;
            break;
        }
        else if(cb_data.length >= MAX_UART_RX_BUFFER)
        {
            ret = -3;
            break;
        }

        time_out -= 10;
        HAL_Delay(10);
    }

    HAL_Delay(500);
    if(time_out <= 0)
        return -4;

    return ret;
}

int drv_esp_init(void)
{
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if(HAL_UART_Init(&huart3) != HAL_OK)
        return -1;

    HAL_UART_Receive_IT(&huart3, &data, 1);
    drv_esp_reset();

    return 0;
}

int drv_esp_reset(void)
{
    int ret = 0;
    char resp[32];
    uint16_t length = 0;

    length = 0;
    memset(resp, 0x00, sizeof(resp));
    if((ret = esp_at_command((uint8_t *)"AT+RST\r\n", (uint8_t *)resp, &length, 1000)) == 0)
    {
        length = 0;
        memset(resp, 0x00, sizeof(resp));
        ret = esp_at_command((uint8_t *)"AT\r\n", (uint8_t *)resp, &length, 1000);
    }

    return ret;
}

int drv_esp_test_command(void)
{
    char resp[MAX_UART_RX_BUFFER+1];
    char command[MAX_UART_COMMAND_LEN];
    uint16_t length = 0;

    while (1)
    {
		drv_uart_tx_buffer((uint8_t *)"esp>", 4);

        memset(command, 0x00, sizeof(command));
		if(drv_uart_rx_buffer((uint8_t *)command, MAX_UART_COMMAND_LEN) == 0)
			continue;

        if(strcmp(command, "help") == 0)
        {
            printf("============================================================\r\n");
            printf("* help                    : help\r\n");
            printf("* quit                    : esp test exit\r\n");
            printf("* reset                   : esp restart\r\n");
            printf("* ap_scan                 : scan ap list\r\n");
            printf("* ap_conn <ssid> <passwd> : connect ap & obtain ip addr\r\n");
            printf("* ip_state                : display ip addr\r\n");
            printf("*\r\n");
            printf("* More <AT COMMAND> information is available on the following website\r\n");
            printf("*  - https://docs.espressif.com/projects/esp-at/en/latest/AT_Command_Set\r\n");
            printf("============================================================\r\n");
        }
        else if(strcmp(command, "quit") == 0)
        {
            printf("esp test exit\r\n");
            break;
        }
        else if(strcmp(command, "reset") == 0)
        {
            printf("esp reset... ");
            if(drv_esp_reset() == 0)
                printf("OK\r\n");
            else
                printf("fail\r\n");
        }
        else if(strcmp(command, "ap_scan") == 0)
        {
            printf("ap scan...\r\n");

            length = 0;
            memset(resp, 0x00, sizeof(resp));
            if(esp_at_command((uint8_t *)"AT+CWLAP\r\n", (uint8_t *)resp, &length, 5000) != 0)
                printf("ap scan command fail\r\n");
            else
            {
                for(int i = 0 ; i < length ; i++)
                    printf("%c", resp[i]);
            }
        }
        else if(strncmp(command, "ap_conn", strlen("ap_conn")) == 0)
        {
            char at_cmd[MAX_UART_COMMAND_LEN];
            char *ssid   = strtok(&command[strlen("ap_conn")+1], " ");
            char *passwd = strtok(NULL, " ");

            if(ssid == NULL || passwd == NULL)
            {
                printf("invalid command : ap_conn <ssid> <passwd>\r\n");
                continue;
            }

            memset(at_cmd, 0x00, sizeof(at_cmd));
            sprintf(at_cmd, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid,passwd);
            
            length = 0;
            memset(resp, 0x00, sizeof(resp));
            if(esp_at_command((uint8_t *)at_cmd, (uint8_t *)resp, &length, 5000) != 0)
                printf("ap scan command fail\r\n");
            else
            {
                for(int i = 0 ; i < length ; i++)
                    printf("%c", resp[i]);
            }
        }
        else if(strcmp(command, "ip_state") == 0)
        {
            length = 0;
            memset(resp, 0x00, sizeof(resp));
            if(esp_at_command((uint8_t *)"AT+CIPSTA?\r\n", (uint8_t *)resp, &length, 1000) != 0)
                printf("ap scan command fail\r\n");
            else
            {
                for(int i = 0 ; i < length ; i++)
                    printf("%c", resp[i]);
            }
        }
        else
        {
            printf("unkwon command\r\n");
        }
    }

    return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART3)
    {
        if(cb_data.length < MAX_UART_RX_BUFFER)
            cb_data.buf[cb_data.length++] = data;
        HAL_UART_Receive_IT(huart, &data, 1);
    }
}
