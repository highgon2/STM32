#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "uart.h"
#include "esp.h"

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
        HAL_Delay(100);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
        |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

    /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

static void board_test_command(void)
{
    char command[MAX_UART_COMMAND_LEN];
    while (1)
    {
		drv_uart_tx_buffer((uint8_t *)"board>", 7);
        memset(command, 0x00, sizeof(command));
		if(drv_uart_rx_buffer((uint8_t *)command, MAX_UART_COMMAND_LEN) == 0)
			continue;

        if(strcmp(command, "help") == 0)
        {
            printf("============================================================\r\n");
            printf("* help                 : help\r\n");
            printf("* quit                 : board test exit\r\n");
            printf("* led_info             : led color information\r\n");
            printf("* led_toggle <number>  : led on/off test\r\n");
            printf("============================================================\r\n");
        }
        else if(strcmp(command, "quit") == 0)
        {
            printf("board test exit\r\n");
            break;
        }
        else if(strcmp(command, "led_info") == 0)
        {
			printf("1 : LED Green\r\n");
			printf("2 : LED Orange\r\n");
			printf("3 : LED Red\r\n");
			printf("4 : LED Blue\r\n");
        }
        else if(strncmp(command, "led_toggle", strlen("led_toggle")) == 0)
        {
            char     buf[32] = {0};
            uint8_t  num  = atoi(&command[strlen("led_toggle")+1]);
            uint16_t gpio_pin = 0xFFFF;

            GPIO_PinState state = 0;

    		switch(num)
			{
				case 1:
					gpio_pin = GPIO_PIN_12;
					sprintf(buf, "LED Green");
					break;

				case 2:
					gpio_pin = GPIO_PIN_13;
					sprintf(buf, "LED Orange");
					break;

				case 3:
					gpio_pin = GPIO_PIN_14;
					sprintf(buf, "LED Red");
					break;
				
				case 4:
					gpio_pin = GPIO_PIN_15;
					sprintf(buf, "LED Blue");
					break;
			}

            if(gpio_pin == 0xFFFF)
            {
                printf("invalid led number : num = %d\r\n", num);
                continue;
            }

			state = HAL_GPIO_ReadPin(GPIOD, gpio_pin);
			printf("LED %d : %s %d -> %d\r\n", num, buf, state, !state);
			HAL_GPIO_TogglePin(GPIOD, gpio_pin);
        }
    }
}

int main(void)
{
    int ret = 0;
    char command[MAX_UART_COMMAND_LEN];

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    ret |= drv_uart_init();
    ret |= drv_esp_init();
    if(ret != 0) Error_Handler();
    while (1)
    {
        memset(command, 0x00, sizeof(command));
		drv_uart_tx_buffer((uint8_t *)">", 1);
        if(drv_uart_rx_buffer((uint8_t *)command, MAX_UART_COMMAND_LEN) == 0)
			continue;

        if(strcmp(command, "help") == 0)
        {
            printf("============================================================\r\n");
            printf("* help                 : help\r\n");
            printf("* board                : board test\r\n");
            printf("* esp                  : esp8266 module test\r\n");
            printf("============================================================\r\n");
        }
        else if(strcmp(command, "board") == 0)
        {
            board_test_command();
        }
        else if(strcmp(command, "esp") == 0)
        {
            drv_esp_test_command();
        }

    }
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
		ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
