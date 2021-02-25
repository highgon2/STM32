#ifndef __ESP_H__
#define __ESP_H__

#include "stm32f4xx_hal.h"

#define MAX_ESP_RX_BUFFER      1024
#define MAX_ESP_COMMAND_LEN    64

#define MAX_ESP_CLIENT_NUM     10

int drv_esp_init(void);
int drv_esp_test_command(void);

#endif