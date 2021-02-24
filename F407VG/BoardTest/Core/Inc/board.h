#ifndef __BOARD_H__
#define __BOARD_H__

#include "stm32f4xx.h"

#define MAX_LED_NUM     4

typedef struct _led_info_t
{
    char    label[16];
    uint8_t state;
    float   voltage;
}led_info_t;

typedef struct _board_info_t
{
    led_info_t led_list[MAX_LED_NUM];
}board_info_t;

#endif