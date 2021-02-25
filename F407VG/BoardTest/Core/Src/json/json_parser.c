#include <stdio.h>
#include <string.h>

#include <json_parser.h>

#include "cJSON.h"
#include "board.h"

static int make_json_led_info(cJSON *root, led_info_t *led_list)
{
    cJSON *led_array = NULL;

    cJSON_AddItemToObject(root, "leds", led_array = cJSON_CreateArray());
    for(int i = 0 ; i < MAX_LED_NUM ; i++)
    {
        cJSON *led = NULL;
        led_info_t *led_info = &led_list[i];

        if((led = cJSON_CreateObject()) == NULL)
            return -1;

        cJSON_AddItemToObject(led_array, "led", led);
        cJSON_AddStringToObject(led, "label", led_info->label);
        cJSON_AddNumberToObject(led, "state", led_info->state);
        cJSON_AddNumberToObject(led, "voltage", led_info->voltage);
    }

    return 0;
}

int make_json_board_state(char *json, void *data)
{
    cJSON *root = NULL;
    board_info_t *board = (board_info_t *)data;

    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "message", "info_state");
    if(make_json_led_info(root, board->led_list) != 0)
    {
        cJSON_Delete(root);
        return -1;
    }

    sprintf(json, "%s", cJSON_PrintUnformatted(root));
    cJSON_Delete(root);
    return 0;
}

int parse_json_req_message(char *json)
{
    int ret = -1;

    cJSON *root = NULL;
    cJSON *message = NULL;

    if((root = cJSON_Parse(json)) == NULL)
    {
        printf("ERROR :: %s() : json = [%s]\r\n", __func__, json);
        return -1;
    }

    message = cJSON_GetObjectItem(root, "message");
    if(strcmp(message->valuestring, "req_led_ctrl") == 0)    
        ret = REQ_LED_CTRL_MESSAGE;
    else if(strcmp(message->valuestring, "req_pwr_ctrl") == 0)
        ret = REQ_PWR_CTRL_MESSAGE;

    cJSON_Delete(root);
    return ret;
}
