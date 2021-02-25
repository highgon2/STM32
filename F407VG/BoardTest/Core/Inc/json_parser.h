#ifndef __JSON_PARSER_H__
#define __JSON_PARSER_H__

#define REQ_LED_CTRL_MESSAGE     0
#define REQ_PWR_CTRL_MESSAGE     1

int make_json_board_state(char *json, void *data);

#endif