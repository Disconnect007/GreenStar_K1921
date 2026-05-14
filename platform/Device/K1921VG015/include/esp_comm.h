#ifndef ESP_COMM_H
#define ESP_COMM_H

#include <stdint.h>
#include <stdbool.h>

#define REQ_GET_DATA       0x01
#define REQ_EXEC_CMD       0x02

#define CMD_START          0x01
#define CMD_STOP           0x02
#define CMD_CLEAR          0x03
#define CMD_SET_CHANNELS   0x04

#define SYS_STATE_OK       0xAA
#define SYS_STATE_STOPPED  0xBB
#define SYS_STATE_ERROR    0xEE

extern uint16_t  g_nchan;
extern float    g_ader;
extern float    g_ltime;
extern float    g_inprate;
extern uint8_t  g_system_state;
extern bool     g_restart_measurement;   // флаг для main: начать измерение заново

void ESP_ProcessRequest(void);

#endif
