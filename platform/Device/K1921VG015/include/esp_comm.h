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

#define ACK_SUCCESS        0xAA
#define ACK_ERROR          0xEE

#define SBS_STATE_CLEAR    0
#define SBS_STATE_STOP     1
#define SBS_STATE_ACQ      2

#define CMD_QUEUE_SIZE     8

typedef struct {
    uint8_t cmd;
    uint8_t param;
    bool    valid;
} QueuedCommand_t;

typedef enum {
    DISP_OK = 0,
    DISP_WRITE_ERROR,
    DISP_PROTOCOL_ERROR,
    DISP_READ_ERROR,
    DISP_STOPPED,
    DISP_ND
} DisplayState_t;

void ESP_Init(void);
void ESP_ProcessRequest(void);
bool ESP_QueueEmpty(void);
void ESP_ExecuteNextCommand(void);
void ESP_HandleDataRequest(void);
DisplayState_t ESP_GetDisplayState(void);

extern int16_t g_nchan;
extern float   g_ader;
extern float   g_ltime;
extern float   g_inprate;
extern volatile bool g_data_requested;

#endif