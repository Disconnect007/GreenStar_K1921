#include "esp_comm.h"
#include "uart_tx.h"
#include "modbus_funcs.h"
#include "modbus_sbs_regs.h"
#include "mtimer.h"
#include "dose_calc.h"
#include <string.h>

int16_t g_nchan   = 0;
float   g_ader    = 0.0f;
float   g_ltime   = 0.0f;
float   g_inprate = 0.0f;
volatile bool g_data_requested = false;

static QueuedCommand_t cmd_queue[CMD_QUEUE_SIZE];
static uint8_t         queue_head = 0;
static uint8_t         queue_tail = 0;
static uint8_t         queue_count = 0;
static uint8_t         last_cmd_ack = ACK_SUCCESS;
static DisplayState_t  disp_state = DISP_ND;

static bool QueueIsFull(void) { return queue_count >= CMD_QUEUE_SIZE; }

static bool QueuePush(uint8_t cmd, uint8_t param) {
    if (QueueIsFull()) return false;
    cmd_queue[queue_tail].cmd   = cmd;
    cmd_queue[queue_tail].param = param;
    cmd_queue[queue_tail].valid = true;
    queue_tail = (queue_tail + 1) % CMD_QUEUE_SIZE;
    queue_count++;
    return true;
}

static bool QueuePop(uint8_t *cmd, uint8_t *param) {
    if (queue_count == 0) return false;
    QueuedCommand_t *item = &cmd_queue[queue_head];
    *cmd   = item->cmd;
    *param = item->param;
    item->valid = false;
    queue_head = (queue_head + 1) % CMD_QUEUE_SIZE;
    queue_count--;
    return true;
}

static void SendResponse(uint8_t ack, bool data_valid, uint8_t sbs_state) {
    uint8_t resp[16];
    resp[0] = ack;
    resp[1] = ((data_valid ? 1 : 0) << 4) | (sbs_state & 0x0F);

    resp[2] = g_nchan & 0xFF;
    resp[3] = (g_nchan >> 8) & 0xFF;

    if (data_valid) {
        uint32_t raw;
        raw = (uint32_t)(g_ader * 1e6f);
        resp[4] = raw & 0xFF; resp[5] = (raw >> 8) & 0xFF;
        resp[6] = (raw >> 16) & 0xFF; resp[7] = (raw >> 24) & 0xFF;

        raw = (uint32_t)(g_ltime * 1e6f);
        resp[8] = raw & 0xFF; resp[9] = (raw >> 8) & 0xFF;
        resp[10] = (raw >> 16) & 0xFF; resp[11] = (raw >> 24) & 0xFF;

        raw = (uint32_t)(g_inprate * 1e6f);
        resp[12] = raw & 0xFF; resp[13] = (raw >> 8) & 0xFF;
        resp[14] = (raw >> 16) & 0xFF; resp[15] = (raw >> 24) & 0xFF;
    } else {
        memset(&resp[4], 0, 12);
    }

    UART2_SendBuffer(resp, sizeof(resp));
}

static uint8_t ReadSbsState(void) {
    int16_t val;
    if (MODBUS_ReadInt16(SBS_ADDR, SBS_STATE_REG, &val)) {
        return (uint8_t)val;
    }
    return SBS_STATE_STOP;
}

void ESP_Init(void) {
    memset(cmd_queue, 0, sizeof(cmd_queue));
    queue_head = queue_tail = queue_count = 0;
    last_cmd_ack = ACK_SUCCESS;
    g_data_requested = false;
    disp_state = DISP_ND;
}

void ESP_ProcessRequest(void) {
    uint8_t req_type;

    uint16_t len = UART2_ReceiveBuffer(&req_type, 1, 50, 5);
    if (len != 1) return;

    if (req_type == REQ_GET_DATA) {
        g_data_requested = true;
        return;
    }

    if (req_type == REQ_EXEC_CMD) {
        uint8_t cmd;
        len = UART2_ReceiveBuffer(&cmd, 1, 50, 5);
        if (len != 1) {
            SendResponse(ACK_ERROR, false, ReadSbsState());
            disp_state = DISP_PROTOCOL_ERROR;
            return;
        }

        uint8_t param = 0;
        if (cmd == CMD_SET_CHANNELS) {
            len = UART2_ReceiveBuffer(&param, 1, 50, 5);
            if (len != 1 || param > 5) {
                SendResponse(ACK_ERROR, false, ReadSbsState());
                disp_state = DISP_PROTOCOL_ERROR;
                return;
            }
        }

        if (!QueuePush(cmd, param)) {
            SendResponse(ACK_ERROR, false, ReadSbsState());
            disp_state = DISP_PROTOCOL_ERROR;
        }
        return;
    }

    UART2_FlushRx();
}

bool ESP_QueueEmpty(void) {
    return (queue_count == 0);
}

void ESP_ExecuteNextCommand(void) {
    if (ESP_QueueEmpty()) return;

    uint8_t cmd, param;
    if (!QueuePop(&cmd, &param)) return;

    bool ok = false;
    switch (cmd) {
        case CMD_START:
            ok = MODBUS_WriteSingleReg(SBS_ADDR, SBS_STATE_REG, SBS_STATE_START);
            break;
        case CMD_STOP:
            ok = MODBUS_WriteSingleReg(SBS_ADDR, SBS_STATE_REG, SBS_STATE_STOP);
            break;
        case CMD_CLEAR:
            ok = MODBUS_WriteSingleReg(SBS_ADDR, SBS_STATE_REG, SBS_STATE_CLEAR);
            break;
        case CMD_SET_CHANNELS:
            ok = MODBUS_WriteSingleReg(SBS_ADDR, SBS_NCHANNELS_REG, 128 << param);
            break;
        default:
            ok = false;
            break;
    }

    uint8_t current_state = ReadSbsState();
    last_cmd_ack = ok ? ACK_SUCCESS : ACK_ERROR;

    if (ok) {
        if (current_state == SBS_STATE_ACQ) disp_state = DISP_OK;
        else disp_state = DISP_STOPPED;
    } else {
        disp_state = DISP_WRITE_ERROR;
    }

    SendResponse(last_cmd_ack, false, current_state);
}

void ESP_HandleDataRequest(void) {
    int16_t nchan;
    float   ltime, inprate;
    bool    read_ok = true;

    if (read_ok) read_ok = MODBUS_ReadInt16(SBS_ADDR, SBS_NCHANNELS_REG, &nchan);
    if (read_ok) {
        uint32_t spectr[4096];
        uint64_t t_start = mtimer_get_raw_time();
        read_ok = MODBUS_ReadSpectrum(SBS_ADDR, SBS_SP0_CHANNEL, nchan, spectr, 60);
        uint64_t sp_rec_time = mtimer_get_raw_time() - t_start;

        if (read_ok) read_ok = MODBUS_ReadFloat(SBS_ADDR, SBS_LTIME_REG, &ltime);
        if (read_ok) read_ok = MODBUS_ReadFloat(SBS_ADDR, SBS_INPRATE_REG, &inprate);

        if (read_ok) {
            float ader;
            bool calc_ok = DoseCalc_Perform(nchan, spectr, sp_rec_time, ltime, inprate, &ader);
            if (calc_ok) {
                g_nchan   = nchan;
                g_ader    = ader;
                g_ltime   = ltime;
                g_inprate = inprate;
                disp_state = DISP_OK;
                SendResponse(ACK_SUCCESS, true, SBS_STATE_ACQ);
                return;
            }
        }
    }

    uint8_t current_state = ReadSbsState();
    if (current_state == SBS_STATE_ACQ) {
        disp_state = DISP_READ_ERROR;
        SendResponse(ACK_ERROR, false, current_state);
    } else {
        disp_state = DISP_STOPPED;
        SendResponse(ACK_SUCCESS, false, current_state);
    }
}

DisplayState_t ESP_GetDisplayState(void) {
    return disp_state;
}