#include "esp_comm.h"
#include "uart_tx.h"
#include "modbus_funcs.h"
#include "modbus_sbs_regs.h"
#include "modbus_crc.h"
#include "mtimer.h"
#include "dose_calc.h"
#include "peak_finder.h"
#include <string.h>

extern const double ENK0[];
extern const double ENK1[];

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
            ok = MODBUS_WriteInt16(SBS_ADDR, SBS_STATE_REG, SBS_STATE_START);
            if (ok) DoseCalc_Init();   
            break;
        case CMD_STOP:
            ok = MODBUS_WriteInt16(SBS_ADDR, SBS_STATE_REG, SBS_STATE_STOP);
            break;
        case CMD_CLEAR:
            ok = MODBUS_WriteInt16(SBS_ADDR, SBS_STATE_REG, SBS_STATE_CLEAR);
            if (ok) DoseCalc_Init();   
            break;
        case CMD_SET_CHANNELS:
            ok = MODBUS_WriteInt16(SBS_ADDR, SBS_NCHANNELS_REG, 128 << param);
            break;
        default:
            ok = false;
            break;
    }

    uint8_t current_state;
    if (ok) {
        mtimer_sleep(50);              
        current_state = ReadSbsState();
    } else {
        current_state = ReadSbsState();
    }
    last_cmd_ack = ok ? ACK_SUCCESS : ACK_ERROR;

    if (ok) {
        if (current_state == SBS_STATE_ACQ) {
            disp_state = DISP_OK;
        } else if (current_state == SBS_STATE_STOP) {
            disp_state = DISP_STOPPED;
        } else if (current_state == SBS_STATE_CLEAR) {
            disp_state = DISP_ND;
        } else {
            disp_state = DISP_STOPPED;  
        }
    } else {
        disp_state = DISP_WRITE_ERROR;
    }

    SendResponse(last_cmd_ack, false, current_state);
}

void ESP_HandleDataRequest(void)
{
    int16_t nchan;
    float   ltime, inprate;
    bool    read_ok = true;

    if (read_ok) read_ok = MODBUS_ReadInt16(SBS_ADDR, SBS_NCHANNELS_REG, &nchan);
    if (read_ok) {
        static uint32_t raw[4096]; 
        uint64_t t_start = mtimer_get_raw_time();
        read_ok = MODBUS_ReadSpectrum(SBS_ADDR, SBS_SP0_CHANNEL, nchan, raw, 60);
        uint64_t sp_rec_time = mtimer_get_raw_time() - t_start;

        if (read_ok) read_ok = MODBUS_ReadFloat(SBS_ADDR, SBS_LTIME_REG, &ltime);
        if (read_ok) read_ok = MODBUS_ReadFloat(SBS_ADDR, SBS_INPRATE_REG, &inprate);

        if (read_ok) {
            float ader;
            bool calc_ok = DoseCalc_Perform(nchan, raw, sp_rec_time, ltime, &ader);

            g_nchan   = nchan;
            g_ltime   = ltime;
            g_inprate = inprate;
            g_ader    = calc_ok ? ader : 0.0f;

            uint8_t k_idx;
            switch (nchan) {
                case 128:  k_idx = 0; break;
                case 256:  k_idx = 1; break;
                case 512:  k_idx = 2; break;
                case 1024: k_idx = 3; break;
                case 2048: k_idx = 4; break;
                case 4096: k_idx = 5; break;
                default:   k_idx = 0; break;
            }
            const double enk0 = ENK0[k_idx];
            const double enk1 = ENK1[k_idx];

            PeakInfo peaks[20];
            int npeaks = find_peaks_simple(raw, nchan, enk0, enk1, 3.5f, peaks, 20);

            PeakInfo top[5];
            int top_count = 0;
            for (int i = 0; i < npeaks && top_count < 5; i++) {
                int pos = top_count;
                while (pos > 0 && peaks[i].max_count > top[pos-1].max_count) {
                    top[pos] = top[pos-1];
                    pos--;
                }
                top[pos] = peaks[i];
                if (top_count < 5) top_count++;
            }

            uint8_t resp[46];
            resp[0] = ACK_SUCCESS;
            resp[1] = ((calc_ok ? 1 : 0) << 4) | SBS_STATE_ACQ;
            resp[2] = g_nchan & 0xFF;
            resp[3] = (g_nchan >> 8) & 0xFF;

            uint32_t ader_raw = (uint32_t)(g_ader * 1e6f);
            resp[4] = ader_raw & 0xFF; resp[5] = (ader_raw >> 8) & 0xFF;
            resp[6] = (ader_raw >> 16) & 0xFF; resp[7] = (ader_raw >> 24) & 0xFF;

            uint32_t ltime_raw = (uint32_t)(g_ltime * 1e6f);
            resp[8] = ltime_raw & 0xFF; resp[9] = (ltime_raw >> 8) & 0xFF;
            resp[10] = (ltime_raw >> 16) & 0xFF; resp[11] = (ltime_raw >> 24) & 0xFF;

            uint32_t inprate_raw = (uint32_t)(g_inprate * 1e6f);
            resp[12] = inprate_raw & 0xFF; resp[13] = (inprate_raw >> 8) & 0xFF;
            resp[14] = (inprate_raw >> 16) & 0xFF; resp[15] = (inprate_raw >> 24) & 0xFF;

            for (int p = 0; p < 5; p++) {
                if (p < top_count) {
                    uint32_t cnt = top[p].max_count;
                    resp[16 + p*6 + 0] = cnt & 0xFF;
                    resp[16 + p*6 + 1] = (cnt >> 8) & 0xFF;
                    resp[16 + p*6 + 2] = (cnt >> 16) & 0xFF;
                    resp[16 + p*6 + 3] = (cnt >> 24) & 0xFF;

                    uint16_t e_raw = (uint16_t)(top[p].energy_keV * 10.0 + 0.5);
                    resp[16 + p*6 + 4] = e_raw & 0xFF;
                    resp[16 + p*6 + 5] = (e_raw >> 8) & 0xFF;
                } else {
                    memset(&resp[16 + p*6], 0, 6);
                }
            }

            UART2_SendBuffer(resp, sizeof(resp));
            disp_state = DISP_OK;
            return;
        }
    }

    uint8_t resp[16];
    resp[0] = ACK_ERROR;
    resp[1] = 0x00;
    memset(&resp[2], 0, 14);
    UART2_SendBuffer(resp, sizeof(resp));
    disp_state = DISP_READ_ERROR;
}

DisplayState_t ESP_GetDisplayState(void) {
    return disp_state;
}