#include "esp_comm.h"
#include "uart_tx.h"
#include "modbus_funcs.h"
#include "modbus_sbs_regs.h"
#include "mtimer.h"
#include <string.h>

// буфер больше не нужен для накопления, удаляем rx_buf и rx_idx

uint16_t  g_nchan = 0;
float    g_ader = 0.0f;
float    g_ltime = 0.0f;
float    g_inprate = 0.0f;
uint8_t  g_system_state = SYS_STATE_OK;
bool     g_restart_measurement = false;

static void SendResponse(void);
static bool WriteSBSRegister(uint16_t reg, uint16_t value);

void ESP_ProcessRequest(void)
{
    uint8_t buf[4];
    uint16_t len = UART2_ReceiveBuffer(buf, 3, 100, 10);

    if (len == 0) return;

    if (buf[0] == REQ_GET_DATA) {
        // просто запрос данных – ничего не делаем, сразу ответим
    } else if (buf[0] == REQ_EXEC_CMD && len >= 2) {
        uint8_t cmd = buf[1];
        bool ok = false;
        switch (cmd) {
        case CMD_START:
            ok = WriteSBSRegister(SBS_STATE_REG, SBS_STATE_START);
            if (ok) {
                g_system_state = SYS_STATE_OK;
                g_restart_measurement = true;
            } else {
                g_system_state = SYS_STATE_ERROR;
            }
            break;
        case CMD_STOP:
            ok = WriteSBSRegister(SBS_STATE_REG, SBS_STATE_STOP);
            if (ok) {
                g_system_state = SYS_STATE_STOPPED;
            } else {
                g_system_state = SYS_STATE_ERROR;
            }
            break;
        case CMD_CLEAR:
            ok = WriteSBSRegister(SBS_STATE_REG, SBS_STATE_CLEAR);
            if (ok) {
                g_restart_measurement = true;
                // g_system_state остаётся прежним (если была норма – будет OK)
            } else {
                g_system_state = SYS_STATE_ERROR;
            }
            break;
        case CMD_SET_CHANNELS:
            if (len >= 3) {
                uint8_t idx = buf[2];
                if (idx <= 5) {
                    int nch = 128 << idx;
                    ok = WriteSBSRegister(SBS_NCHANNELS_REG, nch);
                    if (!ok) {
                        g_system_state = SYS_STATE_ERROR;
                    }
                }
            }
            break;
        default:
            break;
        }
    }

    SendResponse();
}

static void SendResponse(void)
{
    uint8_t resp[15];
    resp[0] = g_system_state;

    resp[1] = g_nchan & 0xFF;
    resp[2] = (g_nchan >> 8) & 0xFF;

    uint32_t ader_raw = (uint32_t)(g_ader * 1000000.0f);
    resp[3] = ader_raw & 0xFF;
    resp[4] = (ader_raw >> 8) & 0xFF;
    resp[5] = (ader_raw >> 16) & 0xFF;
    resp[6] = (ader_raw >> 24) & 0xFF;

    uint32_t ltime_raw = (uint32_t)(g_ltime * 1000000.0f);
    resp[7] = ltime_raw & 0xFF;
    resp[8] = (ltime_raw >> 8) & 0xFF;
    resp[9] = (ltime_raw >> 16) & 0xFF;
    resp[10] = (ltime_raw >> 24) & 0xFF;

    uint32_t inprate_raw = (uint32_t)(g_inprate * 1000000.0f);
    resp[11] = inprate_raw & 0xFF;
    resp[12] = (inprate_raw >> 8) & 0xFF;
    resp[13] = (inprate_raw >> 16) & 0xFF;
    resp[14] = (inprate_raw >> 24) & 0xFF;

    UART2_SendBuffer(resp, 15);
}

static bool WriteSBSRegister(uint16_t reg, uint16_t value)
{
    return MODBUS_WriteSingleReg(SBS_ADDR, reg, value);
}
