#include "uart_tx.h"
#include "plib015_uart.h"
#include "K1921VG015.h"
#include "esp_comm.h"
#include "mtimer.h"
#include <stdarg.h>
#include <string.h>

static EspLinkState_t link_state = LINK_OK;
static bool esp_online = false;

static void process_ack_result(bool ack_received)
{
    switch (link_state) {
    case LINK_OK:
        if (ack_received) {
            esp_online = true;
        } else {
            ESP_HardwareReset();
            link_state = LINK_RECOVERY;
            esp_online = false;
        }
        break;

    case LINK_RECOVERY:
        if (ack_received) {
            link_state = LINK_OK;
            esp_online = true;
        } else {
            link_state = LINK_ERROR;
            esp_online = false;
        }
        break;

    case LINK_ERROR:
        if (ack_received) {
            link_state = LINK_OK;
            esp_online = true;
        } else {
            esp_online = false;
        }
        break;
    }
}

void ESP_InitResetPin(void)
{
    RCU->CGCFGAHB_bit.GPIOBEN = 1;
    RCU->RSTDISAHB_bit.GPIOBEN = 1;
    GPIOB->OUTENSET = ESP_RST_MSK; 
    GPIOB->DATAOUTSET = ESP_RST_MSK;
}

void ESP_HardwareReset(void)
{
    GPIOB->DATAOUTCLR = ESP_RST_MSK;
    mtimer_sleep(100);
    GPIOB->DATAOUTSET = ESP_RST_MSK;
    mtimer_sleep(3000);      
    UART2_FlushRx();      
}

bool ESP_WaitForAck(void)
{
   for (int attempt = 0; attempt < ACK_MAX_ATTEMPTS; attempt++) {
        uint8_t buf[8] = {0};
        int idx = 0;
        uint64_t start_time = mtimer_get_raw_time();
        uint64_t timeout_clocks = MTIMER_MSEC_TO_CLOCKS(ACK_TIMEOUT_MS);
        
        while (1) {
            if (UART2_DataAvailable()) {
                char c = (char)UART_RecieveData(UART2);
                if (c == '\n' || idx >= 7) {
                    if (idx < 7) buf[idx] = '\0';
                    break;
                }
                buf[idx++] = c;
                start_time = mtimer_get_raw_time();
            }
            if (mtimer_get_raw_time() - start_time > timeout_clocks) {
                break; 
            }
        }
        
        if (idx > 0 && strcmp((char*)buf, "ACK") == 0) {
            while (UART2_DataAvailable()) {
                if (UART_RecieveData(UART2) == '\n') break;
            }
            return true;
        }
    }
    return false;
}

void ESP_SendWithAck(const uint8_t *data, uint16_t len)
{
    UART2_SendBuffer(data, len);
    process_ack_result(ESP_WaitForAck());
}

static uint8_t append_int16(uint8_t* buffer, uint8_t idx, int16_t value)
{
    if (value < 0) {
        buffer[idx++] = '-';
        value = -value;
    }
    uint8_t rev[8];
    uint8_t j = 0;
    uint16_t num = (uint16_t)value;
    if (num == 0) {
        buffer[idx++] = '0';
    } else {
        while (num > 0) {
            rev[j++] = '0' + (num % 10);
            num /= 10;
        }
        while (j > 0) {
            buffer[idx++] = rev[--j];
        }
    }
    return idx;
}

static uint8_t append_int32(uint8_t* buffer, uint8_t idx, int32_t value)
{
    if (value < 0) {
        buffer[idx++] = '-';
        value = -value;
    }
    uint8_t rev[16];
    uint8_t j = 0;
    uint32_t num = (uint32_t)value;
    if (num == 0) {
        buffer[idx++] = '0';
    } else {
        while (num > 0) {
            rev[j++] = '0' + (num % 10);
            num /= 10;
        }
        while (j > 0) {
            buffer[idx++] = rev[--j];
        }
    }
    return idx;
}

static uint8_t append_float(uint8_t* buffer, uint8_t idx, float value, uint8_t decimals)
{
    if (value < 0) {
        buffer[idx++] = '-';
        value = -value;
    }

    uint32_t factor = 1;
    for (uint8_t i = 0; i < decimals; i++) factor *= 10;
    uint32_t scaled = (uint32_t)(value * factor + 0.5f);

    uint32_t int_part = scaled / factor;
    uint32_t frac_part = scaled % factor;

    uint8_t temp[16];
    uint8_t j = 0;
    if (int_part == 0) {
        temp[j++] = '0';
    } else {
        uint32_t num = int_part;
        while (num > 0) {
            temp[j++] = '0' + (num % 10);
            num /= 10;
        }
    }
    while (j > 0) {
        buffer[idx++] = temp[--j];
    }

    buffer[idx++] = '.';

    uint32_t divisor = factor / 10;
    uint32_t frac = frac_part;
    for (uint8_t d = 0; d < decimals; d++) {
        buffer[idx++] = '0' + (frac / divisor);
        frac %= divisor;
        divisor /= 10;
    }
    return idx;
}

// Отправка произвольного числа данных на ESP в виде строки формата ("s,i,f...") 
// где 's' - число int16, 'i' - int32, 'f' - float (4 bytes)
void ESP_SendFormattedAck(const char* fmt, ...)
{
    uint8_t buffer[64];
    int i = 0;
    va_list args;
    va_start(args, fmt);
    bool first = true;

    while (*fmt) {
        if (!first) buffer[i++] = ',';
        first = false;
        switch (*fmt) {
            case 's': i = append_int16(buffer, i, (int16_t)va_arg(args, int)); break;
            case 'i': i = append_int32(buffer, i, va_arg(args, int32_t)); break;
            case 'f': i = append_float(buffer, i, (float)va_arg(args, double), 2); break;
            default: first = true; break;
        }
        fmt++;
    }
    buffer[i++] = '\r';
    buffer[i++] = '\n';
    UART2_SendBuffer(buffer, i);
    va_end(args);
    process_ack_result(ESP_WaitForAck());
}

void ESP_SendErrorAck(void)
{
    uint8_t buf[] = "ERROR\r\n";
    UART2_SendBuffer(buf, sizeof(buf)-1);
    process_ack_result(ESP_WaitForAck());
}

void ESP_SendStopAck(void)
{
    uint8_t buf[] = "STOP\r\n";
    UART2_SendBuffer(buf, sizeof(buf)-1);
    process_ack_result(ESP_WaitForAck());
}

bool ESP_IsOnline(void) { return esp_online; }
bool ESP_IsError(void)  { return (link_state == LINK_ERROR); }