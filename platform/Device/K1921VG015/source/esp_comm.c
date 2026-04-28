#include "uart_tx.h"
#include "esp_comm.h"
#include <stdarg.h>

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
void ESP_SendFormatted(const char* fmt, ...)
{
    uint8_t buffer[64];
    uint8_t i = 0;
    va_list args;
    va_start(args, fmt);
    bool first = true;

    while (*fmt) {
        if (!first) {
            buffer[i++] = ',';
        }
        first = false;

        switch (*fmt) {
            case 's': // int16_t
                i = append_int16(buffer, i, (int16_t)va_arg(args, int));
                break;
            case 'i': // int32_t
                i = append_int32(buffer, i, va_arg(args, int32_t));
                break;
            case 'f': // float (2 знака)
                i = append_float(buffer, i, (float)va_arg(args, double), 2);
                break;
            default:
                first = true;
                break;
        }
        fmt++;
    }

    buffer[i++] = '\r';
    buffer[i++] = '\n';
    UART2_SendBuffer(buffer, i);
    va_end(args);
}

void ESP_Send_Error(void)
{
    uint8_t buffer[] = "ERROR\r\n";
    UART2_SendBuffer(buffer, sizeof(buffer) - 1);
}

void ESP_Send_Stop(void)
{
    uint8_t buffer[] = "STOP\r\n";
    UART2_SendBuffer(buffer, sizeof(buffer) - 1);
}