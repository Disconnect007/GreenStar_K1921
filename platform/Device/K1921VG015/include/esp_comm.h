#ifndef ESP_COMM_H_
#define ESP_COMM_H_

#include <stdint.h>

uint8_t append_int16(uint8_t* buffer, uint8_t idx, int16_t value);
uint8_t append_int32(uint8_t* buffer, uint8_t idx, int32_t value);
uint8_t append_float(uint8_t* buffer, uint8_t idx, float value, uint8_t decimals);
void ESP_SendFormatted(const char* fmt, ...);
void ESP_Send_Error(void);

#endif