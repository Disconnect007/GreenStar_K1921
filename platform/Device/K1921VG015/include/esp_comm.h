#ifndef ESP_COMM_H_
#define ESP_COMM_H_

#include <stdint.h>

void ESP_SendFormatted(const char* fmt, ...);
void ESP_Send_Error(void);
void ESP_Send_Stop(void);

#endif