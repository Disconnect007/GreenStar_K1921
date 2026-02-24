#ifndef UART_TX_H_
#define UART_TX_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef union {
    float f;
    uint8_t bytes[4];
} FloatBytes_t;

void UART1_init();
void UART1_SendFloat(float value);
void UART1_SendString(const char *str);
void UART1_SendUInt16(const uint16_t num);
void UART1_SendBuffer(const uint8_t *data, uint16_t length);
bool UART1_DataAvailable(void);
void UART1_FlushRx(void);
uint16_t UART1_ReceiveBuffer(uint8_t *buffer, uint16_t max_len, uint32_t timeout_ms, uint32_t interchar_ms);


#ifdef __cplusplus
};
#endif
#endif 