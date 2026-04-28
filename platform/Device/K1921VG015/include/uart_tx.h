#ifndef UART_TX_H_
#define UART_TX_H_

#include <stdint.h>
#include <stdbool.h>

#define UART_BAUD 115200
#define UART_BUSY_TIMEOUT_US  5000   
#define UART_TX_TIMEOUT_US 1000
#define UART_FLUSH_MAX_BYTES  32

#ifdef __cplusplus
extern "C" {
#endif

typedef union {
    float f;
    uint8_t bytes[4];
} FloatBytes_t;

void UART1_init();
void UART1_SendBuffer(const uint8_t *data, uint16_t length);
void UART1_FlushRx(void);
uint16_t UART1_ReceiveBuffer(uint8_t *buffer, uint16_t max_len, uint32_t timeout_ms, uint32_t interchar_ms);
void UART2_init();
void UART2_SendBuffer(const uint8_t *data, uint16_t length);
void UART2_FlushRx(void);
uint16_t UART2_ReceiveBuffer(uint8_t *buffer, uint16_t max_len, uint32_t timeout_ms, uint32_t interchar_ms);

#ifdef __cplusplus
};
#endif
#endif 