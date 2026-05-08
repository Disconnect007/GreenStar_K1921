#ifndef ESP_COMM_H_
#define ESP_COMM_H_

#include <stdint.h>
#include <stdbool.h>

#define ESP_RST_MSK 0x8000 // PB15

#define ACK_TIMEOUT_MS      1000
#define ACK_MAX_ATTEMPTS    3

typedef enum {
    LINK_OK,           // обмен в порядке
    LINK_RECOVERY,     // один сброс выполнен, ждём ACK
    LINK_ERROR         // сброс не помог, аппаратная ошибка
} EspLinkState_t;

void ESP_InitResetPin(void);
void ESP_HardwareReset(void);
bool ESP_WaitForAck(void);
void ESP_SendWithAck(const uint8_t *data, uint16_t len);
void ESP_SendFormattedAck(const char* fmt, ...);
void ESP_SendErrorAck(void);
void ESP_SendStopAck(void);
bool ESP_IsOnline(void);
void ESP_ClearErrorCount(void);
bool ESP_IsError(void);

#endif