#include <K1921VG015.h>
#include <system_k1921vg015.h>
#include "mtimer.h"
#include "plib015_uart.h"
#include "uart_tx.h"
#include "modbus_funcs.h"
#include "modbus_crc.h"


#define UART1_BAUD 115200
#define MODBUS_TIMEOUT_MS 100     // Таймаут ожидания ответа
#define MODBUS_INTERCHAR_MS 1     // Макс. пауза между символами

void UART1_init()
{

    UART_Cmd(UART1, DISABLE); // 25.5 пункт РП по UART
    while(UART_FlagStatus(UART1, UART_Flag_Busy) == SET); 
    UART_FIFOCmd(UART1, DISABLE); // 25.5 пункт РП по UART

    RCU->CGCFGAHB_bit.GPIOAEN = 1;
    RCU->RSTDISAHB_bit.GPIOAEN = 1;
    RCU->CGCFGAPB_bit.UART1EN = 1;
    RCU->RSTDISAPB_bit.UART1EN = 1;

    GPIOA->ALTFUNCNUM_bit.PIN2 = 1;
    GPIOA->ALTFUNCNUM_bit.PIN3 = 1;
    GPIOA->ALTFUNCSET = GPIO_ALTFUNCSET_PIN2_Msk | GPIO_ALTFUNCSET_PIN3_Msk;

    RCU->UARTCLKCFG[1].UARTCLKCFG_bit.CLKSEL = RCU_UARTCLKCFG_CLKSEL_HSE;
    RCU->UARTCLKCFG[1].UARTCLKCFG_bit.DIVEN = 0;
    RCU->UARTCLKCFG[1].UARTCLKCFG_bit.RSTDIS = 1;
    RCU->UARTCLKCFG[1].UARTCLKCFG_bit.CLKEN = 1;

    UART_Init_TypeDef uart1_config;
    UART_StructInit(&uart1_config);
    
    uart1_config.BaudRate = UART1_BAUD; 
    uart1_config.DataWidth = UART_DataWidth_8;
    uart1_config.StopBit = UART_StopBit_1;
    uart1_config.ParityBit = UART_ParityBit_Disable;
    uart1_config.FIFO = ENABLE;
    uart1_config.Rx = ENABLE;
    uart1_config.Tx = ENABLE;
    
    UART_Init(UART1, &uart1_config);
    UART_Cmd(UART1, ENABLE);
    UART_ErrorStatusClear(UART1, UART_Error_All);
}

void UART1_SendString(const char *str) 
{
    while(*str) {
        while(UART_FlagStatus(UART1, UART_Flag_TxFIFOFull) == SET);
        UART_SendData(UART1, *str++);
    }
}

void UART1_SendBuffer(const uint8_t *data, uint16_t length) 
{
    for(uint16_t i = 0; i < length; i++) {
        while(UART_FlagStatus(UART1, UART_Flag_TxFIFOFull) == SET);
        UART_SendData(UART1, data[i]);
    }
}

bool UART1_DataAvailable(void) 
{
    return (UART_FlagStatus(UART1, UART_Flag_RxFIFOEmpty) == CLEAR);
}

void UART1_FlushRx(void) 
{
    while(UART1_DataAvailable()) {
        (void)UART_RecieveData(UART1);
    }
}

uint16_t UART1_ReceiveBuffer(uint8_t *buffer, uint16_t max_len, uint32_t timeout_ms) 
{
    uint64_t start_time = mtimer_get_raw_time();
    uint64_t timeout_clocks = MTIMER_MSEC_TO_CLOCKS(timeout_ms);
    uint64_t interchar_clocks = MTIMER_MSEC_TO_CLOCKS(MODBUS_INTERCHAR_MS);
    uint64_t last_byte_time = start_time;
    uint16_t received = 0;
    
    while(received < max_len) {
        if(UART1_DataAvailable()) {
            buffer[received++] = UART_RecieveData(UART1);
            last_byte_time = mtimer_get_raw_time();
            
            if((mtimer_get_raw_time() - start_time) > timeout_clocks) {
                break;
            }
        } else {
            if((mtimer_get_raw_time() - last_byte_time) > interchar_clocks) {
                break;
            }
        }
    }
    
    return received;
}

bool MODBUS_ReadRegister(uint8_t slave_addr, uint16_t reg_addr, uint16_t *data) 
{
    uint8_t tx_buffer[8];
    uint8_t rx_buffer[32];
    uint16_t rx_len;
    
    //Формируем запрос
    uint8_t tx_len = modbus_read_holding_request(slave_addr, reg_addr, 1, tx_buffer);
    
    //Очищаем буфер приема перед отправкой
    UART1_FlushRx();
    
    //Отправляем запрос
    UART1_SendBuffer(tx_buffer, tx_len);
    
    //Принимаем ответ
    rx_len = UART1_ReceiveBuffer(rx_buffer, sizeof(rx_buffer), MODBUS_TIMEOUT_MS);
    
    //Проверяем минимальный размер ответа
    if(rx_len < 5) {
        return false; // Слишком короткий ответ
    }
    
    //Проверяем адрес и функцию
    if(rx_buffer[0] != slave_addr || rx_buffer[1] != 0x03) {
        return false; // Не наш ответ или ошибка функции
    }
    
    //Проверяем количество байт данных
    if(rx_buffer[2] != 2) { // 2 байта для 1 регистра
        return false;
    }
    
    //Проверяем CRC ответа
    uint16_t received_crc = (rx_buffer[rx_len-1] << 8) | rx_buffer[rx_len-2];
    uint16_t calculated_crc = modbus_crc16(rx_buffer, rx_len - 2);
    
    if(received_crc != calculated_crc) {
        return false; // Ошибка CRC
    }
    
    //Извлекаем данные (2 байта, Big-Endian)
    *data = (rx_buffer[3] << 8) | rx_buffer[4];
    
    return true;
}