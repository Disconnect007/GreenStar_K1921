#include <K1921VG015.h>
#include <system_k1921vg015.h>
#include "mtimer.h"
#include "plib015_uart.h"
#include "uart_tx.h"

void UART1_init()
{
    UART_Cmd(UART1, DISABLE); // 25.5 пункт РП по UART
    uint64_t busy_timeout = mtimer_get_raw_time() + MTIMER_USEC_TO_CLOCKS(UART_BUSY_TIMEOUT_US);
    while (UART_FlagStatus(UART1, UART_Flag_Busy) == SET) {
        if (mtimer_get_raw_time() >= busy_timeout) return;
    }
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
    
    uart1_config.BaudRate = UART_BAUD; 
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

void UART1_SendBuffer(const uint8_t *data, uint16_t length) 
{
    for (uint16_t i = 0; i < length; i++) {
        uint64_t tx_timeout = mtimer_get_raw_time() + MTIMER_USEC_TO_CLOCKS(UART_TX_TIMEOUT_US);
        while (UART_FlagStatus(UART1, UART_Flag_TxFIFOFull) == SET) {
            if (mtimer_get_raw_time() >= tx_timeout) return;
        }
        UART_SendData(UART1, data[i]);
    }
}

static bool UART1_DataAvailable(void) 
{
    return (UART_FlagStatus(UART1, UART_Flag_RxFIFOEmpty) == CLEAR);
}

void UART1_FlushRx(void) 
{
    uint16_t cnt = 0;
    while (UART1_DataAvailable() && (cnt < UART_FLUSH_MAX_BYTES)) {
        (void)UART_RecieveData(UART1);
        cnt++;
    }
}

uint16_t UART1_ReceiveBuffer(uint8_t *buffer, uint16_t max_len, uint32_t timeout_ms, uint32_t interchar_ms) 
{
    uint64_t start_time = mtimer_get_raw_time();
    uint64_t timeout_clocks = MTIMER_MSEC_TO_CLOCKS(timeout_ms);
    uint64_t interchar_clocks = MTIMER_MSEC_TO_CLOCKS(interchar_ms);
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

void UART2_init(void)
{
    UART_Cmd(UART2, DISABLE);
    uint64_t busy_timeout = mtimer_get_raw_time() + MTIMER_USEC_TO_CLOCKS(UART_BUSY_TIMEOUT_US);
    while (UART_FlagStatus(UART2, UART_Flag_Busy) == SET) {
        if (mtimer_get_raw_time() >= busy_timeout) return;
    }
    UART_FIFOCmd(UART2, DISABLE);

    RCU->CGCFGAHB_bit.GPIOAEN = 1;
    RCU->RSTDISAHB_bit.GPIOAEN = 1;
    RCU->CGCFGAPB_bit.UART2EN = 1;
    RCU->RSTDISAPB_bit.UART2EN = 1;

    GPIOA->ALTFUNCNUM_bit.PIN4 = 1;
    GPIOA->ALTFUNCNUM_bit.PIN5 = 1;
    GPIOA->ALTFUNCSET = GPIO_ALTFUNCSET_PIN4_Msk | GPIO_ALTFUNCSET_PIN5_Msk;

    RCU->UARTCLKCFG[2].UARTCLKCFG_bit.CLKSEL = RCU_UARTCLKCFG_CLKSEL_HSE;
    RCU->UARTCLKCFG[2].UARTCLKCFG_bit.DIVEN = 0;
    RCU->UARTCLKCFG[2].UARTCLKCFG_bit.RSTDIS = 1;
    RCU->UARTCLKCFG[2].UARTCLKCFG_bit.CLKEN = 1;

    UART_Init_TypeDef uart2_config;
    UART_StructInit(&uart2_config);
    
    uart2_config.BaudRate = UART_BAUD;
    uart2_config.DataWidth = UART_DataWidth_8;
    uart2_config.StopBit = UART_StopBit_1;
    uart2_config.ParityBit = UART_ParityBit_Disable;
    uart2_config.FIFO = ENABLE;
    uart2_config.Rx = ENABLE;
    uart2_config.Tx = ENABLE;
    
    UART_Init(UART2, &uart2_config);
    UART_Cmd(UART2, ENABLE);
    UART_ErrorStatusClear(UART2, UART_Error_All);
}

void UART2_SendBuffer(const uint8_t *data, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++) {
        uint64_t tx_timeout = mtimer_get_raw_time() + MTIMER_USEC_TO_CLOCKS(UART_TX_TIMEOUT_US);
        while (UART_FlagStatus(UART2, UART_Flag_TxFIFOFull) == SET) {
            if (mtimer_get_raw_time() >= tx_timeout) return;
        }
        UART_SendData(UART2, data[i]);
    }    
}

static bool UART2_DataAvailable(void)
{
    return (UART_FlagStatus(UART2, UART_Flag_RxFIFOEmpty) == CLEAR);
}

void UART2_FlushRx(void)
{
    uint16_t cnt = 0;
    while (UART2_DataAvailable() && (cnt < UART_FLUSH_MAX_BYTES)) {
        (void)UART_RecieveData(UART1);
        cnt++;
    }
}

uint16_t UART2_ReceiveBuffer(uint8_t *buffer, uint16_t max_len, uint32_t timeout_ms, uint32_t interchar_ms)
{
    uint64_t start_time = mtimer_get_raw_time();
    uint64_t timeout_clocks = MTIMER_MSEC_TO_CLOCKS(timeout_ms);
    uint64_t interchar_clocks = MTIMER_MSEC_TO_CLOCKS(interchar_ms);
    uint64_t last_byte_time = start_time;
    uint16_t received = 0;
    
    while (received < max_len) {
        if (UART2_DataAvailable()) {
            buffer[received++] = UART_RecieveData(UART2);
            last_byte_time = mtimer_get_raw_time();
            if ((mtimer_get_raw_time() - start_time) > timeout_clocks) {
                break;
            }
        } else {
            if ((mtimer_get_raw_time() - last_byte_time) > interchar_clocks) {
                break;
            }
        }
    }
    return received;
}