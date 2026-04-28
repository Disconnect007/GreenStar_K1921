// ===================================================================================
// Basic I2C Master Functions for K1921VG015 (write only)                     
// ===================================================================================

#include <K1921VG015.h>
#include <system_k1921vg015.h>
#include "i2c_tx.h"
#include "plib015_i2c.h"
#include "mtimer.h"

void I2C_init(void) 
{
    /* I2C: SCL - PC.12, SDA - PC.13 */
    RCU->CGCFGAHB_bit.GPIOCEN = 1;  // Разрешение тактирования порта GPIOC
    RCU->RSTDISAHB_bit.GPIOCEN = 1; // Вывод из состояния сброса порта GPIOC
    GPIOC->ALTFUNCSET = GPIO_ALTFUNCSET_PIN12_Msk | GPIO_ALTFUNCSET_PIN13_Msk;
    GPIOC->ALTFUNCNUM_bit.PIN12 = 1; // Выбор альтернативной функции пинов
    GPIOC->ALTFUNCNUM_bit.PIN13 = 1;
    GPIOC->OUTMODE_bit.PIN12 = 1;  // Open Drain
    GPIOC->OUTMODE_bit.PIN13 = 1;
    GPIOC->PULLMODE |= 0x3000; // Pull-up
    
    RCU->CGCFGAPB_bit.I2CEN = 1;
    RCU->RSTDISAPB_bit.I2CEN = 1;
    
    I2C_Cmd(DISABLE); // Отключаем I2C перед настройкой

    I2C_FSFreqConfig(FSFreq, SystemCoreClock);
    I2C_HSFreqConfig(HSFreq, SystemCoreClock);
    
    I2C_Cmd(ENABLE); // Включение модуля I2C
    I2C_ITCmd(ENABLE); // Включение прерываний
    
    mtimer_sleep(1);
}

void I2C_start(uint8_t addr) 
{
    uint64_t timeout;

    timeout = mtimer_get_raw_time() + MTIMER_USEC_TO_CLOCKS(I2C_TIMEOUT_BUS_FREE_US);
    while (I2C_BusBusyStatus() == SET) {
        if (mtimer_get_raw_time() >= timeout) return;
    }

    I2C_StartCmd();
    timeout = mtimer_get_raw_time() + MTIMER_USEC_TO_CLOCKS(I2C_TIMEOUT_STATE_US);
    while (I2C_GetState() != I2C_State_STDONE) {
        if (mtimer_get_raw_time() >= timeout) return;
    }

    I2C_SetData(addr);
    I2C_ITStatusClear();
    I2C_ITCmd(ENABLE);

    timeout = mtimer_get_raw_time() + MTIMER_USEC_TO_CLOCKS(I2C_TIMEOUT_STATE_US);
    while (1) {
        I2C_State_TypeDef state = I2C_GetState();
        if (state == I2C_State_MTADPA) break;     // ACK
        if (state == I2C_State_MTADNA) return;    // NACK
        if (state == I2C_State_BERROR) return;    // Ошибка шины
        if (mtimer_get_raw_time() >= timeout) return;
    }
}

void I2C_write(uint8_t data) 
{
    I2C_State_TypeDef state = I2C_GetState();
    if (state != I2C_State_MTADPA && state != I2C_State_MTDAPA) return;

    I2C_SetData(data);
    I2C_ITStatusClear();
    I2C_ITCmd(ENABLE);

    uint64_t timeout = mtimer_get_raw_time() + MTIMER_USEC_TO_CLOCKS(I2C_TIMEOUT_STATE_US);
    while (1) {
        state = I2C_GetState();
        if (state == I2C_State_MTDAPA) break;
        if (state == I2C_State_MTDANA) break;
        if (mtimer_get_raw_time() >= timeout) return;
    }
}

void I2C_stop(void) 
{
    I2C_StopCmd();
    I2C_ITStatusClear();
    I2C_ITCmd(ENABLE);
    mtimer_sleep(1);
}