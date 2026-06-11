#include <K1921VG015.h>
#include <system_k1921vg015.h>
#include <string.h>
#include <stdio.h>
#include "mtimer.h"
#include "i2c_tx.h"
#include "oled_small.h"
#include "uart_tx.h"
#include "modbus_sbs_regs.h"
#include "modbus_funcs.h"
#include "esp_comm.h"
#include "dose_calc.h"
#include "temp.h"
#include "power_mgmt.h"
#include "bitmaps.h"

#define LEDS_MSK  0xF000
#define LED0_PIN  12
#define LED0_MSK  (1 << LED0_PIN)

static volatile bool tmr_wdt_trigger = false;

static void led_init(void) 
{
    RCU->CGCFGAHB_bit.GPIOAEN = 1;
    RCU->RSTDISAHB_bit.GPIOAEN = 1;
    GPIOA->OUTENSET = LEDS_MSK;
    GPIOA->DATAOUTCLR = LEDS_MSK;
}

static void IWDT_Init(uint32_t ms) 
{
    PMURTC->IWDG_CFG = (2 << PMURTC_IWDG_CFG_CLKSRC_Pos) | PMURTC_IWDG_CFG_RSTDIS_Msk;
    uint32_t ticks = (LSICLK_VAL / 1000) * ms;
    IWDT->LOAD = ticks;
    IWDT->CTRL = IWDT_CTRL_INTEN_Msk | IWDT_CTRL_RESEN_Msk;
    IWDT->LOCK = 0x1ACCE551;
}

static void IWDT_Reset(void) 
{
    IWDT->INTCLR = 0xFFFFFFFF;
}

static void TMR32_IRQHandler(void)
{
    tmr_wdt_trigger = true;
    TMR32->IC = 3;
}

static void TMR32_Init_WDT(uint32_t period_ms)
{
    RCU->CGCFGAPB_bit.TMR32EN = 1;
    RCU->RSTDISAPB_bit.TMR32EN = 1;
    TMR32->CAPCOM[0].VAL = ((SystemCoreClock / 1000) * period_ms) - 1;
    TMR32->CTRL_bit.MODE = 1;
    TMR32->IM = 2;
    PLIC_SetIrqHandler(Plic_Mach_Target, IsrVect_IRQ_TMR32, TMR32_IRQHandler);
    PLIC_SetPriority(IsrVect_IRQ_TMR32, 0x1);
    PLIC_IntEnable(Plic_Mach_Target, IsrVect_IRQ_TMR32);
}

static void UART2_IRQHandler(void)
{
    UART2->ICR_bit.RTIC = 1;
}

static void UART2_EnableRXInterrupt(void)
{
    UART2->IMSC_bit.RTIM = 1;

    PLIC_SetIrqHandler(Plic_Mach_Target, IsrVect_IRQ_UART2, UART2_IRQHandler);
    PLIC_SetPriority(IsrVect_IRQ_UART2, 0x2); 
    PLIC_IntEnable(Plic_Mach_Target, IsrVect_IRQ_UART2);
}

static void periph_init(void) 
{
    SystemInit();
    SystemCoreClockUpdate();
    IWDT_Init(15000);
    led_init();
    UART1_init();
    UART2_init();
    UART2_EnableRXInterrupt();
    I2C_init();
    OLED_init();
    adcsar_init(TSENSOR_ISRC_INT);
    
    MODBUS_WriteInt16(SBS_ADDR, SBS_NCHANNELS_REG, 128);
    mtimer_sleep(50);
    MODBUS_WriteInt16(SBS_ADDR, SBS_SP_OFFSET_REG, 0);
    mtimer_sleep(50);
    MODBUS_WriteFloat(SBS_ADDR, SBS_ENK0_REG, (float)(0.0));
    mtimer_sleep(50);
    MODBUS_WriteFloat(SBS_ADDR, SBS_ENK1_REG, (float)(0.0));
    mtimer_sleep(50);
    MODBUS_WriteInt16(SBS_ADDR, SBS_STATE_REG, SBS_STATE_STOP);
    mtimer_sleep(50);
    MODBUS_WriteInt16(SBS_ADDR, SBS_STATE_REG, SBS_STATE_CLEAR);
    DoseCalc_Init();
    ESP_Init();
}

static void check(void) 
{
    mtimer_sleep(800);
    GPIOA->DATAOUTTGL = LED0_MSK;
    mtimer_sleep(200);
    GPIOA->DATAOUTTGL = LED0_MSK;
}

static void update_oled(void)
{
    DisplayState_t state = ESP_GetDisplayState();
    char st_text[8];
    uint8_t error_code = 5;

    switch (state) {
        case DISP_OK: 
            strcpy(st_text, "ИЗМЕР"); 
            error_code = 0; 
            break;
        case DISP_WRITE_ERROR:
            strcpy(st_text, "КОД 1");
            error_code = 1;
            break;
        case DISP_PROTOCOL_ERROR:
            strcpy(st_text, "КОД 2");
            error_code = 2;
            break;
        case DISP_READ_ERROR:
            strcpy(st_text, "КОД 3");
            error_code = 3;
            break;
        case DISP_STOPPED:
            strcpy(st_text, " СТОП");
            error_code = 4;
            break;
        default:
            strcpy(st_text, "  Н/Д");
            error_code = 5;
            break;
    }

    OLED_setpos(88, 5);
    OLED_printS(st_text, true);

    if (error_code == 0 || error_code == 4) {
        char aderlen = float_num_len(g_ader, 2);
        OLED_setpos((128 - aderlen * 8) / 2 - 12, 3);
        OLED_printF(g_ader, 2, false);
    } else {
        OLED_setpos(32, 3);
        OLED_printS(" Н/Д ", false);
    }
}

int main(void) 
{
    periph_init();
    check();

    OLED_clear();
    OLED_DrawBitmap(0, 2, 128, 40, GSlogo, false);
    mtimer_sleep(5000);
    OLED_clear();
    for (int i = 0; i < 11; i++) {
        OLED_DrawBitmap(115, 0, 12, 16, battery[i], false);
        mtimer_sleep(100);
    }
    OLED_DrawBitmap(0, 0, 12, 16, wifi, false);
    OLED_setpos(0, 3);   OLED_printS("H10", false);
    OLED_setpos(36, 3);  OLED_printS("Н/Д", false);
    OLED_setpos(80, 3);  OLED_printS("мкЗв/ч", false);
    OLED_setpos(0, 5);   OLED_printS("СТАТУС:", false);
    OLED_setpos(88, 5);  OLED_printS("  Н/Д", true);
    OLED_setpos(0, 7);   OLED_printS("ТЕМП:", false);
    OLED_setpos(112, 7); OLED_printS("°С", false);

    InterruptEnable();
    TMR32_Init_WDT(10000); 

    while (1) {

        ESP_ProcessRequest();

        while (!ESP_QueueEmpty()) {
            ESP_ExecuteNextCommand();
        }
        update_oled();

        if (g_data_requested) {
            ESP_HandleDataRequest();
            g_data_requested = false;
            update_oled();
        }

        if (tmr_wdt_trigger) {
            tmr_wdt_trigger = false;
            IWDT_Reset();
            OLED_setpos(72, 7);
            OLED_printF(Get_Temp_Celsius(), 2, false);
        }

        PM_EnterMode(PM_IDLE); 
    }
    
    return 0;
}