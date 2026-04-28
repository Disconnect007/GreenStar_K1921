#include <K1921VG015.h>
#include <system_k1921vg015.h>
#include <string.h>
#include "mtimer.h"
#include "i2c_tx.h"
#include "oled_small.h"
#include "uart_tx.h"
#include "modbus_sbs_regs.h"
#include "modbus_funcs.h"
#include "esp_comm.h"
#include "temp.h"
#include "power_mgmt.h"


#define LEDS_MSK  0xF000
#define LED0_PIN  12
#define LED0_MSK  (1 << LED0_PIN)

#define TIME_EPS    1.0E-5
#define MIN_DELTA   1.0E-2
#define DZ          1.0E-6

//static double ENK0[6] = {-239.137, -118.336, -62.554, -35.514, -24.453, -14.380};
//static double ENK1[6] = {24.549, 12.396, 6.198, 3.122, 1.559, 0.780};
static const double ENK0[6] = {-240.0, -120.0, -60.0, -30.0, -15.0, -7.5};
static const double ENK1[6] = {24.0, 12.0, 6.0, 3.0, 1.5, 0.75};
static int8_t k_idx = 0;

static volatile bool tmr_trigger = false;

static uint32_t spectr[4096] = {};
static uint32_t prev_spectr[4096] = {};

static float DoseRateInstant(uint32_t spectr[], uint16_t nchannels, float ltime, double Dz, uint8_t idx, uint64_t sp_rec_time);
static float DoseRatediff(uint32_t spectr[], uint32_t prev_spectr[], uint16_t nchannels, float delta_ltime, double Dz, uint8_t idx);

static void led_init(void) 
{
    RCU->CGCFGAHB_bit.GPIOAEN = 1;
    RCU->RSTDISAHB_bit.GPIOAEN = 1;
    GPIOA->OUTENSET = LEDS_MSK;
    GPIOA->DATAOUTCLR = LEDS_MSK;
}

static void IWDT_Init(uint32_t timeout_ms) 
{
    PMURTC->IWDG_CFG = (2 << PMURTC_IWDG_CFG_CLKSRC_Pos) | PMURTC_IWDG_CFG_RSTDIS_Msk;
    uint32_t ticks = (LSICLK_VAL / 1000) * timeout_ms;
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
    tmr_trigger = true;            
    TMR32->IC = 3;                    
}

static void TMR32_init(uint32_t period_ms)
{
    RCU->CGCFGAPB_bit.TMR32EN = 1;
    RCU->RSTDISAPB_bit.TMR32EN = 1;

    TMR32->CAPCOM[0].VAL = ((SystemCoreClock / 1000) * period_ms) - 1;
    TMR32->CTRL_bit.MODE = 1;
    TMR32->IM = 2;

    PLIC_SetIrqHandler (Plic_Mach_Target, IsrVect_IRQ_TMR32, TMR32_IRQHandler);
    PLIC_SetPriority   (IsrVect_IRQ_TMR32, 0x1);
    PLIC_IntEnable     (Plic_Mach_Target, IsrVect_IRQ_TMR32);
}

static void periph_init(void) 
{
    SystemInit();
    SystemCoreClockUpdate();
    IWDT_Init(30000);
    led_init();
    UART1_init();
    UART2_init();
    I2C_init();
    OLED_init();
    adcsar_init(TSENSOR_ISRC_INT);
}

static void check(void) 
{
    mtimer_sleep(800);
    GPIOA->DATAOUTTGL = LED0_MSK;
    mtimer_sleep(200);
    GPIOA->DATAOUTTGL = LED0_MSK;
}

int main(void) 
{
    periph_init();
    check();

    int32_t sbs_time = 0;
    float ader = 0.0;
    float ltime = 0.0, inprate = 0.0;
    bool success = false, first_measure = false;
    double prev_ltime = 0.0;
    char aderlen = 0;
    char err = 3;
    int16_t nchan = 0;

    OLED_clear();
    OLED_setpos(32, 1);
    OLED_printS("  МАЭД  ", true);
    OLED_setpos(52, 2);
    OLED_printS("Н/Д", false);
    OLED_setpos(32, 3);
    OLED_printS("[мкЗв/ч]", false);
    OLED_setpos(0, 5);
    OLED_printS("СТАТУС:", false);
    OLED_setpos(88, 5);
    OLED_printS(" Н/Д ", true);
    OLED_setpos(0, 7);
    OLED_printS("ТЕМП:", false);
    OLED_setpos(96, 7);
    OLED_printS("[°С]", false);

    TMR32_init(3000);
    InterruptEnable();

    while(1)
    {
        while (!tmr_trigger) {
            InterruptDisable();
            if (!tmr_trigger) {
                InterruptEnable();
                __asm volatile ("WFI");
                __asm("NOP");
		        __asm("NOP");
		        __asm("NOP");
            } else {
                InterruptEnable();
            }
        }
        tmr_trigger = false;

        OLED_setpos(38, 7);
        OLED_printS(" ", true);
        OLED_printF(Get_Temp_Celsius(), 2, true);
        OLED_printS(" ", true);

        switch(err) {
            case 0:
                OLED_setpos(64, 5);
                OLED_printS("        ", false);
                OLED_setpos(72, 5);
                OLED_printS(" НОРМА ", true);
                break;
            case 1: 
                OLED_setpos(44, 2);
                OLED_printS(" Н/Д ", false);
                OLED_setpos(64, 5);
                OLED_printS("        ", false);
                OLED_setpos(64, 5);
                OLED_printS(" ОШИБКА ", true);
                break;
            case 2:
                OLED_setpos(64, 5);
                OLED_printS("        ", false);
                OLED_setpos(80, 5);
                OLED_printS(" СТОП ", true);
                break;
            default:
                OLED_setpos(64, 5);
                OLED_printS("        ", false);
                OLED_setpos(88, 5);
                OLED_printS(" Н/Д ", true);
                break;
        }

        success = MODBUS_ReadInt32(SBS_ADDR, 0x090F, &sbs_time);
        if (!success) { 
            ESP_Send_Error();
            first_measure = true;
            err = 1; 
            continue; 
        }

        success = MODBUS_ReadInt16(SBS_ADDR, SBS_NCHANNELS_REG, &nchan);
        if (!success) {
            ESP_Send_Error();  
            first_measure = true;
            err = 1; 
            continue; 
        }
        
        InterruptDisable();
        uint64_t t_sp_start = mtimer_get_raw_time();
        success = MODBUS_ReadSpectrum(SBS_ADDR, SBS_SP0_CHANNEL, nchan, spectr, 60);
        uint64_t sp_rec_time = mtimer_get_raw_time() - t_sp_start;
        if (!success) { 
            ESP_Send_Error(); 
            first_measure = true; 
            err = 1; 
            continue; 
        }

        success = MODBUS_ReadFloat(SBS_ADDR, SBS_LTIME_REG, &ltime);
        if (!success || ltime <= 0.0f) {
            ESP_Send_Error();
            first_measure = true;
            err = 1;  
            continue; 
        }
        InterruptEnable();
        
        success = MODBUS_ReadFloat(SBS_ADDR, SBS_INPRATE_REG, &inprate);
        if (!success) {
            ESP_Send_Error(); 
            first_measure = true;
            err = 1;  
            continue; 
        }

        switch(nchan) {
            case 128:  k_idx = 0; break;
            case 256:  k_idx = 1; break;
            case 512:  k_idx = 2; break;
            case 1024: k_idx = 3; break;
            case 2048: k_idx = 4; break;
            case 4096: k_idx = 5; break;
            default:   k_idx = -1;
        }

        if(k_idx == -1) {err = 1; continue;}

        if(first_measure) {
            ader = DoseRateInstant(spectr, nchan, ltime, DZ, k_idx, sp_rec_time);
            if (ader < 0) {IWDT_Reset(); continue;}
            ESP_SendFormatted("s,f,f,f", nchan, ader, ltime, inprate);
            memcpy(prev_spectr, spectr, nchan * sizeof(uint32_t));
            prev_ltime = ltime;
            first_measure = false;
            aderlen = float_num_len(ader, 2);
            OLED_setpos((128 - aderlen * 8) / 2, 2);
            OLED_printF(ader, 2, false);
        } else {
            float delta_ltime = ltime - prev_ltime;
            if (delta_ltime < -TIME_EPS) {
                first_measure = true;
                IWDT_Reset();
                continue;
            }
            if (delta_ltime <= TIME_EPS) {
                ESP_Send_Stop();
                err = 2;
                IWDT_Reset(); 
                continue;   
            }
            if (delta_ltime < MIN_DELTA) {
                IWDT_Reset();
                continue;
            }

            ader = DoseRatediff(spectr, prev_spectr, nchan, delta_ltime, DZ, k_idx);
            ESP_SendFormatted("s,f,f,f", nchan, ader, ltime, inprate);
            memcpy(prev_spectr, spectr, nchan * sizeof(uint32_t));
            prev_ltime = ltime;
            aderlen = float_num_len(ader, 2);
            OLED_setpos((128 - aderlen * 8) / 2, 2);
            OLED_printF(ader, 2, false);
        }
        err = 0;
        IWDT_Reset();
    }
    return 0;
}

static float DoseRatediff(uint32_t spectr[], uint32_t prev_spectr[], uint16_t nchannels, float delta_ltime, double Dz, uint8_t idx)
{
    double enk0 = ENK0[idx];
    double enk1 = ENK1[idx];
    double sum_diff = 0.0;
    for (uint16_t i = 0; i < nchannels; i++) {
        int32_t diff = (int32_t)spectr[i] - (int32_t)prev_spectr[i];
        if (diff < 0) diff = 0;
            double E_kev = enk0 + enk1 * i;
            sum_diff += (double)diff * E_kev;
        }
    double dose_rate = (sum_diff * Dz) / delta_ltime;
    return (float)dose_rate;
}


static float DoseRateInstant(uint32_t spectr[], uint16_t nchannels, float ltime, double Dz, uint8_t idx, uint64_t sp_rec_time)
{
    double delta = (double)sp_rec_time / (double)SystemCoreClock;
    double enk0 = ENK0[idx];
    double enk1 = ENK1[idx];
    double summ = 0.0;
    for (uint16_t i = 0; i < nchannels; i++) {
        double E_kev = enk0 + enk1 * i;
        summ += (double)spectr[i] * E_kev;
    }
    double dose_rate = (summ * Dz) / (ltime - delta);
    return (float)dose_rate;
}
