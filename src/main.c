//-- Includes ----------------------------------------------------------------
#include <K1921VG015.h>
#include <system_k1921vg015.h>
#include "mtimer.h"
#include "i2c_tx.h"
#include "oled_small.h"
#include "uart_tx.h"
#include "modbus_sbs_regs.h"
#include "modbus_funcs.h"
#include "esp_comm.h"
#include <string.h>

//-- Defines ------------------------------------------------------------------
#define LEDS_MSK  0xF000
#define LED0_PIN  12
#define LED0_MSK  (1 << LED0_PIN)

#define TIME_EPS	  1.0E-5f
#define MIN_DELTA     1.0E-3f
#define DZ 		      1.0E-6f

//-- Peripheral init functions ------------------------------------------------
void led_init() 
{
	RCU->CGCFGAHB_bit.GPIOAEN = 1;
	RCU->RSTDISAHB_bit.GPIOAEN = 1;
  	GPIOA->OUTENSET = LEDS_MSK;
	GPIOA->DATAOUTCLR = LEDS_MSK;
}

void periph_init() 
{
	SystemInit();
	SystemCoreClockUpdate();
	led_init();
	UART1_init();
	UART2_init();
	I2C_init();
	OLED_init();
}

//--- USER FUNCTIONS -----------------------------------------------------------
void check() 
{
	mtimer_sleep(800);
	GPIOA->DATAOUTTGL = LED0_MSK;
	mtimer_sleep(200);
	GPIOA->DATAOUTTGL = LED0_MSK;
}

float DoseRateInstant(uint32_t spectr[], uint16_t nchannels, float ltime, float inprate, float Dz);

//-- Main ----------------------------------------------------------------------

int main(void) 
{
    periph_init();
    check();
    
    float ader = 0.0f, 
          ltime = 0.0f, 
          inprate = 0.0f;

    bool success = false,
		 first_measure = false;
		 
    int16_t nchan = 0;
    static uint32_t spectr[4096] = {};
    static uint32_t prev_spectr[4096] = {};
    static float prev_ltime = 0.0f;
    static char aderlen = 0;

    OLED_clear();
    OLED_setpos(48, 1);
    OLED_printS("ADER", false);
    OLED_setpos(52, 4);
    OLED_printS("N/D", false);
    OLED_setpos(36, 6);
    OLED_printS("[uSv/h]", false);

    while(1) {
        mtimer_sleep(1);

        success = MODBUS_ReadInt16(SBS_ADDR, SBS_NCHANNELS_REG, &nchan);
        if (!success) { ESP_Send_Error(); first_measure = true; continue; }
        
        success = MODBUS_ReadSpectrum(SBS_ADDR, SBS_SP0_CHANNEL, nchan, spectr, 60);
        if (!success) { ESP_Send_Error(); first_measure = true; continue; }

        success = MODBUS_ReadFloat(SBS_ADDR, SBS_LTIME_REG, &ltime);
        if (!success || ltime <= 0.0f) { ESP_Send_Error(); first_measure = true; continue; }
        
        success = MODBUS_ReadFloat(SBS_ADDR, SBS_INPRATE_REG, &inprate);
        if (!success) { ESP_Send_Error(); first_measure = true; continue; }

        if (first_measure) {
            // Первое измерение – мгновенная доза
            ader = DoseRateInstant(spectr, nchan, ltime, inprate, DZ);
            ESP_SendFormatted("s,f,f,f", nchan, ader, ltime, inprate);
            memcpy(prev_spectr, spectr, nchan * sizeof(uint32_t));
            prev_ltime = ltime;
            first_measure = false;

            aderlen = float_num_len(ader, 2);
            OLED_setpos((128 - aderlen * 8) / 2, 4);
            OLED_printF(ader, 2, false);
        } else {
            float delta_ltime = ltime - prev_ltime;

            // Сброс набора (дельта отрицательна)
            if (delta_ltime < -TIME_EPS) {
                first_measure = true;
                continue;
            }

            // Остановка набора (время не растёт в пределах погрешности) - пропускаем расчет
            if (delta_ltime <= TIME_EPS) {
                ESP_Send_Stop();
                continue;   
            }

            // Слишком малый интервал дифференцирования – пропускаем расчет
            if (delta_ltime < MIN_DELTA) {
                continue;
            }

            // Дифференциальный расчёт
            uint32_t dose_diff = 0;
            for (uint16_t i = 0; i < nchan; i++) {
                uint32_t diff = spectr[i] - prev_spectr[i];
                dose_diff += diff * i;
            }
            ader = ((float)dose_diff * DZ) / (delta_ltime * 1.6f);
            ESP_SendFormatted("s,f,f,f", nchan, ader, ltime, inprate);
            memcpy(prev_spectr, spectr, nchan * sizeof(uint32_t));
            prev_ltime = ltime;

            aderlen = float_num_len(ader, 2);
            OLED_setpos((128 - aderlen * 8) / 2, 4);
            OLED_printF(ader, 2, false);
        }
    }
    return 0;
}

float DoseRateInstant(uint32_t spectr[], uint16_t nchannels, float ltime, float inprate, float Dz)
{
    uint32_t summ = 0;
    for(uint16_t i = 0; i < nchannels; i++) {
        summ += spectr[i] * i;
    }
    float dose_rate= ((float)summ * Dz) / (ltime * 1.6f);
    return dose_rate;
}