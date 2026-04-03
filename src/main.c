#include <K1921VG015.h>
#include <system_k1921vg015.h>
#include "mtimer.h"
#include "i2c_tx.h"
#include "oled_small.h"
#include "uart_tx.h"
#include "modbus_sbs_regs.h"
#include "modbus_funcs.h"
#include "esp_comm.h"
#include "temp.h"
#include <string.h>

#define LEDS_MSK  0xF000
#define LED0_PIN  12
#define LED0_MSK  (1 << LED0_PIN)

#define TIME_EPS	  1.0E-5
#define MIN_DELTA     1.0E-2
#define DZ 		      1.0E-6

static double ENK0[6] = {-239.137, -118.336, -62.554, -35.514, -24.453, -14.380};
static double ENK1[6] = {24.549, 12.396, 6.198, 3.122, 1.559, 0.780};
static int8_t k_idx = 0;

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
    adcsar_init(TSENSOR_ISRC_INT);
}

void check() 
{
	mtimer_sleep(800);
	GPIOA->DATAOUTTGL = LED0_MSK;
	mtimer_sleep(200);
	GPIOA->DATAOUTTGL = LED0_MSK;
}

float DoseRateInstant(uint32_t spectr[], uint16_t nchannels, float ltime, double Dz, uint8_t idx);

int main(void) 
{
    periph_init();
    check();
    
    static double ader = 0.0;

    static float ltime = 0.0, 
                 inprate = 0.0;

    static bool success = false,
		 first_measure = false;
		 
    static uint32_t spectr[4096] = {};
    static uint32_t prev_spectr[4096] = {};
    static double prev_ltime = 0.0;
    static char aderlen = 0;

    int16_t nchan = 0;

    OLED_clear();
    OLED_setpos(40, 1);
    OLED_printS(" ADER ", true);
    OLED_setpos(52, 2);
    OLED_printS("N/D", false);
    OLED_setpos(36, 3);
    OLED_printS("[uSv/h]", false);
    OLED_setpos(0, 6);
    OLED_printS("TEMP:", true);
    OLED_setpos(96, 6);
    OLED_printS("[C]", false);

    while(1) {
        InterruptDisable();
        mtimer_sleep(1);

        success = MODBUS_ReadInt16(SBS_ADDR, SBS_NCHANNELS_REG, &nchan);
        if (!success) { ESP_Send_Error(); first_measure = true; continue; }
        
        success = MODBUS_ReadSpectrum(SBS_ADDR, SBS_SP0_CHANNEL, nchan, spectr, 60);
        if (!success) { ESP_Send_Error(); first_measure = true; continue; }

        success = MODBUS_ReadFloat(SBS_ADDR, SBS_LTIME_REG, &ltime);
        if (!success || ltime <= 0.0f) { ESP_Send_Error(); first_measure = true; continue; }
        
        success = MODBUS_ReadFloat(SBS_ADDR, SBS_INPRATE_REG, &inprate);
        if (!success) { ESP_Send_Error(); first_measure = true; continue; }

        switch(nchan) {
            case 128: 
                k_idx = 0;
                break;
            case 256: 
                k_idx = 1;
                break;
            case 512: 
                k_idx = 2;
                break;
            case 1024: 
                k_idx = 3;
                break;
            case 2048: 
                k_idx = 4;
                break;
            case 4096: 
                k_idx = 5;
                break;
            default:
                k_idx = -1;
        }

        if(k_idx == -1) continue;

        if(first_measure) {
            // Первое измерение – мгновенная доза
            ader = DoseRateInstant(spectr, nchan, ltime, DZ, k_idx);
            ESP_SendFormatted("s,f,f,f", nchan, ader, ltime, inprate);
            memcpy(prev_spectr, spectr, nchan * sizeof(uint32_t));
            prev_ltime = ltime;
            first_measure = false;

            aderlen = float_num_len(ader, 2);
            OLED_setpos((128 - aderlen * 8) / 2, 2);
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
            double enk0 = ENK0[k_idx];
            double enk1 = ENK1[k_idx];
            double sum_diff = 0.0;
            for (uint16_t i = 0; i < nchan; i++) {
                uint32_t diff = spectr[i] - prev_spectr[i];
                double E_kev = enk0 + enk1 * i;
                sum_diff += (double)diff * E_kev;
            }
            ader = (sum_diff * DZ) / delta_ltime;
            float ader_f = (float)ader;

            ESP_SendFormatted("s,f,f,f", nchan, ader_f, ltime, inprate);
            memcpy(prev_spectr, spectr, nchan * sizeof(uint32_t));
            prev_ltime = ltime;

            aderlen = float_num_len(ader_f, 2);
            OLED_setpos((128 - aderlen * 8) / 2, 2);
            OLED_printF(ader_f, 2, false);
        }
        OLED_setpos(48, 6);
        OLED_printF(Get_Temp_Celsius(), 2, false);
        InterruptEnable();
    }
    return 0;
}

float DoseRateInstant(uint32_t spectr[], uint16_t nchannels, float ltime, double Dz, uint8_t idx)
{
    double enk0 = ENK0[idx];
    double enk1 = ENK1[idx];
    double summ = 0.0;
    for (uint16_t i = 0; i < nchannels; i++) {
        double E_kev = enk0 + enk1 * i;
        summ += (double)spectr[i] * E_kev;
    }
    double dose_rate = (summ * Dz) / ltime;
    return (float)dose_rate;
}