//-- Includes ----------------------------------------------------------------
#include <K1921VG015.h>
#include <system_k1921vg015.h>
#include "mtimer.h"
#include "i2c_tx.h"
#include "oled_small.h"
#include "uart_tx.h"
#include "modbus_sbs_regs.h"
#include "modbus_funcs.h"

//-- Defines ------------------------------------------------------------------
#define LEDS_MSK  0xF000
#define LED0_PIN  12
#define LED0_MSK  (1 << LED0_PIN)


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

//-- Main ----------------------------------------------------------------------
int main(void) 
{
	periph_init();
	check();
	float temp;
    bool success;
	uint16_t nchan;
	static uint32_t spectrum[4096];
	
	while(1) {
		/*
		success = MODBUS_ReadFloat(SBS_ADDR, SBS_TEMP_REG, &temp);
		mtimer_sleep(100);
		success = MODBUS_ReadUInt16(SBS_ADDR, SBS_NCHANNELS_REG, &nchan);
		mtimer_sleep(100);
		OLED_setpos(0, 0); 
		OLED_printF(temp, 2, false);
		OLED_setpos(0, 1); 
		OLED_printD((uint32_t)nchan, false);
		UART2_Send_Uint_Float_AsString(nchan, temp);
		check();
		OLED_clear();
		*/
		success = MODBUS_ReadUInt16(SBS_ADDR, SBS_NCHANNELS_REG, &nchan);
		mtimer_sleep(50);
		uint32_t spectrum[nchan];
		for (uint16_t i = 0; i < 4096; i++) {
            spectrum[i] = (uint32_t)-1;
        }
		success = MODBUS_ReadSpectrumU32(SBS_ADDR, SBS_SP0_CHANNEL, nchan, spectrum);
		if (success) {
            uint16_t filled = 0;
            for (uint16_t i = 0; i < nchan; i++) {
                if (spectrum[i] != (uint32_t)-1) {
                    filled++;
                }
            }
			OLED_setpos(0, 0);
			OLED_printS("Zapolneno kanalov", false);
            OLED_setpos(0, 1);
            OLED_printD((uint32_t)filled, false);
        } else {
            OLED_setpos(0, 0);
            OLED_printS("Read error", false);
        }
		check();
		OLED_clear();
	}

	return 0;
}