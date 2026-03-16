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
#define DZ 		  (float) 0.000001

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
	//OLED_init();
}

//--- USER FUNCTIONS -----------------------------------------------------------
void check() 
{
	mtimer_sleep(800);
	GPIOA->DATAOUTTGL = LED0_MSK;
	mtimer_sleep(200);
	GPIOA->DATAOUTTGL = LED0_MSK;
}

float DoseRate(uint32_t spectr[], uint16_t nchannels, float ltime, float Dz)
{
	float dose;
	for(uint16_t i = 0; i < nchannels; i++) {
		dose += spectr[i] * i;
	}
	dose = dose * Dz / ltime;
	return dose;
}

//-- Main ----------------------------------------------------------------------
int main(void) 
{
	periph_init();
	check();
	float maed, ltime = 0.0;
    bool success;
	uint16_t nchan;
	static uint32_t spectr[4096] = {};

	while(1) {
		mtimer_sleep(50);
		success = MODBUS_ReadUInt16(SBS_ADDR, SBS_NCHANNELS_REG, &nchan);
		mtimer_sleep(50);
		if (success) {
			success = MODBUS_ReadSpectrumU32(SBS_ADDR, SBS_SP0_CHANNEL, nchan, spectr);
			mtimer_sleep(50);
			if (success) {
				success = MODBUS_ReadFloat(SBS_ADDR, SBS_LTIME_REG, &ltime);
				mtimer_sleep(50);
				if (success && ltime > 0.0) {
					maed = DoseRate(spectr, nchan, ltime, DZ);
					UART2_Send_Data(nchan, maed);
					mtimer_sleep(50);
				} else {
					UART2_Send_Error();
				}	
			} else {
				UART2_Send_Error();
			}	
		} else {
			UART2_Send_Error();
		}
	}
	
	return 0;
}