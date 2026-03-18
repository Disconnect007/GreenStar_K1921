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

//-- Defines ------------------------------------------------------------------
#define LEDS_MSK  0xF000
#define LED0_PIN  12
#define LED0_MSK  (1 << LED0_PIN)
#define DZ 		  1.0E-6f

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

float DoseRate(uint32_t spectr[], uint16_t nchannels, float ltime, float inprate, float Dz);

//-- Main ----------------------------------------------------------------------
int main(void) 
{
	periph_init();
	check();
	float adr = 0.0f, 
		  ltime = 0.0f, 
		  inprate = 0.0f;
    bool success = false;
	int16_t nchan = 0;
	static uint32_t spectr[4096] = {};

	while(1) {
		mtimer_sleep(2);
		success = MODBUS_ReadInt16(SBS_ADDR, SBS_NCHANNELS_REG, &nchan);
		if (success) {
			success = MODBUS_ReadSpectrum(SBS_ADDR, SBS_SP0_CHANNEL, nchan, spectr, 60);
			if (success) {
				success = MODBUS_ReadFloat(SBS_ADDR, SBS_LTIME_REG, &ltime);
				if (success && ltime > 0.0f) {
					success = MODBUS_ReadFloat(SBS_ADDR, SBS_INPRATE_REG, &inprate);
					if (success) {
						adr = DoseRate(spectr, nchan, ltime, inprate, DZ);
						ESP_SendFormatted("s,f,f", nchan, adr, inprate);
					} else {
						ESP_Send_Error();
					}
				} else {
					ESP_Send_Error();
				}	
			} else {
				ESP_Send_Error();
			}	
		} else {
			ESP_Send_Error();
		}
	}
	
	return 0;
}

float DoseRate(uint32_t spectr[], uint16_t nchannels, float ltime, float inprate, float Dz)
{
	float dose = 0.0f;
	for(uint16_t i = 0; i < nchannels; i++) {
		dose += (float)spectr[i] * i;
	}
	dose = dose * inprate * Dz / (ltime * 640.0f);
	return dose;
}