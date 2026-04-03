#ifndef TEMP_H_
#define TEMP_H_

#include <K1921VG015.h>
#include <stdint.h>
#include <stdlib.h>
#include <system_k1921vg015.h>

#define TSENSOR_ISEL_INT_ENABLE  TSENS->CTRL = 0x12;
#define TSENSOR_DISABLE TSENS->CTRL = 0x01;
#define TSENSOR_ISEL_ADCSD_ENABLE  TSENS->CTRL = 0x02;
#define TSENSOR_ISRC_INT      0
#define TSENSOR_ISRC_ADCSD    1

#define AREF	3.000

#ifdef __cplusplus
extern "C" {
#endif

uint16_t Get_Temp_adc();
float Get_Temp_Celsius();
float Get_Temp_Celsius_ISEL_ADCSD();
void adcsar_init(uint8_t tsensor_isrc);

#ifdef __cplusplus
};
#endif
#endif