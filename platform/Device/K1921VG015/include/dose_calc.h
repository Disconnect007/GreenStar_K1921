#ifndef DOSE_CALC_H
#define DOSE_CALC_H

#include <stdint.h>
#include <stdbool.h>

#define TIME_EPS    1e-5
#define MIN_DELTA   0.5
#define DZ          1e-6

#define ENK_COUNT 6

void DoseCalc_Init(void);
bool DoseCalc_Perform(uint16_t nchan, const uint32_t *spectr, uint64_t sp_rec_time, float ltime, float *ader);
const char* GetRadStatusText(float ader);

#endif