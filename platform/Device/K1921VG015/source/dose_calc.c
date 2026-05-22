#include "dose_calc.h"
#include "system_k1921vg015.h"
#include <string.h>

static const double ENK0[ENK_COUNT] = {-240.0, -120.0, -60.0, -30.0, -15.0, -7.5};
static const double ENK1[ENK_COUNT] = { 24.0,   12.0,   6.0,   3.0,   1.5,  0.75};

static uint32_t prev_spectr[4096];
static double   prev_ltime;
static bool     first_measure = true;
static int8_t   k_idx;

static float DoseRateInstant(const uint32_t *s, uint16_t n, float lt, double dz, uint8_t idx, uint64_t t_rec);
static float DoseRatediff(const uint32_t *s, const uint32_t *ps, uint16_t n,float dt, double dz, uint8_t idx);

void DoseCalc_Init(void)
{
    first_measure = true;
    memset(prev_spectr, 0, sizeof(prev_spectr));
    prev_ltime = 0.0;
}

bool DoseCalc_Perform(uint16_t nchan, const uint32_t *spectr, uint64_t sp_rec_time, float ltime, float *ader)
{
    switch (nchan) {
        case 128:  k_idx = 0; break;
        case 256:  k_idx = 1; break;
        case 512:  k_idx = 2; break;
        case 1024: k_idx = 3; break;
        case 2048: k_idx = 4; break;
        case 4096: k_idx = 5; break;
        default: return false;
    }

    if (ltime <= MIN_DELTA) {
        first_measure = true;
        return false;
    }

    float dose;
    if (first_measure) {
        dose = DoseRateInstant(spectr, nchan, ltime, DZ, k_idx, sp_rec_time);
        if (dose < 0.0f) return false;
        memcpy(prev_spectr, spectr, nchan * sizeof(uint32_t));
        prev_ltime = ltime;
        first_measure = false;
    } else {
        double delta = ltime - prev_ltime;
        if (delta < -TIME_EPS) {
            first_measure = true;
            return false;
        }
        if (delta <= TIME_EPS) {
            return false;
        }
        if (delta < MIN_DELTA) {
            return false;
        }
        dose = DoseRatediff(spectr, prev_spectr, nchan, (float)delta, DZ, k_idx);
        memcpy(prev_spectr, spectr, nchan * sizeof(uint32_t));
        prev_ltime = ltime;
    }
    *ader = dose;
    return true;
}

static float DoseRateInstant(const uint32_t *s, uint16_t n, float lt, double dz, uint8_t idx, uint64_t t_rec)
{
    double delta = (double)t_rec / (double)SystemCoreClock;
    double enk0 = ENK0[idx];
    double enk1 = ENK1[idx];
    double sum = 0.0;
    for (uint16_t i = 0; i < n; i++) {
        sum += (double)s[i] * (enk0 + enk1 * i);
    }
    double rate = (sum * dz) / (lt - delta);
    return (float)rate;
}

static float DoseRatediff(const uint32_t *s, const uint32_t *ps, uint16_t n, float dt, double dz, uint8_t idx)
{
    double enk0 = ENK0[idx];
    double enk1 = ENK1[idx];
    double sum = 0.0;
    for (uint16_t i = 0; i < n; i++) {
        int32_t diff = (int32_t)s[i] - (int32_t)ps[i];
        if (diff < 0) diff = 0;
        sum += (double)diff * (enk0 + enk1 * i);
    }
    double rate = (sum * dz) / dt;
    return (float)rate;
}

const char* GetRadStatusText(float ader) {
    if (ader < 0.6f)  return "НОРМА";      
    if (ader < 1.2f)  return "ВНИМА";   
    return "ОПАСН";                          
}