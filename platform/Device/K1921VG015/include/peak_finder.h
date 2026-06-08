#ifndef PEAK_FINDER_H
#define PEAK_FINDER_H

#include <stdint.h>

typedef struct {
    double energy_keV;   // Энергия пика (канал с максимальным счётом), кэВ
    uint32_t max_count;  // Максимальный счёт в пике (из исходного спектра) 
} PeakInfo;

int find_peaks_simple(const uint32_t *spectrum, uint16_t n_channels,
                      const double k0, const double k1,
                      float peak_threshold,
                      PeakInfo *peaks, uint8_t max_peaks);

#endif