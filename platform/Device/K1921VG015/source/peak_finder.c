#include "peak_finder.h"
#include <math.h>

int find_peaks_simple(const uint32_t *spectrum, uint16_t n_channels,
                      const double k0, const double k1,
                      float peak_threshold,
                      PeakInfo *peaks, uint8_t max_peaks)
{
    /* Проверка входных параметров */
    if (!spectrum || !peaks || max_peaks <= 0 || n_channels < 3 || n_channels > 4096)
        return 0;

    /* Статические буферы – выделяются один раз, занимают 3×4096×4 = 48 кБ */
    static float work_sm[4096];
    static float work_tmp[4096];
    static float work_bg[4096];

    /* Лёгкое сглаживание (окно 3 канала) -> work_sm */
    for (int i = 0; i < n_channels; i++) {
        float sum = 0.0f;
        int cnt = 0;
        int start = i - 1;
        if (start < 0) start = 0;
        int end = i + 1;
        if (end >= n_channels) end = n_channels - 1;
        for (int j = start; j <= end; j++) {
            sum += (float)spectrum[j];
            cnt++;
        }
        work_sm[i] = sum / (float)cnt;
    }

    /* Адаптивный размер окна для фона (2% от числа каналов) */
    int bg_half = (int)(n_channels * 0.02f);
    if (bg_half < 2) bg_half = 2;
    if (bg_half > 50) bg_half = 50;

    /* Оценка фона: эрозия -> work_tmp, дилатация -> work_bg */
    /* Эрозия (локальный минимум) */
    for (int i = 0; i < n_channels; i++) {
        float min_val = 3.40282347e+38f; /* ~FLT_MAX */
        int start = i - bg_half;
        if (start < 0) start = 0;
        int end = i + bg_half;
        if (end >= n_channels) end = n_channels - 1;
        for (int j = start; j <= end; j++) {
            if (work_sm[j] < min_val) min_val = work_sm[j];
        }
        work_tmp[i] = min_val;
    }
    /* Дилатация (локальный максимум) */
    for (int i = 0; i < n_channels; i++) {
        float max_val = -3.40282347e+38f; /* ~ -FLT_MAX */
        int start = i - bg_half;
        if (start < 0) start = 0;
        int end = i + bg_half;
        if (end >= n_channels) end = n_channels - 1;
        for (int j = start; j <= end; j++) {
            if (work_tmp[j] > max_val) max_val = work_tmp[j];
        }
        work_bg[i] = max_val;
    }

    /* Чистый сигнал (net) сохраняем в work_tmp */
    for (int i = 0; i < n_channels; i++) {
        float net = work_sm[i] - work_bg[i];
        work_tmp[i] = (net > 0.0f) ? net : 0.0f;
    }

    /* Поиск пиков и определение их параметров */
    int num_peaks = 0;
    int last_left_ch = 0, last_right_ch = -1;   /* границы последнего добавленного пика */

    for (int i = 1; i < n_channels - 1; i++) {
        float net_i = work_tmp[i];
        /* Локальный максимум в чистом сигнале? */
        if (net_i > 0.0f && net_i > work_tmp[i-1] && net_i > work_tmp[i+1]) {
            /* Статистическая значимость */
            float sigma = sqrtf(work_bg[i] > 0.0f ? work_bg[i] : 1.0f);
            if (net_i > peak_threshold * sigma) {
                /* Определяем границы пика по уровню 10% от высоты чистого сигнала */
                float threshold = 0.1f * net_i;
                int left = i;
                while (left > 0 && work_tmp[left-1] >= threshold) left--;
                int right = i;
                while (right < n_channels-1 && work_tmp[right+1] >= threshold) right++;

                /* Максимальный счёт в исходном спектре и соответствующий канал */
                uint32_t max_cnt = spectrum[left];
                int ch_max = left;
                for (int k = left + 1; k <= right; k++) {
                    if (spectrum[k] > max_cnt) {
                        max_cnt = spectrum[k];
                        ch_max = k;
                    }
                }

                /* Добавление в выходной массив с учётом возможного перекрытия */
                if (num_peaks == 0) {
                    peaks[0].energy_keV = k0 + k1 * ch_max;
                    peaks[0].max_count  = max_cnt;
                    last_left_ch  = left;
                    last_right_ch = right;
                    num_peaks = 1;
                } else {
                    if (left <= last_right_ch) {
                        /* Перекрытие с предыдущим пиком – расширяем его */
                        if (right > last_right_ch) last_right_ch = right;
                        if (max_cnt > peaks[num_peaks-1].max_count) {
                            peaks[num_peaks-1].energy_keV = k0 + k1 * ch_max;
                            peaks[num_peaks-1].max_count  = max_cnt;
                        }
                    } else {
                        if (num_peaks < max_peaks) {
                            peaks[num_peaks].energy_keV = k0 + k1 * ch_max;
                            peaks[num_peaks].max_count  = max_cnt;
                            last_left_ch  = left;
                            last_right_ch = right;
                            num_peaks++;
                        } else {
                            return num_peaks;
                        }
                    }
                }
            }
        }
    }

    return num_peaks;
}