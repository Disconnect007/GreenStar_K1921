import struct
import sys

# Параметры по умолчанию
LOG_SIZE = 600          # общее число записей
ENTRY_SIZE = 24         # байт на запись (выравнивание)
DEFAULT_FREQ_MHZ = 16.0 # частота по умолчанию (16 МГц)

def parse_log(filename, sysclk_mhz):
    with open(filename, 'rb') as f:
        data = f.read()
    expected = LOG_SIZE * ENTRY_SIZE
    if len(data) != expected:
        print(f"Предупреждение: размер файла {len(data)} байт, ожидалось {expected}")
    entries = []
    for i in range(LOG_SIZE):
        offset = i * ENTRY_SIZE
        # Распаковка: little-endian, два uint64_t, два uint16_t (последний reserved)
        start, end, nch, _ = struct.unpack('<QQHH', data[offset:offset+20])
        if nch == 0:
            break   # если буфер заполнен не полностью
        duration_us = (end - start) / sysclk_mhz
        entries.append((nch, duration_us))
    return entries

def compute_stats(entries):
    # Группировка по nchannels
    stats = {}
    for nch, dur in entries:
        stats.setdefault(nch, []).append(dur)
    
    # Заголовок таблицы
    print(f"{'nch':<6} {'Ср. (мкс)':<12} {'σ (мкс)':<10} {'CV (%)':<8} "
          f"{'Min (мкс)':<10} {'Max (мкс)':<10} {'Размах (мкс)':<12} {'Размах/ср (%)':<12}")
    print('-' * 85)
    
    for nch in sorted(stats.keys()):
        vals = stats[nch]
        avg = sum(vals) / len(vals)
        variance = sum((x - avg)**2 for x in vals) / len(vals)
        std = variance**0.5
        cv = (std / avg) * 100 if avg > 0 else 0
        vmin = min(vals)
        vmax = max(vals)
        span = vmax - vmin
        span_rel = (span / avg) * 100 if avg > 0 else 0
        print(f"{nch:<6} {avg:<12.2f} {std:<10.2f} {cv:<8.2f} "
              f"{vmin:<10.2f} {vmax:<10.2f} {span:<12.2f} {span_rel:<12.2f}")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Использование: python parse_log.py <файл.bin> [частота_МГц]")
        sys.exit(1)
    filename = sys.argv[1]
    freq = float(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_FREQ_MHZ
    entries = parse_log(filename, freq)
    print(f"Всего записей: {len(entries)}")
    compute_stats(entries)