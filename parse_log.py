import struct
import sys

# Параметры
LOG_SIZE = 600          # общее число записей
ENTRY_SIZE = 24         # байт на запись (выравнивание)
SYSCLK_MHZ = 16.0       # частота в МГц (50 для PLL)

def parse_log(filename):
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
        duration_us = (end - start) / SYSCLK_MHZ
        entries.append((nch, duration_us))
    return entries

def compute_stats(entries):
    # Группировка по nchannels
    stats = {}
    for nch, dur in entries:
        stats.setdefault(nch, []).append(dur)
    
    print(f"{'nchannels':<10} {'Среднее (мкс)':<18} {'σ (мкс)':<15} {'CV (%)':<10}")
    print('-' * 55)
    for nch in sorted(stats.keys()):
        vals = stats[nch]
        avg = sum(vals) / len(vals)
        variance = sum((x - avg)**2 for x in vals) / len(vals)
        std = variance**0.5
        cv = (std / avg) * 100 if avg > 0 else 0
        print(f"{nch:<10} {avg:<18.2f} {std:<15.2f} {cv:<10.2f}")

if __name__ == '__main__':
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = 'log16MHz.bin'
    entries = parse_log(filename)
    print(f"Всего записей: {len(entries)}")
    compute_stats(entries)