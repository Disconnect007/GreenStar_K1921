import struct
import os

LOG_SIZE = 200
ENTRY_SIZE = 24   # байт на запись
MTIME_FREQ_MHZ = 16  # частота mtimer 16 МГц

def parse_log_file(filename):
    with open(filename, 'rb') as f:
        data = f.read()
    
    expected_size = LOG_SIZE * ENTRY_SIZE
    if len(data) != expected_size:
        print(f"Предупреждение: размер файла {len(data)} байт, ожидалось {expected_size}")
    
    entries = []
    for i in range(LOG_SIZE):
        offset = i * ENTRY_SIZE
        # Распаковка: < = little-endian, Q = uint64_t, H = uint16_t
        # Читаем только первые 20 байт (8+8+2+2), остальные 4 байта pad игнорируем
        start_ticks, end_ticks, nchannels, reserved = struct.unpack('<QQHH', data[offset:offset+20])
        if nchannels == 0:
            break   # неинициализированная запись
        duration_ticks = end_ticks - start_ticks
        duration_us = duration_ticks / MTIME_FREQ_MHZ
        entries.append((nchannels, duration_us))
    
    return entries

def compute_stats(entries):
    stats = {}
    for nch, dur in entries:
        stats.setdefault(nch, []).append(dur)
    
    print("Результаты измерений времени цикла обработки")
    print(f"{'nchannels':<10} {'Среднее, мкс':<15} {'σ, мкс':<15} {'CV, %':<10}")
    print("-" * 55)
    for nch in sorted(stats.keys()):
        vals = stats[nch]
        avg = sum(vals) / len(vals)
        variance = sum((x - avg) ** 2 for x in vals) / len(vals)
        std = variance ** 0.5
        cv = (std / avg) * 100 if avg > 0 else 0
        print(f"{nch:<10} {avg:<15.2f} {std:<15.2f} {cv:<10.2f}")

if __name__ == "__main__":
    if not os.path.exists("log_data.bin"):
        print("Файл log_data.bin не найден")
    else:
        entries = parse_log_file("log_data.bin")
        print(f"Всего записей: {len(entries)}")
        compute_stats(entries)