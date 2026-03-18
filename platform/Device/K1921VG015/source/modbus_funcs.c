#include "modbus_crc.h"
#include "modbus_funcs.h"
#include "uart_tx.h"
#include "mtimer.h"

union FloatConverter{
    uint32_t u32;
    float f32;
};

// Формирование Modbus запроса
uint8_t modbus_request(uint8_t addr, uint8_t code, uint16_t reg_addr, uint16_t reg_count, uint8_t *tx_buf)
{
    tx_buf[0] = addr;
    tx_buf[1] = code;
    tx_buf[2] = Hi(reg_addr);
    tx_buf[3] = Low(reg_addr);
    tx_buf[4] = Hi(reg_count);
    tx_buf[5] = Low(reg_count);

    uint16_t crc = modbus_crc16(tx_buf, 6);
    tx_buf[6] = Low(crc);
    tx_buf[7] = Hi(crc);

    return 8;
}

// Запрос на чтение holding регистров
uint8_t modbus_read_holding_request(uint8_t slave_addr, uint16_t reg_addr, uint16_t num_registers, uint8_t *tx_buf)
{
    return modbus_request(slave_addr, 0x03, reg_addr, num_registers, tx_buf);
}

// Чтение блока регистров (до 120 регистров)
bool MODBUS_ReadMultipleRegisters(uint8_t slave_addr, uint16_t reg_addr, uint16_t num_registers, uint8_t *data)
{
    uint8_t tx_buffer[8];
    uint8_t rx_buffer[5 + MODBUS_MAX_BYTES_PER_READ + 2];
    uint16_t rx_len;
    uint16_t expected_bytes;

    if (num_registers < 1 || num_registers > MODBUS_MAX_REGISTERS_PER_READ) return false;
    expected_bytes = num_registers * 2;

    uint8_t tx_len = modbus_read_holding_request(slave_addr, reg_addr, num_registers, tx_buffer);

    for (uint8_t retry = 0; retry < MODBUS_MAX_RETRIES; retry++) {
        UART1_FlushRx();
        UART1_SendBuffer(tx_buffer, tx_len);
        rx_len = UART1_ReceiveBuffer(rx_buffer, sizeof(rx_buffer), MODBUS_TIMEOUT_MS, MODBUS_INTERCHAR_MS);

        if (rx_len < 5 + expected_bytes) continue;
        if (rx_buffer[0] != slave_addr || rx_buffer[1] != 0x03) continue;
        if (rx_buffer[2] != expected_bytes) continue;

        uint16_t received_crc = (rx_buffer[rx_len - 1] << 8) | rx_buffer[rx_len - 2];
        uint16_t calculated_crc = modbus_crc16(rx_buffer, rx_len - 2);
        if (received_crc != calculated_crc) continue;

        for (uint16_t i = 0; i < expected_bytes; i++) {
            data[i] = rx_buffer[3 + i];
        }
        return true;
    }
    return false;
}

// Преобразование 4 bytes big-endian для little-endian
static uint32_t modbus_bytes_reorder(const uint8_t bytes[4])
{
    return (bytes[2] << 24) | (bytes[3] << 16) | (bytes[0] << 8) | bytes[1];
}

// Чтение short int (2 bytes)
bool MODBUS_ReadInt16(uint8_t slave_addr, uint16_t reg_addr, int16_t *value)
{
    uint8_t data[2];
    if (!MODBUS_ReadMultipleRegisters(slave_addr, reg_addr, 1, data)) return false;
    *value = (int16_t)((data[0] << 8) | data[1]);
    return true;
}

// Чтение int (4 bytes)
bool MODBUS_ReadInt32(uint8_t slave_addr, uint16_t reg_addr, int32_t *value)
{
    uint8_t data[4];
    if (!MODBUS_ReadMultipleRegisters(slave_addr, reg_addr, 2, data)) return false;
    uint32_t temp = modbus_bytes_reorder(data);
    *value = (int32_t)temp;
    return true;
}

// Чтение float (4 bytes)
bool MODBUS_ReadFloat(uint8_t slave_addr, uint16_t reg_addr, float *value)
{
    union FloatConverter conv;
    uint8_t data[4];
    if (!MODBUS_ReadMultipleRegisters(slave_addr, reg_addr, 2, data)) return false;
    conv.u32 = modbus_bytes_reorder(data);
    *value = conv.f32;
    return true;
}

// Преобразование 4 bytes big-endian для little-endian (для спектра)
static uint32_t spectrum_bytes_reorder(const uint8_t bytes[4])
{
    return (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | bytes[3];
}

// Преобразование блока сырых данных спектра в массив unsigned int (4 bytes)
static void convert_spectrum_block(uint8_t *src, uint32_t *dst, uint16_t count)
{
    for (uint16_t i = 0; i < count; i++) {
        dst[i] = spectrum_bytes_reorder(&src[i * 4]);
    }
}


// Чтение спектра с настраиваемым размером блока
bool MODBUS_ReadSpectrum(uint8_t slave_addr, uint16_t start_reg, uint16_t channels, uint32_t *spectrum, uint16_t block_size_channels)
{
    if (block_size_channels > 60 || channels > 4096) return false;

    const uint8_t REGS_PER_CHANNEL = 2;
    uint16_t channels_remaining = channels;
    uint16_t current_reg = start_reg;
    uint32_t *spectrum_ptr = spectrum;

    if (block_size_channels * REGS_PER_CHANNEL > MODBUS_MAX_REGISTERS_PER_READ) {
        block_size_channels = MODBUS_MAX_REGISTERS_PER_READ / REGS_PER_CHANNEL;
    }

    while (channels_remaining > 0) {
        uint16_t channels_in_block = (channels_remaining > block_size_channels) ? block_size_channels : channels_remaining;
        uint16_t regs_in_block = channels_in_block * REGS_PER_CHANNEL;
        uint8_t raw_data[regs_in_block * 2];

        if (!MODBUS_ReadMultipleRegisters(slave_addr, current_reg, regs_in_block, raw_data)) return false;

        convert_spectrum_block(raw_data, spectrum_ptr, channels_in_block);
        spectrum_ptr += channels_in_block;
        current_reg += regs_in_block;
        channels_remaining -= channels_in_block;

        if (channels_remaining > 0) mtimer_sleep(2);
    }
    return true;
}