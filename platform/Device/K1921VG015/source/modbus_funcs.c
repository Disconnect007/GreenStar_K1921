#include "modbus_crc.h"
#include "modbus_funcs.h"
#include "uart_tx.h"
#include "mtimer.h"

static uint16_t append_crc(uint8_t *buf, uint8_t len)
{
    uint16_t crc = modbus_crc16(buf, len);
    buf[len]   = Low(crc);
    buf[len+1] = Hi(crc);
    return len + 2;
}

static uint8_t build_read_request(uint8_t addr, uint16_t reg, uint16_t count, uint8_t *out)
{
    out[0] = addr;
    out[1] = 0x03;
    out[2] = Hi(reg);
    out[3] = Low(reg);
    out[4] = Hi(count);
    out[5] = Low(count);
    return append_crc(out, 6);
}

static uint8_t build_write_multiple_request(uint8_t addr, uint16_t reg, uint16_t count, const uint8_t *data, uint8_t *out)
{
    uint8_t byte_count = count * 2;
    out[0] = addr;
    out[1] = 0x10;
    out[2] = Hi(reg);
    out[3] = Low(reg);
    out[4] = Hi(count);
    out[5] = Low(count);
    out[6] = byte_count;
    for (uint8_t i = 0; i < byte_count; i++)
        out[7 + i] = data[i];
    return append_crc(out, 7 + byte_count);
}

static bool validate_response(const uint8_t *resp, uint8_t len, uint8_t addr, uint8_t func, uint16_t min_len)
{
    if (len < min_len) return false;
    if (resp[0] != addr || resp[1] != func) return false;
    uint16_t crc_rx = (resp[len-1] << 8) | resp[len-2];
    uint16_t crc_calc = modbus_crc16(resp, len - 2);
    return crc_rx == crc_calc;
}

static bool send_and_receive(const uint8_t *tx, uint8_t tx_len, uint8_t *rx, uint8_t *rx_len,
                             uint8_t max_rx, uint8_t addr, uint8_t func, uint16_t min_resp_len)
{
    for (uint8_t retry = 0; retry < MODBUS_MAX_RETRIES; retry++) {
        UART1_FlushRx();
        UART1_SendBuffer((uint8_t*)tx, tx_len);
        *rx_len = UART1_ReceiveBuffer(rx, max_rx, MODBUS_TIMEOUT_MS, MODBUS_INTERCHAR_MS);
        if (validate_response(rx, *rx_len, addr, func, min_resp_len))
            return true;
    }
    return false;
}

// Преобразование из буфера (word swap, big-endian) в uint32_t
static uint32_t bytes_to_uint32_word_swap(const uint8_t *bytes)
{
    return ((uint32_t)bytes[2] << 24) | ((uint32_t)bytes[3] << 16) |
           ((uint32_t)bytes[0] << 8)  | (uint32_t)bytes[1];
}

// Преобразование uint32_t в буфер (word swap, big-endian)
static void uint32_to_bytes_word_swap(uint32_t val, uint8_t *out)
{
    out[0] = (val >> 8) & 0xFF;   
    out[1] = val & 0xFF;           
    out[2] = (val >> 24) & 0xFF;   
    out[3] = (val >> 16) & 0xFF;   
}

static void int16_to_bytes_be(int16_t val, uint8_t *out)
{
    uint16_t uval = (uint16_t)val;
    out[0] = Hi(uval);
    out[1] = Low(uval);
}

static bool MODBUS_ReadMultipleRegisters(uint8_t slave_addr, uint16_t reg_addr, uint16_t num_registers, uint8_t *data)
{
    if (num_registers < 1 || num_registers > MODBUS_MAX_REGISTERS_PER_READ)
        return false;

    uint8_t tx[8];
    uint8_t tx_len = build_read_request(slave_addr, reg_addr, num_registers, tx);

    uint16_t expected_bytes = num_registers * 2;
    uint16_t min_resp_len = 5 + expected_bytes;
    uint8_t rx[5 + MODBUS_MAX_BYTES_PER_READ + 2];
    uint8_t rx_len;

    if (!send_and_receive(tx, tx_len, rx, &rx_len, sizeof(rx), slave_addr, 0x03, min_resp_len))
        return false;

    if (rx[2] != expected_bytes)
        return false;

    for (uint16_t i = 0; i < expected_bytes; i++)
        data[i] = rx[3 + i];
    return true;
}

static bool MODBUS_WriteMultipleRegisters(uint8_t slave_addr, uint16_t reg_addr, uint16_t num_registers, const uint8_t *data)
{
    if (num_registers < 1 || num_registers > MODBUS_MAX_REGISTERS_PER_WRITE)
        return false;

    uint8_t tx[9 + MODBUS_MAX_REGISTERS_PER_WRITE * 2];
    uint8_t tx_len = build_write_multiple_request(slave_addr, reg_addr, num_registers, data, tx);

    uint8_t rx[8];
    uint8_t rx_len;

    if (!send_and_receive(tx, tx_len, rx, &rx_len, sizeof(rx), slave_addr, 0x10, 8))
        return false;

    return (rx[2] == Hi(reg_addr) && rx[3] == Low(reg_addr) &&
            rx[4] == Hi(num_registers) && rx[5] == Low(num_registers));
}

/* ============================ Чтение ============================ */

bool MODBUS_ReadInt16(uint8_t slave_addr, uint16_t reg_addr, int16_t *value)
{
    uint8_t data[2];
    if (!MODBUS_ReadMultipleRegisters(slave_addr, reg_addr, 1, data))
        return false;
    *value = (int16_t)((data[0] << 8) | data[1]);
    return true;
}

bool MODBUS_ReadInt32(uint8_t slave_addr, uint16_t reg_addr, int32_t *value)
{
    uint8_t data[4];
    if (!MODBUS_ReadMultipleRegisters(slave_addr, reg_addr, 2, data))
        return false;
    *value = (int32_t)bytes_to_uint32_word_swap(data);
    return true;
}

bool MODBUS_ReadFloat(uint8_t slave_addr, uint16_t reg_addr, float *value)
{
    uint8_t data[4];
    if (!MODBUS_ReadMultipleRegisters(slave_addr, reg_addr, 2, data))
        return false;
    union FloatConverter conv = { .u32 = bytes_to_uint32_word_swap(data) };
    *value = conv.f32;
    return true;
}

/* ============================ Запись ============================ */

bool MODBUS_WriteInt16(uint8_t slave_addr, uint16_t reg_addr, int16_t value)
{
    uint8_t data[2];
    int16_to_bytes_be(value, data);
    return MODBUS_WriteMultipleRegisters(slave_addr, reg_addr, 1, data);
}

bool MODBUS_WriteInt32(uint8_t slave_addr, uint16_t reg_addr, int32_t value)
{
    uint8_t data[4];
    uint32_to_bytes_word_swap((uint32_t)value, data);
    return MODBUS_WriteMultipleRegisters(slave_addr, reg_addr, 2, data);
}

bool MODBUS_WriteFloat(uint8_t slave_addr, uint16_t reg_addr, float value)
{
    union FloatConverter conv = { .f32 = value };
    uint8_t data[4];
    uint32_to_bytes_word_swap(conv.u32, data);
    return MODBUS_WriteMultipleRegisters(slave_addr, reg_addr, 2, data);
}

/* ============================ Функции для работы со спектром ============================ */

static uint32_t spectrum_bytes_reorder(const uint8_t bytes[4])
{
    return ((uint32_t)bytes[0] << 24) | ((uint32_t)bytes[1] << 16) |
           ((uint32_t)bytes[2] << 8)  | (uint32_t)bytes[3];
}

static void convert_spectrum_block(uint8_t *src, uint32_t *dst, uint16_t count)
{
    for (uint16_t i = 0; i < count; i++)
        dst[i] = spectrum_bytes_reorder(&src[i * 4]);
}

bool MODBUS_ReadSpectrum(uint8_t slave_addr, uint16_t start_reg, uint16_t channels, uint32_t *spectrum, uint16_t block_size_channels)
{
    if (block_size_channels > 60 || channels > 4096) return false;

    const uint8_t REGS_PER_CHANNEL = 2;
    uint16_t channels_remaining = channels;
    uint16_t current_reg = start_reg;
    uint32_t *spectrum_ptr = spectrum;

    if (block_size_channels * REGS_PER_CHANNEL > MODBUS_MAX_REGISTERS_PER_READ)
        block_size_channels = MODBUS_MAX_REGISTERS_PER_READ / REGS_PER_CHANNEL;

    while (channels_remaining > 0) {
        uint16_t channels_in_block = (channels_remaining > block_size_channels) ? block_size_channels : channels_remaining;
        uint16_t regs_in_block = channels_in_block * REGS_PER_CHANNEL;
        uint8_t raw_data[regs_in_block * 2];

        if (!MODBUS_ReadMultipleRegisters(slave_addr, current_reg, regs_in_block, raw_data))
            return false;

        convert_spectrum_block(raw_data, spectrum_ptr, channels_in_block);
        spectrum_ptr += channels_in_block;
        current_reg += regs_in_block;
        channels_remaining -= channels_in_block;

        if (channels_remaining > 0)
            mtimer_sleep(1);
    }
    return true;
}