#include "modbus_crc.h"
#include "modbus_funcs.h"
#include "uart_tx.h"
#include "mtimer.h"

union {
    uint32_t u32;
    float f32;
} converter;

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

uint8_t modbus_read_holding_request(uint8_t slave_addr, uint16_t reg_addr, uint16_t num_registers, uint8_t *tx_buf)
{
	return modbus_request(slave_addr, 0x03, reg_addr, num_registers, tx_buf);
}

/**
 * @brief Функция чтения 1 или 2 регистров Modbus
 * @param data Буфер для данных (2 bytes для 1 регистра, 4 bytes для 2 регистров)
 */
bool MODBUS_ReadRegisters(uint8_t slave_addr, uint16_t reg_addr, uint8_t num_registers, uint8_t *data) 
{
    uint8_t tx_buffer[8];
    uint8_t rx_buffer[32];
    uint16_t rx_len;
    uint8_t expected_bytes;
    
    // Проверка параметров
    if (num_registers != 1 && num_registers != 2) {return false;}
    expected_bytes = num_registers * 2;
    
    // Формируем запрос
    uint8_t tx_len = modbus_read_holding_request(slave_addr, reg_addr, num_registers, tx_buffer);
    
    for (uint8_t retry = 0; retry < MODBUS_MAX_RETRIES; retry++) {
        // Очищаем буфер приема
        UART1_FlushRx();
        // Отправляем запрос
        UART1_SendBuffer(tx_buffer, tx_len);
        // Принимаем ответ с таймаутами
        rx_len = UART1_ReceiveBuffer(rx_buffer, sizeof(rx_buffer), MODBUS_TIMEOUT_MS, MODBUS_INTERCHAR_MS);

        // Быстрая проверка
        if (rx_len < 5 + expected_bytes) {continue;}
        if (rx_buffer[0] != slave_addr || rx_buffer[1] != 0x03) {continue;}
        if (rx_buffer[2] != expected_bytes) {continue;}

        // Проверяем CRC
        uint16_t received_crc = (rx_buffer[rx_len - 1] << 8) | rx_buffer[rx_len - 2];
        uint16_t calculated_crc = modbus_crc16(rx_buffer, rx_len - 2);
        if (received_crc != calculated_crc) {continue;}

        // Копируем данные (пропускаем 3 байта заголовка)
        for (uint8_t i = 0; i < expected_bytes; i++) {
            data[i] = rx_buffer[3 + i];
        }
        return true;
    }
    return false;
}

/**
 * @brief Чтение одного регистра unsigned int (2 bytes)
 */
bool MODBUS_ReadUInt16(uint8_t slave_addr, uint16_t reg_addr, uint16_t *value) 
{
    uint8_t data[2];
    if (!MODBUS_ReadRegisters(slave_addr, reg_addr, 1, data)) {
        return false;
    }
    *value = (data[0] << 8) | data[1];
    return true;
}

/**
 * @brief Чтение одного регистра signed int (2 bytes)
 */
bool MODBUS_ReadInt16(uint8_t slave_addr, uint16_t reg_addr, int16_t *value) 
{
    uint16_t temp;
    if (!MODBUS_ReadUInt16(slave_addr, reg_addr, &temp)) {
        return false;
    }
    *value = (int16_t)temp;
    return true;
}

/**
 * @brief Вспомогательная функция для преобразования 4-х байт в 32-битное значение
 * @note Формат: big-endian с word swap [A B C D] -> [C D A B]
 */
static uint32_t modbus_bytes_to_u32(const uint8_t bytes[4]) 
{
    return (bytes[2] << 24) | (bytes[3] << 16) | (bytes[0] << 8) | bytes[1];
}

/**
 * @brief Чтение двух регистров как (32 bytes unsigned int) число
 */
bool MODBUS_ReadUInt32(uint8_t slave_addr, uint16_t reg_addr, uint32_t *value) 
{
    uint8_t data[4];
    if (!MODBUS_ReadRegisters(slave_addr, reg_addr, 2, data)) {return false;}
    *value = modbus_bytes_to_u32(data);
    return true;
}

/**
 * @brief Чтение двух регистров как (32 bytes signed int) число
 */
bool MODBUS_ReadInt32(uint8_t slave_addr, uint16_t reg_addr, int32_t *value) 
{
    uint32_t temp;
    if (!MODBUS_ReadUInt32(slave_addr, reg_addr, &temp)) {return false;}
    *value = (int32_t)temp;
    return true;
}

/**
 * @brief Чтение двух регистров как (float) число
 */
bool MODBUS_ReadFloat(uint8_t slave_addr, uint16_t reg_addr, float *value) 
{
    uint8_t data[4];
    if (!MODBUS_ReadRegisters(slave_addr, reg_addr, 2, data)) {return false;}
    converter.u32 = modbus_bytes_to_u32(data);
    *value = converter.f32;
    return true;
}

/**
 * @brief Чтение блока регистров Modbus (1..120 байт на блок)
 */
bool MODBUS_ReadMultipleRegisters(uint8_t slave_addr, uint16_t reg_addr, uint16_t num_registers, uint8_t *data) 
{
    uint8_t tx_buffer[8];
    uint8_t rx_buffer[5 + MODBUS_MAX_BYTES_PER_READ + 2]; // Заголовок + данные + CRC
    uint16_t rx_len;
    uint16_t expected_bytes;
    
    if (num_registers < 1 || num_registers > MODBUS_MAX_REGISTERS_PER_READ) {return false;}
    expected_bytes = num_registers * 2;
    
    // Формируем запрос
    uint8_t tx_len = modbus_read_holding_request(slave_addr, reg_addr, num_registers, tx_buffer);
    
    for (uint8_t retry = 0; retry < MODBUS_MAX_RETRIES; retry++) {
        // Очищаем буфер приема
        UART1_FlushRx();
        
        // Отправляем запрос
        UART1_SendBuffer(tx_buffer, tx_len);
        
        // Принимаем ответ
        rx_len = UART1_ReceiveBuffer(rx_buffer, sizeof(rx_buffer), MODBUS_TIMEOUT_MS, MODBUS_INTERCHAR_MS);
        
        // Быстрая проверка
        if (rx_len < 5 + expected_bytes) {continue;}
        if (rx_buffer[0] != slave_addr || rx_buffer[1] != 0x03) {continue;}
        if (rx_buffer[2] != expected_bytes) {continue;}
        
        // Проверяем CRC
        uint16_t received_crc = (rx_buffer[rx_len - 1] << 8) | rx_buffer[rx_len - 2];
        uint16_t calculated_crc = modbus_crc16(rx_buffer, rx_len - 2);
        if (received_crc != calculated_crc) {continue;}
        
        // Копируем данные (пропускаем 3 байта заголовка)
        for (uint16_t i = 0; i < expected_bytes; i++) {
            data[i] = rx_buffer[3 + i];
        }
        return true;
    }
    return false;
}


/**
 * @brief Преобразование блока сырых данных в массив uint32_t
 */
static void convert_spectrum_block(uint8_t *src, uint32_t *dst, uint16_t count) 
{
    for (uint16_t i = 0; i < count; i++) {
        dst[i] = modbus_bytes_to_u32(&src[i * 4]);
    }
}

/**
 * @brief Чтение спектра в массив uint32_t
 */
bool MODBUS_ReadSpectrumU32(uint8_t slave_addr, uint16_t start_reg, uint16_t channels, uint32_t *spectrum) 
{
    // Проверка допустимого количества каналов
    if (channels == 0 || channels > 4096) {return false;}

    // Оптимальный размер блока: 60 каналов (120 регистров)
    return MODBUS_ReadSpectrumU32_Config(slave_addr, start_reg, channels, spectrum, 60);
}

/**
 * @brief Чтение спектра с настраиваемым размером блока
 */
bool MODBUS_ReadSpectrumU32_Config(uint8_t slave_addr, uint16_t start_reg, uint16_t channels, uint32_t *spectrum, uint16_t block_size_channels) 
{
    if (channels == 0 || block_size_channels == 0) {return false;}
    
    const uint8_t REGS_PER_CHANNEL = 2;
    uint16_t channels_remaining = channels;
    uint16_t current_reg = start_reg;
    uint32_t *spectrum_ptr = spectrum;
    
    // Корректируем размер блока, если он слишком большой
    if (block_size_channels * REGS_PER_CHANNEL > MODBUS_MAX_REGISTERS_PER_READ) {
        block_size_channels = MODBUS_MAX_REGISTERS_PER_READ / REGS_PER_CHANNEL;
    }
    
    while (channels_remaining > 0) {
        uint16_t channels_in_block = (channels_remaining > block_size_channels) ? block_size_channels : channels_remaining;
        uint16_t regs_in_block = channels_in_block * REGS_PER_CHANNEL;
        uint8_t raw_data[regs_in_block * 2];
        
        // Читаем блок
        if (!MODBUS_ReadMultipleRegisters(slave_addr, current_reg, regs_in_block, raw_data)) {return false;}
    
        // Преобразуем и копируем в выходной массив
        convert_spectrum_block(raw_data, spectrum_ptr, channels_in_block);
        spectrum_ptr += channels_in_block;
        current_reg += regs_in_block;
        channels_remaining -= channels_in_block;
        
        //пауза между запросами
        if (channels_remaining > 0) {mtimer_sleep(2);}
    }
    return true;
}