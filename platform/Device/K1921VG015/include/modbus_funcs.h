#ifndef MODBUS_FUNCS_H_
#define MODBUS_FUNCS_H_

#include <stdint.h>
#include <stdbool.h>

#define MODBUS_TIMEOUT_MS 100       // Таймаут ожидания ответа
#define MODBUS_INTERCHAR_MS 2  // Макс. пауза между символами
#define MODBUS_MAX_RETRIES 5    // Попытки
#define MODBUS_MAX_REGISTERS_PER_READ 120
#define MODBUS_MAX_BYTES_PER_READ (MODBUS_MAX_REGISTERS_PER_READ * 2)

#define Hi(Int) (uint8_t) (Int>>8)
#define Low(Int) (uint8_t) (Int)

uint8_t modbus_request(uint8_t addr, uint8_t code, uint16_t reg_addr, uint16_t reg_count, uint8_t *tx_buf);
uint8_t modbus_read_holding_request(uint8_t addr, uint16_t reg_addr, uint16_t reg_count, uint8_t *tx_buf);
bool MODBUS_ReadRegisters(uint8_t slave_addr, uint16_t reg_addr, uint8_t num_registers, uint8_t *data);
bool MODBUS_ReadUInt16(uint8_t slave_addr, uint16_t reg_addr, uint16_t *value);
bool MODBUS_ReadInt16(uint8_t slave_addr, uint16_t reg_addr, int16_t *value);
bool MODBUS_ReadUInt32(uint8_t slave_addr, uint16_t reg_addr, uint32_t *value);
bool MODBUS_ReadInt32(uint8_t slave_addr, uint16_t reg_addr, int32_t *value);
bool MODBUS_ReadFloat(uint8_t slave_addr, uint16_t reg_addr, float *value);
bool MODBUS_ReadMultipleRegisters(uint8_t slave_addr, uint16_t reg_addr, uint16_t num_registers, uint8_t *data);
bool MODBUS_ReadSpectrumU32(uint8_t slave_addr, uint16_t start_reg, uint16_t channels, uint32_t *spectrum);
bool MODBUS_ReadSpectrumU32_Config(uint8_t slave_addr, uint16_t start_reg, uint16_t channels, uint32_t *spectrum, uint16_t block_size_channels);

#endif /* MODBUS_FUNCS_H_ */
