#ifndef MODBUS_CRC_H_
#define MODBUS_CRC_H_
#include <stdint.h>

uint16_t modbus_crc16(uint8_t *data, uint16_t len);

#endif /* MODBUS_CRC_H_ */
