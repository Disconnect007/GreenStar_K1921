#ifndef I2C_TX_H_
#define I2C_TX_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// I2C Parameters
#ifndef FSFreq
#define FSFreq           100000    // Стандартный режим (Гц)
#endif

#ifndef HSFreq  
#define HSFreq           400000    // Скоростной режим (Гц)
#endif

#define I2C_TIMEOUT_BUS_FREE_US   5000   
#define I2C_TIMEOUT_STATE_US       500   

// I2C Functions    
void I2C_init(void);            // Инициализация I2C 
void I2C_start(uint8_t addr);   // Начало передачи I2C, адрес должен содержать бит R/W
void I2C_write(uint8_t data);   // Передача байта
void I2C_stop(void);            // Остановка передачи

#ifdef __cplusplus
};
#endif
#endif 