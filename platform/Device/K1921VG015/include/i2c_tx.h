#ifndef I2C_TX_H_
#define I2C_TX_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// I2C Parameters
#ifndef FSFreq
#define FSFreq           100000    // Standard frequency (Hz)
#endif

#ifndef HSFreq  
#define HSFreq           400000    // Fast Mode frequency (Hz)
#endif

// I2C Functions    
void I2C_init(void);            // I2C initialization function
void I2C_start(uint8_t addr);   // I2C start transmission, addr must contain R/W bit
void I2C_write(uint8_t data);   // I2C transmit one data byte via I2C
void I2C_stop(void);            // I2C stop transmission

#ifdef __cplusplus
};
#endif
#endif 