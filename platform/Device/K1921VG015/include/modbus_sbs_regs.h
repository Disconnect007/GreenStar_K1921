#ifndef MODBUS_SBS_REGS_H_
#define MODBUS_SBS_REGS_H_

#define SBS_ADDR 0x21

#define SBS_TEMP_REG 0x0510
#define SBS_NCHANNELS_REG 0x0902
#define SBS_REGRATE_REG 0x0907
#define SBS_INPRATE_REG 0x0909
#define SBS_LTIME_REG 0x090d
#define SBS_SP0_CHANNEL 0x1000

#define SBS_STATE_REG       0x0901   // регистр управления состоянием
#define SBS_STATE_CLEAR     0        // очистка спектра и сброс
#define SBS_STATE_STOP      1        // остановка набора
#define SBS_STATE_START     2        // запуск набора

#endif
