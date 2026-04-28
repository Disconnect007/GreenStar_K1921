#ifndef POWER_MGMT_H
#define POWER_MGMT_H

#ifdef __cplusplus
extern "C" {
#endif

// Идентификаторы режимов
typedef enum {
    PM_RUN = 0,
    PM_IDLE,
    PM_LP_IDLE,
    PM_STOP,
    PM_LP_STOP,
    PM_POWEROFF
} PowerMode_t;

// Уровни напряжения LDO (значения для регистров VL/VH)
typedef enum {
    LDO_LEVEL_0V80 = 0x00,
    LDO_LEVEL_0V90 = 0x05,
    LDO_LEVEL_1V00 = 0x0A,
    LDO_LEVEL_1V10 = 0x0F,
    LDO_LEVEL_1V20 = 0x14,
    LDO_LEVEL_1V24 = 0x16,
    LDO_LEVEL_1V30 = 0x19
} LdoLevel_t;

void PM_DisableAllPeriph(void);
void PM_SwitchToHSE(void);
void PM_EnterMode(PowerMode_t mode);
void PM_SetupWakeupTimer(uint32_t period_ms); // пробуждение по TMR32
void PM_SetupWakeupPin(uint8_t pin_num, uint8_t polarity); // пробуждение по WAKEUP


#ifdef __cplusplus
};
#endif
#endif 