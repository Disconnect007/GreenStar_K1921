#include "power_mgmt.h"
#include <K1921VG015.h>
#include <system_k1921vg015.h>

// Внутренние вспомогательные функции
static void PM_WriteWFIEntry(uint32_t ldo0_en, uint32_t ldo1_en,
                             uint32_t ldo0_lp, uint32_t ldo1_lp,
                             LdoLevel_t vh, LdoLevel_t vl) {
    uint32_t reg_val = PMURTC->WFI_ENTR; // сохраняем биты, не относящиеся к LDO
    reg_val &= ~(PMURTC_WFI_ENTR_LDO0EN_Msk | PMURTC_WFI_ENTR_LDO1EN_Msk |
                 PMURTC_WFI_ENTR_LDO0LP_Msk | PMURTC_WFI_ENTR_LDO1LP_Msk |
                 PMURTC_WFI_ENTR_VH_Msk | PMURTC_WFI_ENTR_VL_Msk);
    
    reg_val |= (ldo0_en ? PMURTC_WFI_ENTR_LDO0EN_Msk : 0) |
               (ldo1_en ? PMURTC_WFI_ENTR_LDO1EN_Msk : 0) |
               (ldo0_lp ? PMURTC_WFI_ENTR_LDO0LP_Msk : 0) |
               (ldo1_lp ? PMURTC_WFI_ENTR_LDO1LP_Msk : 0) |
               ((uint32_t)vh << PMURTC_WFI_ENTR_VH_Pos) |
               ((uint32_t)vl << PMURTC_WFI_ENTR_VL_Pos);
    
    PMURTC->WFI_ENTR = reg_val;
}

static void PM_WriteWFIExit(uint32_t ldo0_en, uint32_t ldo1_en,
                            uint32_t ldo0_lp, uint32_t ldo1_lp) {
    PMURTC->WFI_EXIT = (ldo0_en ? PMURTC_WFI_EXIT_LDO0EN_Msk : 0) |
                       (ldo1_en ? PMURTC_WFI_EXIT_LDO1EN_Msk : 0) |
                       (ldo0_lp ? PMURTC_WFI_EXIT_LDO0LP_Msk : 0) |
                       (ldo1_lp ? PMURTC_WFI_EXIT_LDO1LP_Msk : 0) |
                       PMURTC_WFI_EXIT_ALR_Msk; // всегда сбрасываем флаг ALARM
}

// Отключение всей периферии (кроме необходимой для пробуждения)
void PM_DisableAllPeriph(void) {
    // Отключаем тактирование и сбрасываем всю периферию на AHB и APB
    RCU->CGCFGAHB = 0x0;
    RCU->CGCFGAPB = 0x0;
    RCU->RSTDISAHB = 0x0;
    RCU->RSTDISAPB = 0x0;
    
    // Отключаем блоки батарейного домена
    PMURTC->PMU_VBATPER_FORCE = PMURTC_PMU_VBATPER_FORCE_CMP0PD_Msk |
                                PMURTC_PMU_VBATPER_FORCE_CMP1PD_Msk |
                                PMURTC_PMU_VBATPER_FORCE_DACPD_Msk;
    // HSI оставляем включенным (может понадобиться для пробуждения)
    // HSE отключаем отдельно, если не используется
}

// Переключение системной частоты на HSE
void PM_SwitchToHSE(void) {
    if (RCU->CLKSTAT_bit.SRC == RCU_SYSCLKCFG_SRC_HSECLK) return;
    RCU->SYSCLKCFG = RCU_SYSCLKCFG_SRC_HSECLK << RCU_SYSCLKCFG_SRC_Pos;
    uint32_t timeout = 10000;
    while ((RCU->CLKSTAT_bit.SRC != RCU_SYSCLKCFG_SRC_HSECLK) && timeout--) {}
    
    // Отключаем PLL
    RCU->PLLSYSCFG0_bit.PLLEN = 0;
    PMUSYS->PDENFORCE_bit.PLLEN = 1; // принудительно в PowerDown
}

// Универсальная функция входа в заданный режим
void PM_EnterMode(PowerMode_t mode) {
    // 1. Убедимся, что системная частота не от PLL (для режимов с выключением LDO1)
    if (mode >= PM_STOP) {
        PM_SwitchToHSE();
    }
    
    // 2. Разрешаем управление питанием по WFI
    PMURTC->WFI_PDEN_bit.EN = 1;
    
    // 3. Настраиваем WFI_ENTR и WFI_EXIT в зависимости от режима
    switch (mode) {
        case PM_RUN:
            // Ничего не делаем, просто возврат
            return;
            
        case PM_IDLE:
            PM_WriteWFIEntry(1, 1, 0, 0, 0, 0);
            PM_WriteWFIExit(1, 1, 0, 0);
            break;
            
        case PM_LP_IDLE:
            PM_WriteWFIEntry(1, 1, 1, 1, LDO_LEVEL_1V30, LDO_LEVEL_1V24);
            PM_WriteWFIExit(1, 1, 0, 0);
            break;
            
        case PM_STOP:
            PM_WriteWFIEntry(1, 0, 0, 0, 0, 0);
            PM_WriteWFIExit(1, 1, 0, 0);
            break;
            
        case PM_LP_STOP:
            PM_WriteWFIEntry(1, 0, 1, 0, LDO_LEVEL_1V30, LDO_LEVEL_1V24);
            PM_WriteWFIExit(1, 1, 0, 0);
            break;
            
        case PM_POWEROFF:
            PM_WriteWFIEntry(0, 0, 0, 0, 0, 0);
            // WFI_EXIT не настраивается, так как пробуждение будет через сброс
            break;
    }
    
    // 4. Выполняем инструкцию WFI
    __asm volatile ("WFI");
    __asm volatile ("NOP");
    __asm volatile ("NOP");
    __asm volatile ("NOP");
    
    // После пробуждения из IDLE/STOP выполнение продолжится здесь.
    // Для POWEROFF сюда не попадём (будет сброс).
}

// Настройка пробуждения по таймеру TMR32 (период в миллисекундах)
void PM_SetupWakeupTimer(uint32_t period_ms) {
    // Включаем тактирование TMR32
    RCU->CGCFGAPB_bit.TMR32EN = 1;
    RCU->RSTDISAPB_bit.TMR32EN = 1;
    
    uint32_t ticks = (SystemCoreClock / 1000) * period_ms;
    TMR32->CAPCOM[0].VAL = ticks - 1;
    TMR32->CTRL_bit.MODE = 1; // счёт вверх
    TMR32->IM = 2; // прерывание по совпадению CAPCOM0
    TMR32->CTRL_bit.CLR = 1; // сброс счётчика
}

// Настройка пробуждения по внешнему выводу AKEUPx
void PM_SetupWakeupPin(uint8_t pin_num, uint8_t polarity) {
    // pin_num: 0,1,2
    if (pin_num > 2) return;
    // Устанавливаем полярность (0 - высокий уровень активен, 1 - низкий)
    if (polarity) {
        PMURTC->RTC_WAKECFG |= (1 << (6 + pin_num)); // WAKEPOL
    } else {
        PMURTC->RTC_WAKECFG &= ~(1 << (6 + pin_num));
    }
    // Разрешаем событие пробуждения
    PMURTC->RTC_WAKECFG |= (1 << pin_num); // WAKEEN
}