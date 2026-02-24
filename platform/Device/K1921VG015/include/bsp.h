/*==============================================================================
 * Определения для периферии платы NIIET-DEV-K1921VG015 (КФДЛ.441461.029)
 *------------------------------------------------------------------------------
 * НИИЭТ, Александр Дыхно <dykhno@niiet.ru>
 *==============================================================================
 * ДАННОЕ ПРОГРАММНОЕ ОБЕСПЕЧЕНИЕ ПРЕДОСТАВЛЯЕТСЯ «КАК ЕСТЬ», БЕЗ КАКИХ-ЛИБО
 * ГАРАНТИЙ, ЯВНО ВЫРАЖЕННЫХ ИЛИ ПОДРАЗУМЕВАЕМЫХ, ВКЛЮЧАЯ ГАРАНТИИ ТОВАРНОЙ
 * ПРИГОДНОСТИ, СООТВЕТСТВИЯ ПО ЕГО КОНКРЕТНОМУ НАЗНАЧЕНИЮ И ОТСУТСТВИЯ
 * НАРУШЕНИЙ, НО НЕ ОГРАНИЧИВАЯСЬ ИМИ. ДАННОЕ ПРОГРАММНОЕ ОБЕСПЕЧЕНИЕ
 * ПРЕДНАЗНАЧЕНО ДЛЯ ОЗНАКОМИТЕЛЬНЫХ ЦЕЛЕЙ И НАПРАВЛЕНО ТОЛЬКО НА
 * ПРЕДОСТАВЛЕНИЕ ДОПОЛНИТЕЛЬНОЙ ИНФОРМАЦИИ О ПРОДУКТЕ, С ЦЕЛЬЮ СОХРАНИТЬ ВРЕМЯ
 * ПОТРЕБИТЕЛЮ. НИ В КАКОМ СЛУЧАЕ АВТОРЫ ИЛИ ПРАВООБЛАДАТЕЛИ НЕ НЕСУТ
 * ОТВЕТСТВЕННОСТИ ПО КАКИМ-ЛИБО ИСКАМ, ЗА ПРЯМОЙ ИЛИ КОСВЕННЫЙ УЩЕРБ, ИЛИ
 * ПО ИНЫМ ТРЕБОВАНИЯМ, ВОЗНИКШИМ ИЗ-ЗА ИСПОЛЬЗОВАНИЯ ПРОГРАММНОГО ОБЕСПЕЧЕНИЯ
 * ИЛИ ИНЫХ ДЕЙСТВИЙ С ПРОГРАММНЫМ ОБЕСПЕЧЕНИЕМ.
 *
 *                              2025 АО "НИИЭТ"
 *==============================================================================
 */

#ifndef BSP_H
#define BSP_H

#ifdef __cplusplus
extern "C" {
#endif

//-- Includes ------------------------------------------------------------------
#include "K1921VG015.h"
#include <system_k1921vg015.h>
#include <stdint.h>

//-- Defines -------------------------------------------------------------------
//LEDs
#define LED_PORT GPIOA
#define LED_PORT_EN GPIOAEN
#define LED_PIN_MSK 0xF000
#define LED0_POS 12
#define LED1_POS 13
#define LED2_POS 14
#define LED3_POS 15
#define LED0_MSK (1 << LED0_POS)
#define LED1_MSK (1 << LED1_POS)
#define LED2_MSK (1 << LED2_POS)
#define LED3_MSK (1 << LED3_POS)
#define LEDS_MSK 0xF000

//Button SB1
#define BTN_PORT GPIOA
#define BTN_PORT_EN GPIOAEN
#define BTN_IRQ_N IsrVect_IRQ_GPIO
//#define BTN_IRQ_HANDLER GPIO_IRQHandler
#define BTN_PIN_POS 11
#define BTN_PIN_MSK (1 << BTN_PIN_POS)

//-- Functions -----------------------------------------------------------------
void BSP_LED_Init(void);
void BSP_LED_On(uint32_t leds);
void BSP_LED_Off(uint32_t leds);
void BSP_LED_Toggle(uint32_t leds);
void BSP_Btn_Init(void);
uint32_t BSP_Btn_IsPressed(void);
void BTN_IRQ_HANDLER(void);

#ifdef __cplusplus
}
#endif

#endif // BSP_H
