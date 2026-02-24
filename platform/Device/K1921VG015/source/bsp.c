/*==============================================================================
 * Управление периферией на плате NIIET-DEV-K1921VG015 (КФДЛ.441461.029)
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

//-- Includes ------------------------------------------------------------------
#include "bsp.h"

//-- Private variables ---------------------------------------------------------
static volatile uint32_t btn_press_event = 0;

//-- Functions -----------------------------------------------------------------
void BSP_LED_Init()
{
	RCU->CGCFGAHB_bit.LED_PORT_EN = 1;
	RCU->RSTDISAHB_bit.LED_PORT_EN = 1;
	LED_PORT->OUTENSET = LEDS_MSK;
	LED_PORT->DATAOUTSET = LEDS_MSK;
}

void BSP_LED_Toggle(uint32_t leds)
{
	LED_PORT->DATAOUTTGL = leds;
}

void BSP_LED_On(uint32_t leds)
{
	LED_PORT->DATAOUTCLR = leds;
}

void BSP_LED_Off(uint32_t leds)
{
	LED_PORT->DATAOUTSET = leds;
}

void BSP_Btn_Init()
{
	RCU->CGCFGAHB_bit.BTN_PORT_EN = 1;
	RCU->RSTDISAHB_bit.BTN_PORT_EN = 1;
	BTN_PORT->OUTENCLR = BTN_PIN_MSK;
	BTN_PORT->INTTYPESET = BTN_PIN_MSK; // фронт
	BTN_PORT->INTPOLSET = BTN_PIN_MSK;  // положительный
	BTN_PORT->INTENSET = BTN_PIN_MSK;
	// Настраиваем обработчик прерывания
	PLIC_SetIrqHandler (Plic_Mach_Target, BTN_IRQ_N, BTN_IRQ_HANDLER);
	PLIC_SetPriority   (BTN_IRQ_N, 0x1);
	PLIC_IntEnable     (Plic_Mach_Target, BTN_IRQ_N);
}

uint32_t BSP_Btn_IsPressed()
{
    if (btn_press_event) {
        btn_press_event = 0;
        return 1;
    } else
        return 0;
}

//-- IRQ handlers --------------------------------------------------------------
void BTN_IRQ_HANDLER()
{
	BTN_PORT->INTSTATUS = BTN_PIN_MSK;
	btn_press_event = 1;
}
