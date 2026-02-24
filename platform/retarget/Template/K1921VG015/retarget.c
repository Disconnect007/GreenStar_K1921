/*==============================================================================
 * Перенаправление printf() для К1921ВГ015
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

#include "retarget.h"
#define SystemCoreClock_uart	16000000 
#define USE_LIBC

//-- Functions -----------------------------------------------------------------
void retarget_init()
{
#if defined RETARGET
    uint32_t baud_icoef = SystemCoreClock_uart / (16 * RETARGET_UART_BAUD);
    uint32_t baud_fcoef = ((SystemCoreClock_uart / (16.0f * RETARGET_UART_BAUD) - baud_icoef) * 64 + 0.5f);
    uint32_t uartclk_ref = 1;//RCU_UARTCFG_UARTCFG_CLKSEL_OSICLK;

    RCU->CGCFGAHB_bit.GPIOAEN = 1;
    RCU->RSTDISAHB_bit.GPIOAEN = 1;
    RCU->CGCFGAPB_bit.UART0EN = 1;
    RCU->RSTDISAPB_bit.UART0EN = 1;

    RETARGET_UART_PORT->ALTFUNCNUM_bit.PIN0 = 1;
    RETARGET_UART_PORT->ALTFUNCNUM_bit.PIN1 = 1;
    RETARGET_UART_PORT->ALTFUNCSET = (1 << RETARGET_UART_PIN_TX_POS) | (1 << RETARGET_UART_PIN_RX_POS);
    RCU->UARTCLKCFG[RETARGET_UART_NUM].UARTCLKCFG = (uartclk_ref << RCU_UARTCLKCFG_CLKSEL_Pos) |
                                              	  	  RCU_UARTCLKCFG_CLKEN_Msk |
													  RCU_UARTCLKCFG_RSTDIS_Msk;
    RETARGET_UART->IBRD = baud_icoef;
    RETARGET_UART->FBRD = baud_fcoef;
    RETARGET_UART->LCRH = UART_LCRH_FEN_Msk | (3 << UART_LCRH_WLEN_Pos);
    RETARGET_UART->CR = UART_CR_TXE_Msk | UART_CR_RXE_Msk | UART_CR_UARTEN_Msk;
#endif //RETARGET
}

int retarget_get_char()
{
#if defined RETARGET
    while (RETARGET_UART->FR_bit.RXFE) {
    };
    return (int)RETARGET_UART->DR_bit.DATA;
#endif //RETARGET
}

int retarget_put_char(int ch)
{
#if defined RETARGET
    while (RETARGET_UART->FR_bit.BUSY) {
    };
    RETARGET_UART->DR = ch;
#endif //RETARGET
    return 0;
}

#ifdef USE_LIBC

// Переопределяем системный вызов write для libc nano
int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
    	retarget_put_char(ptr[i]);
    }
    return len;
}

void *_sbrk(ptrdiff_t incr)
{
    extern char _ebss[];
    static char *heap_end = _ebss;
    char *base = heap_end;
    heap_end += incr;
    return base;
}

// Обработчик для errno (может потребоваться)
__attribute__((weak)) int _getpid(void) {
    return 1;
}

__attribute__((weak)) int _kill(int pid, int sig) {
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

__attribute__((weak)) void _exit(int status) {
    (void)status;
    while(1) {}
}

__attribute__((weak)) int _read(int file, char *ptr, int len) {
    (void)file;
    (void)ptr;
    (void)len;
    return 0;
}

__attribute__((weak)) int _close(int file) {
    (void)file;
    return -1;
}

__attribute__((weak)) int _fstat(int file, void *st) {
    (void)file;
    (void)st;
    return 0;
}

__attribute__((weak)) int _isatty(int file) {
    (void)file;
    return 1;
}

__attribute__((weak)) off_t _lseek(int file, off_t offset, int whence) {
    (void)file;
    (void)offset;
    (void)whence;
    return 0;
}
#endif
