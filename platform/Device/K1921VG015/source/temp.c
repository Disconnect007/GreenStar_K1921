#include "temp.h"

uint16_t Get_Temp_adc()
{
    uint16_t adcsar_res[3];
    uint16_t adc_good;
    uint8_t i;
    ADCSAR->SEQSYNC_bit.GSYNC = 1;
    while (ADCSAR->BSTAT) __asm("nop");
    while (!(ADCSAR->RIS_bit.SEQRIS0)) {};
    while (ADCSAR->SEQ[0].SFLOAD < ADCSAR->SEQ[0].SRQCTL_bit.RQMAX) {};
    for(i=0;i<=ADCSAR->SEQ[0].SRQCTL_bit.RQMAX;i++)
        adcsar_res[i] = ADCSAR->SEQ[0].SFIFO;
    if(ADCSAR->SEQ[0].SRQCTL_bit.RQMAX == 2){ // Если очередь запросов - из 3-х измерений, то выбираем одно из двух похожих
        if (abs(adcsar_res[2] - adcsar_res[1]) < 16) adc_good = adcsar_res[2];
        else if (abs(adcsar_res[0] - adcsar_res[1]) < 16) adc_good = adcsar_res[1];
        else adc_good = adcsar_res[0];
    } else adc_good = adcsar_res[ADCSAR->SEQ[0].SRQCTL_bit.RQMAX];
    return adc_good;
}

// Функция вычисления температуры при подключении датчика температуры к внутреннему источнику тока
float Get_Temp_Celsius()
{
	double tf;
	uint16_t tadc;
	tadc = Get_Temp_adc();
	tf = ((double)tadc / 4096);
	tf = AREF * tf;
	tf = 250.0 * (1.38 - tf);
    return (float)tf;
}

// Функция вычисления температуры при подключении датчика температуры к источнику тока ADCSD
float Get_Temp_Celsius_ISEL_ADCSD()
{
	double tf;
	uint16_t tadc;
	tadc = Get_Temp_adc();
	tf = ((double)tadc / 4096);
	tf = AREF * tf;
	tf = 211.023 * tf;
	tf = 351.8597 - tf;
    return (float)tf;
}

void adcsar_init(uint8_t tsensor_isrc)
{
	// настройка питания ADC
    PMUSYS->ADCPWRCFG_bit.LDOEN = 1;
    PMUSYS->ADCPWRCFG_bit.LVLDIS = 0;

    //Инициализация тактирвоания блока ADC
    RCU->ADCSARCLKCFG_bit.CLKSEL = RCU_ADCSARCLKCFG_CLKSEL_HSE;
    // Устанавливаем максимальный делитель частоты для измерений высокоомных сигналов
    RCU->ADCSARCLKCFG_bit.DIVN = 0; // N=2*(DIVN + 1)
    RCU->ADCSARCLKCFG_bit.DIVEN = 0;
    RCU->ADCSARCLKCFG_bit.CLKEN = 1;
    RCU->ADCSARCLKCFG_bit.RSTDIS = 1;

    //инициализация тактирования и сброса логики АЦП
    RCU->CGCFGAPB_bit.ADCSAREN = 1;
    RCU->RSTDISAPB_bit.ADCSAREN = 1;

    //Настройка модуля АЦП
    //12бит и без калибровки при включении
    ADCSAR->ACTL_bit.SELRES = ADCSAR_ACTL_SELRES_12bit;
    ADCSAR->ACTL_bit.CALEN = 0;
    ADCSAR->ACTL_bit.ADCEN = 1;

    //Настройка секвенсора 0: CH0 - CH7
    ADCSAR->EMUX_bit.EM0 = ADCSAR_EMUX_EM0_SwReq;
    //ADCSAR->EMUX = 0x0F;
    ADCSAR->SEQ[0].SCCTL_bit.ICNT = 0;
    ADCSAR->SEQ[0].SCCTL_bit.RCNT = 0;
    ADCSAR->SEQ[0].SRTMR = 0x0;
    ADCSAR->SEQ[0].SRQCTL_bit.QAVGVAL = ADCSAR_SEQ_SRQCTL_QAVGVAL_Disable;
    ADCSAR->SEQ[0].SRQCTL_bit.QAVGEN = 0;
    ADCSAR->SEQ[0].SRQCTL_bit.RQMAX = 2;
    ADCSAR->SEQ[0].SCCTL_bit.RCNT = 0;
    ADCSAR->SEQ[0].SCCTL_bit.RAVGEN = 1;
    ADCSAR->SEQ[0].SRQSEL_bit.RQ0 = 10; //канал №10, к которому подключен внутренний датчик температуры
    ADCSAR->SEQ[0].SRQSEL_bit.RQ1 = 10; //канал №10, к которому подключен внутренний датчик температуры
    ADCSAR->SEQ[0].SRQSEL_bit.RQ2 = 10; //канал №10, к которому подключен внутренний датчик температуры

    //Включаем секвенсоры
    ADCSAR->SEQSYNC = ADCSAR_SEQSYNC_SYNC0_Msk;
    ADCSAR->SEQEN = ADCSAR_SEQEN_SEQEN0_Msk;
    //Включаем датчик температуры
    if(tsensor_isrc == TSENSOR_ISRC_ADCSD)
    {
        // Если в качестве источника тока датчика температуры выбран внешний - от ADCSD
        //Инициализация тактирования блока ADC
        RCU->ADCSDCLKCFG_bit.CLKSEL = 0;
        RCU->ADCSDCLKCFG_bit.DIVEN = 0;
        RCU->ADCSDCLKCFG_bit.CLKEN = 1;
        RCU->ADCSDCLKCFG_bit.RSTDIS = 1;
        //инициализация тактирования и сброса логики АЦП
        RCU->CGCFGAPB_bit.ADCSDEN = 1;
        RCU->RSTDISAPB_bit.ADCSDEN = 1;
        //Включаем модуль АЦП
        ADCSD->CTRL_bit.MDC = 0;
        ADCSD->CTRL_bit.DR = 0;
        ADCSD->CTRL_bit.ENB = 1;
        ADCSD->CTRL_bit.PUREF = 1;
        ADCSD->CTRL_bit.WTCYC = 3;
        // Включаем один канал для включения источника тока
        ADCSD->ENB_bit.CH0 = 1;
        TSENSOR_ISEL_ADCSD_ENABLE
        ADCSAR->CHDELAY[10].CHDELAY = 14;  //Дополнительные такты ожидания для увеличения времени подключения выхода датчика температуры к зарядной емкости АЦП.
    } else {
        TSENSOR_ISEL_INT_ENABLE
		if(RCU->ADCSARCLKCFG_bit.CLKSEL == RCU_ADCSARCLKCFG_CLKSEL_HSE)
		    ADCSAR->CHDELAY[10].CHDELAY = HSECLK_VAL/4000;  //Дополнительные такты ожидания для увеличения времени подключения выхода датчика температуры к зарядной емкости АЦП. 
        else if(RCU->ADCSARCLKCFG_bit.CLKSEL == RCU_ADCSARCLKCFG_CLKSEL_HSI)  
            ADCSAR->CHDELAY[10].CHDELAY = 1000000/4000;  //Дополнительные такты ожидания для увеличения времени подключения выхода датчика температуры к зарядной емкости АЦП.          
    }    
    //Ждем пока АЦП пройдут инициализацию, начатую в самом начале
    while (!(ADCSAR->ACTL_bit.ADCRDY)) {
    };
}