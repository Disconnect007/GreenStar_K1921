#include "unix_time.h"

/* Функция заполняет структуру календаря unix_time в зависисмости от счетчика секунд timer */
void timer_to_cal (unsigned long timer, unix_cal * unix_time)
{
	unsigned long a;
	char b;
	char c;
	char d;
	unsigned long time;

	time = timer%SEC_A_DAY;
	a = ((timer+43200)/(86400>>1)) + (2440587<<1) + 1;
	a>>=1;
	unix_time->wday = a%7;
	a+=32044;
	b=(4*a+3)/146097;
	a=a-(146097*b)/4;
	c=(4*a+3)/1461;
	a=a-(1461*c)/4;
	d=(5*a+2)/153;
	unix_time->mday=a-(153*d+2)/5+1;
	unix_time->mon=d+3-12*(d/10);
	unix_time->year=100*b+c-4800+(d/10);
	unix_time->hour=time/3600;
	unix_time->min=(time%3600)/60;
	unix_time->sec=(time%3600)%60;
}

/* Функция возвращает значение счетчика секунд в зависисмости от структуры календаря unix_time  */
unsigned long cal_to_timer (unix_cal * unix_time)
{
	char a;
	int y;
	char m;
	unsigned long Uday;
	unsigned long time;

	a=((14-unix_time->mon)/12);
	y=unix_time->year+4800-a;
	m=unix_time->mon+(12*a)-3;
	Uday=(unix_time->mday+((153*m+2)/5)+365*y+(y/4)-(y/100)+(y/400)-32045)-2440588;
	time=Uday*86400;
	time+=unix_time->sec+unix_time->min*60+unix_time->hour*3600;
	return time;
}



