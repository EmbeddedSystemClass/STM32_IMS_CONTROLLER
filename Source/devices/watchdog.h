#ifndef WATCHDOG_H
#define WATCHDOG_H
#include "stm32f4xx.h"

#define TASK_NUM	11

enum
{
	RS232_TASK=0,
	RS485_TASK,
	USB_TASK,
	FREQUENCY_CH1_TASK,
	FREQUENCY_CH2_TASK,
	ADS1220_RTD1_TASK,
	ADS1220_RTD2_TASK,
	ADS1220_CURRENT1_TASK,
	ADS1220_CURRENT2_TASK,
	LOG_TASK,
	CALC_TASK
};

#ifndef TRUE
#define TRUE            1
#endif

#ifndef FALSE
#define FALSE           0
#endif

enum
{
	TASK_ACTIVE=0,
	TASK_IDLE=1
};

struct task_watch
{
	uint32_t counter;
	uint8_t  task_status;
};

extern struct task_watch task_watches[];

void Watchdog_Init(void);
void Watchdog_SetTaskStatus(uint8_t task,uint8_t status);
void Watchdog_IncrementCouter(uint8_t task);

#endif
