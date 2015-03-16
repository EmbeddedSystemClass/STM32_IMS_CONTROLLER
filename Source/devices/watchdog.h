#ifndef WATCHDOG_H
#define WATCHDOG_H
#include "stm32f4xx.h"

#define TASK_NUM	10//количество задач, кроме задачи ватчдога , в системе

enum
{
	PROTO_TASK=0,//ok
	BUZZER_TASK=1,//ok
	DISPLAY_TASK=2,//ok
	DRYING_TASK=3,//ok
	HEATER_INIT_TASK=4,//not required
	ADS1120_TASK=5,//ok
	ADC_TASK=6,//ok
	KEYBOARD_TASK=7,//ok
	HEATER_CONTROL_TASK=8,//ok
	PID_TASK=9,
};

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

void Watchdog_Init(void);

#endif
