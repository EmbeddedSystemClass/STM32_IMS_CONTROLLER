#include "watchdog.h"
#include "stm32f4xx_iwdg.h"
//FreeRTOS:

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define LSI_FREQ	40000

struct task_watch task_watches[TASK_NUM];
static void Watchdog_Task(void *pvParameters);//

void Watchdog_Init(void)
{
	uint8_t i;

	for(i=0;i<TASK_NUM;i++)
	{
		task_watches[i].task_status=TASK_IDLE;
	}

	  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	  IWDG_SetPrescaler(IWDG_Prescaler_256);

	  IWDG_SetReload(LSI_FREQ/64);

	  /* Reload IWDG counter */
	  IWDG_ReloadCounter();

	  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	  IWDG_Enable();

	  xTaskCreate(Watchdog_Task,(signed char*)"INIT",128,NULL, tskIDLE_PRIORITY + 1, NULL);
}

static void Watchdog_Task(void *pvParameters)
{
	while(1)
	{
		uint8_t i=0, task_error_flag=FALSE;

		for(i=0;i<TASK_NUM;i++)
		{
			if((task_watches[i].counter==0)&&(task_watches[i].task_status==TASK_ACTIVE))
			{
				task_error_flag=TRUE;
			}
		}

		if(task_error_flag!=TRUE)
		{
			IWDG_ReloadCounter();
			for(i=0;i<TASK_NUM;i++)
			{
				task_watches[i].counter=0;
			}
		}

		vTaskDelay(2000);
	}
}
