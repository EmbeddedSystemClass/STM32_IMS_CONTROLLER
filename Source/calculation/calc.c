#include "calc.h"
#include "controller.h"
#include "watchdog.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define CALC_PERIOD	1000
#define CALC_STACK_SIZE	256
static void Calc_Task(void *pvParameters);

void Calc_Init(void)
{
	xTaskCreate(Calc_Task,(signed char*)"Calc task",CALC_STACK_SIZE,NULL, tskIDLE_PRIORITY + 1, NULL);
}


static void Calc_Task(void *pvParameters)
{
	Watchdog_SetTaskStatus(CALC_TASK,TASK_ACTIVE);
 	for( ;; )
	{
	    xSemaphoreTake( xMeasureDataMutex, portMAX_DELAY );
	    {
	    	//чтение из структуры результатов измерений
	    }
	    xSemaphoreGive( xMeasureDataMutex );



 		vTaskDelay(CALC_PERIOD);
 		Watchdog_IncrementCouter(CALC_TASK);
	}
}
