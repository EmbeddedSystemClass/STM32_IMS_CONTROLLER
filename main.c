#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"

#include <misc.h>
#include "system_stm32f4xx.h"

#include "controller.h"



int main(void)
{
	SystemInit();
	ControllerInit();
    vTaskStartScheduler();
    while(1);
}


//---------------------------------------------------------------------------------------

void vApplicationTickHook( void )
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
	}
}
