#include "calc.h"
#include "controller.h"
#include "watchdog.h"

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define CALC_PERIOD	1000
#define CALC_STACK_SIZE	256


#define HT_GPIO_RCC 					RCC_AHB1Periph_GPIOE
#define HT_GPIO_CS						GPIOE
#define HT_PIN							GPIO_Pin_1

#define HT_LED_OFF						HT_GPIO_CS->BSRRL|=(HT_PIN)
#define HT_LED_ON						HT_GPIO_CS->BSRRH|=(HT_PIN)

static void Calc_Task(void *pvParameters);

void Calc_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(HT_GPIO_RCC, ENABLE);

	GPIO_InitStructure.GPIO_Pin   = HT_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(HT_GPIO_CS, &GPIO_InitStructure);

	xTaskCreate(Calc_Task,(signed char*)"Calc task",CALC_STACK_SIZE,NULL, tskIDLE_PRIORITY + 1, NULL);
}


static void Calc_Task(void *pvParameters)
{
	Watchdog_SetTaskStatus(CALC_TASK,TASK_ACTIVE);
 	for( ;; )
	{
 		HT_LED_ON;
	    xSemaphoreTake( xMeasureDataMutex, portMAX_DELAY );
	    {
	    	//чтение из структуры результатов измерений
	    }
	    xSemaphoreGive( xMeasureDataMutex );

 		vTaskDelay(CALC_PERIOD);
 		Watchdog_IncrementCouter(CALC_TASK);
 		HT_LED_OFF;
 		vTaskDelay(CALC_PERIOD);
	}
}
