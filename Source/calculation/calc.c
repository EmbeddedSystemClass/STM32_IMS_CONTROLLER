#include "calc.h"
#include "controller.h"
#include "watchdog.h"

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_rtc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include <math.h>
#include "arm_math.h"


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

	RCC_AHB1PeriphClockCmd(HT_GPIO_RCC, ENABLE);

	GPIO_InitStructure.GPIO_Pin   = HT_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(HT_GPIO_CS, &GPIO_InitStructure);

	xTaskCreate(Calc_Task,(signed char*)"Calc task",CALC_STACK_SIZE,NULL, tskIDLE_PRIORITY + 1, NULL);
}

#define TEST_CONST	3.14

static void Calc_Task(void *pvParameters)
{
	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_DateTypeDef RTC_DateStructure;
	portTickType xLastWakeTime;
	const portTickType xFreq = 1000;

	xLastWakeTime = xTaskGetTickCount ();
	Watchdog_SetTaskStatus(CALC_TASK,TASK_ACTIVE);
 	for( ;; )
	{
		vTaskDelayUntil(&xLastWakeTime, xFreq);

		if(fl_led==1){
    		fl_led=0;
    		HT_LED_OFF;
    	}
    	else{
    		fl_led=1;
    		HT_LED_ON;
    	}


 		float val=0.0;
	    xSemaphoreTake( xMeasureDataMutex, portMAX_DELAY );
	    {
	    	//чтение из структуры результатов измерений

			RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
			RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);

//	    	val=(float)stMeasureData.frequency[0]*TEST_CONST-RTC_TimeStructure.RTC_Minutes;
//	    	val=(float)stMeasureData.frequency[0]*3600.0/90000;
//			val=(float)(stMeasureData.pulse_counter[0]-stMeasureData.last_counter[0])/90000;
			val=(stMeasureData.pulse_counter[0]-stMeasureData.last_counter[0])>=0 ? ((float)(stMeasureData.pulse_counter[0]-stMeasureData.last_counter[0])/90000) : ((float)(stMeasureData.pulse_counter[0]+stMeasureData.last_counter[0]-0xffffffff)/90000);
//	    	stMeasureData.test_var=(float)my_counter;//val;

		   	stMeasureData.test_var=(float)my_counter ?  (float)val/((float)my_counter/1000.0) : 0;
//	    	stMeasureData.test_var=((float)my_counter/1000.0);


//	    	stMeasureData.test_var=(float)(stMeasureData.pulse_counter[0]-stMeasureData.last_counter[0]);

			stMeasureData.last_counter[0]=stMeasureData.pulse_counter[0];


	    }
	    xSemaphoreGive( xMeasureDataMutex );


	    my_counter=0;
	    float val1;
    	int j;
	    	for (j = 0; j < 500000; j++) {
	    		//val1=pow(TEST_CONST,3.0);


	    		val1+=sin(TEST_CONST);
	    		//val1+=arm_sin_f32(TEST_CONST);
			}


// 		vTaskDelay(CALC_PERIOD>>1);
// 		vTaskDelay(CALC_PERIOD);

 		Watchdog_IncrementCouter(CALC_TASK);



// 		HT_LED_OFF;  //выкл. светодиод
// 		vTaskDelay(CALC_PERIOD>>1);
	}
}
