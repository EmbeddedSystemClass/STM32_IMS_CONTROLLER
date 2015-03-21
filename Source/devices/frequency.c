#include "frequency.h"

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "misc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define FREQ_TIM					TIM3
#define FREQ_GPIO_AF 				GPIO_AF_TIM3
#define RCC_FREQ_TIM 				RCC_APB1Periph_TIM3
#define	RCC_FREQ_GPIO_PORT			RCC_AHB1Periph_GPIOA
#define FREQ_GPIO_PORT				GPIOA
#define FREQ_TIM_IRQn				TIM3_IRQn
#define FREQ_GPIO_PINS				GPIO_Pin_6 | GPIO_Pin_7
#define FREQ_GPIO_PINSOURCE_1		GPIO_PinSource6
#define FREQ_GPIO_PINSOURCE_2		GPIO_PinSource7
#define Freq_TIM_IRQHandler 		TIM3_IRQHandler
#define FREQ_TIM_PERIOD				60000+1
#define FREQ_TIM_PRESCALER			1
#define FREQ_TIMER_TICK_FREQUENCY	12000000

xQueueHandle xFrequencyMeasureCH1Queue;
xQueueHandle xFrequencyMeasureCH2Queue;

static void FrequencyMeasure_Task(void *pvParameters);

void FrequencyMeasureInit(void)
{
	 GPIO_InitTypeDef GPIO_InitStructure;
	 TIM_ICInitTypeDef TIM_ICInitStructure;
	 NVIC_InitTypeDef NVIC_InitStructure;
	 TIM_TimeBaseInitTypeDef TIM_InitStructure;

	 TIM_DeInit(FREQ_TIM);

	 RCC_APB1PeriphClockCmd(RCC_FREQ_TIM, ENABLE);
	 RCC_AHB1PeriphClockCmd(RCC_FREQ_GPIO_PORT, ENABLE);


	 TIM_TimeBaseStructInit(&TIM_InitStructure);
	 TIM_InitStructure.TIM_Prescaler = FREQ_TIM_PERIOD;
	 TIM_InitStructure.TIM_Period = FREQ_TIM_PRESCALER;
	 TIM_TimeBaseInit(FREQ_TIM, &TIM_InitStructure);
	 TIM_Cmd(FREQ_TIM, ENABLE);

	 GPIO_InitStructure.GPIO_Pin = FREQ_GPIO_PINS;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	 GPIO_Init(FREQ_GPIO_PORT, &GPIO_InitStructure);

	 GPIO_PinAFConfig(FREQ_GPIO_PORT, FREQ_GPIO_PINSOURCE_1, FREQ_GPIO_AF );
	 GPIO_PinAFConfig(FREQ_GPIO_PORT, FREQ_GPIO_PINSOURCE_2, FREQ_GPIO_AF );

	 TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	 TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	 TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	 TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	 TIM_ICInitStructure.TIM_ICFilter = 0;
	 TIM_ICInit(FREQ_TIM, &TIM_ICInitStructure);

	 TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	 TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	 TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	 TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	 TIM_ICInitStructure.TIM_ICFilter = 0;
	 TIM_ICInit(FREQ_TIM, &TIM_ICInitStructure);

	 TIM_Cmd(FREQ_TIM, ENABLE);

	 NVIC_EnableIRQ(FREQ_TIM_IRQn);
	 NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	 NVIC_InitStructure.NVIC_IRQChannel = FREQ_TIM_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 14;
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 NVIC_Init(&NVIC_InitStructure);

	 TIM_ITConfig(FREQ_TIM, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2, ENABLE);

	 TIM_ClearFlag(FREQ_TIM, TIM_FLAG_Update | TIM_FLAG_CC1 | TIM_FLAG_CC2 );

	 xFrequencyMeasureCH1Queue = xQueueCreate( 2, sizeof( uint8_t ) );
	 xFrequencyMeasureCH2Queue = xQueueCreate( 2, sizeof( uint8_t ) );

	 xTaskCreate(FrequencyMeasure_Task,(signed char*)"Freq CH1",64,NULL, tskIDLE_PRIORITY + 1, NULL);
}

typedef struct
{
	uint32_t capture;
//	uint32_t overflow_count;
	uint32_t impulse_count;
}stFrequencyData;

#define OVERFLOW_250MSEC		50
#define FREQUENCY_DATA_LEN		64
#define FREQUENCY_TIME_INTERVAL	40

static volatile stFrequencyData FrequencyDataCH1[FREQUENCY_DATA_LEN];
static volatile stFrequencyData FrequencyDataCH2[FREQUENCY_DATA_LEN];

static volatile uint8_t FrequencyDataCH1counter=0;
static volatile uint8_t FrequencyDataCH2counter=0;

static volatile uint32_t FrequencyCH1overflow_count=0;

void Freq_TIM_IRQHandler()
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	 if (FREQ_TIM->SR & TIM_IT_CC1 )
	 {
		 FREQ_TIM->SR &= (~TIM_IT_CC1 );
		 FrequencyDataCH1[FrequencyDataCH1counter].capture =FREQ_TIM->CCR1;
		 FrequencyDataCH1[FrequencyDataCH1counter].impulse_count++;
	 }

	 if (FREQ_TIM->SR & TIM_IT_CC2 )
	 {
		 FREQ_TIM->SR &= (~TIM_IT_CC2 );
		 FrequencyDataCH2[FrequencyDataCH2counter].capture =FREQ_TIM->CCR2;
		 FrequencyDataCH2[FrequencyDataCH2counter].impulse_count++;
	 }

	 if ( FREQ_TIM->SR & TIM_IT_Update )
	 {
		 FREQ_TIM->SR &= (~TIM_IT_Update );

		 FrequencyCH1overflow_count++;
//		 FrequencyDataCH2[FrequencyDataCH2counter].overflow_count++;

		 if(FrequencyCH1overflow_count>=OVERFLOW_250MSEC)
		 {
			 FrequencyCH1overflow_count=0;
			 FrequencyDataCH1counter++;
			 FrequencyDataCH1counter&=(FREQUENCY_DATA_LEN-1);
			 FrequencyDataCH1[FrequencyDataCH1counter].impulse_count=0;

			 xQueueSendFromISR(xFrequencyMeasureCH1Queue,  &FrequencyDataCH1counter, &xHigherPriorityTaskWoken);

		     if (xHigherPriorityTaskWoken)
		     {
		    	 taskYIELD();
		     }
		 }

//		 if(FrequencyDataCH2[FrequencyDataCH2counter].overflow_count>=OVERFLOW_250MSEC)
//		 {
//			 FrequencyDataCH2[FrequencyDataCH2counter].overflow_count=0;
//			 FrequencyDataCH2counter++;
//			 FrequencyDataCH2counter&=(FREQUENCY_DATA_LEN-1);
//			 FrequencyDataCH2[FrequencyDataCH2counter].impulse_count=0;
//
//			   xQueueSendFromISR(xFrequencyMeasureCH2Queue,  &FrequencyDataCH2counter, &xHigherPriorityTaskWoken);
//
//			   if (xHigherPriorityTaskWoken)
//			   {
//				  taskYIELD();
//			   }
//		 }
	 }
}

static void FrequencyMeasure_Task(void *pvParameters)
{
	uint8_t ch1_current_pos=0, ch2_current_pos=0;
	uint8_t i;

	float ch1_frequency;
	uint32_t ch1_sum_impulse=0;

 	for( ;; )
	{
		 if (uxQueueMessagesWaiting(xFrequencyMeasureCH1Queue) > 0)
		 {
			 xQueueReceive(xFrequencyMeasureCH1Queue, &ch1_current_pos, 0 );

			 uint8_t first_element=(ch1_current_pos-FREQUENCY_TIME_INTERVAL)&(FREQUENCY_DATA_LEN-1);

			 for(i=first_element;i<ch1_current_pos;i++)
			 {
				 ch1_sum_impulse+=FrequencyDataCH1[i].impulse_count;
			 }

			 ch1_frequency= (float)FREQ_TIMER_TICK_FREQUENCY*ch1_sum_impulse/((FREQUENCY_TIME_INTERVAL+1)*FREQ_TIM_PERIOD-FrequencyDataCH1[first_element].capture+FrequencyDataCH1[ch1_current_pos].capture);
			 ch1_sum_impulse=0;
		 }

//		 if (uxQueueMessagesWaiting(xFrequencyMeasureCH2Queue) > 0)
//		 {
//			 xQueueReceive(xFrequencyMeasureCH2Queue, &ch2_current_pos, 0 );
//		 }

	}
}

