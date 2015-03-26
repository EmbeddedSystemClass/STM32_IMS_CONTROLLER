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
#define FREQ_TIM_PERIOD				(65535)
#define FREQ_TIM_PRESCALER			0
#define FREQ_TIMER_TICK_FREQUENCY	20000000//20000000///12000000

xQueueHandle xFrequencyMeasureCH1Queue;
xQueueHandle xFrequencyMeasureCH2Queue;

static void FrequencyCH1Measure_Task(void *pvParameters);
static void FrequencyCH2Measure_Task(void *pvParameters);

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
	 TIM_InitStructure.TIM_Prescaler = FREQ_TIM_PRESCALER;
	 TIM_InitStructure.TIM_Period = FREQ_TIM_PERIOD ;
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
	 TIM_ICInitStructure.TIM_ICFilter = 5;
	 TIM_ICInit(FREQ_TIM, &TIM_ICInitStructure);

	 TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	 TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	 TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	 TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	 TIM_ICInitStructure.TIM_ICFilter = 5;
	 TIM_ICInit(FREQ_TIM, &TIM_ICInitStructure);



	 NVIC_EnableIRQ(FREQ_TIM_IRQn);
	 NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	 NVIC_InitStructure.NVIC_IRQChannel = FREQ_TIM_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 NVIC_Init(&NVIC_InitStructure);

	 TIM_ITConfig(FREQ_TIM,TIM_IT_CC1|TIM_IT_CC2, ENABLE);
	 TIM_SelectOutputTrigger(FREQ_TIM, TIM_TRGOSource_Update);
	 TIM_ClearFlag(FREQ_TIM, TIM_FLAG_CC1|TIM_FLAG_CC2 );




	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_InitStructure.TIM_Period = 65535;
	TIM_InitStructure.TIM_Prescaler = 0;
	TIM_InitStructure.TIM_ClockDivision = 0;
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_InitStructure);


	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	 GPIO_Init(FREQ_GPIO_PORT, &GPIO_InitStructure);

	 GPIO_PinAFConfig(FREQ_GPIO_PORT, GPIO_PinSource1, GPIO_AF_TIM2 );
	 GPIO_PinAFConfig(FREQ_GPIO_PORT, GPIO_PinSource2, GPIO_AF_TIM2 );


	 TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	 TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	 TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	 TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	 TIM_ICInitStructure.TIM_ICFilter = 5;
	 TIM_ICInit(TIM2, &TIM_ICInitStructure);

	 TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	 TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	 TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	 TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	 TIM_ICInitStructure.TIM_ICFilter = 5;
	 TIM_ICInit(TIM2, &TIM_ICInitStructure);


	TIM_ARRPreloadConfig(TIM2, ENABLE);

	TIM_SelectInputTrigger(TIM2, TIM_TS_ITR2);

	TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);
	TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_External1);

	TIM_Cmd(TIM2,  ENABLE);


	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	 GPIO_Init(FREQ_GPIO_PORT, &GPIO_InitStructure);

	 GPIO_PinAFConfig(FREQ_GPIO_PORT, GPIO_PinSource12, GPIO_AF_TIM1 );

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	TIM_TimeBaseStructInit(&TIM_InitStructure);
	TIM_InitStructure.TIM_Prescaler = 0;
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_Period= 65535;
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM1, &TIM_InitStructure);
	TIM_ETRClockMode2Config(TIM1, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0x00);
	TIM_Cmd(TIM1, ENABLE);

	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	 GPIO_Init(FREQ_GPIO_PORT, &GPIO_InitStructure);

	 GPIO_PinAFConfig(FREQ_GPIO_PORT, GPIO_PinSource0, GPIO_AF_TIM8 );

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	TIM_TimeBaseStructInit(&TIM_InitStructure);
	TIM_InitStructure.TIM_Prescaler = 0;
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_Period= 65535;
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM8, &TIM_InitStructure);
	TIM_ETRClockMode2Config(TIM8, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0x00);
	TIM_Cmd(TIM8, ENABLE);


	 xFrequencyMeasureCH1Queue = xQueueCreate( 2, sizeof( uint8_t ) );
	 xFrequencyMeasureCH2Queue = xQueueCreate( 2, sizeof( uint8_t ) );

	 xTaskCreate(FrequencyCH1Measure_Task,(signed char*)"Freq CH1",64,NULL, tskIDLE_PRIORITY + 1, NULL);
	 xTaskCreate(FrequencyCH2Measure_Task,(signed char*)"Freq CH2",64,NULL, tskIDLE_PRIORITY + 1, NULL);

}

typedef struct
{
	uint32_t capture;
	uint32_t div_cap;
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

portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
void Freq_TIM_IRQHandler()
{
//	 TIM_ITConfig(FREQ_TIM,TIM_IT_CC1|TIM_IT_CC2, DISABLE);

	 if (TIM_GetITStatus(FREQ_TIM,TIM_FLAG_CC1 ))
	 {
		 TIM_ClearFlag(FREQ_TIM, TIM_FLAG_CC1 );
		 FREQ_TIM->DIER &= (uint16_t)~TIM_IT_CC1;
		 FrequencyDataCH1[FrequencyDataCH1counter].capture =(uint32_t)FREQ_TIM->CCR1|(((uint32_t)TIM2->CCR2)<<16);
		 FrequencyDataCH1[FrequencyDataCH1counter].impulse_count=TIM8->CNT;
		 TIM8->CNT=0x0;

		 FrequencyDataCH1counter++;
		 FrequencyDataCH1counter&=(FREQUENCY_DATA_LEN-1);
		 xQueueSendFromISR(xFrequencyMeasureCH1Queue,  &FrequencyDataCH1counter, &xHigherPriorityTaskWoken);
	 }

	 if (TIM_GetITStatus(FREQ_TIM,TIM_FLAG_CC2 ))
	 {
		 FREQ_TIM->SR &= (~TIM_FLAG_CC2 );
		 FREQ_TIM->DIER &= (uint16_t)~TIM_IT_CC2;
		 FrequencyDataCH2[FrequencyDataCH2counter].capture =(uint32_t)FREQ_TIM->CCR2|(((uint32_t)TIM2->CCR3)<<16);
		 FrequencyDataCH2[FrequencyDataCH2counter].impulse_count=TIM1->CNT;
		 TIM1->CNT=0x0;

		 FrequencyDataCH2counter++;
		 FrequencyDataCH2counter&=(FREQUENCY_DATA_LEN-1);
		 xQueueSendFromISR(xFrequencyMeasureCH2Queue,  &FrequencyDataCH2counter, &xHigherPriorityTaskWoken);
	 }

//	 TIM_ITConfig(FREQ_TIM,TIM_IT_CC1|TIM_IT_CC2, ENABLE);
}


float ch1_frequency;
static void FrequencyCH1Measure_Task(void *pvParameters)
{
	uint8_t ch1_current_pos=0, ch2_current_pos=0;
	uint8_t i;


	uint32_t ch1_sum_impulse=0;
	uint32_t ch1_sum_tick_impulse=0;

 	for( ;; )
	{
 		vTaskDelay(250);
 		FREQ_TIM->SR &= (~TIM_IT_CC1 );
 		FREQ_TIM->DIER |= TIM_IT_CC1;

		if (uxQueueMessagesWaiting(xFrequencyMeasureCH1Queue) > 0)
		{
			 xQueueReceive(xFrequencyMeasureCH1Queue, &ch1_current_pos, 0 );

			 ch1_sum_impulse=FrequencyDataCH1[(ch1_current_pos-1)&(FREQUENCY_DATA_LEN-1)].impulse_count;

			 if(FrequencyDataCH1[(ch1_current_pos-1)&(FREQUENCY_DATA_LEN-1)].capture>FrequencyDataCH1[(ch1_current_pos-2)&(FREQUENCY_DATA_LEN-1)].capture)
			 {
				 ch1_sum_tick_impulse=FrequencyDataCH1[(ch1_current_pos-1)&(FREQUENCY_DATA_LEN-1)].capture-FrequencyDataCH1[(ch1_current_pos-2)&(FREQUENCY_DATA_LEN-1)].capture;
			 }
			 else
			 {
				 ch1_sum_tick_impulse=FrequencyDataCH1[(ch1_current_pos-2)&(FREQUENCY_DATA_LEN-1)].capture-FrequencyDataCH1[(ch1_current_pos-1)&(FREQUENCY_DATA_LEN-1)].capture;
			 }

			 FrequencyDataCH1[(ch1_current_pos-1)&(FREQUENCY_DATA_LEN-1)].div_cap=ch1_sum_tick_impulse;

			 ch1_frequency= (float)FREQ_TIMER_TICK_FREQUENCY*ch1_sum_impulse/ch1_sum_tick_impulse;
		}
	}
}


float ch2_frequency;
static void FrequencyCH2Measure_Task(void *pvParameters)
{
	uint8_t ch2_current_pos=0;
	uint8_t i;


	uint32_t ch2_sum_impulse=0;
	uint32_t ch2_sum_tick_impulse=0;

 	for( ;; )
	{
 		vTaskDelay(250);
 		FREQ_TIM->SR &= (~TIM_IT_CC2 );
 		FREQ_TIM->DIER |= TIM_IT_CC2;

		if (uxQueueMessagesWaiting(xFrequencyMeasureCH2Queue) > 0)
		{
			 xQueueReceive(xFrequencyMeasureCH2Queue, &ch2_current_pos, 0 );

			 ch2_sum_impulse=FrequencyDataCH2[(ch2_current_pos-1)&(FREQUENCY_DATA_LEN-1)].impulse_count;

			 if(FrequencyDataCH2[(ch2_current_pos-1)&(FREQUENCY_DATA_LEN-1)].capture>FrequencyDataCH2[(ch2_current_pos-2)&(FREQUENCY_DATA_LEN-1)].capture)
			 {
				 ch2_sum_tick_impulse=FrequencyDataCH2[(ch2_current_pos-1)&(FREQUENCY_DATA_LEN-1)].capture-FrequencyDataCH2[(ch2_current_pos-2)&(FREQUENCY_DATA_LEN-1)].capture;
			 }
			 else
			 {
				 ch2_sum_tick_impulse=FrequencyDataCH2[(ch2_current_pos-2)&(FREQUENCY_DATA_LEN-1)].capture-FrequencyDataCH2[(ch2_current_pos-1)&(FREQUENCY_DATA_LEN-1)].capture;
			 }

			 FrequencyDataCH2[(ch2_current_pos-1)&(FREQUENCY_DATA_LEN-1)].div_cap=ch2_sum_tick_impulse;

			 ch2_frequency= (float)FREQ_TIMER_TICK_FREQUENCY*ch2_sum_impulse/ch2_sum_tick_impulse;
		}
	}
}

