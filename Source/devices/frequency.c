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

#define FREQ_CAPTURE_TIM					TIM2
#define FREQ_CAPTURE_GPIO_AF 				GPIO_AF_TIM2
#define RCC_FREQ_CAPTURE_TIM 				RCC_APB1Periph_TIM2
#define	RCC_FREQ_CAPTURE_GPIO_PORT			RCC_AHB1Periph_GPIOA
#define FREQ_CAPTURE_GPIO_PORT				GPIOA
#define FREQ_CAPTURE_TIM_IRQn				TIM2_IRQn
#define FREQ_CAPTURE_GPIO_PINS				GPIO_Pin_1 | GPIO_Pin_2
#define FREQ_CAPTURE_GPIO_PINSOURCE_1		GPIO_PinSource1
#define FREQ_CAPTURE_GPIO_PINSOURCE_2		GPIO_PinSource2
#define FREQ_CAPTURE_TIM_IRQHandler 		TIM2_IRQHandler
#define FREQ_CAPTURE_TIM_PERIOD				(0xFFFFFFFF)
#define FREQ_CAPTURE_TIMER_TICK_FREQUENCY	19999592//20000000///12000000
#define FREQ_CAPTURE_PERIOD_0_HZ			10000
#define FREQ_CAPTURE_IC_FILTER				5
#define FREQ_CAPTURE_CHN_1					TIM_Channel_2
#define FREQ_CAPTURE_CHN_2					TIM_Channel_3
#define FREQ_CAPTURE_IT_1					TIM_IT_CC2
#define FREQ_CAPTURE_IT_2					TIM_IT_CC3
#define FREQ_CAP_REG_1						CCR2
#define FREQ_CAP_REG_2						CCR3

#define FREQ_COUNT_2_TIM					TIM1
#define FREQ_COUNT_2_GPIO_AF 				GPIO_AF_TIM1
#define RCC_FREQ_COUNT_2_TIM 				RCC_APB2Periph_TIM1
#define	RCC_FREQ_COUNT_2_GPIO_PORT			RCC_AHB1Periph_GPIOA
#define FREQ_COUNT_2_GPIO_PORT				GPIOA
#define FREQ_COUNT_2_GPIO_PINS				GPIO_Pin_12
#define FREQ_COUNT_2_GPIO_PINSOURCE			GPIO_PinSource12
#define FREQ_COUNT_2_TIM_PERIOD				(65535)

#define FREQ_COUNT_1_TIM					TIM8
#define FREQ_COUNT_1_GPIO_AF 				GPIO_AF_TIM8
#define RCC_FREQ_COUNT_1_TIM 				RCC_APB2Periph_TIM8
#define	RCC_FREQ_COUNT_1_GPIO_PORT			RCC_AHB1Periph_GPIOA
#define FREQ_COUNT_1_GPIO_PORT				GPIOA
#define FREQ_COUNT_1_GPIO_PINS				GPIO_Pin_0
#define FREQ_COUNT_1_GPIO_PINSOURCE			GPIO_PinSource0
#define FREQ_COUNT_1_TIM_PERIOD				(65535)

#define FREQ_MEASURE_TIME					250


xQueueHandle xFrequencyMeasureCH1Queue;
xQueueHandle xFrequencyMeasureCH2Queue;

xQueueHandle xFrequencyResultQueue[2];

static void FrequencyCH1Measure_Task(void *pvParameters);
static void FrequencyCH2Measure_Task(void *pvParameters);

void FrequencyMeasureInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_FREQ_CAPTURE_GPIO_PORT | RCC_FREQ_COUNT_1_GPIO_PORT | RCC_FREQ_COUNT_2_GPIO_PORT, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_FREQ_CAPTURE_TIM, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_FREQ_COUNT_1_TIM, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_FREQ_COUNT_2_TIM, ENABLE);

	TIM_InitStructure.TIM_Period = FREQ_CAPTURE_TIM_PERIOD;
	TIM_InitStructure.TIM_Prescaler = 0;
	TIM_InitStructure.TIM_ClockDivision = 0;
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(FREQ_CAPTURE_TIM, &TIM_InitStructure);

	GPIO_InitStructure.GPIO_Pin = FREQ_CAPTURE_GPIO_PINS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(FREQ_CAPTURE_GPIO_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(FREQ_CAPTURE_GPIO_PORT, FREQ_CAPTURE_GPIO_PINSOURCE_1, FREQ_CAPTURE_GPIO_AF );
	GPIO_PinAFConfig(FREQ_CAPTURE_GPIO_PORT, FREQ_CAPTURE_GPIO_PINSOURCE_2, FREQ_CAPTURE_GPIO_AF );

	TIM_ICInitStructure.TIM_Channel = FREQ_CAPTURE_CHN_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = FREQ_CAPTURE_IC_FILTER;
	TIM_ICInit(FREQ_CAPTURE_TIM, &TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel = FREQ_CAPTURE_CHN_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = FREQ_CAPTURE_IC_FILTER;
	TIM_ICInit(FREQ_CAPTURE_TIM, &TIM_ICInitStructure);

	TIM_ARRPreloadConfig(FREQ_CAPTURE_TIM, ENABLE);

	NVIC_EnableIRQ(FREQ_CAPTURE_TIM_IRQn);
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	NVIC_InitStructure.NVIC_IRQChannel = FREQ_CAPTURE_TIM_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(FREQ_CAPTURE_TIM, FREQ_CAPTURE_IT_1 | FREQ_CAPTURE_IT_2, ENABLE);
	TIM_ClearFlag(FREQ_CAPTURE_TIM , FREQ_CAPTURE_IT_1 | FREQ_CAPTURE_IT_2 );

	TIM_Cmd(FREQ_CAPTURE_TIM,  ENABLE);

	GPIO_InitStructure.GPIO_Pin = FREQ_COUNT_1_GPIO_PINS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(FREQ_COUNT_1_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(FREQ_COUNT_1_GPIO_PORT, FREQ_COUNT_1_GPIO_PINSOURCE, FREQ_COUNT_1_GPIO_AF );

	GPIO_InitStructure.GPIO_Pin = FREQ_COUNT_2_GPIO_PINS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(FREQ_COUNT_2_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(FREQ_COUNT_2_GPIO_PORT, FREQ_COUNT_2_GPIO_PINSOURCE, FREQ_COUNT_2_GPIO_AF );

	TIM_TimeBaseStructInit(&TIM_InitStructure);
	TIM_InitStructure.TIM_Prescaler = 0;
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_Period= FREQ_COUNT_1_TIM_PERIOD;
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(FREQ_COUNT_1_TIM, &TIM_InitStructure);
	TIM_ETRClockMode2Config(FREQ_COUNT_1_TIM, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0x00);
	TIM_Cmd(FREQ_COUNT_1_TIM, ENABLE);

	TIM_TimeBaseInit(FREQ_COUNT_2_TIM, &TIM_InitStructure);
	TIM_ETRClockMode2Config(FREQ_COUNT_2_TIM, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0x00);
	TIM_Cmd(FREQ_COUNT_2_TIM, ENABLE);

	xFrequencyMeasureCH1Queue = xQueueCreate( 2, sizeof( uint8_t ) );
	xFrequencyMeasureCH2Queue = xQueueCreate( 2, sizeof( uint8_t ) );

	xFrequencyResultQueue[0] = xQueueCreate( 2, sizeof( float ) );
	xFrequencyResultQueue[1] = xQueueCreate( 2, sizeof( float ) );

	xTaskCreate(FrequencyCH1Measure_Task,(signed char*)"Freq CH1",64,NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(FrequencyCH2Measure_Task,(signed char*)"Freq CH2",64,NULL, tskIDLE_PRIORITY + 1, NULL);
}

typedef struct
{
	uint32_t capture;
	uint32_t impulse_count;
}stFrequencyData;


#define FREQUENCY_DATA_LEN		64
#define FREQUENCY_TIME_INTERVAL	40

static volatile stFrequencyData FrequencyDataCH1[FREQUENCY_DATA_LEN];
static volatile stFrequencyData FrequencyDataCH2[FREQUENCY_DATA_LEN];

static volatile uint8_t FrequencyDataCH1counter=0;
static volatile uint8_t FrequencyDataCH2counter=0;

static volatile uint32_t FrequencyCH1overflow_count=0;

portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

void FREQ_CAPTURE_TIM_IRQHandler()
{
	 if ((FREQ_CAPTURE_TIM->SR & FREQ_CAPTURE_IT_1) && (FREQ_CAPTURE_TIM->DIER & FREQ_CAPTURE_IT_1))
	 {
		 FREQ_CAPTURE_TIM->SR &= (~FREQ_CAPTURE_IT_1 );
		 FREQ_CAPTURE_TIM->DIER &= (uint16_t)~FREQ_CAPTURE_IT_1;
		 FrequencyDataCH1[FrequencyDataCH1counter].capture =FREQ_CAPTURE_TIM->FREQ_CAP_REG_1;
		 FrequencyDataCH1[FrequencyDataCH1counter].impulse_count=FREQ_COUNT_1_TIM->CNT;
		 FREQ_COUNT_1_TIM->CNT=0x0;

		 FrequencyDataCH1counter++;
		 FrequencyDataCH1counter&=(FREQUENCY_DATA_LEN-1);
		 xQueueSendFromISR(xFrequencyMeasureCH1Queue,  &FrequencyDataCH1counter, &xHigherPriorityTaskWoken);
	 }

	 if ((FREQ_CAPTURE_TIM->SR & FREQ_CAPTURE_IT_2) && (FREQ_CAPTURE_TIM->DIER & FREQ_CAPTURE_IT_2))
	 {
		 FREQ_CAPTURE_TIM->SR &= (~FREQ_CAPTURE_IT_2 );
		 FREQ_CAPTURE_TIM->DIER &= (uint16_t)~FREQ_CAPTURE_IT_2;
		 FrequencyDataCH2[FrequencyDataCH2counter].capture =FREQ_CAPTURE_TIM->FREQ_CAP_REG_2;
		 FrequencyDataCH2[FrequencyDataCH2counter].impulse_count=FREQ_COUNT_2_TIM->CNT;
		 FREQ_COUNT_2_TIM->CNT=0x0;

		 FrequencyDataCH2counter++;
		 FrequencyDataCH2counter&=(FREQUENCY_DATA_LEN-1);
		 xQueueSendFromISR(xFrequencyMeasureCH2Queue,  &FrequencyDataCH2counter, &xHigherPriorityTaskWoken);
	 }
}



static void FrequencyCH1Measure_Task(void *pvParameters)
{
	uint8_t current_pos=0;
	uint32_t sum_impulse=0;
	uint32_t sum_tick_impulse=0;
	float frequency;

	portBASE_TYPE xStatus;

 	for( ;; )
	{
 		vTaskDelay(FREQ_MEASURE_TIME);
 		FREQ_CAPTURE_TIM->SR &= (~FREQ_CAPTURE_IT_1 );
 		FREQ_CAPTURE_TIM->DIER |= FREQ_CAPTURE_IT_1;

 			xStatus=xQueueReceive(xFrequencyMeasureCH1Queue, &current_pos, FREQ_CAPTURE_PERIOD_0_HZ );
 			if(xStatus==pdPASS)
 			{
				 sum_impulse=FrequencyDataCH1[(current_pos-1)&(FREQUENCY_DATA_LEN-1)].impulse_count;

				 if(FrequencyDataCH1[(current_pos-1)&(FREQUENCY_DATA_LEN-1)].capture>FrequencyDataCH1[(current_pos-2)&(FREQUENCY_DATA_LEN-1)].capture)
				 {
					 sum_tick_impulse=FrequencyDataCH1[(current_pos-1)&(FREQUENCY_DATA_LEN-1)].capture-FrequencyDataCH1[(current_pos-2)&(FREQUENCY_DATA_LEN-1)].capture;
				 }
				 else
				 {
					 sum_tick_impulse=FrequencyDataCH1[(current_pos-2)&(FREQUENCY_DATA_LEN-1)].capture-FrequencyDataCH1[(current_pos-1)&(FREQUENCY_DATA_LEN-1)].capture;
				 }

				 frequency= (float)FREQ_CAPTURE_TIMER_TICK_FREQUENCY*sum_impulse/sum_tick_impulse;
 			}
 			else
 			{
 				frequency=0.0;
 			}
 			xQueueSend( xFrequencyResultQueue[0], &frequency, 0 );
	}
}



static void FrequencyCH2Measure_Task(void *pvParameters)
{
	uint8_t current_pos=0;
	uint32_t sum_impulse=0;
	uint32_t sum_tick_impulse=0;
	float frequency;

	portBASE_TYPE xStatus;

 	for( ;; )
	{
 		vTaskDelay(FREQ_MEASURE_TIME);
 		FREQ_CAPTURE_TIM->SR &= (~FREQ_CAPTURE_IT_2 );
 		FREQ_CAPTURE_TIM->DIER |= FREQ_CAPTURE_IT_2;

		xStatus=xQueueReceive(xFrequencyMeasureCH2Queue, &current_pos, FREQ_CAPTURE_PERIOD_0_HZ );
		if (xStatus==pdPASS)
		{
			 sum_impulse=FrequencyDataCH2[(current_pos-1)&(FREQUENCY_DATA_LEN-1)].impulse_count;

			 if(FrequencyDataCH2[(current_pos-1)&(FREQUENCY_DATA_LEN-1)].capture>FrequencyDataCH2[(current_pos-2)&(FREQUENCY_DATA_LEN-1)].capture)
			 {
				 sum_tick_impulse=FrequencyDataCH2[(current_pos-1)&(FREQUENCY_DATA_LEN-1)].capture-FrequencyDataCH2[(current_pos-2)&(FREQUENCY_DATA_LEN-1)].capture;
			 }
			 else
			 {
				 sum_tick_impulse=FrequencyDataCH2[(current_pos-2)&(FREQUENCY_DATA_LEN-1)].capture-FrequencyDataCH2[(current_pos-1)&(FREQUENCY_DATA_LEN-1)].capture;
			 }

			 frequency= (float)FREQ_CAPTURE_TIMER_TICK_FREQUENCY*sum_impulse/sum_tick_impulse;
		}
		else
		{
			frequency=0.0;
		}
		xQueueSend( xFrequencyResultQueue[1], &frequency, 0 );
	}
}

