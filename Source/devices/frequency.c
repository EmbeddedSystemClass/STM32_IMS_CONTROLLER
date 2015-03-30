#include "frequency.h"
#include "controller.h"

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "misc.h"


typedef struct
{
	uint32_t	capture_1;
	uint32_t	capture_2;
	uint32_t impulse_count;
}stFrequencyData;

xSemaphoreHandle xFrequencySemaphore[2];
static volatile stFrequencyData FrequencyData[2];

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

	vSemaphoreCreateBinary( xFrequencySemaphore[0] );
	vSemaphoreCreateBinary( xFrequencySemaphore[1] );

	xTaskCreate(FrequencyCH1Measure_Task,(signed char*)"Freq CH1",64,NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(FrequencyCH2Measure_Task,(signed char*)"Freq CH2",64,NULL, tskIDLE_PRIORITY + 1, NULL);
}

portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

void FREQ_CAPTURE_TIM_IRQHandler()
{
	 if ((FREQ_CAPTURE_TIM->SR & FREQ_CAPTURE_IT_1) && (FREQ_CAPTURE_TIM->DIER & FREQ_CAPTURE_IT_1))
	 {
		 FREQ_CAPTURE_TIM->SR &= (~FREQ_CAPTURE_IT_1 );
		 FREQ_CAPTURE_TIM->DIER &= (uint16_t)~FREQ_CAPTURE_IT_1;
		 FrequencyData[0].capture_1=FrequencyData[0].capture_2;
		 FrequencyData[0].capture_2 =FREQ_CAPTURE_TIM->FREQ_CAP_REG_1;
		 FrequencyData[0].impulse_count=FREQ_COUNT_1_TIM->CNT;
		 FREQ_COUNT_1_TIM->CNT=0x0;
		 xSemaphoreGiveFromISR( xFrequencySemaphore[0], &xHigherPriorityTaskWoken );
	 }

	 if ((FREQ_CAPTURE_TIM->SR & FREQ_CAPTURE_IT_2) && (FREQ_CAPTURE_TIM->DIER & FREQ_CAPTURE_IT_2))
	 {
		 FREQ_CAPTURE_TIM->SR &= (~FREQ_CAPTURE_IT_2 );
		 FREQ_CAPTURE_TIM->DIER &= (uint16_t)~FREQ_CAPTURE_IT_2;
		 FrequencyData[1].capture_1=FrequencyData[1].capture_2;
		 FrequencyData[1].capture_2 =FREQ_CAPTURE_TIM->FREQ_CAP_REG_2;
		 FrequencyData[1].impulse_count=FREQ_COUNT_2_TIM->CNT;
		 FREQ_COUNT_2_TIM->CNT=0x0;
		 xSemaphoreGiveFromISR( xFrequencySemaphore[1], &xHigherPriorityTaskWoken );
	 }
}

static void FrequencyCH1Measure_Task(void *pvParameters)
{
	uint32_t sum_tick_impulse=0;
	float frequency;

 	for( ;; )
	{
 		vTaskDelay(FREQ_MEASURE_TIME);
 		FREQ_CAPTURE_TIM->SR &= (~FREQ_CAPTURE_IT_1 );
 		FREQ_CAPTURE_TIM->DIER |= FREQ_CAPTURE_IT_1;

		if ( xSemaphoreTake( xFrequencySemaphore[0], ( portTickType ) FREQ_CAPTURE_PERIOD_0_HZ ) == pdTRUE )
		{
			 if(FrequencyData[0].capture_2>FrequencyData[0].capture_1)
			 {
				 sum_tick_impulse=FrequencyData[0].capture_2-FrequencyData[0].capture_1;
			 }
			 else
			 {
				 sum_tick_impulse=FrequencyData[0].capture_1-FrequencyData[0].capture_2;
			 }

			 frequency= (float)FREQ_CAPTURE_TIMER_TICK_FREQUENCY*FrequencyData[0].impulse_count/sum_tick_impulse;
		}
		else
		{
			frequency=0.0;
		}

	    xSemaphoreTake( xFrequencyMutex[0], portMAX_DELAY );
	    {
	    	stMeasureData.frequency[0]=frequency;
	    }
	    xSemaphoreGive( xFrequencyMutex[0] );
	}
}


static void FrequencyCH2Measure_Task(void *pvParameters)
{
	uint32_t sum_tick_impulse=0;
	float frequency;

 	for( ;; )
	{
 		vTaskDelay(FREQ_MEASURE_TIME);
 		FREQ_CAPTURE_TIM->SR &= (~FREQ_CAPTURE_IT_2 );
 		FREQ_CAPTURE_TIM->DIER |= FREQ_CAPTURE_IT_2;

		if ( xSemaphoreTake( xFrequencySemaphore[1], ( portTickType ) FREQ_CAPTURE_PERIOD_0_HZ ) == pdTRUE )
		{
			 if(FrequencyData[1].capture_2>FrequencyData[1].capture_1)
			 {
				 sum_tick_impulse=FrequencyData[1].capture_2-FrequencyData[1].capture_1;
			 }
			 else
			 {
				 sum_tick_impulse=FrequencyData[1].capture_1-FrequencyData[1].capture_2;
			 }

			 frequency= (float)FREQ_CAPTURE_TIMER_TICK_FREQUENCY*FrequencyData[1].impulse_count/sum_tick_impulse;
		}
		else
		{
			frequency=0.0;
		}

	    xSemaphoreTake( xFrequencyMutex[1], portMAX_DELAY );
	    {
	    	stMeasureData.frequency[1]=frequency;
	    }
	    xSemaphoreGive( xFrequencyMutex[1] );
	}
}

