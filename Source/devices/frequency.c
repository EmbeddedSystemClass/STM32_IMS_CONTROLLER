#include "frequency.h"
#include "controller.h"
#include "watchdog.h"

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"

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
//#define FREQ_CAPTURE_TIMER_TICK_FREQUENCY	19999592//20000000///12000000
#define FREQ_CAPTURE_PERIOD_0_HZ			1000
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
#define	RCC_FREQ_COUNT_2_GPIO_PORT			RCC_AHB1Periph_GPIOE
#define FREQ_COUNT_2_GPIO_PORT				GPIOE
#define FREQ_COUNT_2_GPIO_PINS				GPIO_Pin_7
#define FREQ_COUNT_2_GPIO_PINSOURCE			GPIO_PinSource7
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
#define FREQ_TCXO_MULTIPLIER				3

#define FREQ_COUNT_PP_OD_SELECT_PORT				GPIOA
#define FREQ_COUNT_1_PP_OD_SELECT_PIN				GPIO_Pin_4
#define FREQ_COUNT_2_PP_OD_SELECT_PIN				GPIO_Pin_3

#define FREQ_COUNT_1_PP	FREQ_COUNT_PP_OD_SELECT_PORT->BSRRL|=FREQ_COUNT_1_PP_OD_SELECT_PIN
#define FREQ_COUNT_1_OD	FREQ_COUNT_PP_OD_SELECT_PORT->BSRRH|=FREQ_COUNT_1_PP_OD_SELECT_PIN

#define FREQ_COUNT_2_PP	FREQ_COUNT_PP_OD_SELECT_PORT->BSRRL|=FREQ_COUNT_2_PP_OD_SELECT_PIN
#define FREQ_COUNT_2_OD	FREQ_COUNT_PP_OD_SELECT_PORT->BSRRH|=FREQ_COUNT_2_PP_OD_SELECT_PIN

#define IMPULSE_SENSOR_PORT					GPIOB
#define IMPULSE_SENSOR_1_1					GPIO_Pin_4
#define IMPULSE_SENSOR_1_2					GPIO_Pin_7
#define IMPULSE_SENSOR_2_1					GPIO_Pin_8
#define IMPULSE_SENSOR_2_2					GPIO_Pin_9

#define IMPULSE_SENSOR_PORT_EXTI  			EXTI_PortSourceGPIOB

#define IMPULSE_SENSOR_1_1_EXTI_Pin_Source 			EXTI_PinSource4
#define IMPULSE_SENSOR_1_2_EXTI_Pin_Source 			EXTI_PinSource7
#define IMPULSE_SENSOR_2_1_EXTI_Pin_Source 			EXTI_PinSource8
#define IMPULSE_SENSOR_2_2_EXTI_Pin_Source 			EXTI_PinSource9

#define IMPULSE_SENSOR_1_1_EXTI 			EXTI_Line4
#define IMPULSE_SENSOR_1_2_EXTI 			EXTI_Line7
#define IMPULSE_SENSOR_2_1_EXTI 			EXTI_Line8
#define IMPULSE_SENSOR_2_2_EXTI 			EXTI_Line9


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
	EXTI_InitTypeDef   EXTI_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_InitStructure;

	RCC_AHB1PeriphClockCmd(/*RCC_FREQ_CAPTURE_GPIO_PORT |*/ RCC_FREQ_COUNT_1_GPIO_PORT/* | RCC_FREQ_COUNT_2_GPIO_PORT*/, ENABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	  /* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

//	RCC_APB1PeriphClockCmd(RCC_FREQ_CAPTURE_TIM, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_FREQ_COUNT_1_TIM, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_FREQ_COUNT_2_TIM, ENABLE);

//	TIM_InitStructure.TIM_Period = FREQ_CAPTURE_TIM_PERIOD;
//	TIM_InitStructure.TIM_Prescaler = 0;
//	TIM_InitStructure.TIM_ClockDivision = 0;
//	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseInit(FREQ_CAPTURE_TIM, &TIM_InitStructure);
//
//	GPIO_InitStructure.GPIO_Pin = FREQ_CAPTURE_GPIO_PINS;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_Init(FREQ_CAPTURE_GPIO_PORT, &GPIO_InitStructure);
//
//	GPIO_PinAFConfig(FREQ_CAPTURE_GPIO_PORT, FREQ_CAPTURE_GPIO_PINSOURCE_1, FREQ_CAPTURE_GPIO_AF );
//	GPIO_PinAFConfig(FREQ_CAPTURE_GPIO_PORT, FREQ_CAPTURE_GPIO_PINSOURCE_2, FREQ_CAPTURE_GPIO_AF );
//
//	TIM_ICInitStructure.TIM_Channel = FREQ_CAPTURE_CHN_1;
//	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
//	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
//	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
//	TIM_ICInitStructure.TIM_ICFilter = FREQ_CAPTURE_IC_FILTER;
//	TIM_ICInit(FREQ_CAPTURE_TIM, &TIM_ICInitStructure);
//
//	TIM_ICInitStructure.TIM_Channel = FREQ_CAPTURE_CHN_2;
//	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
//	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
//	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
//	TIM_ICInitStructure.TIM_ICFilter = FREQ_CAPTURE_IC_FILTER;
//	TIM_ICInit(FREQ_CAPTURE_TIM, &TIM_ICInitStructure);
//
//	TIM_ARRPreloadConfig(FREQ_CAPTURE_TIM, ENABLE);
//
//	NVIC_EnableIRQ(FREQ_CAPTURE_TIM_IRQn);
//	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
//	NVIC_InitStructure.NVIC_IRQChannel = FREQ_CAPTURE_TIM_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//
//	TIM_ITConfig(FREQ_CAPTURE_TIM, FREQ_CAPTURE_IT_1 | FREQ_CAPTURE_IT_2, ENABLE);
//	TIM_ClearFlag(FREQ_CAPTURE_TIM , FREQ_CAPTURE_IT_1 | FREQ_CAPTURE_IT_2 );
//
//	TIM_Cmd(FREQ_CAPTURE_TIM,  ENABLE);

	GPIO_InitStructure.GPIO_Pin = FREQ_COUNT_1_GPIO_PINS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(FREQ_COUNT_1_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(FREQ_COUNT_1_GPIO_PORT, FREQ_COUNT_1_GPIO_PINSOURCE, FREQ_COUNT_1_GPIO_AF );

//	GPIO_InitStructure.GPIO_Pin = FREQ_COUNT_2_GPIO_PINS;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_Init(FREQ_COUNT_2_GPIO_PORT, &GPIO_InitStructure);
//	GPIO_PinAFConfig(FREQ_COUNT_2_GPIO_PORT, FREQ_COUNT_2_GPIO_PINSOURCE, FREQ_COUNT_2_GPIO_AF );

	TIM_TimeBaseStructInit(&TIM_InitStructure);
	TIM_InitStructure.TIM_Prescaler = 0;
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_Period= FREQ_COUNT_1_TIM_PERIOD;
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(FREQ_COUNT_1_TIM, &TIM_InitStructure);
	TIM_ETRClockMode2Config(FREQ_COUNT_1_TIM, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0x00);


    NVIC_InitStructure.NVIC_IRQChannel =  TIM8_UP_TIM13_IRQn  ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(FREQ_COUNT_1_TIM, ENABLE);
    TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);

//	TIM_TimeBaseInit(FREQ_COUNT_2_TIM, &TIM_InitStructure);
//	TIM_ETRClockMode2Config(FREQ_COUNT_2_TIM, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0x00);
//	TIM_Cmd(FREQ_COUNT_2_TIM, ENABLE);

	GPIO_InitStructure.GPIO_Pin = FREQ_COUNT_1_PP_OD_SELECT_PIN | FREQ_COUNT_2_PP_OD_SELECT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(FREQ_COUNT_PP_OD_SELECT_PORT, &GPIO_InitStructure);

	FREQ_COUNT_1_PP;//FREQ_COUNT_1_OD; //
	FREQ_COUNT_2_OD;//FREQ_COUNT_2_OD;


	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = IMPULSE_SENSOR_1_1 | IMPULSE_SENSOR_1_2 | IMPULSE_SENSOR_2_1 | IMPULSE_SENSOR_2_2;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(IMPULSE_SENSOR_PORT, &GPIO_InitStructure);

    SYSCFG_EXTILineConfig(IMPULSE_SENSOR_PORT_EXTI,IMPULSE_SENSOR_1_1_EXTI_Pin_Source);
    SYSCFG_EXTILineConfig(IMPULSE_SENSOR_PORT_EXTI,IMPULSE_SENSOR_1_2_EXTI_Pin_Source);
    SYSCFG_EXTILineConfig(IMPULSE_SENSOR_PORT_EXTI,IMPULSE_SENSOR_2_1_EXTI_Pin_Source);
    SYSCFG_EXTILineConfig(IMPULSE_SENSOR_PORT_EXTI,IMPULSE_SENSOR_2_2_EXTI_Pin_Source);

    EXTI_InitStructure.EXTI_Line = IMPULSE_SENSOR_1_1_EXTI  | IMPULSE_SENSOR_2_1_EXTI ;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_InitStructure.EXTI_Line =   IMPULSE_SENSOR_1_2_EXTI | IMPULSE_SENSOR_2_2_EXTI;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
//	vSemaphoreCreateBinary( xFrequencySemaphore[0] );
//	vSemaphoreCreateBinary( xFrequencySemaphore[1] );

//	xTaskCreate(FrequencyCH1Measure_Task,(signed char*)"Freq CH1",128,NULL, tskIDLE_PRIORITY + 2, NULL);
//	xTaskCreate(FrequencyCH2Measure_Task,(signed char*)"Freq CH2",128,NULL, tskIDLE_PRIORITY + 2, NULL);
}

//portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
//
//void FREQ_CAPTURE_TIM_IRQHandler()
//{
//	 if ((FREQ_CAPTURE_TIM->SR & FREQ_CAPTURE_IT_1) && (FREQ_CAPTURE_TIM->DIER & FREQ_CAPTURE_IT_1))
//	 {
//		 FREQ_CAPTURE_TIM->SR &= (~FREQ_CAPTURE_IT_1 );
//		 FREQ_CAPTURE_TIM->DIER &= (uint16_t)~FREQ_CAPTURE_IT_1;
//		 FrequencyData[0].capture_1=FrequencyData[0].capture_2;
//		 FrequencyData[0].capture_2 =FREQ_CAPTURE_TIM->FREQ_CAP_REG_1;
//		 FrequencyData[0].impulse_count=FREQ_COUNT_1_TIM->CNT;
//		 FREQ_COUNT_1_TIM->CNT=0x0;
//		 xSemaphoreGiveFromISR( xFrequencySemaphore[0], &xHigherPriorityTaskWoken );
//		  if( xHigherPriorityTaskWoken == pdTRUE )
//		  {
//			  portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
//		  }
//	 }
//
//	 if ((FREQ_CAPTURE_TIM->SR & FREQ_CAPTURE_IT_2) && (FREQ_CAPTURE_TIM->DIER & FREQ_CAPTURE_IT_2))
//	 {
//		 FREQ_CAPTURE_TIM->SR &= (~FREQ_CAPTURE_IT_2 );
//		 FREQ_CAPTURE_TIM->DIER &= (uint16_t)~FREQ_CAPTURE_IT_2;
//		 FrequencyData[1].capture_1=FrequencyData[1].capture_2;
//		 FrequencyData[1].capture_2 =FREQ_CAPTURE_TIM->FREQ_CAP_REG_2;
//		 FrequencyData[1].impulse_count=FREQ_COUNT_2_TIM->CNT;
//		 FREQ_COUNT_2_TIM->CNT=0x0;
//		 xSemaphoreGiveFromISR( xFrequencySemaphore[1], &xHigherPriorityTaskWoken );
//		  if( xHigherPriorityTaskWoken == pdTRUE )
//		  {
//			  portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
//		  }
//	 }
//}


//static void FrequencyCH1Measure_Task(void *pvParameters)
//{
//	uint32_t sum_tick_impulse=0;
//	float frequency;
//
// 	for( ;; )
//	{
// 		vTaskDelay(FREQ_MEASURE_TIME);
// 		FREQ_CAPTURE_TIM->SR &= (~FREQ_CAPTURE_IT_1 );
// 		FREQ_CAPTURE_TIM->DIER |= FREQ_CAPTURE_IT_1;
//
//		if ( xSemaphoreTake( xFrequencySemaphore[0], ( portTickType ) FREQ_CAPTURE_PERIOD_0_HZ ) == pdTRUE )
//		{
//			Watchdog_SetTaskStatus(FREQUENCY_CH1_TASK,TASK_ACTIVE);
//			if(FrequencyData[0].capture_2>FrequencyData[0].capture_1)
//			 {
//				 sum_tick_impulse=FrequencyData[0].capture_2-FrequencyData[0].capture_1;
//			 }
//			 else
//			 {
//				 sum_tick_impulse=(FREQ_CAPTURE_TIM_PERIOD-FrequencyData[0].capture_1)+FrequencyData[0].capture_2;
//			 }
//
//
//			 xSemaphoreTake( xSettingsMutex, portMAX_DELAY );
//			 {
//				 frequency= sum_tick_impulse ? (float)stSettings.TCXO_frequency*FREQ_TCXO_MULTIPLIER*FrequencyData[0].impulse_count/sum_tick_impulse : 0;
//			 }
//			 xSemaphoreGive( xSettingsMutex );
//
//			 Watchdog_IncrementCouter(FREQUENCY_CH1_TASK);
//			 Watchdog_SetTaskStatus(FREQUENCY_CH1_TASK,TASK_IDLE);
//		}
//		else
//		{
//			frequency=0.0;
//			FrequencyData[0].impulse_count=FREQ_COUNT_1_TIM->CNT;
//			FREQ_COUNT_1_TIM->CNT=0x0;
//		}
//
//		Watchdog_SetTaskStatus(FREQUENCY_CH1_TASK,TASK_ACTIVE);
//	    xSemaphoreTake( xMeasureDataMutex, portMAX_DELAY );
//	    {
//	    	stMeasureData.frequency[0]=frequency;
//	    	stMeasureData.pulse_counter[0]+=FrequencyData[0].impulse_count;
//	    }
//	    xSemaphoreGive( xMeasureDataMutex );
//		Watchdog_IncrementCouter(FREQUENCY_CH1_TASK);
//		Watchdog_SetTaskStatus(FREQUENCY_CH1_TASK,TASK_IDLE);
//	}
//}
//
//
//static void FrequencyCH2Measure_Task(void *pvParameters)
//{
//	uint32_t sum_tick_impulse=0;
//	float frequency;
//
// 	for( ;; )
//	{
// 		vTaskDelay(FREQ_MEASURE_TIME);
// 		FREQ_CAPTURE_TIM->SR &= (~FREQ_CAPTURE_IT_2 );
// 		FREQ_CAPTURE_TIM->DIER |= FREQ_CAPTURE_IT_2;
//
//		if ( xSemaphoreTake( xFrequencySemaphore[1], ( portTickType ) FREQ_CAPTURE_PERIOD_0_HZ ) == pdTRUE )
//		{
//			 Watchdog_SetTaskStatus(FREQUENCY_CH2_TASK,TASK_ACTIVE);
//			 if(FrequencyData[1].capture_2>FrequencyData[1].capture_1)
//			 {
//				 sum_tick_impulse=FrequencyData[1].capture_2-FrequencyData[1].capture_1;
//			 }
//			 else
//			 {
//				 sum_tick_impulse= (FREQ_CAPTURE_TIM_PERIOD-FrequencyData[1].capture_1)+FrequencyData[1].capture_2;
//			 }
//
//			 xSemaphoreTake( xSettingsMutex, portMAX_DELAY );
//			 {
//			     frequency= sum_tick_impulse ? (float)stSettings.TCXO_frequency*FREQ_TCXO_MULTIPLIER*FrequencyData[1].impulse_count/sum_tick_impulse : 0;
//			 }
//			 xSemaphoreGive( xSettingsMutex );
//
//			 Watchdog_IncrementCouter(FREQUENCY_CH2_TASK);
//			 Watchdog_SetTaskStatus(FREQUENCY_CH2_TASK,TASK_IDLE);
//		}
//		else
//		{
//			frequency=0.0;
//			FrequencyData[1].impulse_count=FREQ_COUNT_2_TIM->CNT;
//			FREQ_COUNT_2_TIM->CNT=0x0;
//		}
//
//		Watchdog_SetTaskStatus(FREQUENCY_CH2_TASK,TASK_ACTIVE);
//	    xSemaphoreTake( xMeasureDataMutex, portMAX_DELAY );
//	    {
//	    	stMeasureData.frequency[1]=frequency;
//	    	stMeasureData.pulse_counter[1]+=FrequencyData[1].impulse_count;
//	    }
//	    xSemaphoreGive( xMeasureDataMutex );
//		Watchdog_IncrementCouter(FREQUENCY_CH2_TASK);
//		Watchdog_SetTaskStatus(FREQUENCY_CH2_TASK,TASK_IDLE);
//	}
//}


void EXTI4_IRQHandler(void)
{
  if(EXTI_GetITStatus(IMPULSE_SENSOR_1_1_EXTI) != RESET)
  {

    EXTI_ClearITPendingBit(IMPULSE_SENSOR_1_1_EXTI);
  }
}

void EXTI9_5_IRQHandler(void)
{
  if(EXTI_GetITStatus(IMPULSE_SENSOR_1_2_EXTI) != RESET)
  {

    EXTI_ClearITPendingBit(IMPULSE_SENSOR_1_2_EXTI);
  }

  if(EXTI_GetITStatus(IMPULSE_SENSOR_2_1_EXTI) != RESET)
  {

    EXTI_ClearITPendingBit(IMPULSE_SENSOR_2_1_EXTI);
  }

  if(EXTI_GetITStatus(IMPULSE_SENSOR_2_2_EXTI) != RESET)
  {

    EXTI_ClearITPendingBit(IMPULSE_SENSOR_2_2_EXTI);
  }
}

void  TIM8_UP_TIM13_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET)
    {
    	TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
    }
}
