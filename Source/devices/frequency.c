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


#define IMPULSE_COUNT_1_TIM						TIM8
#define IMPULSE_COUNT_1_GPIO_AF 				GPIO_AF_TIM8
#define RCC_IMPULSE_COUNT_1_TIM 				RCC_APB2Periph_TIM8
#define	RCC_IMPULSE_COUNT_1_GPIO_PORT			RCC_AHB1Periph_GPIOA
#define IMPULSE_COUNT_1_GPIO_PORT				GPIOA
#define IMPULSE_COUNT_1_GPIO_PINS				GPIO_Pin_0
#define IMPULSE_COUNT_1_GPIO_PINSOURCE			GPIO_PinSource0
#define IMPULSE_COUNT_1_TIM_PERIOD				(65535)
#define IMPULSE_COUNT_1_IRQHandler				TIM8_UP_TIM13_IRQHandler

#define IMPULSE_COUNT_PP_OD_SELECT_PORT	  	GPIOA
#define IMPULSE_COUNT_1_PP_OD_SELECT_PIN	GPIO_Pin_4
#define IMPULSE_COUNT_2_PP_OD_SELECT_PIN	GPIO_Pin_3

#define IMPULSE_COUNT_1_PP					IMPULSE_COUNT_PP_OD_SELECT_PORT->BSRRL|=IMPULSE_COUNT_1_PP_OD_SELECT_PIN
#define IMPULSE_COUNT_1_OD					IMPULSE_COUNT_PP_OD_SELECT_PORT->BSRRH|=IMPULSE_COUNT_1_PP_OD_SELECT_PIN

#define IMPULSE_COUNT_2_PP					IMPULSE_COUNT_PP_OD_SELECT_PORT->BSRRL|=IMPULSE_COUNT_2_PP_OD_SELECT_PIN
#define IMPULSE_COUNT_2_OD					IMPULSE_COUNT_PP_OD_SELECT_PORT->BSRRH|=IMPULSE_COUNT_2_PP_OD_SELECT_PIN

#define IMPULSE_SENSOR_PORT					GPIOB
#define IMPULSE_SENSOR_1_1					GPIO_Pin_4
#define IMPULSE_SENSOR_1_2					GPIO_Pin_7
#define IMPULSE_SENSOR_2_1					GPIO_Pin_8
#define IMPULSE_SENSOR_2_2					GPIO_Pin_9

#define IMPULSE_SENSOR_PORT_EXTI  			EXTI_PortSourceGPIOB

#define IMPULSE_SENSOR_1_1_EXTI_Pin_Source 	EXTI_PinSource4
#define IMPULSE_SENSOR_1_2_EXTI_Pin_Source 	EXTI_PinSource7
#define IMPULSE_SENSOR_2_1_EXTI_Pin_Source 	EXTI_PinSource8
#define IMPULSE_SENSOR_2_2_EXTI_Pin_Source 	EXTI_PinSource9

#define IMPULSE_SENSOR_1_1_EXTI 			EXTI_Line4
#define IMPULSE_SENSOR_1_2_EXTI 			EXTI_Line7
#define IMPULSE_SENSOR_2_1_EXTI 			EXTI_Line8
#define IMPULSE_SENSOR_2_2_EXTI 			EXTI_Line9

#define IMPULSE_SENSORS_IRQHandler_1		EXTI4_IRQHandler
#define IMPULSE_SENSORS_IRQHandler_2		EXTI9_5_IRQHandler

#define LINE1_ANTIBOUNCE_TIMER				TIM11
#define LINE2_ANTIBOUNCE_TIMER				TIM14

#define LINE1_ANTIBOUNCE_TIMER_IRQHandler 	TIM1_TRG_COM_TIM11_IRQHandler
#define LINE2_ANTIBOUNCE_TIMER_IRQHandler	TIM8_TRG_COM_TIM14_IRQHandler

#define IMPULSE_FAST_TIM					TIM2
#define IMPULSE_FAST_GPIO_AF 				GPIO_AF_TIM2
#define RCC_IMPULSE_FAST_TIM 				RCC_APB1Periph_TIM2
#define	RCC_IMPULSE_FAST_GPIO_PORT			RCC_AHB1Periph_GPIOA
#define IMPULSE_FAST_GPIO_PORT				GPIOA
#define IMPULSE_FAST_TIM_IRQn				TIM2_IRQn
#define IMPULSE_FAST_GPIO_PINS				GPIO_Pin_1 /*| GPIO_Pin_2*/
#define IMPULSE_FAST_GPIO_PINSOURCE_1		GPIO_PinSource1
#define IMPULSE_FAST_GPIO_PINSOURCE_2		GPIO_PinSource2
#define IMPULSE_FAST_TIM_IRQHandler 		TIM2_IRQHandler
#define IMPULSE_FAST_TIM_PERIOD				(0xFFFFFFFF)
#define IMPULSE_FAST_TIM_FREQ				30000000


//#define IMPULSE_FAST_TIMER_TICK_FREQUENCY	19999592//20000000///12000000
#define IMPULSE_FAST_PERIOD_0_HZ			1000
#define IMPULSE_FAST_IC_FILTER				5
#define IMPULSE_FAST_CHN_1					TIM_Channel_2
#define IMPULSE_FAST_CHN_2					TIM_Channel_3
#define IMPULSE_FAST_IT_1					TIM_IT_CC2
#define IMPULSE_FAST_IT_2					TIM_IT_CC3
#define IMPULSE_FAST_REG_1					CCR2
#define IMPULSE_FAST_REG_2					CCR3

#define SENSOR_EVENT_LEVEL					0

#define FREQ_CAPTURE_PERIOD_0_HZ			1000
#define FREQ_MEASURE_TIME					250
#define FREQ_TCXO_MULTIPLIER				3
#define FREQ_CAPTURE_TIM_PERIOD				(0xFFFFFFFF)

enum
{
	IMPULSE_SENSOR_1_1_EVENT=0,
	IMPULSE_SENSOR_1_2_EVENT,
	IMPULSE_SENSOR_2_1_EVENT,
	IMPULSE_SENSOR_2_2_EVENT,
	IMPULSE_SENSOR_EVENT_NONE,
};

/*typedef enum
{
	MEASURE_DIRECT_1_2=0,
	MEASURE_DIRECT_2_1,
}enMeasureDirect;

enMeasureDirect line_1_MeasureDirect,line_2_MeasureDirect;*/

typedef struct
{
	uint64_t start_value;
	uint64_t stop_value;
}stImpulseCounter;

uint64_t reload_counter=0, reload_fast_tim=0;
stImpulseCounter Line1_ImpulseCounter, Line2_ImpulseCounter, Line1_SensorTimer, Line2_SensorTimer, Line1_ImpulseTimer, Line2_ImpulseTimer;
uint32_t  exti_base_addr = (uint32_t)EXTI_BASE+EXTI_Mode_Interrupt;
uint8_t line_1_event, line_2_event;



xSemaphoreHandle xFrequencySemaphore;
typedef enum
{
	FREQ_MEASURE_NONE,
	FREQ_MEASURE_START,
	FREQ_MEASURE_COMPLETE,
}enFreqMeasure;
enFreqMeasure flagFreqMeasure=FREQ_MEASURE_NONE;

typedef struct
{
	uint32_t	capture_1;
	uint32_t	capture_2;
	uint32_t impulse_count_1;
	uint32_t impulse_count_2;
}stFrequencyData;

static volatile stFrequencyData FrequencyData;


static void FrequencyCH1Measure_Task(void *pvParameters);

void ImpulseMeasureInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_InitStructure;

//enable clock for all devices
	RCC_AHB1PeriphClockCmd( RCC_IMPULSE_COUNT_1_GPIO_PORT, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_IMPULSE_FAST_GPIO_PORT, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_IMPULSE_COUNT_1_TIM, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_IMPULSE_FAST_GPIO_PORT, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_IMPULSE_FAST_TIM,ENABLE);

//impulse counter init
	GPIO_InitStructure.GPIO_Pin = IMPULSE_COUNT_1_GPIO_PINS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(IMPULSE_COUNT_1_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(IMPULSE_COUNT_1_GPIO_PORT, IMPULSE_COUNT_1_GPIO_PINSOURCE, IMPULSE_COUNT_1_GPIO_AF );

	TIM_TimeBaseStructInit(&TIM_InitStructure);
	TIM_InitStructure.TIM_Prescaler = 0;
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_Period= IMPULSE_COUNT_1_TIM_PERIOD;
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(IMPULSE_COUNT_1_TIM, &TIM_InitStructure);
	TIM_ETRClockMode2Config(IMPULSE_COUNT_1_TIM, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0x00);

    NVIC_InitStructure.NVIC_IRQChannel =  TIM8_UP_TIM13_IRQn  ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ClearITPendingBit(IMPULSE_COUNT_1_TIM, TIM_IT_Update);
    TIM_Cmd(IMPULSE_COUNT_1_TIM, ENABLE);
    TIM_ITConfig(IMPULSE_COUNT_1_TIM, TIM_IT_Update, ENABLE);

	GPIO_InitStructure.GPIO_Pin = IMPULSE_COUNT_1_PP_OD_SELECT_PIN | IMPULSE_COUNT_2_PP_OD_SELECT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(IMPULSE_COUNT_PP_OD_SELECT_PORT, &GPIO_InitStructure);

	IMPULSE_COUNT_1_PP;//IMPULSE_COUNT_1_OD; //
	IMPULSE_COUNT_2_OD;//IMPULSE_COUNT_2_OD;

//impulse sensors init
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
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_InitStructure.EXTI_Line =   IMPULSE_SENSOR_1_2_EXTI | IMPULSE_SENSOR_2_2_EXTI;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_1_1_EXTI;
	*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_1_2_EXTI;
	*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_2_1_EXTI;
	*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_2_2_EXTI;

//antibounce timers init
	TIM_TimeBaseStructInit(&TIM_InitStructure);
	TIM_InitStructure.TIM_Prescaler = 60;
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_Period= 0xFFFF;
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(LINE1_ANTIBOUNCE_TIMER, &TIM_InitStructure);

	TIM_InitStructure.TIM_Prescaler = 30;

	TIM_TimeBaseInit(LINE2_ANTIBOUNCE_TIMER, &TIM_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel =  TIM1_TRG_COM_TIM11_IRQn  ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel =  TIM8_TRG_COM_TIM14_IRQn  ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ClearITPendingBit(LINE1_ANTIBOUNCE_TIMER, TIM_IT_Update);
    TIM_ITConfig(LINE1_ANTIBOUNCE_TIMER, TIM_IT_Update, ENABLE);

    TIM_ClearITPendingBit(LINE2_ANTIBOUNCE_TIMER, TIM_IT_Update);
    TIM_ITConfig(LINE2_ANTIBOUNCE_TIMER, TIM_IT_Update, ENABLE);

//fast timer init
	TIM_TimeBaseStructInit(&TIM_InitStructure);
	TIM_InitStructure.TIM_Prescaler = 0;
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_Period= IMPULSE_FAST_TIM_PERIOD;
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(IMPULSE_FAST_TIM, &TIM_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel =  IMPULSE_FAST_TIM_IRQn  ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ClearITPendingBit(IMPULSE_FAST_TIM, TIM_IT_Update);
    TIM_ITConfig(IMPULSE_FAST_TIM, TIM_IT_Update, ENABLE);

	GPIO_InitStructure.GPIO_Pin = IMPULSE_FAST_GPIO_PINS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(IMPULSE_FAST_GPIO_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(IMPULSE_FAST_GPIO_PORT, IMPULSE_FAST_GPIO_PINSOURCE_1, IMPULSE_FAST_GPIO_AF );

	TIM_ICInitStructure.TIM_Channel = IMPULSE_FAST_CHN_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = IMPULSE_FAST_IC_FILTER;
	TIM_ICInit(IMPULSE_FAST_TIM, &TIM_ICInitStructure);

	TIM_ARRPreloadConfig(IMPULSE_FAST_TIM, ENABLE);

	TIM_ITConfig(IMPULSE_FAST_TIM, IMPULSE_FAST_IT_1, ENABLE);
	TIM_ClearFlag(IMPULSE_FAST_TIM , IMPULSE_FAST_IT_1);


	TIM_Cmd(IMPULSE_FAST_TIM, ENABLE);

	//---------------------------------
    stMeasureData.pulse_line_measure_state[0]=MEASURE_UNCERTAIN;
    stMeasureData.pulse_line_measure_state[1]=MEASURE_UNCERTAIN;
    line_1_event=IMPULSE_SENSOR_EVENT_NONE;
    line_2_event=IMPULSE_SENSOR_EVENT_NONE;

 /*   line_1_MeasureDirect=MEASURE_DIRECT_1_2;
    line_2_MeasureDirect=MEASURE_DIRECT_1_2;*/

    Impulse_SetAntiBounceDelay(1);
    ImpulseLine1_StartMeasure();//test measure
    //------------------------------------

	vSemaphoreCreateBinary( xFrequencySemaphore );
	xTaskCreate(FrequencyCH1Measure_Task,(signed char*)"Freq measure",128,NULL, tskIDLE_PRIORITY + 2, NULL);

}

void ImpulseLine1_StartMeasure(void)//переделать по обоим датчикам
{
	if((stMeasureData.pulse_line_measure_state[0]==MEASURE_FINISHED)||(stMeasureData.pulse_line_measure_state[0]==MEASURE_UNCERTAIN))
	{
		*(__IO uint32_t *) exti_base_addr |= IMPULSE_SENSOR_1_1_EXTI;
		//*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_1_2_EXTI;
		*(__IO uint32_t *) exti_base_addr |= IMPULSE_SENSOR_1_2_EXTI;
		stMeasureData.pulse_line_measure_state[0]=MEASURE_STARTED;
	    line_1_event=IMPULSE_SENSOR_EVENT_NONE;
	}
}

void ImpulseLine2_StartMeasure(void)//переделать по обоим датчикам
{
	if((stMeasureData.pulse_line_measure_state[1]==MEASURE_FINISHED)||(stMeasureData.pulse_line_measure_state[1]==MEASURE_UNCERTAIN))
	{
		*(__IO uint32_t *) exti_base_addr |= IMPULSE_SENSOR_2_1_EXTI;
		//*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_2_2_EXTI;
		*(__IO uint32_t *) exti_base_addr |= IMPULSE_SENSOR_2_2_EXTI;
		stMeasureData.pulse_line_measure_state[1]=MEASURE_STARTED;
	    line_2_event=IMPULSE_SENSOR_EVENT_NONE;
	}
}

void ImpulseLine1_EmergencyStopMeasure(void)
{
	*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_1_1_EXTI;
	*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_1_2_EXTI;
	stMeasureData.pulse_line_measure_state[0]=MEASURE_UNCERTAIN;
}

void ImpulseLine2_EmergencyStopMeasure(void)
{
	*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_2_1_EXTI;
	*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_2_2_EXTI;
	stMeasureData.pulse_line_measure_state[1]=MEASURE_UNCERTAIN;
}

void Impulse_SetAntiBounceDelay(uint16_t time_us)
{
	if(time_us==0)
	{
		time_us=1;
	}

	if(time_us>1000)
	{
		time_us=1000;
	}

	TIM_SetAutoreload(LINE1_ANTIBOUNCE_TIMER,time_us);
	TIM_SetAutoreload(LINE2_ANTIBOUNCE_TIMER,time_us);
}

/*
 * Impulse sensors IRQs
 */
void IMPULSE_SENSORS_IRQHandler_1(void)
{
  if(EXTI_GetITStatus(IMPULSE_SENSOR_1_1_EXTI) != RESET)
  {
	  	LINE1_ANTIBOUNCE_TIMER->CNT=0;
	    TIM_Cmd(LINE1_ANTIBOUNCE_TIMER, ENABLE);
		EXTI_ClearITPendingBit(IMPULSE_SENSOR_1_1_EXTI);
		*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_1_1_EXTI;
		line_1_event=IMPULSE_SENSOR_1_1_EVENT;
  }
}

void IMPULSE_SENSORS_IRQHandler_2(void)
{
  if(EXTI_GetITStatus(IMPULSE_SENSOR_1_2_EXTI) != RESET)
  {
	    LINE1_ANTIBOUNCE_TIMER->CNT=0;
	  	TIM_Cmd(LINE1_ANTIBOUNCE_TIMER, ENABLE);
		EXTI_ClearITPendingBit(IMPULSE_SENSOR_1_2_EXTI);
		*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_1_2_EXTI;
		line_1_event=IMPULSE_SENSOR_1_2_EVENT;
  }

  if(EXTI_GetITStatus(IMPULSE_SENSOR_2_1_EXTI) != RESET)
  {
	  	LINE2_ANTIBOUNCE_TIMER->CNT=0;
	    TIM_Cmd(LINE2_ANTIBOUNCE_TIMER, ENABLE);
		EXTI_ClearITPendingBit(IMPULSE_SENSOR_2_1_EXTI);
		*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_2_1_EXTI;
		line_2_event=IMPULSE_SENSOR_2_1_EVENT;
  }

  if(EXTI_GetITStatus(IMPULSE_SENSOR_2_2_EXTI) != RESET)
  {
	  	LINE2_ANTIBOUNCE_TIMER->CNT=0;
	    TIM_Cmd(LINE2_ANTIBOUNCE_TIMER, ENABLE);
		EXTI_ClearITPendingBit(IMPULSE_SENSOR_2_2_EXTI);
		*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_2_2_EXTI;
		line_2_event=IMPULSE_SENSOR_2_2_EVENT;
  }
}

/*
 * Impulse counter IRQ
 */
void  IMPULSE_COUNT_1_IRQHandler(void)
{
    if (TIM_GetITStatus(IMPULSE_COUNT_1_TIM, TIM_IT_Update) != RESET)
    {
    	reload_counter+=0x10000;
    	TIM_ClearITPendingBit(IMPULSE_COUNT_1_TIM, TIM_IT_Update);
    }
}

/*
 * Antibounce delay IRQs
 */
void  LINE1_ANTIBOUNCE_TIMER_IRQHandler(void)//delay_1
{
    if (TIM_GetITStatus(LINE1_ANTIBOUNCE_TIMER, TIM_IT_Update) != RESET)
    {
    	TIM_Cmd(LINE1_ANTIBOUNCE_TIMER, DISABLE);
    	if(line_1_event==IMPULSE_SENSOR_1_1_EVENT)
    	{
    		if(GPIO_ReadInputDataBit(IMPULSE_SENSOR_PORT,IMPULSE_SENSOR_1_1)==SENSOR_EVENT_LEVEL)//level ok
    		{
				Line1_ImpulseCounter.start_value=reload_counter+IMPULSE_COUNT_1_TIM->CNT;
				Line1_SensorTimer.start_value=reload_fast_tim+IMPULSE_FAST_TIM->CNT;
				*(__IO uint32_t *) exti_base_addr |= IMPULSE_SENSOR_1_2_EXTI;
				TIM_ITConfig(IMPULSE_FAST_TIM, IMPULSE_FAST_IT_1, ENABLE);
				stMeasureData.pulse_line_measure_state[0]=MEASURE_IN_PROCESS;
    		}
    		else
    		{
    			*(__IO uint32_t *) exti_base_addr |= IMPULSE_SENSOR_1_1_EXTI;
    		}
    	}
    	else if(line_1_event==IMPULSE_SENSOR_1_2_EVENT)
    	{
    		if(GPIO_ReadInputDataBit(IMPULSE_SENSOR_PORT,IMPULSE_SENSOR_1_2)==SENSOR_EVENT_LEVEL)//level ok
    		{

    			Line1_SensorTimer.stop_value=reload_fast_tim+IMPULSE_FAST_TIM->CNT;

    			if(Line1_SensorTimer.stop_value>=Line1_SensorTimer.start_value)
    			{
    				stMeasureData.sensor_tim_value[0]=Line1_SensorTimer.stop_value-Line1_SensorTimer.start_value;
    			}
    			else
    			{
    				stMeasureData.sensor_tim_value[0]=Line1_SensorTimer.start_value-Line1_SensorTimer.stop_value;
    			}
    			TIM_ITConfig(IMPULSE_FAST_TIM, IMPULSE_FAST_IT_1, ENABLE);

    		}
    		else
    		{
    			*(__IO uint32_t *) exti_base_addr |= IMPULSE_SENSOR_1_2_EXTI;
    		}
    	}
    	TIM_ClearITPendingBit(LINE1_ANTIBOUNCE_TIMER, TIM_IT_Update);
    }
}

void  LINE2_ANTIBOUNCE_TIMER_IRQHandler(void)//delay_2
{
    if (TIM_GetITStatus(LINE2_ANTIBOUNCE_TIMER, TIM_IT_Update) != RESET)
    {
    	TIM_Cmd(LINE2_ANTIBOUNCE_TIMER, DISABLE);
    	if(line_2_event==IMPULSE_SENSOR_2_1_EVENT)
    	{
    		if(GPIO_ReadInputDataBit(IMPULSE_SENSOR_PORT,IMPULSE_SENSOR_1_2)==SENSOR_EVENT_LEVEL)//level ok
    		{
				Line2_ImpulseCounter.start_value=reload_counter+IMPULSE_COUNT_1_TIM->CNT;
				Line2_SensorTimer.start_value=reload_fast_tim+IMPULSE_FAST_TIM->CNT;
				*(__IO uint32_t *) exti_base_addr |= IMPULSE_SENSOR_2_2_EXTI;
				TIM_ITConfig(IMPULSE_FAST_TIM, IMPULSE_FAST_IT_1, ENABLE);
				stMeasureData.pulse_line_measure_state[1]=MEASURE_IN_PROCESS;
    		}
    		else
    		{
    			*(__IO uint32_t *) exti_base_addr |= IMPULSE_SENSOR_2_1_EXTI;
    		}
    	}
    	else if(line_2_event==IMPULSE_SENSOR_2_2_EVENT)
    	{
    		if(GPIO_ReadInputDataBit(IMPULSE_SENSOR_PORT,IMPULSE_SENSOR_2_2)==SENSOR_EVENT_LEVEL)//level ok
    		{
    			Line2_SensorTimer.stop_value=reload_fast_tim+IMPULSE_FAST_TIM->CNT;

    			if(Line2_SensorTimer.stop_value>=Line2_SensorTimer.start_value)
    			{
    				stMeasureData.sensor_tim_value[1]=Line2_SensorTimer.stop_value-Line2_SensorTimer.start_value;
    			}
    			else
    			{
    				stMeasureData.sensor_tim_value[1]=Line2_SensorTimer.start_value-Line2_SensorTimer.stop_value;
    			}
    			TIM_ITConfig(IMPULSE_FAST_TIM, IMPULSE_FAST_IT_1, ENABLE);
    		}
    		else
    		{
    			*(__IO uint32_t *) exti_base_addr |= IMPULSE_SENSOR_2_2_EXTI;
    		}
    	}
    	TIM_ClearITPendingBit(LINE2_ANTIBOUNCE_TIMER, TIM_IT_Update);
    }
}

/*
 * Fast timer interrupt
 */
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
void  IMPULSE_FAST_TIM_IRQHandler(void)
{
    if (TIM_GetITStatus(IMPULSE_FAST_TIM, TIM_IT_Update) != RESET)
    {
    	reload_fast_tim+=(IMPULSE_FAST_TIM_PERIOD+1);
    	TIM_ClearITPendingBit(IMPULSE_FAST_TIM, TIM_IT_Update);
    }

    if (TIM_GetITStatus(IMPULSE_FAST_TIM, IMPULSE_FAST_IT_1) != RESET)
    {
    	if(line_1_event==IMPULSE_SENSOR_1_1_EVENT)
    	{
    		Line1_ImpulseTimer.start_value=IMPULSE_FAST_TIM->IMPULSE_FAST_REG_1;
    		line_1_event=IMPULSE_SENSOR_EVENT_NONE;
    	}
    	else if(line_1_event==IMPULSE_SENSOR_1_2_EVENT)
    	{
    		Line1_ImpulseTimer.stop_value=IMPULSE_FAST_TIM->IMPULSE_FAST_REG_1;
    		Line1_ImpulseCounter.stop_value=reload_counter+IMPULSE_COUNT_1_TIM->CNT;
    		line_1_event=IMPULSE_SENSOR_EVENT_NONE;
    		stMeasureData.pulse_line_measure_state[0]=MEASURE_FINISHED;

			if(Line1_ImpulseTimer.stop_value>=Line1_ImpulseTimer.start_value)
			{
				stMeasureData.impulse_tim_value[0]=Line1_ImpulseTimer.stop_value-Line1_ImpulseTimer.start_value;
			}
			else
			{
				stMeasureData.impulse_tim_value[0]=Line1_ImpulseTimer.start_value-Line1_ImpulseTimer.stop_value;
			}

			if(Line1_ImpulseCounter.stop_value>=Line1_ImpulseCounter.start_value)
			{
				stMeasureData.pulse_counter[0]=Line1_ImpulseCounter.stop_value-Line1_ImpulseCounter.start_value;
			}
			else
			{
				stMeasureData.pulse_counter[0]=Line1_ImpulseCounter.start_value-Line1_ImpulseCounter.stop_value;
			}
    	}

    	if(line_2_event==IMPULSE_SENSOR_2_1_EVENT)
    	{
    		Line2_ImpulseTimer.start_value=IMPULSE_FAST_TIM->IMPULSE_FAST_REG_1;
    		line_2_event=IMPULSE_SENSOR_EVENT_NONE;
    	}
    	else if(line_2_event==IMPULSE_SENSOR_2_2_EVENT)
    	{
    		Line2_ImpulseTimer.stop_value=IMPULSE_FAST_TIM->IMPULSE_FAST_REG_1;
    		Line2_ImpulseCounter.stop_value=reload_counter+IMPULSE_COUNT_1_TIM->CNT;
    		line_2_event=IMPULSE_SENSOR_EVENT_NONE;
			stMeasureData.pulse_line_measure_state[1]=MEASURE_FINISHED;

			if(Line2_ImpulseTimer.stop_value>=Line2_ImpulseTimer.start_value)
			{
				stMeasureData.impulse_tim_value[1]=Line2_ImpulseTimer.stop_value-Line2_ImpulseTimer.start_value;
			}
			else
			{
				stMeasureData.impulse_tim_value[1]=Line2_ImpulseTimer.start_value-Line2_ImpulseTimer.stop_value;
			}

			if(Line2_ImpulseCounter.stop_value>=Line2_ImpulseCounter.start_value)
			{
				stMeasureData.pulse_counter[1]=Line2_ImpulseCounter.stop_value-Line2_ImpulseCounter.start_value;
			}
			else
			{
				stMeasureData.pulse_counter[1]=Line2_ImpulseCounter.start_value-Line2_ImpulseCounter.stop_value;
			}
    	}

    	if(flagFreqMeasure==FREQ_MEASURE_START)
    	{
			  FrequencyData.capture_1=FrequencyData.capture_2;
			  FrequencyData.capture_2 =IMPULSE_FAST_TIM->IMPULSE_FAST_REG_1;
			  FrequencyData.impulse_count_1=FrequencyData.impulse_count_2;
			  FrequencyData.impulse_count_2=IMPULSE_COUNT_1_TIM->CNT;

    		  flagFreqMeasure==FREQ_MEASURE_COMPLETE;
    		  xSemaphoreGiveFromISR( xFrequencySemaphore, &xHigherPriorityTaskWoken );
			  if( xHigherPriorityTaskWoken == pdTRUE )
			  {
				  portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
			  }
    	}

    	TIM_ITConfig(IMPULSE_FAST_TIM, IMPULSE_FAST_IT_1, DISABLE);
    	TIM_ClearITPendingBit(IMPULSE_FAST_TIM, IMPULSE_FAST_IT_1);
    }
}


static void FrequencyCH1Measure_Task(void *pvParameters)
{
	uint32_t sum_tick_impulse=0, sum_impulse=0;
	float frequency;

 	for( ;; )
	{
 		vTaskDelay(FREQ_MEASURE_TIME);
 		flagFreqMeasure=FREQ_MEASURE_START;
// 		FREQ_CAPTURE_TIM->SR &= (~FREQ_CAPTURE_IT_1 );
// 		FREQ_CAPTURE_TIM->DIER |= FREQ_CAPTURE_IT_1;

		if ( xSemaphoreTake( xFrequencySemaphore, ( portTickType ) FREQ_CAPTURE_PERIOD_0_HZ ) == pdTRUE )
		{
//			Watchdog_SetTaskStatus(FREQUENCY_CH1_TASK,TASK_ACTIVE);
			 if(FrequencyData.capture_2>FrequencyData.capture_1)
			 {
				 sum_tick_impulse=FrequencyData.capture_2-FrequencyData.capture_1;
			 }
			 else
			 {
				 sum_tick_impulse=(FREQ_CAPTURE_TIM_PERIOD-FrequencyData.capture_1)+FrequencyData.capture_2;
			 }

			 if(FrequencyData.impulse_count_2>FrequencyData.impulse_count_1)
			 {
				 sum_impulse=FrequencyData.impulse_count_2-FrequencyData.impulse_count_1;
			 }
			 else
			 {
				 sum_impulse=(z/*FREQ_CAPTURE_TIM_PERIOD*/-FrequencyData.impulse_count_1)+FrequencyData.impulse_count_2;
			 }


			 xSemaphoreTake( xSettingsMutex, portMAX_DELAY );
			 {
			     frequency= (float)stSettings.TCXO_frequency*FREQ_TCXO_MULTIPLIER*sum_impulse/sum_tick_impulse;
			 }
//			 xSemaphoreGive( xSettingsMutex );
//
//			 Watchdog_IncrementCouter(FREQUENCY_CH1_TASK);
//			 Watchdog_SetTaskStatus(FREQUENCY_CH1_TASK,TASK_IDLE);
		}
		else
		{
			frequency=0.0;
//			FrequencyData[0].impulse_count=FREQ_COUNT_1_TIM->CNT;
//			FREQ_COUNT_1_TIM->CNT=0x0;
		}

//		Watchdog_SetTaskStatus(FREQUENCY_CH1_TASK,TASK_ACTIVE);
//	    xSemaphoreTake( xMeasureDataMutex, portMAX_DELAY );
//	    {
	    	stMeasureData.frequency[0]=frequency;
	    	//stMeasureData.pulse_counter[0]+=FrequencyData[0].impulse_count;
//	    }
//	    xSemaphoreGive( xMeasureDataMutex );
//		Watchdog_IncrementCouter(FREQUENCY_CH1_TASK);
//		Watchdog_SetTaskStatus(FREQUENCY_CH1_TASK,TASK_IDLE);
	}
}
