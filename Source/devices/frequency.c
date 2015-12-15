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

//#define FREQ_CAPTURE_TIM					TIM2
//#define FREQ_CAPTURE_GPIO_AF 				GPIO_AF_TIM2
//#define RCC_FREQ_CAPTURE_TIM 				RCC_APB1Periph_TIM2
//#define	RCC_FREQ_CAPTURE_GPIO_PORT			RCC_AHB1Periph_GPIOA
//#define FREQ_CAPTURE_GPIO_PORT				GPIOA
//#define FREQ_CAPTURE_TIM_IRQn				TIM2_IRQn
//#define FREQ_CAPTURE_GPIO_PINS				GPIO_Pin_1 | GPIO_Pin_2
//#define FREQ_CAPTURE_GPIO_PINSOURCE_1		GPIO_PinSource1
//#define FREQ_CAPTURE_GPIO_PINSOURCE_2		GPIO_PinSource2
//#define FREQ_CAPTURE_TIM_IRQHandler 		TIM2_IRQHandler
//#define FREQ_CAPTURE_TIM_PERIOD				(0xFFFFFFFF)
////#define FREQ_CAPTURE_TIMER_TICK_FREQUENCY	19999592//20000000///12000000
//#define FREQ_CAPTURE_PERIOD_0_HZ			1000
//#define FREQ_CAPTURE_IC_FILTER				5
//#define FREQ_CAPTURE_CHN_1					TIM_Channel_2
//#define FREQ_CAPTURE_CHN_2					TIM_Channel_3
//#define FREQ_CAPTURE_IT_1					TIM_IT_CC2
//#define FREQ_CAPTURE_IT_2					TIM_IT_CC3
//#define FREQ_CAP_REG_1						CCR2
//#define FREQ_CAP_REG_2						CCR3

//#define FREQ_COUNT_2_TIM					TIM1
//#define FREQ_COUNT_2_GPIO_AF 				GPIO_AF_TIM1
//#define RCC_FREQ_COUNT_2_TIM 				RCC_APB2Periph_TIM1
//#define	RCC_FREQ_COUNT_2_GPIO_PORT			RCC_AHB1Periph_GPIOE
//#define FREQ_COUNT_2_GPIO_PORT				GPIOE
//#define FREQ_COUNT_2_GPIO_PINS				GPIO_Pin_7
//#define FREQ_COUNT_2_GPIO_PINSOURCE			GPIO_PinSource7
//#define FREQ_COUNT_2_TIM_PERIOD				(65535)

#define FREQ_COUNT_1_TIM					TIM8
#define FREQ_COUNT_1_GPIO_AF 				GPIO_AF_TIM8
#define RCC_FREQ_COUNT_1_TIM 				RCC_APB2Periph_TIM8
#define	RCC_FREQ_COUNT_1_GPIO_PORT			RCC_AHB1Periph_GPIOA
#define FREQ_COUNT_1_GPIO_PORT				GPIOA
#define FREQ_COUNT_1_GPIO_PINS				GPIO_Pin_0
#define FREQ_COUNT_1_GPIO_PINSOURCE			GPIO_PinSource0
#define FREQ_COUNT_1_TIM_PERIOD				(65535)

//#define FREQ_MEASURE_TIME					250
//#define FREQ_TCXO_MULTIPLIER				3

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


void Impulse_SetAntiBounceDelay(uint16_t time_us);

uint64_t reload_counter=0;

void FrequencyMeasureInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_InitStructure;

	RCC_AHB1PeriphClockCmd( RCC_FREQ_COUNT_1_GPIO_PORT, ENABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_FREQ_COUNT_1_TIM, ENABLE);


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

	GPIO_InitStructure.GPIO_Pin = FREQ_COUNT_1_GPIO_PINS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(FREQ_COUNT_1_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(FREQ_COUNT_1_GPIO_PORT, FREQ_COUNT_1_GPIO_PINSOURCE, FREQ_COUNT_1_GPIO_AF );

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

    TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
    TIM_Cmd(FREQ_COUNT_1_TIM, ENABLE);
    TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);

//-------------------
	GPIO_InitStructure.GPIO_Pin = FREQ_COUNT_1_PP_OD_SELECT_PIN | FREQ_COUNT_2_PP_OD_SELECT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(FREQ_COUNT_PP_OD_SELECT_PORT, &GPIO_InitStructure);

	FREQ_COUNT_1_PP;//FREQ_COUNT_1_OD; //
	FREQ_COUNT_2_OD;//FREQ_COUNT_2_OD;
//--------------------
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
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //---------------------------------

	TIM_TimeBaseStructInit(&TIM_InitStructure);
	TIM_InitStructure.TIM_Prescaler = 60;
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_Period= 0xFFFF;
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(TIM11, &TIM_InitStructure);

	TIM_InitStructure.TIM_Prescaler = 30;

	TIM_TimeBaseInit(TIM14, &TIM_InitStructure);
	Impulse_SetAntiBounceDelay(500);

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

   // TIM_Cmd(TIM11, ENABLE);
    TIM_ClearITPendingBit(TIM11, TIM_IT_Update);
    TIM_ITConfig(TIM11, TIM_IT_Update, ENABLE);

   // TIM_Cmd(TIM14, ENABLE);
    TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
    TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);

    ImpulseLine1_StartMeasure();

    //---------------------------------
}

typedef struct
{
	uint64_t start_value;
	uint64_t stop_value;
}stImpulseCounter;

stImpulseCounter Line1_ImpulseCounter, Line2_ImpulseCounter;
uint32_t  exti_base_addr = (uint32_t)EXTI_BASE;

void ImpulseLine1_StartMeasure(void)
{
	*(__IO uint32_t *) exti_base_addr |= IMPULSE_SENSOR_1_1_EXTI;
	*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_1_2_EXTI;
}

void ImpulseLine2_StartMeasure(void)
{
	*(__IO uint32_t *) exti_base_addr |= IMPULSE_SENSOR_2_1_EXTI;
	*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_2_2_EXTI;
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

	TIM_SetAutoreload(TIM11,time_us);
	TIM_SetAutoreload(TIM14,time_us);
}

enum
{
	IMPULSE_SENSOR_1_1_EVENT=0,
	IMPULSE_SENSOR_1_2_EVENT,
	IMPULSE_SENSOR_2_1_EVENT,
	IMPULSE_SENSOR_2_2_EVENT
};

#define SENSOR_EVENT_LEVEL	0

uint8_t line_1_event, line_2_event;


void EXTI4_IRQHandler(void)
{
  if(EXTI_GetITStatus(IMPULSE_SENSOR_1_1_EXTI) != RESET)
  {
	    TIM11->CNT=0;
	    TIM_Cmd(TIM11, ENABLE);
		EXTI_ClearITPendingBit(IMPULSE_SENSOR_1_1_EXTI);
		*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_1_1_EXTI;
		line_1_event=IMPULSE_SENSOR_1_1_EVENT;
  }
}



void EXTI9_5_IRQHandler(void)
{
  if(EXTI_GetITStatus(IMPULSE_SENSOR_1_2_EXTI) != RESET)
  {
	    TIM11->CNT=0;
	  	TIM_Cmd(TIM11, ENABLE);
		EXTI_ClearITPendingBit(IMPULSE_SENSOR_1_2_EXTI);
		*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_1_2_EXTI;
		line_1_event=IMPULSE_SENSOR_1_2_EVENT;
  }

  if(EXTI_GetITStatus(IMPULSE_SENSOR_2_1_EXTI) != RESET)
  {
	  	TIM14->CNT=0;
	    TIM_Cmd(TIM14, ENABLE);
		EXTI_ClearITPendingBit(IMPULSE_SENSOR_2_1_EXTI);
		*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_2_1_EXTI;
		line_2_event=IMPULSE_SENSOR_2_1_EVENT;
  }

  if(EXTI_GetITStatus(IMPULSE_SENSOR_2_2_EXTI) != RESET)
  {
	  	TIM14->CNT=0;
	    TIM_Cmd(TIM14, ENABLE);
		EXTI_ClearITPendingBit(IMPULSE_SENSOR_2_2_EXTI);
		*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_2_2_EXTI;
		line_2_event=IMPULSE_SENSOR_2_2_EVENT;
  }
}


void  TIM8_UP_TIM13_IRQHandler(void)//counter
{
    if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET)
    {
    	reload_counter++;
    	TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
    }
}


void  TIM1_TRG_COM_TIM11_IRQHandler(void)//delay_1
{
    if (TIM_GetITStatus(TIM11, TIM_IT_Update) != RESET)
    {
    	TIM_Cmd(TIM11, DISABLE);
    	if(line_1_event==IMPULSE_SENSOR_1_1_EVENT)
    	{
    		if(GPIO_ReadInputDataBit(IMPULSE_SENSOR_PORT,IMPULSE_SENSOR_1_1)==SENSOR_EVENT_LEVEL)//level ok
    		{
				Line1_ImpulseCounter.start_value=reload_counter+TIM8->CNT;
				*(__IO uint32_t *) exti_base_addr |= IMPULSE_SENSOR_1_2_EXTI;
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
    			Line1_ImpulseCounter.stop_value=reload_counter+TIM8->CNT;
    		}
    		else
    		{
    			*(__IO uint32_t *) exti_base_addr |= IMPULSE_SENSOR_1_2_EXTI;
    		}
    	}
    	TIM_ClearITPendingBit(TIM11, TIM_IT_Update);
    }
}

void  TIM8_TRG_COM_TIM14_IRQHandler(void)//delay_2
{
    if (TIM_GetITStatus(TIM14, TIM_IT_Update) != RESET)
    {
    	TIM_Cmd(TIM14, DISABLE);
    	if(line_2_event==IMPULSE_SENSOR_2_1_EVENT)
    	{
    		if(GPIO_ReadInputDataBit(IMPULSE_SENSOR_PORT,IMPULSE_SENSOR_1_2)==SENSOR_EVENT_LEVEL)//level ok
    		{
				Line2_ImpulseCounter.start_value=reload_counter+TIM8->CNT;
				*(__IO uint32_t *) exti_base_addr |= IMPULSE_SENSOR_2_2_EXTI;
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
    			Line2_ImpulseCounter.stop_value=reload_counter+TIM8->CNT;
    		}
    		else
    		{
    			*(__IO uint32_t *) exti_base_addr |= IMPULSE_SENSOR_2_2_EXTI;
    		}
    	}
    	TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
    }
}
