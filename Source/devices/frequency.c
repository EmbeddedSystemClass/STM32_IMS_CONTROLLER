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

#define LINE1_ANTIBOUNCE_TIMER				TIM11
#define LINE2_ANTIBOUNCE_TIMER				TIM14

#define IMPULSE_FAST_TIM					TIM2
#define IMPULSE_FAST_GPIO_AF 				GPIO_AF_TIM2
#define RCC_IMPULSE_FAST_TIM 				RCC_APB1Periph_TIM2
#define	RCC_IMPULSE_FAST_GPIO_PORT			RCC_AHB1Periph_GPIOA
#define IMPULSE_FAST_GPIO_PORT				GPIOA
#define IMPULSE_FAST_TIM_IRQn				TIM2_IRQn
#define IMPULSE_FAST_GPIO_PINS				GPIO_Pin_1 | GPIO_Pin_2
#define IMPULSE_FAST_GPIO_PINSOURCE_1		GPIO_PinSource1
#define IMPULSE_FAST_GPIO_PINSOURCE_2		GPIO_PinSource2
#define IMPULSE_FAST_TIM_IRQHandler 		TIM2_IRQHandler
#define IMPULSE_FAST_TIM_PERIOD				(0xFFFF)

void Impulse_SetAntiBounceDelay(uint16_t time_us);

uint64_t reload_counter=0, reload_fast_tim=0;

typedef struct
{
	uint64_t start_value;
	uint64_t stop_value;
}stImpulseCounter;

stImpulseCounter Line1_ImpulseCounter, Line2_ImpulseCounter, Line1_FastTimer, Line2_FastTimer;
uint32_t  exti_base_addr = (uint32_t)EXTI_BASE;

void FrequencyMeasureInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_InitStructure;

	RCC_AHB1PeriphClockCmd( RCC_FREQ_COUNT_1_GPIO_PORT, ENABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_IMPULSE_FAST_GPIO_PORT, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_FREQ_COUNT_1_TIM, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_IMPULSE_FAST_GPIO_PORT, ENABLE);

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

    TIM_ClearITPendingBit(FREQ_COUNT_1_TIM, TIM_IT_Update);
    TIM_Cmd(FREQ_COUNT_1_TIM, ENABLE);
    TIM_ITConfig(FREQ_COUNT_1_TIM, TIM_IT_Update, ENABLE);

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
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //---------------------------------

	TIM_TimeBaseStructInit(&TIM_InitStructure);
	TIM_InitStructure.TIM_Prescaler = 60;
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_Period= 0xFFFF;
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(LINE1_ANTIBOUNCE_TIMER, &TIM_InitStructure);

	TIM_InitStructure.TIM_Prescaler = 30;

	TIM_TimeBaseInit(LINE2_ANTIBOUNCE_TIMER, &TIM_InitStructure);
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
    TIM_ClearITPendingBit(LINE1_ANTIBOUNCE_TIMER, TIM_IT_Update);
    TIM_ITConfig(LINE1_ANTIBOUNCE_TIMER, TIM_IT_Update, ENABLE);

   // TIM_Cmd(TIM14, ENABLE);
    TIM_ClearITPendingBit(LINE2_ANTIBOUNCE_TIMER, TIM_IT_Update);
    TIM_ITConfig(LINE2_ANTIBOUNCE_TIMER, TIM_IT_Update, ENABLE);

    stMeasureData.pulse_line_measure_state[0]=MEASURE_FINISHED;
    stMeasureData.pulse_line_measure_state[1]=MEASURE_FINISHED;



    //---------------------------------
	TIM_TimeBaseStructInit(&TIM_InitStructure);
	TIM_InitStructure.TIM_Prescaler = 0;
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_Period= 0xFFFF;
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(IMPULSE_FAST_TIM, &TIM_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel =  IMPULSE_FAST_TIM_IRQn  ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ClearITPendingBit(IMPULSE_FAST_TIM, TIM_IT_Update);
    TIM_ITConfig(IMPULSE_FAST_TIM, TIM_IT_Update, ENABLE);
    TIM_Cmd(IMPULSE_FAST_TIM, ENABLE);
    //---------------------------------

    ImpulseLine1_StartMeasure();
}



void ImpulseLine1_StartMeasure(void)
{
	if(stMeasureData.pulse_line_measure_state[0]==MEASURE_FINISHED)
	{
		*(__IO uint32_t *) exti_base_addr |= IMPULSE_SENSOR_1_1_EXTI;
		*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_1_2_EXTI;
		stMeasureData.pulse_line_measure_state[0]=MEASURE_STARTED;
	}
}

void ImpulseLine2_StartMeasure(void)
{
	if(stMeasureData.pulse_line_measure_state[1]==MEASURE_FINISHED)
	{
		*(__IO uint32_t *) exti_base_addr |= IMPULSE_SENSOR_2_1_EXTI;
		*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_2_2_EXTI;
		stMeasureData.pulse_line_measure_state[1]=MEASURE_STARTED;
	}
}

void ImpulseLine1_EmergencyStopMeasure(void)
{
	*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_1_1_EXTI;
	*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_1_2_EXTI;
	stMeasureData.pulse_line_measure_state[0]=MEASURE_FINISHED;
}

void ImpulseLine2_EmergencyStopMeasure(void)
{
	*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_2_1_EXTI;
	*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_2_2_EXTI;
	stMeasureData.pulse_line_measure_state[1]=MEASURE_FINISHED;
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
	  	LINE1_ANTIBOUNCE_TIMER->CNT=0;
	    TIM_Cmd(LINE1_ANTIBOUNCE_TIMER, ENABLE);
		EXTI_ClearITPendingBit(IMPULSE_SENSOR_1_1_EXTI);
		*(__IO uint32_t *) exti_base_addr &= ~IMPULSE_SENSOR_1_1_EXTI;
		line_1_event=IMPULSE_SENSOR_1_1_EVENT;
  }
}


void EXTI9_5_IRQHandler(void)
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


void  TIM8_UP_TIM13_IRQHandler(void)//counter
{
    if (TIM_GetITStatus(FREQ_COUNT_1_TIM, TIM_IT_Update) != RESET)
    {
    	reload_counter+=0xFFFF;
    	TIM_ClearITPendingBit(FREQ_COUNT_1_TIM, TIM_IT_Update);
    }
}


void  TIM1_TRG_COM_TIM11_IRQHandler(void)//delay_1
{
    if (TIM_GetITStatus(LINE1_ANTIBOUNCE_TIMER, TIM_IT_Update) != RESET)
    {
    	TIM_Cmd(LINE1_ANTIBOUNCE_TIMER, DISABLE);
    	if(line_1_event==IMPULSE_SENSOR_1_1_EVENT)
    	{
    		if(GPIO_ReadInputDataBit(IMPULSE_SENSOR_PORT,IMPULSE_SENSOR_1_1)==SENSOR_EVENT_LEVEL)//level ok
    		{
				Line1_ImpulseCounter.start_value=reload_counter+FREQ_COUNT_1_TIM->CNT;
				Line1_FastTimer.start_value=reload_fast_tim+IMPULSE_FAST_TIM->CNT;
				*(__IO uint32_t *) exti_base_addr |= IMPULSE_SENSOR_1_2_EXTI;
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
    			Line1_ImpulseCounter.stop_value=reload_counter+FREQ_COUNT_1_TIM->CNT;
    			Line1_FastTimer.stop_value=reload_fast_tim+IMPULSE_FAST_TIM->CNT;

    			if(Line1_ImpulseCounter.stop_value>=Line1_ImpulseCounter.start_value)
    			{
    				stMeasureData.pulse_counter[0]=Line1_ImpulseCounter.stop_value-Line1_ImpulseCounter.start_value;
    			}
    			else
    			{
    				stMeasureData.pulse_counter[0]=Line1_ImpulseCounter.start_value-Line1_ImpulseCounter.stop_value;
    			}

    			if(Line1_FastTimer.stop_value>=Line1_FastTimer.start_value)
    			{
    				stMeasureData.fast_tim_value[0]=Line1_FastTimer.stop_value-Line1_FastTimer.start_value;
    			}
    			else
    			{
    				stMeasureData.fast_tim_value[0]=Line1_FastTimer.start_value-Line1_FastTimer.stop_value;
    			}
    			stMeasureData.pulse_line_measure_state[0]=MEASURE_FINISHED;
    		}
    		else
    		{
    			*(__IO uint32_t *) exti_base_addr |= IMPULSE_SENSOR_1_2_EXTI;
    		}
    	}
    	TIM_ClearITPendingBit(LINE1_ANTIBOUNCE_TIMER, TIM_IT_Update);
    }
}

void  TIM8_TRG_COM_TIM14_IRQHandler(void)//delay_2
{
    if (TIM_GetITStatus(LINE2_ANTIBOUNCE_TIMER, TIM_IT_Update) != RESET)
    {
    	TIM_Cmd(LINE2_ANTIBOUNCE_TIMER, DISABLE);
    	if(line_2_event==IMPULSE_SENSOR_2_1_EVENT)
    	{
    		if(GPIO_ReadInputDataBit(IMPULSE_SENSOR_PORT,IMPULSE_SENSOR_1_2)==SENSOR_EVENT_LEVEL)//level ok
    		{
				Line2_ImpulseCounter.start_value=reload_counter+FREQ_COUNT_1_TIM->CNT;
				Line2_FastTimer.start_value=reload_fast_tim+IMPULSE_FAST_TIM->CNT;
				*(__IO uint32_t *) exti_base_addr |= IMPULSE_SENSOR_2_2_EXTI;
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
    			Line2_ImpulseCounter.stop_value=reload_counter+FREQ_COUNT_1_TIM->CNT;
    			Line2_FastTimer.stop_value=reload_fast_tim+IMPULSE_FAST_TIM->CNT;

    			if(Line2_ImpulseCounter.stop_value>=Line2_ImpulseCounter.start_value)
    			{
    				stMeasureData.pulse_counter[1]=Line2_ImpulseCounter.stop_value-Line2_ImpulseCounter.start_value;
    			}
    			else
    			{
    				stMeasureData.pulse_counter[1]=Line2_ImpulseCounter.start_value-Line2_ImpulseCounter.stop_value;
    			}

    			if(Line2_FastTimer.stop_value>=Line2_FastTimer.start_value)
    			{
    				stMeasureData.fast_tim_value[1]=Line2_FastTimer.stop_value-Line2_FastTimer.start_value;
    			}
    			else
    			{
    				stMeasureData.fast_tim_value[1]=Line2_FastTimer.start_value-Line2_FastTimer.stop_value;
    			}


    			stMeasureData.pulse_line_measure_state[1]=MEASURE_FINISHED;
    		}
    		else
    		{
    			*(__IO uint32_t *) exti_base_addr |= IMPULSE_SENSOR_2_2_EXTI;
    		}
    	}
    	TIM_ClearITPendingBit(LINE2_ANTIBOUNCE_TIMER, TIM_IT_Update);
    }
}


void  IMPULSE_FAST_TIM_IRQHandler(void)//fast timer
{
    if (TIM_GetITStatus(IMPULSE_FAST_TIM, TIM_IT_Update) != RESET)
    {
    	reload_fast_tim+=0xFFFF;
    	TIM_ClearITPendingBit(IMPULSE_FAST_TIM, TIM_IT_Update);
    }
}
