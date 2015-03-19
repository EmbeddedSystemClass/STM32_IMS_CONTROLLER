static TIM_ICInitTypeDef TIM_CH1_ICInitStructure;

#define FREQ_GPIO_AF GPIO_AF_2

#define FREQ_TIM					TIM3
#define RCC_APB1Periph_FREQ_TIM 	RCC_APB1Periph_FREQ_TIM
#define FREQ_GPIO_PORT				FREQ_GPIO_PORT
#define FREQ_TIM_IRQn				FREQ_TIM_IRQn
#define FREQ_GPIO_PINS				GPIO_Pin_3 | GPIO_Pin_4



#define	FREQ_GPIO_AF				FREQ_GPIO_AF

#define FREQ_GPIO_PINSOURCE_1		GPIO_PinSource3
#define FREQ_GPIO_PINSOURCE_2		GPIO_PinSource4

#define Freq_TIM_IRQHandler 		TIM3_IRQHandler



void FrequencyMeasureInit(void)
{
	 GPIO_InitTypeDef GPIO_InitStructure;
	 TIM_ICInitTypeDef TIM_ICInitStructure;
	 NVIC_InitTypeDef NVIC_InitStructure;

	 TIM_DeInit(FREQ_TIM);

	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_FREQ_TIM, ENABLE);

	 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FREQ_GPIO_PORT, ENABLE);

	 GPIO_InitStructure.GPIO_Pin = FREQ_GPIO_PINS;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	 GPIO_Init(FREQ_GPIO_PORT, &GPIO_InitStructure);

	 GPIO_PinAFConfig(FREQ_GPIO_PORT, FREQ_GPIO_PINSOURCE_1, FREQ_GPIO_AF );
	 GPIO_PinAFConfig(FREQ_GPIO_PORT, FREQ_GPIO_PINSOURCE_2, FREQ_GPIO_AF );


	 NVIC_InitStructure.NVIC_IRQChannel = FREQ_TIM_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 NVIC_Init(&NVIC_InitStructure);

	 /* Enable capture*/
	 TIM_CH1_ICInitStructure.TIM_Channel = TIM_Channel_1;
	 TIM_CH1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	 TIM_CH1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	 TIM_CH1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	 TIM_CH1_ICInitStructure.TIM_ICFilter = 0;
	 TIM_ICInit(FREQ_TIM, &TIM_ICInitStructure);

	 TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	 TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	 TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	 TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	 TIM_ICInitStructure.TIM_ICFilter = 0;
	 TIM_ICInit(FREQ_TIM, &TIM_ICInitStructure);

	 TIM_Cmd(FREQ_TIM, ENABLE);

	 TIM_ITConfig(FREQ_TIM, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);

	 TIM_ClearFlag(FREQ_TIM, TIM_FLAG_CC1 | TIM_FLAG_CC2 );
}

static volatile uint32_t ccr[4];
static volatile char pulseState = 0;

void Freq_TIM_IRQHandler()
{
	 if (FREQ_TIM->SR & TIM_IT_CC1 )
	 {
		 FREQ_TIM->SR &= (~TIM_IT_CC1 );
	     ccr[0] = TIM_GetCapture1(FREQ_TIM);
	     TIM_SetCounter(FREQ_TIM, 0);
	 }

	 if (FREQ_TIM->SR & TIM_IT_CC2 )
	 {
		 FREQ_TIM->SR &= (~TIM_IT_CC2 );
		 ccr[1] = TIM_GetCapture2(FREQ_TIM);
	 }
}
