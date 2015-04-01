#include "rtc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_rtc.h"

//#include "FreeRTOS.h"
//#include "task.h"
//#include "queue.h"
//#include "semphr.h"

//static void RTC_Task(void *pvParameters);


void RTC_Clock_Init(void)
{
	uint32_t AsynchPrediv = 0, SynchPrediv = 0;

	RTC_InitTypeDef RTC_InitStruct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	/* Allow access to RTC */
	//RTC_WriteProtectionCmd(DISABLE);
	PWR_BackupAccessCmd(ENABLE);

	RCC_LSEConfig(RCC_LSE_ON);

	/* Wait till LSE is ready */
	while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);

	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

    SynchPrediv = 0xFF;
    AsynchPrediv = 0x7F;

	RCC_RTCCLKCmd(ENABLE);
	RTC_WaitForSynchro();

	RTC_WriteBackupRegister(RTC_BKP_DR0, FIRST_DATA);

	RTC_StructInit(&RTC_InitStruct);

	RTC_InitStruct.RTC_AsynchPrediv = AsynchPrediv;
	RTC_InitStruct.RTC_SynchPrediv = SynchPrediv;
	RTC_InitStruct.RTC_HourFormat = RTC_HourFormat_24;
    RTC_Init(&RTC_InitStruct);
}


