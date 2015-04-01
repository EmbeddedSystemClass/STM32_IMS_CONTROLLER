#include "controller.h"
#include "protocol.h"
#include "frequency.h"
#include "ADS1220.h"
#include "rtc.h"

/*xSemaphoreHandle xFrequencyMutex[FREQ_CHN_NUM];
xSemaphoreHandle xPulseCounterMutex[PULSE_COUNT_CHN_NUM];
xSemaphoreHandle xRTDMutex[RTD_CHN_NUM];
xSemaphoreHandle xCurrentMutex[CURRENT_CHN_NUM];*/

xSemaphoreHandle xMeasureDataMutex;
xSemaphoreHandle xSettingsMutex;

stControllerSettings stSettings;
stControllerMeasureData stMeasureData;

void ControllerInit(void)
{
	uint8_t i=0;

/*	for(i=0;i<FREQ_CHN_NUM;i++)
	{
		xFrequencyMutex[i]=xSemaphoreCreateMutex() ;
	}

	for(i=0;i<PULSE_COUNT_CHN_NUM;i++)
	{
		xPulseCounterMutex[i]=xSemaphoreCreateMutex() ;
	}

	for(i=0;i<RTD_CHN_NUM;i++)
	{
		xRTDMutex[i]=xSemaphoreCreateMutex() ;
	}

	for(i=0;i<CURRENT_CHN_NUM;i++)
	{
		xCurrentMutex[i]=xSemaphoreCreateMutex() ;
	}*/

	xMeasureDataMutex=xSemaphoreCreateMutex() ;
	xSettingsMutex=xSemaphoreCreateMutex() ;

	RTC_Clock_Init();
	Protocol_Init();
	FrequencyMeasureInit();
	ADS1220_init();
}
