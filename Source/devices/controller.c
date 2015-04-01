#include "controller.h"
#include "protocol.h"
#include "frequency.h"
#include "ADS1220.h"
#include "rtc.h"

xSemaphoreHandle xMeasureDataMutex;
xSemaphoreHandle xSettingsMutex;

stControllerSettings stSettings;
stControllerMeasureData stMeasureData;

uint8_t Controller_RestoreSettings(void);

void ControllerInit(void)
{
	uint8_t i=0;

	xMeasureDataMutex=xSemaphoreCreateMutex() ;
	xSettingsMutex=xSemaphoreCreateMutex() ;
	Controller_RestoreSettings();
	RTC_Clock_Init();
	Protocol_Init();
	FrequencyMeasureInit();
	ADS1220_init();
}

uint8_t Controller_RestoreSettings(void)
{
	/*read struct settings from ROM*/
	/*CRC check*/
	if((stSettings.TCXO_frequency<TCXO_FREQ_MIN) || (stSettings.TCXO_frequency>TCXO_FREQ_MAX))
	{
		stSettings.TCXO_frequency=TCXO_FREQ_DEFAULT;
	}

	return 0;
}
