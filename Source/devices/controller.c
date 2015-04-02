#include "controller.h"
#include "protocol.h"
#include "frequency.h"
#include "ADS1220.h"
#include "rtc.h"
#include "fram_i2c.h"

xSemaphoreHandle xMeasureDataMutex;
xSemaphoreHandle xSettingsMutex;

stControllerSettings stSettings;
const stControllerSettings stSettingsDefault={TCXO_FREQ_DEFAULT};
stControllerMeasureData stMeasureData;

uint8_t Controller_RestoreSettings(void);

void ControllerInit(void)
{
	uint8_t i=0;

	xMeasureDataMutex=xSemaphoreCreateMutex() ;
	xSettingsMutex=xSemaphoreCreateMutex() ;
	FRAM_I2C_Init();
	Controller_RestoreSettings();
	RTC_Clock_Init();
	Protocol_Init();
	FrequencyMeasureInit();
	ADS1220_init();
}

uint8_t Controller_RestoreSettings(void)
{
//	uint8_t buf[16];
//	uint8_t buf_2[16]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
//	FRAM_I2C_Write_Buffer(0,buf_2,16);
//
//	FRAM_I2C_Read_Buffer(0x0,buf,16);

	FRAM_Read_Settings(&stSettings);
	if((stSettings.TCXO_frequency<TCXO_FREQ_MIN) || (stSettings.TCXO_frequency>TCXO_FREQ_MAX))
	{
		stSettings.TCXO_frequency=TCXO_FREQ_DEFAULT;
	}

	return 0;
}
