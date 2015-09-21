#include "controller.h"
#include "protocol.h"
#include "frequency.h"
#include "ADS1220.h"
#include "rtc.h"
#include "fram_i2c.h"
#include "log.h"
#include "watchdog.h"
#include "sdio_sd.h"
#include "ff.h"
#include "diskio.h"
#include <stdio.h>
#include "wiznet.h"



xSemaphoreHandle xMeasureDataMutex;
xSemaphoreHandle xSettingsMutex;

stControllerSettings stSettings;
const stControllerSettings stSettingsDefault={TCXO_FREQ_DEFAULT};
stControllerMeasureData stMeasureData;






uint8_t Controller_RestoreSettings(void);

volatile FATFS fs;         /* Work area (file system object) for logical drive */
volatile FIL fsrc;         /* file objects */
volatile FRESULT res;
volatile UINT br;

void ControllerInit(void)
{
	xMeasureDataMutex=xSemaphoreCreateMutex() ;
	xSettingsMutex=xSemaphoreCreateMutex() ;


	//Watchdog_Init();

	FRAM_I2C_Init();
	Controller_RestoreSettings();
	RTC_Clock_Init();

	Protocol_Init();
	FrequencyMeasureInit();
	ADS1220_init();

	spiW5500_init();
//
//	if(disk_initialize(0)==0)
//	{
//		  if (f_mount(0, &fs) == FR_OK)
//		  {
//			  res = f_open( &fsrc , "0:/test.txt" ,  FA_OPEN_EXISTING|FA_WRITE);
//			  if (res==FR_OK)
//			  {
////			      uint8_t read_stat=0;
//			      uint8_t buf[64];
////				  read_stat=f_read(&fsrc, &buf[0], sizeof(buf), &br);
//			      uint32_t i=0;
//			      for(i=0;i<100000;i++)
//			      {
//					  sprintf(buf,"%d\n",i);
//					  f_write(&fsrc,buf,strlen(buf),&br);
//			      }
//			      f_close(&fsrc);
//			  }
//			  else
//			  {
//
//			  }
//		  }
//		  else
//		  {
//
//		  }
//	}
//	else
//	{
//
//	}

	//Log_Init();
}

uint8_t Controller_RestoreSettings(void)
{
	FRAM_Read_Settings(&stSettings);
	if((stSettings.TCXO_frequency<TCXO_FREQ_MIN) || (stSettings.TCXO_frequency>TCXO_FREQ_MAX))
	{
		stSettings.TCXO_frequency=TCXO_FREQ_DEFAULT;
	}

	uint8_t i=0;
	for(i=0;i<CURRENT_CHN_NUM;i++)
	{
		if((stSettings.CurChannelCalibrate[i].code_pnt0<REG_CUR_CODE_MIN) || (stSettings.CurChannelCalibrate[i].code_pnt0>REG_CUR_CODE_MAX))
		{
			stSettings.CurChannelCalibrate[i].code_pnt0=REG_CUR_CODE_DEFAULT;
		}

		if((stSettings.CurChannelCalibrate[i].code_pnt1<REG_CUR_CODE_MIN) || (stSettings.CurChannelCalibrate[i].code_pnt1>REG_CUR_CODE_MAX))
		{
			stSettings.CurChannelCalibrate[i].code_pnt1=REG_CUR_CODE_DEFAULT;
		}

		if((stSettings.CurChannelCalibrate[i].current_ma_pnt0<REG_CUR_MA_MIN) || (stSettings.CurChannelCalibrate[i].current_ma_pnt0>REG_CUR_MA_MAX))
		{
			stSettings.CurChannelCalibrate[i].current_ma_pnt0=REG_CUR_MA_DEFAULT;
		}

		if((stSettings.CurChannelCalibrate[i].current_ma_pnt1<REG_CUR_MA_MIN) || (stSettings.CurChannelCalibrate[i].current_ma_pnt1>REG_CUR_MA_MAX))
		{
			stSettings.CurChannelCalibrate[i].current_ma_pnt1=REG_CUR_MA_DEFAULT;
		}
	}

	return 0;
}
