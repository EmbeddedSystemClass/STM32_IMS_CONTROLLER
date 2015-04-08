#include "log.h"
#include "stm32f4xx_rtc.h"

volatile uint16_t current_entry_index=0;
static void Log_Task(void *pvParameters);

#define LOG_PERIOD	2000

void Log_Init(void)
{
	FRAM_I2C_Read_Buffer(FRAM_LOG_CURRENT_ENTRY_ADDR,&current_entry_index,sizeof(current_entry_index));
	/*
	 *
	 */
	current_entry_index=0;
	xTaskCreate(Log_Task,(signed char*)"Log",256,NULL, tskIDLE_PRIORITY + 1, NULL);
}

eErrorCode Log_Read_LogEntry(uint16_t entry_index, uint8_t *buf)
{
	eErrorCode err=ENOERR;

	if(entry_index<FRAM_LOG_LEN)
	{
		err=FRAM_I2C_Read_Buffer(FRAM_LOG_BASE_ADDR+entry_index*(CONTROLLER_MEASURE_DATA_LEN+TIMESTAMP_LEN),buf,(CONTROLLER_MEASURE_DATA_LEN+TIMESTAMP_LEN));
	}
	else
	{
		return EIO;
	}
	return err;
}

eErrorCode Log_Write_LogEntry(uint8_t *buf)
{
	eErrorCode err=ENOERR;

	if(current_entry_index<FRAM_LOG_LEN)
	{
		err=FRAM_I2C_Write_Buffer(FRAM_LOG_BASE_ADDR+current_entry_index*(CONTROLLER_MEASURE_DATA_LEN+TIMESTAMP_LEN),buf,(CONTROLLER_MEASURE_DATA_LEN+TIMESTAMP_LEN));
//		current_entry_index++;
//
//		if(current_entry_index>FRAM_LOG_LEN)
//		{
//			current_entry_index=0;
//		}

		FRAM_I2C_Write_Buffer(FRAM_LOG_CURRENT_ENTRY_ADDR,&current_entry_index,sizeof(current_entry_index));
	}
	else
	{
		return EIO;
	}
	return err;
}

volatile uint8_t log_buf[128];
static void Log_Task(void *pvParameters)
{

	uint8_t i=0;
	uint8_t *buf=log_buf;
	RTC_DateTypeDef RTC_DateStructure;
	RTC_TimeTypeDef RTC_TimeStructure;
	stControllerMeasureData TempControllerMeasureData;

	while(1)
	{
		vTaskDelay(LOG_PERIOD);
		buf=log_buf;
		RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);
		RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);

		xSemaphoreTake( xMeasureDataMutex, portMAX_DELAY );
	    {
	    	TempControllerMeasureData=stMeasureData;
	    }
	    xSemaphoreGive( xMeasureDataMutex );

	    *buf++=RTC_DateStructure.RTC_Date;
	    *buf++=RTC_DateStructure.RTC_Month;
	    *buf++=RTC_DateStructure.RTC_Year;
	    *buf++=RTC_TimeStructure.RTC_Hours;
	    *buf++=RTC_TimeStructure.RTC_Minutes;
	    *buf++=RTC_TimeStructure.RTC_Seconds;

	    for(i=0;i<FREQ_CHN_NUM;i++)
	    {
	    	*(float *)buf=TempControllerMeasureData.frequency[i];
	    	buf+=sizeof(float);
	    }

	    for(i=0;i<PULSE_COUNT_CHN_NUM;i++)
	    {
	    	((uint32_t *)buf)[0]=((uint32_t *)&TempControllerMeasureData.pulse_counter[i])[0];
	    	((uint32_t *)buf)[1]=((uint32_t *)&TempControllerMeasureData.pulse_counter[i])[1];
	    	buf+=2*sizeof(uint32_t);
	    }

	    for(i=0;i<RTD_CHN_NUM;i++)
	    {
	    	*(float *)buf=TempControllerMeasureData.rtd[i];
	    	buf+=sizeof(float);
	    }

	    for(i=0;i<CURRENT_CHN_NUM;i++)
	    {
	    	*(float *)buf=TempControllerMeasureData.current[i];
	    	buf+=sizeof(float);
	    }

		Log_Write_LogEntry(log_buf);
	}
}
