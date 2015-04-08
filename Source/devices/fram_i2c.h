#ifndef FRAM_I2C_H
#define FRAM_I2C_H

#include "stm32f4xx.h"
#include "stm32f4xx_rtc.h"
#include "controller.h"

//typedef struct
//{
//	stControllerMeasureData stData;
//	RTC_DateTypeDef RTC_DateStructure;
//	RTC_TimeTypeDef RTC_TimeStructure;
//}stLogEntry;


#define FRAM_SETTINGS_ADDR					0x0
#define FRAM_SETTINGS_CRC_ADDR				0x2A
#define FRAM_LOG_CURRENT_ENTRY_ADDR			0x28
#define FRAM_LOG_BASE_ADDR					0x30
#define FRAM_LOG_LEN						100
#define FRAM_MAX_ADDR						0x7FFF


void 	   FRAM_I2C_Init(void);
eErrorCode FRAM_I2C_Read_Buffer(uint16_t addr,uint8_t *buf, uint16_t buf_len);
eErrorCode FRAM_I2C_Write_Buffer(uint16_t addr,uint8_t *buf, uint16_t buf_len);

eErrorCode FRAM_Read_Settings(stControllerSettings *stSettings);
eErrorCode FRAM_Write_Settings(stControllerSettings stSettings);



#endif
