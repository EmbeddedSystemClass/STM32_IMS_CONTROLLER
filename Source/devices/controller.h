#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f4xx_rtc.h"
#include "stdint.h"

typedef enum
{
    ENOERR,                  /*!< no error. */
    ENOREG,                  /*!< illegal register address. */
    EINVAL,                  /*!< illegal argument. */
    ENORES,                  /*!< insufficient resources. */
    EIO,                     /*!< I/O error. */
    ETIMEDOUT,                /*!< timeout error occurred. */
    ECRCERR
} eErrorCode;


#define TCXO_FREQ_MIN		19999000
#define TCXO_FREQ_MAX		20001000
#define TCXO_FREQ_DEFAULT	20000000

#define REG_CUR_CODE_MIN	0
#define REG_CUR_CODE_MAX	0x7FFFFF

#define REG_CUR_MA_MIN		 0.0
#define REG_CUR_MA_MAX		20.0

#define FREQ_CHN_NUM		2
#define PULSE_COUNT_CHN_NUM	2
#define RTD_CHN_NUM			2
#define CURRENT_CHN_NUM		8

#define CONTROLLER_MEASURE_DATA_LEN	64
#define TIMESTAMP_LEN				6

typedef struct
{
	uint32_t code_pnt0;
	float current_ma_pnt0;
	uint32_t code_pnt1;
	float current_ma_pnt1;
}stCurChannelCalibrate;

typedef struct
{
	uint32_t TCXO_frequency;
	stCurChannelCalibrate CurChannelCalibrate[CURRENT_CHN_NUM];
}stControllerSettings;



typedef struct
{
	float frequency[FREQ_CHN_NUM];
	uint64_t pulse_counter[PULSE_COUNT_CHN_NUM];
	float rtd[RTD_CHN_NUM];
	float current_raw[CURRENT_CHN_NUM];
	float current[CURRENT_CHN_NUM];
}stControllerMeasureData;

//typedef struct
//{
//
//}stControllerError;

extern xSemaphoreHandle xMeasureDataMutex;
extern xSemaphoreHandle xSettingsMutex;

extern stControllerSettings stSettings;
extern const stControllerSettings stSettingsDefault;
extern stControllerMeasureData stMeasureData;

void ControllerInit(void);

#endif
