#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f4xx_rtc.h"
#include "stdint.h"

#define TCXO_FREQ_MIN		19999000
#define TCXO_FREQ_MAX		20001000
#define TCXO_FREQ_DEFAULT	20000000

typedef struct
{
	uint32_t TCXO_frequency;

}stControllerSettings;

#define FREQ_CHN_NUM		2
#define PULSE_COUNT_CHN_NUM	2
#define RTD_CHN_NUM			2
#define CURRENT_CHN_NUM		6

typedef struct
{
	float frequency[FREQ_CHN_NUM];
	uint64_t pulse_counter[PULSE_COUNT_CHN_NUM];
	float rtd[RTD_CHN_NUM];
	float current[CURRENT_CHN_NUM];
}stControllerMeasureData;


extern xSemaphoreHandle xMeasureDataMutex;
extern xSemaphoreHandle xSettingsMutex;

extern stControllerSettings stSettings;
extern const stControllerSettings stSettingsDefault;
extern stControllerMeasureData stMeasureData;

void ControllerInit(void);

#endif
