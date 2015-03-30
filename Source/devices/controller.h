#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define FREQ_COEFF_MIN	1.0
#define FREQ_COEFF_MAX	1.1

typedef struct
{
	float frequency_coefficient;

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

extern xSemaphoreHandle xFrequencyMutex[FREQ_CHN_NUM];
extern xSemaphoreHandle xPulseCounterMutex[PULSE_COUNT_CHN_NUM];
extern xSemaphoreHandle xRTDMutex[RTD_CHN_NUM];
extern xSemaphoreHandle xCurrentMutex[CURRENT_CHN_NUM];

extern stControllerSettings stSettings;
extern stControllerMeasureData stMeasureData;

void ControllerInit(void);

#endif
