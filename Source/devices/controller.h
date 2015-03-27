#ifndef CONTROLLER_H
#define CONTROLLER_H

#define FREQ_COEFF_MIN	1.0
#define FREQ_COEFF_MAX	1.1

typedef struct
{
	float frequency_coefficient;

}stControllerSettings;

void ControllerInit(void);

#endif
