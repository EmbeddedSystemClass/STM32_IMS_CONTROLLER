#ifndef FREQUENCY_H
#define FREQUENCY_H

void FrequencyMeasureInit(void);
void ImpulseLine1_StartMeasure(void);
void ImpulseLine2_StartMeasure(void);
void ImpulseLine1_EmergencyStopMeasure(void);
void ImpulseLine2_EmergencyStopMeasure(void);
void Impulse_SetAntiBounceDelay(uint16_t time_us);

#endif
