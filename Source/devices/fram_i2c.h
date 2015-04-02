#ifndef FRAM_I2C_H
#define FRAM_I2C_H

#include "stm32f4xx.h"
#include "controller.h"

#define FRAM_SETTINGS_ADDR		0x0
#define FRAM_SETTINGS_CRC_ADDR	0x2A
#define FRAM_LOG_ADDR			0x30

void FRAM_I2C_Init(void);
uint8_t FRAM_I2C_Read_Buffer(uint16_t addr,uint8_t *buf, uint16_t buf_len);
uint8_t FRAM_I2C_Write_Buffer(uint16_t addr,uint8_t *buf, uint16_t buf_len);

uint8_t FRAM_Read_Settings(stControllerSettings *stSettings);
uint8_t FRAM_Write_Settings(stControllerSettings stSettings);

#endif
