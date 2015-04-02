#ifndef FRAM_I2C_H
#define FRAM_I2C_H

#include "stm32f4xx.h"

void FRAM_I2C_Init(void);
uint8_t FRAM_I2C_Read_Buffer(uint16_t addr,uint8_t *buf, uint16_t buf_len);
uint8_t FRAM_I2C_Write_Buffer(uint16_t addr,uint8_t *buf, uint16_t buf_len);

#endif
