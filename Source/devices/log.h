#ifndef LOG_H
#define LOG_H
#include "controller.h"
#include "fram_i2c.h"

void Log_Init(void);
eErrorCode Log_Write_LogEntry(uint8_t *buf);
eErrorCode Log_Read_LogEntry(uint16_t entry_index, uint8_t *buf);

#endif
