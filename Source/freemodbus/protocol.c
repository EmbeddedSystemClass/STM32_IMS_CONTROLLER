#include "protocol.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "mb.h"
#include "mbport.h"

#include "watchdog.h"

#include "stdio.h"
#include "errno.h"

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include "misc.h"
extern struct task_watch task_watches[];

static void Modbus_RS485_Task(void *pvParameters);
static void Modbus_RS232_Task(void *pvParameters);

stMBContext stContext_RS485;
stMBContext stContext_RS232;

void Protocol_Init(void)
{
	eMBErrorCode    eStatus;

	RS485SerialContextInit(&stContext_RS485);
	RS485TimerContextInit(&stContext_RS485);

	RS232SerialContextInit(&stContext_RS232);
	RS232TimerContextInit(&stContext_RS232);

	eStatus = eMBInit(&stContext_RS485, MB_RTU, 0x0A, 0, 57600, 0 );
	eStatus = eMBInit(&stContext_RS232, MB_RTU, 0x0A, 0, 57600, 0 );

	xTaskCreate(Modbus_RS485_Task,(signed char*)"Modbus RS485",512,NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(Modbus_RS232_Task,(signed char*)"Modbus RS232",512,NULL, tskIDLE_PRIORITY + 1, NULL);
}



static void Modbus_RS485_Task(void *pvParameters)
{
    eMBEnable(&stContext_RS485);
    Watchdog_SetTaskStatus(RS485_TASK,TASK_ACTIVE);
    for( ;; )
    {
        eMBPoll(&stContext_RS485);

        vTaskDelay(10);
        Watchdog_IncrementCouter(RS485_TASK);
    }
}

static void Modbus_RS232_Task(void *pvParameters)
{
    eMBEnable(&stContext_RS232);
    Watchdog_SetTaskStatus(RS232_TASK,TASK_ACTIVE);
    for( ;; )
    {
        eMBPoll(&stContext_RS232);

        vTaskDelay(10);
        Watchdog_IncrementCouter(RS232_TASK);
    }

}
