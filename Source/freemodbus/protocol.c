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

void Protocol_Init(void)
{
	eMBErrorCode    eStatus;

	eStatus = eMBInit( MB_RTU, 0x0A, 0, 57600, 0 );
	xTaskCreate(Modbus_RS485_Task,(signed char*)"Modbus RS485",256,NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(Modbus_RS232_Task,(signed char*)"Modbus RS232",256,NULL, tskIDLE_PRIORITY + 1, NULL);
}



static void Modbus_RS485_Task(void *pvParameters)
{
    portTickType    xLastWakeTime;
    stMBPoll stPoll_RS485;

    eMBEnable();
    task_watches[PROTO_TASK].task_status=TASK_ACTIVE;
    for( ;; )
    {
        eMBPoll(&stPoll_RS485);

        vTaskDelay(10);
        task_watches[PROTO_TASK].counter++;
    }
}

static void Modbus_RS232_Task(void *pvParameters)
{
    for( ;; )
    {
    	vTaskDelay(10);
    }

}
