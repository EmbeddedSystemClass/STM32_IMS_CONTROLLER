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

void Protocol_Init(void)
{
	eMBErrorCode    eStatus;


	RS485SerialContextInit(&stContext_RS485);
	RS485TimerContextInit(&stContext_RS485);

	eStatus = eMBInit(&stContext_RS485, MB_RTU, 0x0A, 0, 57600, 0 );
	xTaskCreate(Modbus_RS485_Task,(signed char*)"Modbus RS485",1024,NULL, tskIDLE_PRIORITY + 1, NULL);
	//xTaskCreate(Modbus_RS232_Task,(signed char*)"Modbus RS232",256,NULL, tskIDLE_PRIORITY + 1, NULL);
}



static void Modbus_RS485_Task(void *pvParameters)
{
    portTickType    xLastWakeTime;


    eMBEnable(&stContext_RS485);
    task_watches[PROTO_TASK].task_status=TASK_ACTIVE;
    for( ;; )
    {
        eMBPoll(&stContext_RS485);

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
