#include "protocol.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "mb.h"
#include "mbport.h"

#include "watchdog.h"

extern struct task_watch task_watches[];

static void Modbus_Task(void *pvParameters);

void Protocol_Init(void)
{
	eMBErrorCode    eStatus;

	eStatus = eMBInit( MB_RTU, 0x0A, 0, 57600, 0 );
	xTaskCreate(Modbus_Task,(signed char*)"Modbus",256,NULL, tskIDLE_PRIORITY + 1, NULL);
}



static void Modbus_Task(void *pvParameters)
{
    portTickType    xLastWakeTime;

    eMBEnable();
    task_watches[PROTO_TASK].task_status=TASK_ACTIVE;
    for( ;; )
    {
        eMBPoll();
        vTaskDelay(10);
        task_watches[PROTO_TASK].counter++;
    }
}
