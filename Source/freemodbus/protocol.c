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

#define MODBUS_RS232_DEV_ADDR	0x0A
#define MODBUS_RS485_DEV_ADDR	0x0A
#define MODBUS_USB_DEV_ADDR	    0x0A

#define MODBUS_TASK_STACK_SIZE	512
#define MODBUS_BAUDRATE			57600
#define MODBUS_POLL_TIME		10

extern struct task_watch task_watches[];

static void Modbus_RS485_Task(void *pvParameters);
static void Modbus_RS232_Task(void *pvParameters);
static void Modbus_USB_CDC_Task(void *pvParameters);

stMBContext stContext_RS485;
stMBContext stContext_RS232;
stMBContext	stContext_USB_CDC;

static stMBContext *stRS232Context;

void Protocol_Init(void)
{
	eMBErrorCode    eStatus;

	RS485SerialContextInit(&stContext_RS485);
	RS485TimerContextInit(&stContext_RS485);

	RS232SerialContextInit(&stContext_RS232);
	RS232TimerContextInit(&stContext_RS232);

	USB_CDC_SerialContextInit(&stContext_USB_CDC);
	USB_CDC_TimerContextInit(&stContext_USB_CDC);

	eStatus = eMBInit(&stContext_RS485, MB_RTU, MODBUS_RS485_DEV_ADDR, 0, MODBUS_BAUDRATE, 0 );
	eStatus = eMBInit(&stContext_RS232, MB_RTU, MODBUS_RS232_DEV_ADDR, 0, MODBUS_BAUDRATE, 0 );
	eStatus = eMBInit(&stContext_USB_CDC, MB_RTU, MODBUS_USB_DEV_ADDR, 0, MODBUS_BAUDRATE, 0 );

	xTaskCreate(Modbus_RS485_Task,(signed char*)"Modbus RS485",MODBUS_TASK_STACK_SIZE,NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(Modbus_RS232_Task,(signed char*)"Modbus RS232",MODBUS_TASK_STACK_SIZE,NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(Modbus_USB_CDC_Task,(signed char*)"Modbus USB",MODBUS_TASK_STACK_SIZE,NULL, tskIDLE_PRIORITY + 1, NULL);
}



static void Modbus_RS485_Task(void *pvParameters)
{
    eMBEnable(&stContext_RS485);
    Watchdog_SetTaskStatus(RS485_TASK,TASK_ACTIVE);
    for( ;; )
    {
        eMBPoll(&stContext_RS485);
        vTaskDelay(MODBUS_POLL_TIME);
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
        vTaskDelay(MODBUS_POLL_TIME);
        Watchdog_IncrementCouter(RS232_TASK);
    }

}

static void Modbus_USB_CDC_Task(void *pvParameters)
{
    eMBEnable(&stContext_USB_CDC);
    Watchdog_SetTaskStatus(USB_TASK,TASK_ACTIVE);
    for( ;; )
    {
        eMBPoll(&stContext_USB_CDC);
        vTaskDelay(MODBUS_POLL_TIME);
        Watchdog_IncrementCouter(USB_TASK);
    }

}
