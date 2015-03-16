/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

#include "mbport.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"



static void Serial_Task(void *pvParameters);
static volatile uint8_t temp_char;
/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
	if(TRUE==xRxEnable)
	{
		//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	}
	else
	{
		//USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
	}

	if(TRUE==xTxEnable)
	{
		//USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		pxMBFrameCBTransmitterEmpty();
	}
	else
	{
	   //USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	}
}
/**********************************************************************************/
BOOL 
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
	//USBD_Init(&USB_OTG_dev,USB_OTG_FS_CORE_ID,&USR_desc,&USBD_CDC_cb,&USR_cb);
	xTaskCreate(Serial_Task,(signed char*)"Serial",64,NULL, tskIDLE_PRIORITY + 1, NULL);
	return TRUE;
}


BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
	if(VCP_DataTx(&ucByte,1)==0)
	{
		pxMBFrameCBTransmitterEmpty();
	}
    return TRUE;
}


BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
	*pucByte=temp_char;
//	VCP_get_char(pucByte);
    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
 void prvvUARTTxReadyISR( void )
{
    pxMBFrameCBTransmitterEmpty(  );
}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
void prvvUARTRxISR( void )
{
     pxMBFrameCBByteReceived();
}

static void Serial_Task(void *pvParameters)
{
	while(1)
	{
//		if(VCP_get_char(&temp_char))
//		{
//			pxMBFrameCBByteReceived();
//		}
		 //vTaskDelay(1);
	}
}
