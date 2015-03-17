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
#include "mb.h"
#include "mbport.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include "misc.h"


#define USART_RS485 			USART3
#define GPIO_AF_USART_RS485 	GPIO_AF_USART3
#define USART_RS485_IRQn		USART3_IRQn
#define RCC_USART_RS485 		RCC_APB1Periph_USART3
#define USART_RS485_IRQHandler  USART3_IRQHandler

#define RCC_USART_RS485_GPIO 	RCC_AHB1Periph_GPIOB

#define USART_RS485_GPIO 	GPIOB

#define USART_RS485_TXD	GPIO_Pin_10
#define USART_RS485_RXD	GPIO_Pin_11

#define USART_RS485_TXD_PIN_SOURCE GPIO_PinSource10
#define USART_RS485_RXD_PIN_SOURCE GPIO_PinSource11

#define USART_RS485_DE	GPIO_Pin_1
#define USART_RS485_RE	GPIO_Pin_2

#define RS_485_RECEIVE  USART_RS485_GPIO->BSRRH|=USART_RS485_DE; USART_RS485_GPIO->BSRRH|=USART_RS485_RE;
#define RS_485_TRANSMIT USART_RS485_GPIO->BSRRL|=USART_RS485_DE; USART_RS485_GPIO->BSRRL|=USART_RS485_RE;


/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
	if(TRUE==xRxEnable)
	{
		USART_ITConfig(USART_RS485, USART_IT_RXNE, ENABLE);
		RS_485_RECEIVE;
	}
	else
	{
		USART_ITConfig(USART_RS485, USART_IT_RXNE, DISABLE);
		RS_485_TRANSMIT;
	}

	if(TRUE==xTxEnable)
	{
		USART_ITConfig(USART_RS485, USART_IT_TXE, ENABLE);
		RS_485_TRANSMIT;
		pxMBFrameCBTransmitterEmpty();
	}
	else
	{
	   USART_ITConfig(USART_RS485, USART_IT_TXE, DISABLE);
	   RS_485_RECEIVE;
	}
}

BOOL 
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
		GPIO_InitTypeDef GPIO_InitStruct;
		USART_InitTypeDef USART_InitStruct;
		NVIC_InitTypeDef NVIC_InitStructure;

		RCC_APB1PeriphClockCmd(RCC_USART_RS485, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_USART_RS485_GPIO, ENABLE);

		GPIO_InitStruct.GPIO_Pin = USART_RS485_TXD | USART_RS485_RXD;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(USART_RS485_GPIO, &GPIO_InitStruct);

		GPIO_PinAFConfig(USART_RS485_GPIO, USART_RS485_TXD_PIN_SOURCE, GPIO_AF_USART_RS485);
		GPIO_PinAFConfig(USART_RS485_GPIO, USART_RS485_RXD_PIN_SOURCE, GPIO_AF_USART_RS485);

		USART_InitStruct.USART_BaudRate = ulBaudRate;
		USART_InitStruct.USART_WordLength = USART_WordLength_8b;
		USART_InitStruct.USART_StopBits = USART_StopBits_1;
		USART_InitStruct.USART_Parity = USART_Parity_No;
		USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
		USART_Init(USART_RS485, &USART_InitStruct);

		GPIO_InitStruct.GPIO_Pin   = USART_RS485_DE|USART_RS485_RE;
		GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd =  GPIO_PuPd_NOPULL;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(USART_RS485_GPIO, &GPIO_InitStruct);

		USART_ClearFlag(USART_RS485,  USART_FLAG_TXE  | USART_FLAG_RXNE );

		USART_Cmd(USART_RS485, ENABLE);

		NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

		NVIC_InitStructure.NVIC_IRQChannel = USART_RS485_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		NVIC_EnableIRQ(USART_RS485_IRQn);

		vMBPortSerialEnable(TRUE,FALSE);

		return TRUE;
}


BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
	USART_RS485->DR=ucByte;
    return TRUE;
}


BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
	*pucByte=USART_RS485->DR;
    return TRUE;
}

void USART_RS485_IRQHandler(void)
{
 	static portBASE_TYPE xHigherPriorityTaskWoken;
 	xHigherPriorityTaskWoken = pdFALSE;

 	if(USART_GetITStatus(USART_RS485, USART_IT_RXNE) != RESET)
   	{
 		USART_ClearITPendingBit(USART_RS485, USART_IT_RXNE);
 		pxMBFrameCBByteReceived();
   	}

   	if(USART_GetITStatus(USART_RS485, USART_IT_TXE) != RESET)
   	{
   		USART_ClearITPendingBit(USART_RS485, USART_IT_TXE);
   		pxMBFrameCBTransmitterEmpty(  );
   	}

   	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

