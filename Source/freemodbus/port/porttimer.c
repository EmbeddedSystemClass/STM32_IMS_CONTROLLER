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
 * File: $Id: porttimer.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "mbport.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "misc.h"

#include "stdlib.h"

static stMBContext *stRS485Context;
/* ----------------------- Start implementation -----------------------------*/
BOOL
RS485TimersInit( USHORT usTim1Timerout50us )
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	uint16_t PrescalerValue = 0;


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	PrescalerValue = (uint16_t) (SystemCoreClock / 20000) - 1; // 1/20000=50us
	TIM_TimeBaseStructure.TIM_Period = (uint16_t) usTim1Timerout50us;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_ARRPreloadConfig(TIM4, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);


	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);

	TIM_Cmd(TIM4,  DISABLE);
	return TRUE;
}


void RS485TimersEnable(  )
{
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	TIM_SetCounter(TIM4,0x0000);
	TIM_Cmd(TIM4, ENABLE);
}

void RS485TimersDisable(  )
{
	TIM_Cmd(TIM4, DISABLE);
	TIM_SetCounter(TIM4,0x0000);
	TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
void TIM4_IRQHandler( void ) //
{
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	( void )stRS485Context->pxMBPortCBTimerExpired(&stRS485Context->stRTUContext,&stRS485Context->stTimer,&stRS485Context->stEvent  );//!!!
}

void RS485TimerContextInit(  stMBContext *stContext)
{
	stRS485Context=stContext;
	stRS485Context->stTimer.vMBPortTimersDelay=NULL;
	stRS485Context->stTimer.vMBPortTimersDisable=RS485TimersDisable;
	stRS485Context->stTimer.vMBPortTimersEnable=RS485TimersEnable;
	stRS485Context->stTimer.xMBPortTimersClose=NULL;
	stRS485Context->stTimer.xMBPortTimersInit=RS485TimersInit;
}



static stMBContext *stRS232Context;
/* ----------------------- Start implementation -----------------------------*/
BOOL
RS232TimersInit( USHORT usTim1Timerout50us )
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	uint16_t PrescalerValue = 0;


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	PrescalerValue = (uint16_t) (SystemCoreClock / 20000) - 1; // 1/20000=50us
	TIM_TimeBaseStructure.TIM_Period = (uint16_t) usTim1Timerout50us;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	TIM_ARRPreloadConfig(TIM5, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);


	TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
	TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);

	TIM_Cmd(TIM5,  DISABLE);
	return TRUE;
}


void RS232TimersEnable(  )
{
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	TIM_SetCounter(TIM5,0x0000);
	TIM_Cmd(TIM5, ENABLE);
}

void RS232TimersDisable(  )
{
	TIM_Cmd(TIM5, DISABLE);
	TIM_SetCounter(TIM5,0x0000);
	TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
void TIM5_IRQHandler( void ) //
{
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	( void )stRS232Context->pxMBPortCBTimerExpired(&stRS232Context->stRTUContext,&stRS232Context->stTimer,&stRS232Context->stEvent  );//!!!
}

void RS232TimerContextInit(  stMBContext *stContext)
{
	stRS232Context=stContext;
	stRS232Context->stTimer.vMBPortTimersDelay=NULL;
	stRS232Context->stTimer.vMBPortTimersDisable=RS232TimersDisable;
	stRS232Context->stTimer.vMBPortTimersEnable=RS232TimersEnable;
	stRS232Context->stTimer.xMBPortTimersClose=NULL;
	stRS232Context->stTimer.xMBPortTimersInit=RS232TimersInit;
}


