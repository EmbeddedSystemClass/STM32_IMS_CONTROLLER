/**
  ******************************************************************************
  * @file    stm32f4_discovery.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    28-October-2011
  * @brief   This file contains definitions for STM32F4-Discovery Kit's Leds and
  *          push-button hardware resources.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************

  PATCHED BY CLIVE SOURCER32@GMAIL.COM TO SDIO

  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOW_LEVEL_SDIO_H
#define __LOW_LEVEL_SDIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_sdio.h"
#include "stm32f4xx_exti.h"
#include "misc.h"
#include "stm32f4xx_syscfg.h"


#define SD_DETECT_PIN                    GPIO_Pin_1                 /* PC.02 */
#define SD_DETECT_GPIO_PORT              GPIOD                      /* GPIOC */
#define SD_DETECT_GPIO_CLK               RCC_AHB1Periph_GPIOD

#define SDIO_FIFO_ADDRESS                ((uint32_t)0x40012C80)
/**
  * @brief  SDIO Intialization Frequency (400KHz max)
  */
#define SDIO_INIT_CLK_DIV                ((uint8_t)0x76)
/**
  * @brief  SDIO Data Transfer Frequency (25MHz max)
  */
#define SDIO_TRANSFER_CLK_DIV            ((uint8_t)0x2)

#define SD_SDIO_DMA                   DMA2
#define SD_SDIO_DMA_CLK               RCC_AHB1Periph_DMA2

#define SD_SDIO_DMA_STREAM3	          3
//#define SD_SDIO_DMA_STREAM6           6

#ifdef SD_SDIO_DMA_STREAM3
 #define SD_SDIO_DMA_STREAM            DMA2_Stream3
 #define SD_SDIO_DMA_CHANNEL           DMA_Channel_4
 #define SD_SDIO_DMA_FLAG_FEIF         DMA_FLAG_FEIF3
 #define SD_SDIO_DMA_FLAG_DMEIF        DMA_FLAG_DMEIF3
 #define SD_SDIO_DMA_FLAG_TEIF         DMA_FLAG_TEIF3
 #define SD_SDIO_DMA_FLAG_HTIF         DMA_FLAG_HTIF3
 #define SD_SDIO_DMA_FLAG_TCIF         DMA_FLAG_TCIF3
 #define SD_SDIO_DMA_IRQn              DMA2_Stream3_IRQn
 #define SD_SDIO_DMA_IRQHANDLER        DMA2_Stream3_IRQHandler
#elif defined SD_SDIO_DMA_STREAM6
 #define SD_SDIO_DMA_STREAM            DMA2_Stream6
 #define SD_SDIO_DMA_CHANNEL           DMA_Channel_4
 #define SD_SDIO_DMA_FLAG_FEIF         DMA_FLAG_FEIF6
 #define SD_SDIO_DMA_FLAG_DMEIF        DMA_FLAG_DMEIF6
 #define SD_SDIO_DMA_FLAG_TEIF         DMA_FLAG_TEIF6
 #define SD_SDIO_DMA_FLAG_HTIF         DMA_FLAG_HTIF6
 #define SD_SDIO_DMA_FLAG_TCIF         DMA_FLAG_TCIF6
 #define SD_SDIO_DMA_IRQn              DMA2_Stream6_IRQn
 #define SD_SDIO_DMA_IRQHANDLER        DMA2_Stream6_IRQHandler
#endif /* SD_SDIO_DMA_STREAM3 */



void SD_LowLevel_DeInit(void);
void SD_LowLevel_Init(void);
void SD_LowLevel_DMA_TxConfig(uint32_t *BufferSRC, uint32_t BufferSize);
void SD_LowLevel_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __LOW_LEVEL_SDIO_H */

