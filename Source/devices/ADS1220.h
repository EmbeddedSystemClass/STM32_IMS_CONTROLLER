#ifndef ADS1220_H
#define ADS1220_H

#include "stm32f4xx.h"

#define ADC_SPI							SPI1
#define ADC_SPI_RCC						RCC_APB2Periph_SPI1
#define ADC_SPI_GPIO_AF					GPIO_AF_SPI1
#define ADC_SPI_GPIO_RCC 	    		RCC_AHB1Periph_GPIOA
#define ADC_SPI_CS_GPIO_RCC 			RCC_AHB1Periph_GPIOE
#define ADC_SPI_GPIO					GPIOA
#define ADC_SPI_GPIO_CS					GPIOE

#define ADC_SPI_SCK						GPIO_Pin_5
#define ADC_SPI_MISO					GPIO_Pin_6
#define ADC_SPI_MOSI					GPIO_Pin_7

#define ADC_SPI_SCK_PinSource			GPIO_PinSource5
#define ADC_SPI_MISO_PinSource			GPIO_PinSource6
#define ADC_SPI_MOSI_PinSource			GPIO_PinSource7

#define ADC_SPI_CS1						GPIO_Pin_8
#define ADC_SPI_CS2						GPIO_Pin_10
#define ADC_SPI_CS3						GPIO_Pin_12
#define ADC_SPI_CS4						GPIO_Pin_14

#define ADC_DRDY1						GPIO_Pin_9
#define ADC_DRDY2						GPIO_Pin_11
#define ADC_DRDY3						GPIO_Pin_13
#define ADC_DRDY4						GPIO_Pin_15

#define ADS_RESET		0x06
#define ADS_START		0x08
#define ADS_POWERDOWN	0x02
#define ADS_RDATA		0x10
#define ADS_RREG		0x20
#define ADS_WREG		0x40

//reg config
#define ADC_REG_CONFIG_RTD_00	0x06//gain =8
#define ADC_REG_CONFIG_RTD_01	0x00
#define ADC_REG_CONFIG_RTD_02	0x56
#define ADC_REG_CONFIG_RTD_03	0x62

#define ADC_REG_CONFIG_CUR_00	0x80
#define ADC_REG_CONFIG_CUR_01	0x00
#define ADC_REG_CONFIG_CUR_02	0x00//0x10
#define ADC_REG_CONFIG_CUR_03	0x00

#define ADC_MEASURE_DELAY	100



uint8_t ADS1220_init(void);


#endif
