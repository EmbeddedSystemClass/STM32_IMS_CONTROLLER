#ifndef ADS1120_H
#define ADS1120_H

#include "stm32f4xx.h"


#define SPI2_GPIO_BUS 	    		RCC_AHB1Periph_GPIOB
#define SPI2_CS_GPIO_BUS 			RCC_AHB1Periph_GPIOB
#define SPI2_GPIO					GPIOB
#define SPI2_GPIO_CS				GPIOB

#define SPI2_SCK					GPIO_Pin_13
#define SPI2_MISO					GPIO_Pin_14
#define SPI2_MOSI					GPIO_Pin_15

#define SPI2_SCK_PinSource			GPIO_PinSource13
#define SPI2_MISO_PinSource			GPIO_PinSource14
#define SPI2_MOSI_PinSource			GPIO_PinSource15

#define SPI2_CS1					GPIO_Pin_12


#define ADS_RESET		0x06
#define ADS_START		0x08
#define ADS_POWERDOWN	0x02
#define ADS_RDATA		0x10
#define ADS_RREG		0x20
#define ADS_WREG		0x40

//reg config
#define ADC_REG_CONFIG_00	0x06//gain =8
#define ADC_REG_CONFIG_01	0x00
#define ADC_REG_CONFIG_02	0x56
#define ADC_REG_CONFIG_03	0x62

#define ADC_FILTER_BUFFER_LEN	8

//struct ADS1120_result
//{
//	int32_t  result;
//	int32_t filter_buffer[ADC_FILTER_BUFFER_LEN];
//	uint8_t  filter_counter;
//};

uint8_t ADS1120_init(void);
float PT100_Code_To_Temperature(int32_t adc_code);

#endif
