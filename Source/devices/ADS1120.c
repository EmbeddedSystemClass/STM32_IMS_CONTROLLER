#include "ADS1120.h"

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"
#include <misc.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "watchdog.h"

#include "stdio.h"
#include "string.h"



//static void ADS1120_task(void *pvParameters);//

static void ADC_RTD1_Task(void *pvParameters);
static void ADC_RTD2_Task(void *pvParameters);
static void ADC_Current1_Task(void *pvParameters);
static void ADC_Current2_Task(void *pvParameters);

static void	ADC_SPI_config(void);


xSemaphoreHandle xADC_SPI_Mutex;
extern struct task_watch task_watches[];

uint8_t ADS1220_init(void)//
{
	ADC_SPI_config();

	xADC_SPI_Mutex=xSemaphoreCreateMutex() ;

	xTaskCreate(ADC_RTD1_Task,(signed char*)"ADS1220 rtd1 task",128,NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(ADC_RTD2_Task,(signed char*)"ADS1220 rtd2 task",128,NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(ADC_Current1_Task,(signed char*)"ADS1220 current1 task",128,NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(ADC_Current2_Task,(signed char*)"ADS1220 current2 task",128,NULL, tskIDLE_PRIORITY + 1, NULL);
	return 0;
}

void	ADC_SPI_config(void)//
{
    	GPIO_InitTypeDef GPIO_InitStructure;
    	SPI_InitTypeDef SPI_InitStructure;

		RCC_AHB1PeriphClockCmd(ADC_SPI_GPIO_RCC, ENABLE);
	    RCC_APB1PeriphClockCmd(ADC_SPI_RCC, ENABLE);

	    GPIO_InitStructure.GPIO_Pin   = ADC_SPI_SCK|ADC_SPI_MOSI;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	    GPIO_Init(ADC_SPI_GPIO, &GPIO_InitStructure);

	    GPIO_InitStructure.GPIO_Pin   = ADC_SPI_MISO;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	    GPIO_Init(ADC_SPI_GPIO, &GPIO_InitStructure);

	    GPIO_InitStructure.GPIO_Pin   = ADC_SPI_CS1 | ADC_SPI_CS2 | ADC_SPI_CS3 | ADC_SPI_CS4;
	    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_Init(ADC_SPI_GPIO_CS, &GPIO_InitStructure);

		GPIO_PinAFConfig(ADC_SPI_GPIO, ADC_SPI_SCK_PinSource,  ADC_SPI_GPIO_AF);
		GPIO_PinAFConfig(ADC_SPI_GPIO, ADC_SPI_MOSI_PinSource, ADC_SPI_GPIO_AF);
		GPIO_PinAFConfig(ADC_SPI_GPIO, ADC_SPI_MISO_PinSource, ADC_SPI_GPIO_AF);

	    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	    SPI_Init(ADC_SPI, &SPI_InitStructure);

	    SPI_CalculateCRC(ADC_SPI, DISABLE);
	    SPI_Cmd(ADC_SPI, ENABLE);

	    ADC_SPI_GPIO_CS->BSRRL|=(ADC_SPI_CS1 | ADC_SPI_CS2 | ADC_SPI_CS3 | ADC_SPI_CS4);// pin up
}



uint8_t ADC_SPI_send (uint8_t data)
{
  while (!(ADC_SPI->SR & SPI_SR_TXE)) taskYIELD();
  ADC_SPI->DR = data;
  while (!(ADC_SPI->SR & SPI_SR_RXNE))taskYIELD();
  return (ADC_SPI->DR);
}

uint8_t ADC_SPI_read (void)
{
  return ADC_SPI_send(0xff);		  //читаем принятые данные
}

uint32_t ADC_result_temp;
int32_t ADC_result;
uint8_t adc_reg;

//struct ADS1120_result ADS1120_res;

enum
{
	ADS_REG_0=0x0,
	ADS_REG_1,
	ADS_REG_2,
	ADS_REG_3,
};

static void ADC_RTD1_Task(void *pvParameters)
{
    xSemaphoreTake( xADC_SPI_Mutex, portMAX_DELAY );
    {

    }
    xSemaphoreGive( xADC_SPI_Mutex );

	while(1)
	{
	    xSemaphoreTake( xADC_SPI_Mutex, portMAX_DELAY );
	    {

	    }
	    xSemaphoreGive( xADC_SPI_Mutex );
	    vTaskDelay(ADC_MEASURE_DELAY);
	}
}

static void ADC_RTD2_Task(void *pvParameters)
{
    xSemaphoreTake( xADC_SPI_Mutex, portMAX_DELAY );
    {

    }
    xSemaphoreGive( xADC_SPI_Mutex );

	while(1)
	{
	    xSemaphoreTake( xADC_SPI_Mutex, portMAX_DELAY );
	    {

	    }
	    xSemaphoreGive( xADC_SPI_Mutex );
	    vTaskDelay(ADC_MEASURE_DELAY);
	}
}

static void ADC_Current1_Task(void *pvParameters)
{
    xSemaphoreTake( xADC_SPI_Mutex, portMAX_DELAY );
    {

    }
    xSemaphoreGive( xADC_SPI_Mutex );

	while(1)
	{
	    xSemaphoreTake( xADC_SPI_Mutex, portMAX_DELAY );
	    {

	    }
	    xSemaphoreGive( xADC_SPI_Mutex );
	    vTaskDelay(ADC_MEASURE_DELAY);
	}
}

static void ADC_Current2_Task(void *pvParameters)
{
    xSemaphoreTake( xADC_SPI_Mutex, portMAX_DELAY );
    {

    }
    xSemaphoreGive( xADC_SPI_Mutex );

	while(1)
	{
	    xSemaphoreTake( xADC_SPI_Mutex, portMAX_DELAY );
	    {

	    }
	    xSemaphoreGive( xADC_SPI_Mutex );
	    vTaskDelay(ADC_MEASURE_DELAY);
	}
}


//----------------------------------------------------------------
//static void ADS1120_task(void *pvParameters)//
//{
////	uint8_t str_buf[32];
//
//	ADC_SPI_GPIO_CS->BSRRH|=ADC_SPI_CS1;// pin down SPI1_CS1
//	ADC_SPI_send (ADS_RESET);
//	vTaskDelay(10);
//	ADC_SPI_send (ADS_WREG|(ADS_REG_0<<2)|(0x0));//1 reg 0x0
//	ADC_SPI_send (ADC_REG_CONFIG_00);
//	vTaskDelay(10);
//
//	ADC_SPI_send (ADS_WREG|(ADS_REG_2<<2)|(0x0));//1 reg 0x2
//	ADC_SPI_send (ADC_REG_CONFIG_02);
//	vTaskDelay(10);
//
//	ADC_SPI_send (ADS_WREG|(ADS_REG_3<<2)|(0x0));//1 reg 0x3
//	ADC_SPI_send (ADC_REG_CONFIG_03);
//	vTaskDelay(10);
//
////	spi_send (ADS_RREG|(ADS_REG_0<<2)|(0x0));//1 reg 0x0
////	adc_reg=spi_read ();
////	vTaskDelay(10);
////	spi_send (ADS_RREG|(ADS_REG_3<<2)|(0x0));//1 reg 0x3
////	adc_reg=spi_read ();
////	vTaskDelay(10);
//
//	task_watches[ADS1120_TASK].task_status=TASK_ACTIVE;
//
//	while(1)
//	{
//		ADC_SPI_send (ADS_START);
//		while(GPIO_ReadInputDataBit(ADC_SPI_GPIO, ADC_SPI_MISO)==Bit_SET);//wait
//		//spi_send (ADS_RDATA);
//		ADC_result_temp=ADC_SPI_read ();
//		ADC_result_temp=ADC_result_temp<<8;
//		ADC_result_temp|=ADC_SPI_read ();
//		ADC_result_temp=ADC_result_temp<<8;
//		ADC_result_temp|=ADC_SPI_read ();
//
//
//
//		if(ADC_result_temp>0x7FFFFF)
//		{
//			ADC_result=-(0xFFFFFF-(int32_t)ADC_result_temp);
//		}
//		else
//		{
//			ADC_result=(int32_t)ADC_result_temp;
//		}
//
//
//		//ADS1120_res.result=ADC_result;
//		//uks_channels.heater_temperature=PT100_Code_To_Temperature(ADC_result);
//
//		vTaskDelay(100);
//		task_watches[ADS1120_TASK].counter++;
//	}
//}

#define CURRENT_SOURCE  0.001014
#define VOLTAGE_REF		2.0289
#define	TEMP_0_RES		100.0
#define TEMP_0			0.0
#define TEMP_200_RES	175.86
#define TEMP_200		200.0
#define ADC_MAX			0x7FFFFF
#define ADC_GAIN		8

float PT100_Code_To_Temperature(int32_t adc_code)
{
	int32_t code_temp_0, code_temp_200;
	float result_temp=0;

	code_temp_0  =(int32_t)(((float)(ADC_MAX)/VOLTAGE_REF)*CURRENT_SOURCE*TEMP_0_RES*ADC_GAIN);
	code_temp_200=(int32_t)(((float)(ADC_MAX)/VOLTAGE_REF)*CURRENT_SOURCE*TEMP_200_RES*ADC_GAIN);

	result_temp=(float)(((float)(adc_code-code_temp_0))*(TEMP_200-TEMP_0)/(code_temp_200-code_temp_0)+TEMP_0);
	return result_temp;
}
