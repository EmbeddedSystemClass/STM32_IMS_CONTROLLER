#include "fram_i2c.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "misc.h"

#include "mbcrc.h"
#include "controller.h"

#define FRAM_I2C_GPIO				GPIOB
#define FRAM_I2C_GPIO_RCC			RCC_AHB1Periph_GPIOB
#define FRAM_I2C_GPIO_PIN_SDA		GPIO_Pin_9
#define FRAM_I2C_GPIO_PIN_SCL		GPIO_Pin_8
#define FRAM_I2C_GPIO_PINSOURCE_SDA	GPIO_PinSource9
#define FRAM_I2C_GPIO_PINSOURCE_SCL	GPIO_PinSource8

#define FRAM_I2C					I2C1
#define FRAM_I2C_RCC				RCC_APB1Periph_I2C1
#define FRAM_I2C_AF					GPIO_AF_I2C1
#define FRAM_I2C_ADDRESS			0xA0
#define FRAM_I2C_TIMEOUT			10000

xSemaphoreHandle xI2CBusMutex;

void FRAM_I2C_Init(void)
{
		 GPIO_InitTypeDef GPIO_InitStructure;
	     I2C_InitTypeDef  I2C_InitStructure;
	     RCC_APB1PeriphClockCmd(FRAM_I2C_RCC, ENABLE);

	     RCC_AHB1PeriphClockCmd(FRAM_I2C_GPIO_RCC, ENABLE);

	     GPIO_PinAFConfig(FRAM_I2C_GPIO, FRAM_I2C_GPIO_PINSOURCE_SDA, FRAM_I2C_AF);
	     GPIO_PinAFConfig(FRAM_I2C_GPIO, FRAM_I2C_GPIO_PINSOURCE_SCL, FRAM_I2C_AF);

	     GPIO_InitStructure.GPIO_Pin = FRAM_I2C_GPIO_PIN_SCL | FRAM_I2C_GPIO_PIN_SDA;
	     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	     GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	     GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	     GPIO_Init(FRAM_I2C_GPIO, &GPIO_InitStructure);

	     I2C_DeInit(FRAM_I2C);

	     I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	     I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	     I2C_InitStructure.I2C_OwnAddress1 = FRAM_I2C_ADDRESS;
	     I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	     I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	     I2C_InitStructure.I2C_ClockSpeed = 400000;

	     I2C_Init(FRAM_I2C, &I2C_InitStructure);
	     I2C_Cmd(FRAM_I2C, ENABLE);

	     xI2CBusMutex=xSemaphoreCreateMutex() ;
}

eErrorCode FRAM_I2C_Read_Buffer(uint16_t addr,uint8_t *buf, uint16_t buf_len)
{
	uint32_t i2c_timeout=0;
	 xSemaphoreTake( xI2CBusMutex, portMAX_DELAY );
	 {
		uint16_t i=0;
		 I2C_AcknowledgeConfig(FRAM_I2C, ENABLE);

	    i2c_timeout=0;
	    while(I2C_GetFlagStatus(FRAM_I2C, I2C_FLAG_BUSY))
	    {
	    	i2c_timeout++;
	    	if(i2c_timeout>=FRAM_I2C_TIMEOUT)
	    	{
	    		return ETIMEDOUT;
	    	}
	    }

	    I2C_GenerateSTART(FRAM_I2C, ENABLE);

	    i2c_timeout=0;
	    while(!I2C_CheckEvent(FRAM_I2C, I2C_EVENT_MASTER_MODE_SELECT))
	    {
	    	i2c_timeout++;
	    	if(i2c_timeout>=FRAM_I2C_TIMEOUT)
	    	{
	    		return ETIMEDOUT;
	    	}
	    }

	    I2C_Send7bitAddress(FRAM_I2C, FRAM_I2C_ADDRESS, I2C_Direction_Transmitter);

	    i2c_timeout=0;
	    while(!I2C_CheckEvent(FRAM_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	    {
	    	i2c_timeout++;
	    	if(i2c_timeout>=FRAM_I2C_TIMEOUT)
	    	{
	    		return ETIMEDOUT;
	    	}
	    }

	    I2C_SendData(FRAM_I2C, (uint8_t)((addr & 0xFF00) >> 8));

	    i2c_timeout=0;
	    while(!I2C_CheckEvent(FRAM_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	    {
	    	i2c_timeout++;
	    	if(i2c_timeout>=FRAM_I2C_TIMEOUT)
	    	{
	    		return ETIMEDOUT;
	    	}
	    }

	    I2C_SendData(FRAM_I2C, (uint8_t)(addr & 0x00FF));

	    i2c_timeout=0;
	    while(!I2C_CheckEvent(FRAM_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	    {
	    	i2c_timeout++;
	    	if(i2c_timeout>=FRAM_I2C_TIMEOUT)
	    	{
	    		return ETIMEDOUT;
	    	}
	    }
//----------------------------------------------------------------------------------
	    I2C_GenerateSTART(FRAM_I2C, ENABLE);

	    i2c_timeout=0;
	    while(!I2C_CheckEvent(FRAM_I2C, I2C_EVENT_MASTER_MODE_SELECT))
	    {
	    	i2c_timeout++;
	    	if(i2c_timeout>=FRAM_I2C_TIMEOUT)
	    	{
	    		return ETIMEDOUT;
	    	}
	    }

	    I2C_Send7bitAddress(FRAM_I2C, FRAM_I2C_ADDRESS, I2C_Direction_Receiver);

	    for(i=0;i<buf_len;i++)
	    {
	    	i2c_timeout=0;
	    	while(!I2C_CheckEvent(FRAM_I2C,I2C_EVENT_MASTER_BYTE_RECEIVED))
	    	{
		    	i2c_timeout++;
		    	if(i2c_timeout>=FRAM_I2C_TIMEOUT)
		    	{
		    		return ETIMEDOUT;
		    	}
	    	}

	    	buf[i]=I2C_ReceiveData(FRAM_I2C);

	    	if(i==(buf_len-1))
	    	{
	    		 I2C_AcknowledgeConfig(FRAM_I2C, DISABLE);
	    	}
	    	else
	    	{
	    		 I2C_AcknowledgeConfig(FRAM_I2C, ENABLE);
	    	}
	    }

	    I2C_GenerateSTOP(FRAM_I2C, ENABLE);
	 }
	 xSemaphoreGive( xI2CBusMutex );

	    return ENOERR;
}

eErrorCode FRAM_I2C_Write_Buffer(uint16_t addr,uint8_t *buf, uint16_t buf_len)
{
	uint32_t i2c_timeout=0;
	 xSemaphoreTake( xI2CBusMutex, portMAX_DELAY );
	 {
		uint16_t i=0;

	    I2C_GenerateSTART(FRAM_I2C, ENABLE);

	    i2c_timeout=0;
	    while(!I2C_CheckEvent(FRAM_I2C, I2C_EVENT_MASTER_MODE_SELECT))
	    {
	    	i2c_timeout++;
	    	if(i2c_timeout>=FRAM_I2C_TIMEOUT)
	    	{
	    		return ETIMEDOUT;
	    	}
	    }

	    I2C_Send7bitAddress(FRAM_I2C, FRAM_I2C_ADDRESS, I2C_Direction_Transmitter);

	    i2c_timeout=0;
	    while(!I2C_CheckEvent(FRAM_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	    {
	    	i2c_timeout++;
	    	if(i2c_timeout>=FRAM_I2C_TIMEOUT)
	    	{
	    		return ETIMEDOUT;
	    	}
	    }

	    I2C_SendData(FRAM_I2C, (uint8_t)((addr & 0xFF00) >> 8));

	    i2c_timeout=0;
	    while(!I2C_CheckEvent(FRAM_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	    {
	    	i2c_timeout++;
	    	if(i2c_timeout>=FRAM_I2C_TIMEOUT)
	    	{
	    		return ETIMEDOUT;
	    	}
	    }

	    I2C_SendData(FRAM_I2C, (uint8_t)(addr & 0x00FF));

	    i2c_timeout=0;
	    while(! I2C_CheckEvent(FRAM_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	    {
	    	i2c_timeout++;
	    	if(i2c_timeout>=FRAM_I2C_TIMEOUT)
	    	{
	    		return ETIMEDOUT;
	    	}
	    }

	    for(i=0;i<buf_len;i++)
	    {
		    I2C_SendData(FRAM_I2C, buf[i]);

		    i2c_timeout=0;
		    while (!I2C_CheckEvent(FRAM_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		    {
		    	i2c_timeout++;
		    	if(i2c_timeout>=FRAM_I2C_TIMEOUT)
		    	{
		    		return ETIMEDOUT;
		    	}
		    }
	    }

	    I2C_GenerateSTOP(FRAM_I2C, ENABLE);
	 }
	 xSemaphoreGive( xI2CBusMutex );

	    return ENOERR;
}


eErrorCode FRAM_Read_Settings(stControllerSettings *stSettings)
{
	stControllerSettings stSettings_temp;
	uint16_t Settings_CRC=0;
	if(FRAM_I2C_Read_Buffer(FRAM_SETTINGS_ADDR,&stSettings_temp,sizeof(stSettings_temp))!=ENOERR)
	{
		return EIO;
	}

	if(FRAM_I2C_Read_Buffer(FRAM_SETTINGS_CRC_ADDR,&Settings_CRC,sizeof(Settings_CRC))!=ENOERR)
	{
		return EIO;
	}

	if(Settings_CRC==usMBCRC16(&stSettings_temp,sizeof(stSettings_temp)))
	{
		*stSettings=stSettings_temp;
		return ENOERR;
	}
	else
	{
		*stSettings=stSettingsDefault;
		return ECRCERR;
	}
}

eErrorCode FRAM_Write_Settings(stControllerSettings stSettings)
{
	uint16_t Settings_CRC=0;
	Settings_CRC=usMBCRC16(&stSettings,sizeof(stSettings));

	if(FRAM_I2C_Write_Buffer(FRAM_SETTINGS_ADDR,&stSettings,sizeof(stSettings))!=ENOERR)
	{
		return EIO;
	}

	if(FRAM_I2C_Write_Buffer(FRAM_SETTINGS_CRC_ADDR,&Settings_CRC,sizeof(Settings_CRC))!=ENOERR)
	{
		return EIO;
	}

	return ENOERR;
}


