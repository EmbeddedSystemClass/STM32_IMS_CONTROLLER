#include "fram_i2c.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "misc.h"

#define FRAM_I2C_GPIO				GPIOB
#define FRAM_I2C_GPIO_RCC			RCC_AHB1Periph_GPIOB
#define FRAM_I2C_GPIO_PIN_SDA		GPIO_Pin_9
#define FRAM_I2C_GPIO_PIN_SCL		GPIO_Pin_8
#define FRAM_I2C_GPIO_PINSOURCE_SDA	GPIO_PinSource9
#define FRAM_I2C_GPIO_PINSOURCE_SCL	GPIO_PinSource8

#define FRAM_I2C					I2C1
#define FRAM_I2C_RCC				RCC_APB1Periph_I2C1
#define FRAM_I2C_AF					GPIO_AF_I2C1
#define FRAM_I2C_ADDRESS			0x50

void FRAM_I2C_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

    RCC_APB1PeriphClockCmd(FRAM_I2C_RCC, ENABLE);
    RCC_AHB1PeriphClockCmd(FRAM_I2C_GPIO_RCC, ENABLE);

    I2C_InitStructure.I2C_ClockSpeed = 400000;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;

    I2C_InitStructure.I2C_OwnAddress1 = FRAM_I2C_ADDRESS;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(FRAM_I2C, &I2C_InitStructure);

    GPIO_InitStructure.GPIO_Pin = FRAM_I2C_GPIO_PIN_SDA | FRAM_I2C_GPIO_PINSOURCE_SCL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(FRAM_I2C_GPIO, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB, FRAM_I2C_GPIO_PINSOURCE_SDA, FRAM_I2C_AF);
    GPIO_PinAFConfig(GPIOB, FRAM_I2C_GPIO_PINSOURCE_SCL, FRAM_I2C_AF);

    I2C_Cmd(FRAM_I2C, ENABLE);
}

uint8_t FRAM_I2C_Read_Buffer(uint16_t addr,uint8_t *buf, uint16_t buf_len)
{
		uint16_t i=0;
	        /* While the bus is busy */
	    while(I2C_GetFlagStatus(FRAM_I2C, I2C_FLAG_BUSY));

	    /* Send START condition */
	    I2C_GenerateSTART(FRAM_I2C, ENABLE);

	    /* Test on EV5 and clear it */
	    while(!I2C_CheckEvent(FRAM_I2C, I2C_EVENT_MASTER_MODE_SELECT));

	    /* Send EEPROM address for write */
	    I2C_Send7bitAddress(FRAM_I2C, FRAM_I2C_ADDRESS, I2C_Direction_Transmitter);

	    /* Test on EV6 and clear it */
	    while(!I2C_CheckEvent(FRAM_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));


	    /* Send the EEPROM's internal address to read from: MSB of the address first */
	    I2C_SendData(FRAM_I2C, (uint8_t)((addr & 0xFF00) >> 8));

	    /* Test on EV8 and clear it */
	    while(!I2C_CheckEvent(FRAM_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	    /* Send the EEPROM's internal address to read from: LSB of the address */
	    I2C_SendData(FRAM_I2C, (uint8_t)(addr & 0x00FF));

	    /* Test on EV8 and clear it */
	    while(!I2C_CheckEvent(FRAM_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
//----------------------------------------------------------------------------------

	    /* Send STRAT condition a second time */
	    I2C_GenerateSTART(FRAM_I2C, ENABLE);

	    /* Test on EV5 and clear it */
	    while(!I2C_CheckEvent(FRAM_I2C, I2C_EVENT_MASTER_MODE_SELECT));

	    /* Send EEPROM address for read */
	    I2C_Send7bitAddress(FRAM_I2C, FRAM_I2C_ADDRESS, I2C_Direction_Receiver);

	    /* Test on EV6 and clear it */

	    for(i=0;i<buf_len;i++)
	    {
	    	while(!I2C_CheckEvent(FRAM_I2C,I2C_EVENT_MASTER_BYTE_RECEIVED));//I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

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

	    /* Send STOP Condition */
	    I2C_GenerateSTOP(FRAM_I2C, ENABLE);

	    return 0;
}

uint8_t FRAM_I2C_Write_Buffer(uint16_t addr,uint8_t *buf, uint16_t buf_len)
{
		uint16_t i=0;
		/* Send START condition */
	    I2C_GenerateSTART(FRAM_I2C, ENABLE);

	    /* Test on EV5 and clear it */
	    while(!I2C_CheckEvent(FRAM_I2C, I2C_EVENT_MASTER_MODE_SELECT));

	    /* Send EEPROM address for write */
	    I2C_Send7bitAddress(FRAM_I2C, FRAM_I2C_ADDRESS, I2C_Direction_Transmitter);

	    /* Test on EV6 and clear it */
	    while(!I2C_CheckEvent(FRAM_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));


	    /* Send the EEPROM's internal address to write to : MSB of the address first */
	    I2C_SendData(FRAM_I2C, (uint8_t)((addr & 0xFF00) >> 8));

	    /* Test on EV8 and clear it */
	    while(!I2C_CheckEvent(FRAM_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));



	    /* Send the EEPROM's internal address to write to : LSB of the address */
	    I2C_SendData(FRAM_I2C, (uint8_t)(addr & 0x00FF));

	    /* Test on EV8 and clear it */
	    while(! I2C_CheckEvent(FRAM_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	    for(i=0;i<buf_len;i++)
	    {
		    I2C_SendData(FRAM_I2C, buf[i]);
		        /* Test on EV8 and clear it */
		    while (!I2C_CheckEvent(FRAM_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	    }

	    /* Send STOP condition */
	    I2C_GenerateSTOP(FRAM_I2C, ENABLE);

	    return 0;
}
