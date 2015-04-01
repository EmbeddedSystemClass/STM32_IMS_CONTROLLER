#include "fram_i2c.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"

#define FRAM_I2C_GPIO				GPIOB
#define FRAM_I2C_GPIO_RCC			RCC_AHB1Periph_GPIOB
#define FRAM_I2C_GPIO_PIN_SDA		GPIO_Pin_6
#define FRAM_I2C_GPIO_PIN_SCL		GPIO_Pin_7
#define FRAM_I2C_GPIO_PINSOURCE_SDA	GPIO_PinSource6
#define FRAM_I2C_GPIO_PINSOURCE_SCL	GPIO_PinSource7

#define FRAM_I2C					I2C1
#define FRAM_I2C_RCC				RCC_APB1Periph_I2C1
#define FRAM_I2C_AF					GPIO_AF_I2C1
#define FRAM_I2C_ADDRESS			0x5

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
