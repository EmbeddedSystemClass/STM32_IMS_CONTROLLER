#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "wiznet.h"
#include <stdio.h>
#include <string.h>
#include "socket.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

////////////////////////////////////////////////
// Shared Buffer Definition for LOOPBACK TEST //
////////////////////////////////////////////////
#define DATA_BUF_SIZE   2048
uint8_t gDATABUF[DATA_BUF_SIZE];

///////////////////////////////////
// Default Network Configuration //
///////////////////////////////////
wiz_NetInfo gWIZNETINFO = { .mac = {0x00, 0x08, 0xdc,0x00, 0xab, 0xcd},
                            .ip = {10, 189, 62, 123},
                            .sn = {255,255,255,0},
                            .gw = {10, 189, 62, 1},
                            .dns = {0,0,0,0},
                            .dhcp = NETINFO_STATIC };


// initialize the dependent host peripheral
void platform_init(void);
void reverse(char s[]);
void itoa(int n, char s[]);


//////////
// TODO //
//////////////////////////////////////////////////////////////////////////////////////////////
// Call back function for W5500 SPI - Theses used as parameter of reg_wizchip_xxx_cbfunc()  //
// Should be implemented by WIZCHIP users because host is dependent                         //
//////////////////////////////////////////////////////////////////////////////////////////////
void  wizchip_select(void);
void  wizchip_deselect(void);
void  wizchip_write(uint8_t wb);
uint8_t wizchip_read();
//////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////
// For example of ioLibrary_BSD //
//////////////////////////////////
void network_init(void);								// Initialize Network information and display it
int32_t tcp_http_mt(uint8_t, uint8_t*, uint16_t);		// Multythread TCP server
//void HTTP_reset(uint8_t sockn);
////////////////////////////////////
//
//
////states for multythread http
//#define HTTP_IDLE 0
//#define HTTP_SENDING 1
//
////variables for multythread http
//uint32_t sentsize[_WIZCHIP_SOCK_NUM_];
//uint8_t http_state[_WIZCHIP_SOCK_NUM_];
//FIL fs[_WIZCHIP_SOCK_NUM_];



static void W5500_task(void *pvParameters);

uint8_t W5500_rxtx(uint8_t data)
{
	while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE)==RESET);
	SPI_I2S_SendData(SPI3,data);

	while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE)==RESET);
	return SPI_I2S_ReceiveData(SPI3);
}




void spiW5500_init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;

    /* Configure SPI1 pins: SCK, MISO and MOSI -------------------------------*/
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);


    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI3);

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI3, &SPI_InitStructure);



    //SPI_CalculateCRC(SPI3, DISABLE);
    SPI_Cmd(SPI3, ENABLE);

    GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_RESET);
//--------------------------------------------------------------------------
//	DMA_InitTypeDef DMA_InitStructure;
//
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
//	DMA_StructInit(&DMA_InitStructure);
//	DMA_DeInit(DMA1_Stream5);
//
//	//-----------------------------
//	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
//	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI3->DR)) ;
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
//	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
//	DMA_Init(DMA1_Stream5,&DMA_InitStructure);
//
//
//	DMA_ClearFlag(DMA1_Stream5,DMA_FLAG_TCIF5);
//	SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);
//
//
//	DMA_DeInit(DMA1_Stream3);
//
//	//-----------------------------
//	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
//	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI3->DR)) ;
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
//	DMA_Init(DMA1_Stream3,&DMA_InitStructure);
//
//	DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);
//	SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Rx, ENABLE);

	xTaskCreate(W5500_task,(signed char*)"W5500 task",128,NULL, tskIDLE_PRIORITY + 1, NULL);
}

//void spiW5500_write_buf(uint16_t* pBuffer, uint16_t len)
//{
//	  DMA1_Stream5->M0AR= (uint32_t)pBuffer;
//	  DMA1_Stream5->NDTR= len;
//
//	  DMA_ClearFlag(DMA1_Stream5,DMA_FLAG_TCIF5);
//
//	  DMA_Cmd(DMA1_Stream5, ENABLE);
//}
//
//void spiW5500_read_buf(uint16_t* pBuffer, uint16_t len)
//{
//	  DMA1_Stream5->M0AR= (uint32_t)pBuffer;
//	  DMA1_Stream5->NDTR= len;
//
//	  DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);
//
//	  DMA_Cmd(DMA1_Stream3, ENABLE);
//}

static void W5500_task(void *pvParameters)
{
//	uint8_t i=0;

	while(1)
	{
				//spi3_write_buf(&tab.buses[BUS_SPI_3].bus_buf[i][0],IND_SPI_BUS_3_NUM);

//				while(DMA_GetFlagStatus(DMA1_Stream5,DMA_FLAG_TCIF5)==RESET)
//				{
//					taskYIELD();
//				}
//				DMA_Cmd(DMA1_Stream5, DISABLE);
//				DMA_ClearFlag(DMA1_Stream5,DMA_FLAG_TCIF5);
//
//				while(SPI3->SR & SPI_SR_BSY)
//				{
//					taskYIELD();
//				}

		//W5500_rxtx(0xAA);
		//task_watches[SPI_TASK_3].counter++;
		vTaskDelay(50);
	}
}

void  wizchip_select(void)
{
	W5500_select();
}

void  wizchip_deselect(void)
{
	W5500_release();
}

void  wizchip_write(uint8_t wb)
{
	W5500_tx(wb);
}

uint8_t wizchip_read()
{
   return W5500_rx();
}
//////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////
// Intialize the network information to be used in WIZCHIP //
/////////////////////////////////////////////////////////////
void network_init(void)
{
   uint8_t tmpstr[6];

	ctlnetwork(CN_SET_NETINFO, (void*)&gWIZNETINFO);

	ctlwizchip(CW_GET_ID,(void*)tmpstr);
}
/////////////////////////////////////////////////////////////
void platform_init(void)
{
	spiW5500_init();

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//****************************************************
// reverse:
void reverse(char s[])
{
    int i, j;
    char c;

    for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
        c = s[i];
        s[i] = s[j];
        s[j] = c;
    }
}

//*******************************************************
// itoa
 void itoa(int n, char s[])
 {
     int i, sign;

     if ((sign = n) < 0)  /* sign */
         n = -n;          /* make positive n */
     i = 0;
     do {       /* generate reverse */
         s[i++] = n % 10 + '0';   /* next digit */
     } while ((n /= 10) > 0);     /* delete */
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
 }

 //*********************************************

