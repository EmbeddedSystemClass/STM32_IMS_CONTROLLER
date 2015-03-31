#include "mb.h"
#include "backup_sram.h"
#include "stm32f4xx_rtc.h"
#include "controller.h"

void ENTER_CRITICAL_SECTION(void)
{

}

void EXIT_CRITICAL_SECTION(void)
{

}



//static volatile u16 usRegInputBuf[128];
//u16 *usRegHoldingBuf=usRegInputBuf;

u8 REG_INPUT_START=1,REG_HOLDING_START=1;
u8 REG_INPUT_NREGS=20,REG_HOLDING_NREGS=48;
u8 usRegInputStart=1,usRegHoldingStart=1;


eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
	eMBErrorCode    eStatus = MB_ENOERR;
	int             iRegIndex;
	u16 usRegInputBuf[32];
 //   uint16_t i=0;

 //   REG_INPUT_NREGS=16;

    xSemaphoreTake( xMeasureDataMutex, portMAX_DELAY );
    {
    	((float*)usRegInputBuf)[0]=stMeasureData.frequency[0];
    	((float*)usRegInputBuf)[1]=stMeasureData.frequency[1];
    	((float*)usRegInputBuf)[2]=stMeasureData.rtd[0];
    	((float*)usRegInputBuf)[3]=stMeasureData.rtd[1];
    	((float*)usRegInputBuf)[4]=stMeasureData.current[0];
    	((float*)usRegInputBuf)[5]=stMeasureData.current[1];
    	((float*)usRegInputBuf)[6]=stMeasureData.current[2];
    	((float*)usRegInputBuf)[7]=stMeasureData.current[3];
    	((float*)usRegInputBuf)[8]=stMeasureData.current[4];
    	((float*)usRegInputBuf)[9]=stMeasureData.current[5];
    }
    xSemaphoreGive( xMeasureDataMutex );

    if( ( usAddress >= REG_INPUT_START )&& ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}


#define REG_RTC_SECOND			0
#define REG_RTC_MINUTE			1
#define REG_RTC_HOUR			2
#define REG_RTC_DAY_OF_MONTH	3
#define REG_RTC_MONTH			4
#define REG_RTC_YEAR			5
#define REG_RTC_DAY_OF_WEEK		6


eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    u16 usRegHoldingBuf[64];
//    int             iRegIndex;
//	u16 *PRT=(u16*)pucRegBuffer;
	uint16_t i=0;

    if( ( usAddress >= REG_HOLDING_START ) && ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        REG_HOLDING_NREGS=(DRYING_CHANNELS_NUM+8)*2+2;//���������
        switch ( eMode )
        {
			case MB_REG_READ:
			{
				RTC_TimeTypeDef RTC_TimeStructure;
				RTC_DateTypeDef RTC_DateStructure;

				RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
				RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);

			     usRegHoldingBuf[REG_RTC_SECOND] = uks_channels.uks_params.measuring_frame_time;


				while( usNRegs > 0 )
				{
	 				*pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
	                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
					iRegIndex++;

					usNRegs--;
				}
			}
			break;

			case MB_REG_WRITE:
			{
				while( usNRegs > 0 )
				{
						switch(iRegIndex)
						{
							case REG_RTC_SECOND:
							{
								uint16_t temp=0;

								((uint8_t*)(&temp))[1]=*pucRegBuffer++;
								((uint8_t*)(&temp))[0]=*pucRegBuffer++;

								iRegIndex+=2;
								usNRegs-=2;

								if((temp>=HEATER_TEMP_MIN)&&(temp<=HEATER_TEMP_MAX))
								{
									//uks_channels.uks_params.heater_temperature_1=temp;
								    //Backup_SRAM_Write_Reg(&uks_channels.backup_uks_params->heater_temperature_1,&uks_channels.uks_params.heater_temperature_1,sizeof(float));
								}
							}
							break;


							default:
							{
								usNRegs--;
							}
							break;
						}
				}
			}
			break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}



eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode )
{
    ( void )pucRegBuffer;
    ( void )usAddress;
    ( void )usNCoils;
    ( void )eMode;
    return MB_ENOREG;
}


eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    ( void )pucRegBuffer;
    ( void )usAddress;
    ( void )usNDiscrete;
    return MB_ENOREG;
}
