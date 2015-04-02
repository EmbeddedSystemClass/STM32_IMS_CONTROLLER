#include "mb.h"
//#include "backup_sram.h"
#include "fram_i2c.h"
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
u8 REG_INPUT_NREGS=20,REG_HOLDING_NREGS=20;
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
#define REG_TCXO_FREQ			7


eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    u16 usRegHoldingBuf[64];
    int             iRegIndex;

	//uint16_t i=0;

    if( ( usAddress >= REG_HOLDING_START ) && ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );

        switch ( eMode )
        {
			case MB_REG_READ:
			{
				RTC_TimeTypeDef RTC_TimeStructure;
				RTC_DateTypeDef RTC_DateStructure;

				RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
				RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);

			     usRegHoldingBuf[REG_RTC_SECOND] 				= RTC_TimeStructure.RTC_Seconds;
			     usRegHoldingBuf[REG_RTC_MINUTE] 				= RTC_TimeStructure.RTC_Minutes;
			     usRegHoldingBuf[REG_RTC_HOUR] 	 				= RTC_TimeStructure.RTC_Hours;
			     usRegHoldingBuf[REG_RTC_DAY_OF_MONTH] 			= RTC_DateStructure.RTC_Date;
			     usRegHoldingBuf[REG_RTC_MONTH] 				= RTC_DateStructure.RTC_Month;
			     usRegHoldingBuf[REG_RTC_YEAR]  				= RTC_DateStructure.RTC_Year;
			     usRegHoldingBuf[REG_RTC_DAY_OF_WEEK]  			= RTC_DateStructure.RTC_WeekDay;
			     *(uint32_t*)(&usRegHoldingBuf[REG_TCXO_FREQ])	= stSettings.TCXO_frequency;


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

								RTC_TimeTypeDef RTC_TimeStructure;
								RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);

								((uint8_t*)(&temp))[1]=*pucRegBuffer++;
								((uint8_t*)(&temp))[0]=*pucRegBuffer++;

								RTC_TimeStructure.RTC_Seconds=temp;

								RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);

								iRegIndex++;
								usNRegs--;

//								if((temp>=HEATER_TEMP_MIN)&&(temp<=HEATER_TEMP_MAX))
//								{
//									//uks_channels.uks_params.heater_temperature_1=temp;
//								    //Backup_SRAM_Write_Reg(&uks_channels.backup_uks_params->heater_temperature_1,&uks_channels.uks_params.heater_temperature_1,sizeof(float));
//								}
							}
							break;

							case REG_RTC_MINUTE:
							{
								uint16_t temp=0;

								RTC_TimeTypeDef RTC_TimeStructure;
								RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);

								((uint8_t*)(&temp))[1]=*pucRegBuffer++;
								((uint8_t*)(&temp))[0]=*pucRegBuffer++;

								RTC_TimeStructure.RTC_Minutes=temp;

								RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);

								iRegIndex++;
								usNRegs--;
							}
							break;

							case REG_RTC_HOUR:
							{
								uint16_t temp=0;

								RTC_TimeTypeDef RTC_TimeStructure;
								RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);

								((uint8_t*)(&temp))[1]=*pucRegBuffer++;
								((uint8_t*)(&temp))[0]=*pucRegBuffer++;

								RTC_TimeStructure.RTC_Hours=temp;

								RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);

								iRegIndex++;
								usNRegs--;
							}
							break;

							case REG_RTC_DAY_OF_MONTH:
							{
								uint16_t temp=0;

								RTC_DateTypeDef RTC_DateStructure;
								RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);

								((uint8_t*)(&temp))[1]=*pucRegBuffer++;
								((uint8_t*)(&temp))[0]=*pucRegBuffer++;

								RTC_DateStructure.RTC_Date=temp;

								RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);

								iRegIndex++;
								usNRegs--;
							}
							break;

							case REG_RTC_MONTH:
							{
								uint16_t temp=0;

								RTC_DateTypeDef RTC_DateStructure;
								RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);

								((uint8_t*)(&temp))[1]=*pucRegBuffer++;
								((uint8_t*)(&temp))[0]=*pucRegBuffer++;

								RTC_DateStructure.RTC_Month=temp;

								RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);

								iRegIndex++;
								usNRegs--;
							}
							break;

							case REG_RTC_YEAR:
							{
								uint16_t temp=0;

								RTC_DateTypeDef RTC_DateStructure;
								RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);

								((uint8_t*)(&temp))[1]=*pucRegBuffer++;
								((uint8_t*)(&temp))[0]=*pucRegBuffer++;

								RTC_DateStructure.RTC_Year=temp;

								RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);

								iRegIndex++;
								usNRegs--;
							}
							break;

							case REG_RTC_DAY_OF_WEEK:
							{
								uint16_t temp=0;

								RTC_DateTypeDef RTC_DateStructure;
								RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);

								((uint8_t*)(&temp))[1]=*pucRegBuffer++;
								((uint8_t*)(&temp))[0]=*pucRegBuffer++;

								RTC_DateStructure.RTC_WeekDay=temp;

								RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);

								iRegIndex++;
								usNRegs--;
							}
							break;

							case REG_TCXO_FREQ:
							{
								uint32_t temp=0;
								stControllerSettings stSettingsCopy;

								((uint8_t*)(&temp))[1]=*pucRegBuffer++;
								((uint8_t*)(&temp))[0]=*pucRegBuffer++;
								((uint8_t*)(&temp))[3]=*pucRegBuffer++;
								((uint8_t*)(&temp))[2]=*pucRegBuffer++;

								if((temp>=TCXO_FREQ_MIN) && (temp<=TCXO_FREQ_MAX))
								{
									 xSemaphoreTake( xSettingsMutex, portMAX_DELAY );
									 {
									     stSettings.TCXO_frequency=temp;
									     stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
									 /*Write settings to ROM*/
								}

								iRegIndex+=2;
								usNRegs-=2;
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
