#include "mb.h"
#include "fram_i2c.h"
#include "stm32f4xx_rtc.h"
#include "controller.h"
#include "log.h"

void ENTER_CRITICAL_SECTION(void)
{

}

void EXIT_CRITICAL_SECTION(void)
{

}

//static volatile u16 usRegInputBuf[128];
//u16 *usRegHoldingBuf=usRegInputBuf;

u8 REG_INPUT_START=1,REG_HOLDING_START=1;
u8 REG_INPUT_NREGS=50,REG_HOLDING_NREGS=80;
u8 usRegInputStart=1,usRegHoldingStart=1;


eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
	eMBErrorCode    eStatus = MB_ENOERR;
	int             iRegIndex;
	u16 usRegInputBuf[REG_INPUT_NREGS];

    xSemaphoreTake( xMeasureDataMutex, portMAX_DELAY );
    {
    	((float*)usRegInputBuf)[0]=stMeasureData.frequency[0];
    	((float*)usRegInputBuf)[1]=stMeasureData.frequency[1];
    	((uint32_t*)usRegInputBuf)[2]=(uint32_t)stMeasureData.pulse_counter[0];
    	((uint32_t*)usRegInputBuf)[3]=(uint32_t)stMeasureData.pulse_counter[1];
    	((float*)usRegInputBuf)[4]=stMeasureData.rtd[0];
    	((float*)usRegInputBuf)[5]=stMeasureData.rtd[1];
    	((float*)usRegInputBuf)[6]=stMeasureData.current[2];
    	((float*)usRegInputBuf)[7]=stMeasureData.current[3];
    	((float*)usRegInputBuf)[8]=stMeasureData.current[1];
    	((float*)usRegInputBuf)[9]=stMeasureData.current[0];
    	((float*)usRegInputBuf)[10]=stMeasureData.current[6];
    	((float*)usRegInputBuf)[11]=stMeasureData.current[7];
    	((float*)usRegInputBuf)[12]=stMeasureData.current[5];
    	((float*)usRegInputBuf)[13]=stMeasureData.current[4];

    	((uint32_t*)usRegInputBuf)[14]=stMeasureData.current_raw[2];
    	((uint32_t*)usRegInputBuf)[15]=stMeasureData.current_raw[3];
    	((uint32_t*)usRegInputBuf)[16]=stMeasureData.current_raw[1];
    	((uint32_t*)usRegInputBuf)[17]=stMeasureData.current_raw[0];
    	((uint32_t*)usRegInputBuf)[18]=stMeasureData.current_raw[6];
    	((uint32_t*)usRegInputBuf)[19]=stMeasureData.current_raw[7];
    	((uint32_t*)usRegInputBuf)[20]=stMeasureData.current_raw[5];
    	((uint32_t*)usRegInputBuf)[21]=stMeasureData.current_raw[4];
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

#define REG_CUR0_CODE_PNT0		9
#define REG_CUR0_MA_PNT0		11
#define REG_CUR0_CODE_PNT1		13
#define REG_CUR0_MA_PNT1		15

#define REG_CUR1_CODE_PNT0		17
#define REG_CUR1_MA_PNT0		19
#define REG_CUR1_CODE_PNT1		21
#define REG_CUR1_MA_PNT1		23

#define REG_CUR2_CODE_PNT0		25
#define REG_CUR2_MA_PNT0		27
#define REG_CUR2_CODE_PNT1		29
#define REG_CUR2_MA_PNT1		31

#define REG_CUR3_CODE_PNT0		33
#define REG_CUR3_MA_PNT0		35
#define REG_CUR3_CODE_PNT1		37
#define REG_CUR3_MA_PNT1		39

#define REG_CUR4_CODE_PNT0		41
#define REG_CUR4_MA_PNT0		43
#define REG_CUR4_CODE_PNT1		45
#define REG_CUR4_MA_PNT1		47

#define REG_CUR5_CODE_PNT0		49
#define REG_CUR5_MA_PNT0		51
#define REG_CUR5_CODE_PNT1		53
#define REG_CUR5_MA_PNT1		55

#define REG_CUR6_CODE_PNT0		57
#define REG_CUR6_MA_PNT0		59
#define REG_CUR6_CODE_PNT1		61
#define REG_CUR6_MA_PNT1		63

#define REG_CUR7_CODE_PNT0		65
#define REG_CUR7_MA_PNT0		67
#define REG_CUR7_CODE_PNT1		69
#define REG_CUR7_MA_PNT1		71

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    u16 usRegHoldingBuf[REG_HOLDING_NREGS];
    int             iRegIndex;

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

			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR0_CODE_PNT0])	= stSettings.CurChannelCalibrate[2].code_pnt0;
			     *(float*)(&usRegHoldingBuf[REG_CUR0_MA_PNT0])	= stSettings.CurChannelCalibrate[2].current_ma_pnt0;
			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR0_CODE_PNT1])	= stSettings.CurChannelCalibrate[2].code_pnt1;
			     *(float*)(&usRegHoldingBuf[REG_CUR0_MA_PNT1])	= stSettings.CurChannelCalibrate[2].current_ma_pnt1;
//
//			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR1_CODE_PNT0])	= stSettings.CurChannelCalibrate[3].code_pnt0;
//			     *(float*)(&usRegHoldingBuf[REG_CUR1_MA_PNT0])	= stSettings.CurChannelCalibrate[3].current_ma_pnt0;
//			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR1_CODE_PNT1])	= stSettings.CurChannelCalibrate[3].code_pnt1;
//			     *(float*)(&usRegHoldingBuf[REG_CUR1_MA_PNT1])	= stSettings.CurChannelCalibrate[3].current_ma_pnt1;
//
//			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR2_CODE_PNT0])	= stSettings.CurChannelCalibrate[1].code_pnt0;
//			     *(float*)(&usRegHoldingBuf[REG_CUR2_MA_PNT0])	= stSettings.CurChannelCalibrate[1].current_ma_pnt0;
//			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR2_CODE_PNT1])	= stSettings.CurChannelCalibrate[1].code_pnt1;
//			     *(float*)(&usRegHoldingBuf[REG_CUR2_MA_PNT1])	= stSettings.CurChannelCalibrate[1].current_ma_pnt1;
//
//			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR3_CODE_PNT0])	= stSettings.CurChannelCalibrate[0].code_pnt0;
//			     *(float*)(&usRegHoldingBuf[REG_CUR3_MA_PNT0])	= stSettings.CurChannelCalibrate[0].current_ma_pnt0;
//			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR3_CODE_PNT1])	= stSettings.CurChannelCalibrate[0].code_pnt1;
//			     *(float*)(&usRegHoldingBuf[REG_CUR3_MA_PNT1])	= stSettings.CurChannelCalibrate[0].current_ma_pnt1;
//
//			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR4_CODE_PNT0])	= stSettings.CurChannelCalibrate[6].code_pnt0;
//			     *(float*)(&usRegHoldingBuf[REG_CUR4_MA_PNT0])	= stSettings.CurChannelCalibrate[6].current_ma_pnt0;
//			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR4_CODE_PNT1])	= stSettings.CurChannelCalibrate[6].code_pnt1;
//			     *(float*)(&usRegHoldingBuf[REG_CUR4_MA_PNT1])	= stSettings.CurChannelCalibrate[6].current_ma_pnt1;
//
//			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR5_CODE_PNT0])	= stSettings.CurChannelCalibrate[7].code_pnt0;
//			     *(float*)(&usRegHoldingBuf[REG_CUR5_MA_PNT0])	= stSettings.CurChannelCalibrate[7].current_ma_pnt0;
//			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR5_CODE_PNT1])	= stSettings.CurChannelCalibrate[7].code_pnt1;
//			     *(float*)(&usRegHoldingBuf[REG_CUR5_MA_PNT1])	= stSettings.CurChannelCalibrate[7].current_ma_pnt1;
//
//			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR6_CODE_PNT0])	= stSettings.CurChannelCalibrate[5].code_pnt0;
//			     *(float*)(&usRegHoldingBuf[REG_CUR6_MA_PNT0])	= stSettings.CurChannelCalibrate[5].current_ma_pnt0;
//			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR6_CODE_PNT1])	= stSettings.CurChannelCalibrate[5].code_pnt1;
//			     *(float*)(&usRegHoldingBuf[REG_CUR6_MA_PNT1])	= stSettings.CurChannelCalibrate[5].current_ma_pnt1;
//
//			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR7_CODE_PNT0])	= stSettings.CurChannelCalibrate[4].code_pnt0;
//			     *(float*)(&usRegHoldingBuf[REG_CUR7_MA_PNT0])	= stSettings.CurChannelCalibrate[4].current_ma_pnt0;
//			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR7_CODE_PNT1])	= stSettings.CurChannelCalibrate[4].code_pnt1;
//			     *(float*)(&usRegHoldingBuf[REG_CUR7_MA_PNT1])	= stSettings.CurChannelCalibrate[4].current_ma_pnt1;


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
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;

							case REG_CUR0_CODE_PNT0:
							{
								uint32_t temp=0;
								stControllerSettings stSettingsCopy;

								((uint8_t*)(&temp))[1]=*pucRegBuffer++;
								((uint8_t*)(&temp))[0]=*pucRegBuffer++;
								((uint8_t*)(&temp))[3]=*pucRegBuffer++;
								((uint8_t*)(&temp))[2]=*pucRegBuffer++;

								if((temp>=REG_CUR_CODE_MIN) && (temp<=REG_CUR_CODE_MAX))
								{
									 xSemaphoreTake( xSettingsMutex, portMAX_DELAY );
									 {
									     stSettings.CurChannelCalibrate[2].code_pnt0=temp;
									     stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;

							case REG_CUR0_MA_PNT0:
							{
								float temp=0;
								stControllerSettings stSettingsCopy;

								((uint8_t*)(&temp))[1]=*pucRegBuffer++;
								((uint8_t*)(&temp))[0]=*pucRegBuffer++;
								((uint8_t*)(&temp))[3]=*pucRegBuffer++;
								((uint8_t*)(&temp))[2]=*pucRegBuffer++;

								if((temp>=REG_CUR_MA_MIN) && (temp<=REG_CUR_MA_MAX))
								{
									 xSemaphoreTake( xSettingsMutex, portMAX_DELAY );
									 {
									     stSettings.CurChannelCalibrate[2].current_ma_pnt0=temp;
									     stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;


							case REG_CUR0_CODE_PNT1:
							{
								uint32_t temp=0;
								stControllerSettings stSettingsCopy;

								((uint8_t*)(&temp))[1]=*pucRegBuffer++;
								((uint8_t*)(&temp))[0]=*pucRegBuffer++;
								((uint8_t*)(&temp))[3]=*pucRegBuffer++;
								((uint8_t*)(&temp))[2]=*pucRegBuffer++;

								if((temp>=REG_CUR_CODE_MIN) && (temp<=REG_CUR_CODE_MAX))
								{
									 xSemaphoreTake( xSettingsMutex, portMAX_DELAY );
									 {
									     stSettings.CurChannelCalibrate[2].code_pnt1=temp;
									     stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;

							case REG_CUR0_MA_PNT1:
							{
								float temp=0;
								stControllerSettings stSettingsCopy;

								((uint8_t*)(&temp))[1]=*pucRegBuffer++;
								((uint8_t*)(&temp))[0]=*pucRegBuffer++;
								((uint8_t*)(&temp))[3]=*pucRegBuffer++;
								((uint8_t*)(&temp))[2]=*pucRegBuffer++;

								if((temp>=REG_CUR_MA_MIN) && (temp<=REG_CUR_MA_MAX))
								{
									 xSemaphoreTake( xSettingsMutex, portMAX_DELAY );
									 {
									     stSettings.CurChannelCalibrate[2].current_ma_pnt1=temp;
									     stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;
//------------------------------------------------------------------------------------------------------------------


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

#define FILE_0	0

#define MB_FRAM_REG_LEN						( CONTROLLER_MEASURE_DATA_LEN + TIMESTAMP_LEN )

eMBErrorCode    eMBFileCB( UCHAR * pucFileBuffer, xMBReadFileRequest* xReadFileRequest, USHORT usNFiles )
{
	eMBErrorCode    eStatus = MB_ENOERR;
	eErrorCode		buf_err=ENOERR;
	USHORT file_cnt=0,reg_cnt=0;
	UCHAR reg_buf[MB_FRAM_REG_LEN];

	for(file_cnt=0;file_cnt<usNFiles;file_cnt++)
	{
		switch(xReadFileRequest[file_cnt].usFileNum)
		{
			case FILE_0:
			{
				for(reg_cnt=xReadFileRequest[file_cnt].usStartAddress;reg_cnt<(xReadFileRequest[file_cnt].usStartAddress+xReadFileRequest[file_cnt].usRegNum);reg_cnt++)
				{
					*pucFileBuffer++=MB_FRAM_REG_LEN+1;
					*pucFileBuffer++=0x6;
					buf_err=Log_Read_LogEntry(reg_cnt,reg_buf);

					if(buf_err==ENOERR)
					{
						UCHAR i=0;
						for(i=0;i<MB_FRAM_REG_LEN;i++)
						{
							*pucFileBuffer++=reg_buf[i];
						}
					}
					else
					{
						eStatus=MB_ENOREG;
					}
				}
			}
			break;

			default:
			{
				eStatus=MB_ENOREG;
			}
			break;
		}
	}
	return eStatus;
}
