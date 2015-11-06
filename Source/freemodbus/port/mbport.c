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

#define CURRENT_CHANNEL_0	2
#define CURRENT_CHANNEL_1	3
#define CURRENT_CHANNEL_2	1
#define CURRENT_CHANNEL_3	0
#define CURRENT_CHANNEL_4	6
#define CURRENT_CHANNEL_5	7
#define CURRENT_CHANNEL_6	5
#define CURRENT_CHANNEL_7	4

#define REG_PULSE_COUNTER_0		0
#define REG_PULSE_COUNTER_1		2

#define REG_CURRENT_RAW_0		4
#define REG_CURRENT_RAW_1		6
#define REG_CURRENT_RAW_2		8
#define REG_CURRENT_RAW_3		10
#define REG_CURRENT_RAW_4		12
#define REG_CURRENT_RAW_5		14
#define REG_CURRENT_RAW_6		16
#define REG_CURRENT_RAW_7		18

#define REG_FREQUENCY_0			20
#define REG_FREQUENCY_1			22
#define REG_RTD_0				24
#define REG_RTD_1				26

#define REG_CURRENT_0			28
#define REG_CURRENT_1			30
#define REG_CURRENT_2			32
#define REG_CURRENT_3			34
#define REG_CURRENT_4			36
#define REG_CURRENT_5			38
#define REG_CURRENT_6			40
#define REG_CURRENT_7			42


#define REG_VAL_0				44
#define REG_VAL_1				46
#define REG_VAL_2				48
#define REG_VAL_3				50
#define REG_VAL_4				52
#define REG_VAL_5				54
#define REG_VAL_6				56
#define REG_VAL_7				58


#define REG_TEST				60

u8 REG_INPUT_START=1,REG_HOLDING_START=1;
u8 REG_INPUT_NREGS=100,REG_HOLDING_NREGS=80;
u8 usRegInputStart=1,usRegHoldingStart=1;


eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
	eMBErrorCode    eStatus = MB_ENOERR;
	int             iRegIndex, j;
	u16 usRegInputBuf[REG_INPUT_NREGS];

    xSemaphoreTake( xMeasureDataMutex, portMAX_DELAY );
    {
    	*((uint32_t*)&usRegInputBuf[REG_PULSE_COUNTER_0])=(uint32_t)stMeasureData.pulse_counter[0];
    	*((uint32_t*)&usRegInputBuf[REG_PULSE_COUNTER_1])=(uint32_t)stMeasureData.pulse_counter[1];

    	*((uint32_t*)&usRegInputBuf[REG_CURRENT_RAW_0])=stMeasureData.current_raw[CURRENT_CHANNEL_0];
    	*((uint32_t*)&usRegInputBuf[REG_CURRENT_RAW_1])=stMeasureData.current_raw[CURRENT_CHANNEL_1];
    	*((uint32_t*)&usRegInputBuf[REG_CURRENT_RAW_2])=stMeasureData.current_raw[CURRENT_CHANNEL_2];
    	*((uint32_t*)&usRegInputBuf[REG_CURRENT_RAW_3])=stMeasureData.current_raw[CURRENT_CHANNEL_3];
    	*((uint32_t*)&usRegInputBuf[REG_CURRENT_RAW_4])=stMeasureData.current_raw[CURRENT_CHANNEL_4];
    	*((uint32_t*)&usRegInputBuf[REG_CURRENT_RAW_5])=stMeasureData.current_raw[CURRENT_CHANNEL_5];
    	*((uint32_t*)&usRegInputBuf[REG_CURRENT_RAW_6])=stMeasureData.current_raw[CURRENT_CHANNEL_6];
    	*((uint32_t*)&usRegInputBuf[REG_CURRENT_RAW_7])=stMeasureData.current_raw[CURRENT_CHANNEL_7];

    	*((float*)&usRegInputBuf[REG_FREQUENCY_0])=stMeasureData.frequency[0];
    	*((float*)&usRegInputBuf[REG_FREQUENCY_1])=stMeasureData.frequency[1];
    	*((float*)&usRegInputBuf[REG_RTD_0])=stMeasureData.rtd[0];
    	*((float*)&usRegInputBuf[REG_RTD_1])=stMeasureData.rtd[1];
    	*((float*)&usRegInputBuf[REG_CURRENT_0])=stMeasureData.current[CURRENT_CHANNEL_0];
    	*((float*)&usRegInputBuf[REG_CURRENT_1])=stMeasureData.current[CURRENT_CHANNEL_1];
    	*((float*)&usRegInputBuf[REG_CURRENT_2])=stMeasureData.current[CURRENT_CHANNEL_2];
    	*((float*)&usRegInputBuf[REG_CURRENT_3])=stMeasureData.current[CURRENT_CHANNEL_3];
    	*((float*)&usRegInputBuf[REG_CURRENT_4])=stMeasureData.current[CURRENT_CHANNEL_4];
    	*((float*)&usRegInputBuf[REG_CURRENT_5])=stMeasureData.current[CURRENT_CHANNEL_5];
    	*((float*)&usRegInputBuf[REG_CURRENT_6])=stMeasureData.current[CURRENT_CHANNEL_6];
    	*((float*)&usRegInputBuf[REG_CURRENT_7])=stMeasureData.current[CURRENT_CHANNEL_7];

    	for (j = 0; j < CURRENT_CHN_NUM; ++j) {
        	*((float*)&usRegInputBuf[REG_VAL_0+j*2])=stSettings.CurChannelCalibrate[REG_VAL_0+j*2].value;
		}
    	*((float*)&usRegInputBuf[REG_TEST])=stMeasureData.test_var;
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

			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR0_CODE_PNT0])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_0].code_pnt0;
//			     *(float*)(&usRegHoldingBuf[REG_CUR0_MA_PNT0])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_0].current_ma_pnt0;
			     *(float*)(&usRegHoldingBuf[REG_CUR0_MA_PNT0])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_0].current_4ma;
			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR0_CODE_PNT1])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_0].code_pnt1;
//			     *(float*)(&usRegHoldingBuf[REG_CUR0_MA_PNT1])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_0].current_ma_pnt1;
			     *(float*)(&usRegHoldingBuf[REG_CUR0_MA_PNT1])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_0].current_20ma;

			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR1_CODE_PNT0])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_1].code_pnt0;
			     *(float*)(&usRegHoldingBuf[REG_CUR1_MA_PNT0])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_1].current_4ma;
			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR1_CODE_PNT1])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_1].code_pnt1;
			     *(float*)(&usRegHoldingBuf[REG_CUR1_MA_PNT1])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_1].current_20ma;

			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR2_CODE_PNT0])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_2].code_pnt0;
			     *(float*)(&usRegHoldingBuf[REG_CUR2_MA_PNT0])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_2].current_4ma;
			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR2_CODE_PNT1])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_2].code_pnt1;
			     *(float*)(&usRegHoldingBuf[REG_CUR2_MA_PNT1])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_2].current_20ma;

			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR3_CODE_PNT0])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_3].code_pnt0;
			     *(float*)(&usRegHoldingBuf[REG_CUR3_MA_PNT0])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_3].current_4ma;
			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR3_CODE_PNT1])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_3].code_pnt1;
			     *(float*)(&usRegHoldingBuf[REG_CUR3_MA_PNT1])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_3].current_20ma;

			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR4_CODE_PNT0])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_4].code_pnt0;
			     *(float*)(&usRegHoldingBuf[REG_CUR4_MA_PNT0])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_4].current_4ma;
			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR4_CODE_PNT1])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_4].code_pnt1;
			     *(float*)(&usRegHoldingBuf[REG_CUR4_MA_PNT1])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_4].current_20ma;

			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR5_CODE_PNT0])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_5].code_pnt0;
			     *(float*)(&usRegHoldingBuf[REG_CUR5_MA_PNT0])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_5].current_4ma;
			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR5_CODE_PNT1])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_5].code_pnt1;
			     *(float*)(&usRegHoldingBuf[REG_CUR5_MA_PNT1])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_5].current_20ma;

			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR6_CODE_PNT0])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_6].code_pnt0;
			     *(float*)(&usRegHoldingBuf[REG_CUR6_MA_PNT0])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_6].current_4ma;
			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR6_CODE_PNT1])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_6].code_pnt1;
			     *(float*)(&usRegHoldingBuf[REG_CUR6_MA_PNT1])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_6].current_20ma;

			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR7_CODE_PNT0])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_7].code_pnt0;
			     *(float*)(&usRegHoldingBuf[REG_CUR7_MA_PNT0])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_7].current_4ma;
			     *(uint32_t*)(&usRegHoldingBuf[REG_CUR7_CODE_PNT1])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_7].code_pnt1;
			     *(float*)(&usRegHoldingBuf[REG_CUR7_MA_PNT1])	= stSettings.CurChannelCalibrate[CURRENT_CHANNEL_7].current_20ma;


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
//---------------------------------------------------------------------------------------------------------
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
									     stSettings.CurChannelCalibrate[CURRENT_CHANNEL_0].code_pnt0=temp;
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
									     stSettings.CurChannelCalibrate[CURRENT_CHANNEL_0].current_4ma=temp;
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
									     stSettings.CurChannelCalibrate[CURRENT_CHANNEL_0].code_pnt1=temp;
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
									     stSettings.CurChannelCalibrate[CURRENT_CHANNEL_0].current_20ma=temp;
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
							case REG_CUR1_CODE_PNT0:
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
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_1].code_pnt0=temp;
										 stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;

							case REG_CUR1_MA_PNT0:
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
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_1].current_4ma=temp;
										 stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;


							case REG_CUR1_CODE_PNT1:
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
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_1].code_pnt1=temp;
										 stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;

							case REG_CUR1_MA_PNT1:
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
//										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_1].current_ma_pnt1=temp;
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_1].current_20ma=temp;
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
							case REG_CUR2_CODE_PNT0:
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
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_2].code_pnt0=temp;
										 stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;

							case REG_CUR2_MA_PNT0:
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
//										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_2].current_ma_pnt0=temp;
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_2].current_4ma=temp;
										 stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;


							case REG_CUR2_CODE_PNT1:
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
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_2].code_pnt1=temp;
										 stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;

							case REG_CUR2_MA_PNT1:
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
//										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_2].current_ma_pnt1=temp;
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_2].current_20ma=temp;
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
							case REG_CUR3_CODE_PNT0:
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
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_3].code_pnt0=temp;
										 stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;

							case REG_CUR3_MA_PNT0:
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
//										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_3].current_ma_pnt0=temp;
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_3].current_4ma=temp;
										 stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;


							case REG_CUR3_CODE_PNT1:
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
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_3].code_pnt1=temp;
										 stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;

							case REG_CUR3_MA_PNT1:
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
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_3].current_20ma=temp;
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
							case REG_CUR4_CODE_PNT0:
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
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_4].code_pnt0=temp;
										 stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;

							case REG_CUR4_MA_PNT0:
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
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_4].current_4ma=temp;
										 stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;


							case REG_CUR4_CODE_PNT1:
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
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_4].code_pnt1=temp;
										 stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;

							case REG_CUR4_MA_PNT1:
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
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_4].current_20ma=temp;
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
							case REG_CUR5_CODE_PNT0:
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
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_5].code_pnt0=temp;
										 stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;

							case REG_CUR5_MA_PNT0:
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
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_5].current_4ma=temp;
										 stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;


							case REG_CUR5_CODE_PNT1:
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
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_5].code_pnt1=temp;
										 stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;

							case REG_CUR5_MA_PNT1:
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
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_5].current_20ma=temp;
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
							case REG_CUR6_CODE_PNT0:
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
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_6].code_pnt0=temp;
										 stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;

							case REG_CUR6_MA_PNT0:
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
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_6].current_4ma=temp;
										 stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;


							case REG_CUR6_CODE_PNT1:
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
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_6].code_pnt1=temp;
										 stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;

							case REG_CUR6_MA_PNT1:
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
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_6].current_20ma=temp;
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
							case REG_CUR7_CODE_PNT0:
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
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_7].code_pnt0=temp;
										 stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;

							case REG_CUR7_MA_PNT0:
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
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_7].current_4ma=temp;
										 stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;


							case REG_CUR7_CODE_PNT1:
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
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_7].code_pnt1=temp;
										 stSettingsCopy=stSettings;
									 }
									 xSemaphoreGive( xSettingsMutex );

									 FRAM_Write_Settings(stSettingsCopy);
								}

								iRegIndex+=2;
								usNRegs-=2;
							}
							break;

							case REG_CUR7_MA_PNT1:
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
										 stSettings.CurChannelCalibrate[CURRENT_CHANNEL_7].current_20ma=temp;
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
