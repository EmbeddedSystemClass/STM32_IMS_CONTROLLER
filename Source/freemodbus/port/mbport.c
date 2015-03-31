#include "mb.h"
#include "backup_sram.h"

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


#define REG_INDEX_HEATER_TEMP_1			14
#define REG_INDEX_HEATER_TEMP_2			16
#define REG_INDEX_P_FACTOR				18
#define REG_INDEX_I_FACTOR				20
#define REG_INDEX_D_FACTOR				22
#define REG_DELTA_TEMP_START_DRYING		24
#define REG_TRESHOLD_TEMP_START_DRYING	26
#define REG_DELTA_TEMP_CANCEL_DRYING 	28
#define REG_HEATER_INIT_TIMEOUT			30
#define REG_MEASURING_FRAME_TIME		31

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
//    int             iRegIndex;
//	u16 *PRT=(u16*)pucRegBuffer;
//	uint16_t i=0;
//
//    if( ( usAddress >= REG_HOLDING_START ) && ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
//    {
//        iRegIndex = ( int )( usAddress - usRegHoldingStart );
//        REG_HOLDING_NREGS=(DRYING_CHANNELS_NUM+8)*2+2;//исправить
//        switch ( eMode )
//        {
//			case MB_REG_READ:
//			{
//
//			    for(i=0;i<DRYING_CHANNELS_NUM;i++)
//			    {
//			    	((float*)usRegHoldingBuf)[i] = uks_channels.uks_params.end_drying_temperature[i];
//			    }
//
//			    ((float*)usRegHoldingBuf)[DRYING_CHANNELS_NUM] = uks_channels.uks_params.heater_temperature_1;
//			    ((float*)usRegHoldingBuf)[DRYING_CHANNELS_NUM+1] = uks_channels.uks_params.heater_temperature_2;
//			    ((float*)usRegHoldingBuf)[DRYING_CHANNELS_NUM+2] = uks_channels.uks_params.p_factor;
//			    ((float*)usRegHoldingBuf)[DRYING_CHANNELS_NUM+3] = uks_channels.uks_params.i_factor;
//			    ((float*)usRegHoldingBuf)[DRYING_CHANNELS_NUM+4] = uks_channels.uks_params.d_factor;
//			    ((float*)usRegHoldingBuf)[DRYING_CHANNELS_NUM+5] = uks_channels.uks_params.delta_temp_start_drying;
//			    ((float*)usRegHoldingBuf)[DRYING_CHANNELS_NUM+6] = uks_channels.uks_params.treshold_temp_start_drying;
//			    ((float*)usRegHoldingBuf)[DRYING_CHANNELS_NUM+7] = uks_channels.uks_params.delta_temp_cancel_drying;
//			     usRegHoldingBuf[(DRYING_CHANNELS_NUM+8)*2] = uks_channels.uks_params.heater_init_timeout;
//			     usRegHoldingBuf[(DRYING_CHANNELS_NUM+8)*2+1] = uks_channels.uks_params.measuring_frame_time;
//
//
//				while( usNRegs > 0 )
//				{
//	 				*pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
//	                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
//					iRegIndex++;
//
//					usNRegs--;
//				}
//			}
//			break;
//
//			case MB_REG_WRITE:
//			{
//				while( usNRegs > 0 )
//				{
//					if((iRegIndex>=0)&&(iRegIndex<(DRYING_CHANNELS_NUM*2)))
//					{
//						float temp=0;
//
//						((uint8_t*)(&temp))[1]=*pucRegBuffer++;
//						((uint8_t*)(&temp))[0]=*pucRegBuffer++;
//						((uint8_t*)(&temp))[3]=*pucRegBuffer++;
//						((uint8_t*)(&temp))[2]=*pucRegBuffer++;
//
//						if((temp>=END_DRYING_TEMP_MIN)&&(temp<=END_DRYING_TEMP_MAX))
//						{
//							uks_channels.uks_params.end_drying_temperature[(iRegIndex>>1)]=temp;
//							Backup_SRAM_Write_Reg(&(uks_channels.backup_uks_params->end_drying_temperature[(iRegIndex>>1)]),&(uks_channels.uks_params.end_drying_temperature[(iRegIndex>>1)]),sizeof(float));
//						}
//
//						iRegIndex+=2;
//						usNRegs-=2;
//					}
//					else
//					{
//						switch(iRegIndex)
//						{
//							case REG_INDEX_HEATER_TEMP_1:
//							{
//								float temp=0;
//
//								((uint8_t*)(&temp))[1]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[0]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[3]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[2]=*pucRegBuffer++;
//
//								iRegIndex+=2;
//								usNRegs-=2;
//
//
//								if((temp>=HEATER_TEMP_MIN)&&(temp<=HEATER_TEMP_MAX))
//								{
//									uks_channels.uks_params.heater_temperature_1=temp;
//								    Backup_SRAM_Write_Reg(&uks_channels.backup_uks_params->heater_temperature_1,&uks_channels.uks_params.heater_temperature_1,sizeof(float));
//								}
//							}
//							break;
//
//							case REG_INDEX_HEATER_TEMP_2:
//							{
//								float temp=0;
//
//								((uint8_t*)(&temp))[1]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[0]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[3]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[2]=*pucRegBuffer++;
//
//								iRegIndex+=2;
//								usNRegs-=2;
//
//
//								if((temp>=HEATER_TEMP_MIN)&&(temp<=HEATER_TEMP_MAX))
//								{
//									uks_channels.uks_params.heater_temperature_2=temp;
//								    Backup_SRAM_Write_Reg(&uks_channels.backup_uks_params->heater_temperature_2,&uks_channels.uks_params.heater_temperature_2,sizeof(float));
//								}
//							}
//							break;
//
//							case REG_INDEX_P_FACTOR:
//							{
//								float temp=0;
//
//								((uint8_t*)(&temp))[1]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[0]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[3]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[2]=*pucRegBuffer++;
//								iRegIndex+=2;
//								usNRegs-=2;
//
//								if((temp>=P_FACTOR_MIN)&&(temp<=P_FACTOR_MAX))
//								{
//									uks_channels.uks_params.p_factor=temp;
//									Backup_SRAM_Write_Reg(&uks_channels.backup_uks_params->p_factor,&uks_channels.uks_params.p_factor,sizeof(float));
//								}
//							}
//							break;
//
//							case REG_INDEX_I_FACTOR:
//							{
//								float temp=0;
//
//								((uint8_t*)(&temp))[1]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[0]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[3]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[2]=*pucRegBuffer++;
//								iRegIndex+=2;
//								usNRegs-=2;
//
//								if((temp>=I_FACTOR_MIN)&&(temp<=I_FACTOR_MAX))
//								{
//									uks_channels.uks_params.i_factor=temp;
//									Backup_SRAM_Write_Reg(&uks_channels.backup_uks_params->i_factor,&uks_channels.uks_params.i_factor,sizeof(float));
//								}
//							}
//							break;
//
//							case REG_INDEX_D_FACTOR:
//							{
//								float temp=0;
//
//								((uint8_t*)(&temp))[1]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[0]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[3]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[2]=*pucRegBuffer++;
//								iRegIndex+=2;
//								usNRegs-=2;
//
//								if((temp>=D_FACTOR_MIN)&&(temp<=D_FACTOR_MAX))
//								{
//									uks_channels.uks_params.d_factor=temp;
//									Backup_SRAM_Write_Reg(&uks_channels.backup_uks_params->d_factor,&uks_channels.uks_params.d_factor,sizeof(float));
//								}
//							}
//							break;
////---------------------
//							case REG_DELTA_TEMP_START_DRYING:
//							{
//								float temp=0;
//
//								((uint8_t*)(&temp))[1]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[0]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[3]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[2]=*pucRegBuffer++;
//								iRegIndex+=2;
//								usNRegs-=2;
//
//								if((temp>=DELTA_TEMP_START_DRYING_MIN)&&(temp<=DELTA_TEMP_START_DRYING_MAX))
//								{
//									uks_channels.uks_params.delta_temp_start_drying=temp;
//									Backup_SRAM_Write_Reg(&uks_channels.backup_uks_params->delta_temp_start_drying,&uks_channels.uks_params.delta_temp_start_drying,sizeof(float));
//								}
//							}
//							break;
//
//							case REG_TRESHOLD_TEMP_START_DRYING:
//							{
//								float temp=0;
//
//								((uint8_t*)(&temp))[1]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[0]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[3]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[2]=*pucRegBuffer++;
//								iRegIndex+=2;
//								usNRegs-=2;
//
//								if((temp>=TRESHOLD_TEMP_START_DRYING_MIN)&&(temp<=TRESHOLD_TEMP_START_DRYING_MAX))
//								{
//									uks_channels.uks_params.treshold_temp_start_drying=temp;
//									Backup_SRAM_Write_Reg(&uks_channels.backup_uks_params->treshold_temp_start_drying,&uks_channels.uks_params.treshold_temp_start_drying,sizeof(float));
//								}
//							}
//							break;
//
//							case REG_DELTA_TEMP_CANCEL_DRYING:
//							{
//								float temp=0;
//
//								((uint8_t*)(&temp))[1]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[0]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[3]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[2]=*pucRegBuffer++;
//								iRegIndex+=2;
//								usNRegs-=2;
//
//								if((temp>=DELTA_TEMP_CANCEL_DRYING_MIN)&&(temp<=DELTA_TEMP_CANCEL_DRYING_MAX))
//								{
//									uks_channels.uks_params.delta_temp_cancel_drying=temp;
//									Backup_SRAM_Write_Reg(&uks_channels.backup_uks_params->delta_temp_cancel_drying,&uks_channels.uks_params.delta_temp_cancel_drying,sizeof(float));
//								}
//							}
//							break;
////--------
//							case REG_HEATER_INIT_TIMEOUT:
//							{
//								uint16_t temp=0;
//
//								((uint8_t*)(&temp))[1]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[0]=*pucRegBuffer++;
//
//								iRegIndex+=1;
//								usNRegs-=1;
//
//								if((temp>=HEATER_INIT_TIMEOUT_MIN)&&(temp<=HEATER_INIT_TIMEOUT_MAX))
//								{
//									uks_channels.uks_params.heater_init_timeout=temp;
//									Backup_SRAM_Write_Reg(&uks_channels.backup_uks_params->heater_init_timeout,&uks_channels.uks_params.heater_init_timeout,sizeof(uint16_t));
//								}
//							}
//							break;
//
//							case REG_MEASURING_FRAME_TIME:
//							{
//								uint16_t temp=0;
//
//								((uint8_t*)(&temp))[1]=*pucRegBuffer++;
//								((uint8_t*)(&temp))[0]=*pucRegBuffer++;
//
//								iRegIndex+=1;
//								usNRegs-=1;
//
//								if((temp>=MEASURING_FRAME_TIME_MIN)&&(temp<=MEASURING_FRAME_TIME_MAX))
//								{
//									uks_channels.uks_params.measuring_frame_time=temp;
//									Backup_SRAM_Write_Reg(&uks_channels.backup_uks_params->measuring_frame_time,&uks_channels.uks_params.measuring_frame_time,sizeof(uint16_t));
//								}
//							}
//							break;
//
//							default:
//							{
//								usNRegs--;
//							}
//							break;
//						}
//					}
//				}
//			}
//			break;
//        }
//    }
//    else
//    {
//        eStatus = MB_ENOREG;
//    }
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
