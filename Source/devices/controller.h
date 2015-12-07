#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f4xx_rtc.h"
#include "stdint.h"

typedef enum
{
    ENOERR,                  /*!< no error. */
    ENOREG,                  /*!< illegal register address. */
    EINVAL,                  /*!< illegal argument. */
    ENORES,                  /*!< insufficient resources. */
    EIO,                     /*!< I/O error. */
    ETIMEDOUT,                /*!< timeout error occurred. */
    ECRCERR
} eErrorCode;


#define TCXO_FREQ_MIN		19999000
#define TCXO_FREQ_MAX		20001000
#define TCXO_FREQ_DEFAULT	20000000

#define REG_CUR_CODE_MIN		0
#define REG_CUR_CODE_MAX		0x7FFFFF
#define REG_CUR_CODE_DEFAULT 	0

#define REG_CUR_MA_MIN		 	0.0
#define REG_CUR_MA_MAX			20.0
#define REG_CUR_4MA_DEFAULT		4.0

#define FREQ_CHN_NUM			2
#define PULSE_COUNT_CHN_NUM		2
#define RTD_CHN_NUM				2
#define CURRENT_CHN_NUM			8

#define CONTROLLER_MEASURE_DATA_LEN	64
#define TIMESTAMP_LEN				6

typedef struct
{
	uint32_t code_pnt0;
//	float current_ma_pnt0;
	float current_4ma;
	uint32_t code_pnt1;
//	float current_ma_pnt1;
	float current_20ma;

	float zero;
	float full;
	float value;
}stCurChannelCalibrate;

typedef struct
{
	uint32_t TCXO_frequency;
	stCurChannelCalibrate CurChannelCalibrate[CURRENT_CHN_NUM];
}stControllerSettings;

//---------------------------------------
#define KFACTOR_POINT_NUM	10

typedef enum
{
	SENSOR_TYPE_PRESSURE_ABS=0,
	SENSOR_TYPE_PRESSURE_OVER,
	SENSOR_TYPE_TEMPERATURE,
	SENSOR_TYPE_IMPULSE,
}enSensorType;

typedef enum
{
	INPUT_VAL_REAL=0,
	INPUT_VAL_FIXED,
}enInputValType;

typedef struct
{
	uint32_t code;
	float    value;
}stCalibr;

typedef struct
{
	uint32_t frequency;
	float	 value;
}stKfactor;

//typedef struct
//{
//	uint8_t 		channel_num;
//	enSensorType 	sensor_type;
//	stCalibr		calibr_low;
//	stCalibr		calibr_hi;
//	float 			threshold_low;
//	float			threshold_hi;
//	uint8_t 		fix_flag;
//	uint32_t		fix_val;
//	stKfactor		Kfactor[KFACTOR_POINT_NUM];
//
//
//}stSensor;
//---------------------------------------
//typedef struct
//{
//	uint8_t 		channel_num;
//	enSensorType 	sensor_type;
//	stCalibr		calibr_low;
//	stCalibr		calibr_hi;
//	float 			threshold_low;
//	float			threshold_hi;
//	enInputValType 	fix_flag;
//	float			fix_val;
//}stMBSettingsAnalogInput;
//
//typedef struct
//{
//	uint8_t 		channel_num;
//	stKfactor		Kfactor[KFACTOR_POINT_NUM];
//	float 			threshold_low;
//	float			threshold_hi;
//	enInputValType 	fix_flag;
//	float			fix_val;
//}stMBSettingsFrequencyInput;
#define MODBUS_PASS_LEN	20
#define MEASURING_CHANNELS_NUM	12
typedef struct//���� �������� ��� (holding ��������)
{
	uint8_t	Pass_Modbus[MODBUS_PASS_LEN];					//������ �������
	uint8_t	RTC_Seconds	;									//������� ���� � �����
	uint8_t	RTC_Minutes;
	uint8_t	RTC_Hours;
	uint8_t	RTC_Date;
	uint8_t	RTC_Month;
	uint8_t	RTC_Year;

	uint8_t	Arch_Num;											//	����� ������������ ������
	uint8_t	Arch_Seconds;										//	���� � ����� �������������  �������� ������
	uint8_t	Arch_Minutes;
	uint8_t	Arch_Hours;
	uint8_t	Arch_Date;
	uint8_t	Arch_Month;
	uint8_t	Arch_Year;

	uint8_t	Sensor_Measuring_Channel[MEASURING_CHANNELS_NUM];	//	������� ������������ ������� �������������� ������
	uint8_t	Sensor_Type[MEASURING_CHANNELS_NUM];				//	��� �������
	uint8_t	Sensor_Use_Fix_Value[MEASURING_CHANNELS_NUM];		//	������������ ������������� �������� ������� ��� �������

	float	Analog_Channel_4mA_Value[CURRENT_CHN_NUM];			//	�������� ���������� ������� ��� ���� 4mA
	float	Analog_Channel_20mA_Value[CURRENT_CHN_NUM];			//	�������� ���������� ������� ��� ���� 20mA

	float	Frequency_Channel_1_Kfactor_Freq[KFACTOR_POINT_NUM];//	������ ������ �������� �������
	float	Frequency_Channel_1_Kfactor_Value[KFACTOR_POINT_NUM];//	�������� �-������� ��� �������� �������
	float	Frequency_Channel_2_Kfactor_Freq[KFACTOR_POINT_NUM];//	������ ������ �������� �������
	float	Frequency_Channel_2_Kfactor_Value[KFACTOR_POINT_NUM];//	�������� �-������� ��� �������� �������

	float	Channel_Treshold_Low[MEASURING_CHANNELS_NUM];		//	������ �������� ������ ����������� �������
	float	Channel_Treshold_High[MEASURING_CHANNELS_NUM];		//	������� �������� ������ ����������� �������
	float	Channel_Fix_Value[MEASURING_CHANNELS_NUM];			//	������������� ��������, ������������ ��� ������������� �������
}stSettings;

#define MEASURING_LINE_NUM	2

typedef struct//������ ���������� �������� (input ��������)
{
	float		V_RU[MEASURING_LINE_NUM];//	����� ��� ��
	float		V_SU[MEASURING_LINE_NUM];//	����� ��� ��
	float		Mass[MEASURING_LINE_NUM];//		�����
	float		Temperature[MEASURING_LINE_NUM];//		�����������
	float		Pressure[MEASURING_LINE_NUM];//		��������
//	float[2];	//������
//	float[2];	//������
	uint32_t	Alarm;//	32-������ ����� ��������� ��������
	uint32_t	Warning;//	32-������ ����� ��������������
	uint32_t	Time_Work;//	����� ������ � ��������

}stInstantVals;

typedef struct//������ �������� ������(input ��������)
{
	uint8_t	Arch_Seconds;//	���� � ����� ��������� ������������  �������� ������
	uint8_t	Arch_Minutes;
	uint8_t	Arch_Hours;
	uint8_t	Arch_Date;
	uint8_t	Arch_Month;
	uint8_t	Arch_Year;

	float	V_RU[MEASURING_LINE_NUM];//	����� ��� ��
	float	V_SU[MEASURING_LINE_NUM];//	����� ��� ��
	float	Mass[MEASURING_LINE_NUM];//	�����
	float	Temperature[MEASURING_LINE_NUM];//	������� �����������
	float	Pressure[MEASURING_LINE_NUM];//	������� ��������
	float	Energy[MEASURING_LINE_NUM];//	�������
	//float[2]	������
	//float[2]	������
	uint16_t	Code_Alarm;//	��� ��������� ��������
	uint16_t	CRC_Sum;//	����������� �����
}stArchiveRead;
//---------------------------------------

typedef struct
{
	float frequency[FREQ_CHN_NUM];
	uint64_t last_counter[PULSE_COUNT_CHN_NUM];
	uint64_t pulse_counter[PULSE_COUNT_CHN_NUM];

	float rtd[RTD_CHN_NUM];
	float current_raw[CURRENT_CHN_NUM];
	float current[CURRENT_CHN_NUM];
	float test_var;


}stControllerMeasureData;

//typedef struct
//{
//
//}stControllerError;

extern xSemaphoreHandle xMeasureDataMutex;
extern xSemaphoreHandle xSettingsMutex;

extern stControllerSettings stSettings;
extern const stControllerSettings stSettingsDefault;
extern stControllerMeasureData stMeasureData;

short fl_led;
uint32_t	my_counter;

void ControllerInit(void);

#endif
