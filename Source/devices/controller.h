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
typedef struct//База настроек ИВК (holding регистры)
{
	uint8_t	Pass_Modbus[MODBUS_PASS_LEN];					//Пароль доступа
	uint8_t	RTC_Seconds	;									//Текущие дата и время
	uint8_t	RTC_Minutes;
	uint8_t	RTC_Hours;
	uint8_t	RTC_Date;
	uint8_t	RTC_Month;
	uint8_t	RTC_Year;

	uint8_t	Arch_Num;											//	Номер считываемого архива
	uint8_t	Arch_Seconds;										//	Дата и время запрашиваемой  архивной записи
	uint8_t	Arch_Minutes;
	uint8_t	Arch_Hours;
	uint8_t	Arch_Date;
	uint8_t	Arch_Month;
	uint8_t	Arch_Year;

	uint8_t	Sensor_Measuring_Channel[MEASURING_CHANNELS_NUM];	//	Таблица соответствия датчика измерительному каналу
	uint8_t	Sensor_Type[MEASURING_CHANNELS_NUM];				//	Тип датчика
	uint8_t	Sensor_Use_Fix_Value[MEASURING_CHANNELS_NUM];		//	Использовать фиксированное значение датчика при расчете

	float	Analog_Channel_4mA_Value[CURRENT_CHN_NUM];			//	Значение аналоговых каналов при токе 4mA
	float	Analog_Channel_20mA_Value[CURRENT_CHN_NUM];			//	Значение аналоговых каналов при токе 20mA

	float	Frequency_Channel_1_Kfactor_Freq[KFACTOR_POINT_NUM];//	Список частот вращения турбины
	float	Frequency_Channel_1_Kfactor_Value[KFACTOR_POINT_NUM];//	Значение К-фактора при заданной частоте
	float	Frequency_Channel_2_Kfactor_Freq[KFACTOR_POINT_NUM];//	Список частот вращения турбины
	float	Frequency_Channel_2_Kfactor_Value[KFACTOR_POINT_NUM];//	Значение К-фактора при заданной частоте

	float	Channel_Treshold_Low[MEASURING_CHANNELS_NUM];		//	Нижнее значение верных показателей датчика
	float	Channel_Treshold_High[MEASURING_CHANNELS_NUM];		//	Верхнее значение верных показателей датчика
	float	Channel_Fix_Value[MEASURING_CHANNELS_NUM];			//	Фиксированное значение, используемое при неисправности датчика
}stSettings;

#define MEASURING_LINE_NUM	2

typedef struct//Чтение мгновенных значений (input регистры)
{
	float		V_RU[MEASURING_LINE_NUM];//	Объем при РУ
	float		V_SU[MEASURING_LINE_NUM];//	Объем при СУ
	float		Mass[MEASURING_LINE_NUM];//		Масса
	float		Temperature[MEASURING_LINE_NUM];//		Температура
	float		Pressure[MEASURING_LINE_NUM];//		Давление
//	float[2];	//Резерв
//	float[2];	//Резерв
	uint32_t	Alarm;//	32-битное слово нештатной ситуации
	uint32_t	Warning;//	32-битное слово предупреждений
	uint32_t	Time_Work;//	Время работы в секундах

}stInstantVals;

typedef struct//Чтение архивной записи(input регистры)
{
	uint8_t	Arch_Seconds;//	Дата и время окончания формирования  архивной записи
	uint8_t	Arch_Minutes;
	uint8_t	Arch_Hours;
	uint8_t	Arch_Date;
	uint8_t	Arch_Month;
	uint8_t	Arch_Year;

	float	V_RU[MEASURING_LINE_NUM];//	Объем при РУ
	float	V_SU[MEASURING_LINE_NUM];//	Объем при СУ
	float	Mass[MEASURING_LINE_NUM];//	Масса
	float	Temperature[MEASURING_LINE_NUM];//	Средняя температура
	float	Pressure[MEASURING_LINE_NUM];//	Среднее давление
	float	Energy[MEASURING_LINE_NUM];//	Энергия
	//float[2]	Резерв
	//float[2]	Резерв
	uint16_t	Code_Alarm;//	Код нештатной ситуации
	uint16_t	CRC_Sum;//	Контрольная сумма
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
