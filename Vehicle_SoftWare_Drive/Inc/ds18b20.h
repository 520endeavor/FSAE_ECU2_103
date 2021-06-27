#ifndef __DS18B20_H
#define __DS18B20_H 

#include "stm32f1xx_hal.h"
#include "gpio.h"

//IO方向设置   	
void DS18B20_IO_OUT(void);
void DS18B20_IO_IN(void);

////IO操作函数											   
#define	DS18B20_DQ_OUT_HIGH  HAL_GPIO_WritePin(ECU2_TemSensor18B20DQ_GPIO_Port, ECU2_TemSensor18B20DQ_Pin, GPIO_PIN_SET)    //数据端口	PA0
#define	DS18B20_DQ_OUT_LOW   HAL_GPIO_WritePin(ECU2_TemSensor18B20DQ_GPIO_Port, ECU2_TemSensor18B20DQ_Pin, GPIO_PIN_RESET)  //数据端口	PA0
#define	DS18B20_DQ_IN    		 HAL_GPIO_ReadPin(ECU2_TemSensor18B20DQ_GPIO_Port, ECU2_TemSensor18B20DQ_Pin)  //数据端口	PA0


void DS18B20_Rst(void);									//复位DS18B20
uint8_t DS18B20_Check(void);						//检测是否存在DS18B20
uint8_t DS18B20_Read_Bit(void);					//读出一个位
uint8_t DS18B20_Read_Byte(void);				//读出一个字节
void DS18B20_Write_Byte(uint8_t dat);		//写入一个字节
void DS18B20_Start(void);								//开始温度转换
uint8_t DS18B20_Init(void);							//初始化DS18B20
float DS18B20_Get_Temp(void);						//获取温度
#endif















