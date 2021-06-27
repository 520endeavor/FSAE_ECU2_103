#ifndef __DS18B20_H
#define __DS18B20_H 

#include "stm32f1xx_hal.h"
#include "gpio.h"

//IO��������   	
void DS18B20_IO_OUT(void);
void DS18B20_IO_IN(void);

////IO��������											   
#define	DS18B20_DQ_OUT_HIGH  HAL_GPIO_WritePin(ECU2_TemSensor18B20DQ_GPIO_Port, ECU2_TemSensor18B20DQ_Pin, GPIO_PIN_SET)    //���ݶ˿�	PA0
#define	DS18B20_DQ_OUT_LOW   HAL_GPIO_WritePin(ECU2_TemSensor18B20DQ_GPIO_Port, ECU2_TemSensor18B20DQ_Pin, GPIO_PIN_RESET)  //���ݶ˿�	PA0
#define	DS18B20_DQ_IN    		 HAL_GPIO_ReadPin(ECU2_TemSensor18B20DQ_GPIO_Port, ECU2_TemSensor18B20DQ_Pin)  //���ݶ˿�	PA0


void DS18B20_Rst(void);									//��λDS18B20
uint8_t DS18B20_Check(void);						//����Ƿ����DS18B20
uint8_t DS18B20_Read_Bit(void);					//����һ��λ
uint8_t DS18B20_Read_Byte(void);				//����һ���ֽ�
void DS18B20_Write_Byte(uint8_t dat);		//д��һ���ֽ�
void DS18B20_Start(void);								//��ʼ�¶�ת��
uint8_t DS18B20_Init(void);							//��ʼ��DS18B20
float DS18B20_Get_Temp(void);						//��ȡ�¶�
#endif















