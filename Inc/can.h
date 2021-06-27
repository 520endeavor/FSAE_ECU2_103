/**
  ******************************************************************************
  * File Name          : CAN.h
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __can_H
#define __can_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
	 
#include "tim.h"	
	 
typedef enum{
	/**********DRSd舵机角度信息和风扇、喇叭、尾灯、电机run、水泵、IMD和制动可靠性触发信号及保留变量**********/	
	DRS_SteeringEngine1_Angle_n=1,
	DRS_SteeringEngine2_Angle_n,
	FanControl_n,
	Speaker_Control_n,
	Taillight_Control_n,
	MotRun_Control_n,
	WaterPump_Control_n,
	IMDAndBrakeReliability_Trigger_n,
	Reserve0_n,
	Reserve1_n	
} Analysis_Data_n;
#include "gpio.h"
	 
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */
/*------------ECU2接收ECU模块报文ID定义---------------*/
#define ECU_ECU2_SteeringEngine_ID		0x18FF1E4A		//ECU2接收ECU模块报文，横稳舵机
#define	ECU_ECU2_SteeringEngine_NUM		0							//ECU2接收报文编号0 
#define ECU_ECU2_CONTROL_ID						0x18FF1F4A		//ECU2接收ECU模块报文，风扇、喇叭、尾灯、电机RUN信号（备用）、水泵控制信号、继电器6IMD和制动可靠性触发信号 
#define	ECU_ECU2_CONTROL_NUM					1							//ECU2接收报文编号1
/*------------ECU2发送ECU报文ID及编号定义---------------*/
#define	ECU2_ECU_INFO0_ID			0x18FF5A4E		//ECU2模块发给ECU报文1，霍尔相关信息——后轮速度
#define	ECU2_ECU_INFO1_ID			0x18FF5B4E		//ECU2模块发给ECU报文2，霍尔相关信息——前轮速度，水箱温度等
#define	ECU2_ECU_INFO2_ID			0x18FF5C4E		//ECU2模块发给ECU报文3，AMS,IMD复位信号输入
#define	ECU2_ECU_STATE_ID			0x18FF5D4E		//ECU2模块发给ECU报文4，水箱温度，水泵，风扇，尾灯，喇叭等状态信息


typedef struct{
	uint32_t CANTransmitID;
	uint8_t CANTransmitData[8];
}CANTransmitMessageINFO;

/* USER CODE END Private defines */

extern void Error_Handler(void);

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */
/*外部接口函数*/
void APP_CAN_Config(void);
void CSU_CAN_Send(CAN_HandleTypeDef* canHandle,uint32_t ID,uint8_t *pData, uint8_t Size);//CAN发送函数,使用CAN控制器发送报文
void Get_CAN_Analysis_Data(Analysis_Data_n name_num,void * extern_data);	//取值函数，搬运DRSd舵机角度信息和风扇、喇叭、尾灯、电机run、水泵、IMD和制动可靠性触发信号等数据@Analysis_Data_n
void CAN_DATA_Send(uint32_t Protocol_ID);																	//CAN发送协议数据函数
void Set_CAN_Analysis_Data(Analysis_Data_n name_num, void * extern_data);	//修改CAN接收并解析好的数据@Analysis_Data_n

TimerHandle_t Get_CAN_100PeriodicSendTimer_Handle(void);											//获取CAN周期发送软件定时器句柄，周期100ms
TimerHandle_t Get_CAN_10PeriodicSendTimer_Handle(void);											//获取CAN周期发送软件定时器句柄，周期10ms
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ can_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
