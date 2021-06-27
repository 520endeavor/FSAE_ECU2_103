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
	/**********DRSd����Ƕ���Ϣ�ͷ��ȡ����ȡ�β�ơ����run��ˮ�á�IMD���ƶ��ɿ��Դ����źż���������**********/	
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
/*------------ECU2����ECUģ�鱨��ID����---------------*/
#define ECU_ECU2_SteeringEngine_ID		0x18FF1E4A		//ECU2����ECUģ�鱨�ģ����ȶ��
#define	ECU_ECU2_SteeringEngine_NUM		0							//ECU2���ձ��ı��0 
#define ECU_ECU2_CONTROL_ID						0x18FF1F4A		//ECU2����ECUģ�鱨�ģ����ȡ����ȡ�β�ơ����RUN�źţ����ã���ˮ�ÿ����źš��̵���6IMD���ƶ��ɿ��Դ����ź� 
#define	ECU_ECU2_CONTROL_NUM					1							//ECU2���ձ��ı��1
/*------------ECU2����ECU����ID����Ŷ���---------------*/
#define	ECU2_ECU_INFO0_ID			0x18FF5A4E		//ECU2ģ�鷢��ECU����1�����������Ϣ���������ٶ�
#define	ECU2_ECU_INFO1_ID			0x18FF5B4E		//ECU2ģ�鷢��ECU����2�����������Ϣ����ǰ���ٶȣ�ˮ���¶ȵ�
#define	ECU2_ECU_INFO2_ID			0x18FF5C4E		//ECU2ģ�鷢��ECU����3��AMS,IMD��λ�ź�����
#define	ECU2_ECU_STATE_ID			0x18FF5D4E		//ECU2ģ�鷢��ECU����4��ˮ���¶ȣ�ˮ�ã����ȣ�β�ƣ����ȵ�״̬��Ϣ


typedef struct{
	uint32_t CANTransmitID;
	uint8_t CANTransmitData[8];
}CANTransmitMessageINFO;

/* USER CODE END Private defines */

extern void Error_Handler(void);

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */
/*�ⲿ�ӿں���*/
void APP_CAN_Config(void);
void CSU_CAN_Send(CAN_HandleTypeDef* canHandle,uint32_t ID,uint8_t *pData, uint8_t Size);//CAN���ͺ���,ʹ��CAN���������ͱ���
void Get_CAN_Analysis_Data(Analysis_Data_n name_num,void * extern_data);	//ȡֵ����������DRSd����Ƕ���Ϣ�ͷ��ȡ����ȡ�β�ơ����run��ˮ�á�IMD���ƶ��ɿ��Դ����źŵ�����@Analysis_Data_n
void CAN_DATA_Send(uint32_t Protocol_ID);																	//CAN����Э�����ݺ���
void Set_CAN_Analysis_Data(Analysis_Data_n name_num, void * extern_data);	//�޸�CAN���ղ������õ�����@Analysis_Data_n

TimerHandle_t Get_CAN_100PeriodicSendTimer_Handle(void);											//��ȡCAN���ڷ��������ʱ�����������100ms
TimerHandle_t Get_CAN_10PeriodicSendTimer_Handle(void);											//��ȡCAN���ڷ��������ʱ�����������10ms
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
