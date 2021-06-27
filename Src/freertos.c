/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "gpio.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "ds18b20.h"
#include "Wireless_AS32_TTL_1W.h"
//#include "AT24Cxx.h" 
/* USER CODE END Includes */
/* Variables -----------------------------------------------------------------*/
osThreadId DRSEngineTaskHandle;
osThreadId Speaker_Waterpump_TaillightTaskHandle;
osThreadId Ecu2_LeakageCurrentLimitProcessTaskHandle;
//osThreadId WirelessTaskHandle;
//osThreadId ds18b20TaskHandle;
osThreadId LEDTaskHandle;
//osThreadId AT24CxxTaskHandle;

void DRSEngineTask(void const * argument);
void Speaker_Waterpump_TaillightTask(void const * argument);
void Ecu2_LeakageCurrentLimitProcessTask(void const * argument);
//void WirelessTask(void const * argument);
//void ds18b20Task(void const * argument);
void LEDTask(void const * argument);
//void AT24CxxTask(void const * argument);

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
	/* definition and creation of DRSEngineTask */
  osThreadDef(DRSEngineTask, DRSEngineTask, osPriorityNormal, 0, 128);
  DRSEngineTaskHandle = osThreadCreate(osThread(DRSEngineTask), NULL);	
  /* definition and creation of FirstStartUpTask */
  osThreadDef(Speaker_Waterpump_TaillightTask, Speaker_Waterpump_TaillightTask, osPriorityNormal, 0, 128);
  Speaker_Waterpump_TaillightTaskHandle = osThreadCreate(osThread(Speaker_Waterpump_TaillightTask), NULL);
	/* definition and creation of Ecu2_LeakageCurrentLimitProcessTask */
  osThreadDef(Ecu2_LeakageCurrentLimitProcessTask, Ecu2_LeakageCurrentLimitProcessTask, osPriorityNormal, 5, 128);
  Ecu2_LeakageCurrentLimitProcessTaskHandle = osThreadCreate(osThread(Ecu2_LeakageCurrentLimitProcessTask), NULL);
	/* definition and creation of ds18b20Task */
//  osThreadDef(ds18b20Task, ds18b20Task, osPriorityNormal, 0, 128);
//  ds18b20TaskHandle = osThreadCreate(osThread(ds18b20Task), NULL);	
	/* definition and creation of LEDTask */
  osThreadDef(LEDTask, LEDTask, osPriorityNormal, 0, 64);
  LEDTaskHandle = osThreadCreate(osThread(LEDTask), NULL);		
	/* definition and creation of WirelessTask */
//  osThreadDef(WirelessTask, WirelessTask, osPriorityNormal, 0, 128);
//  WirelessTaskHandle = osThreadCreate(osThread(WirelessTask), NULL);			
	/* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */  
  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

void DRSEngineTask(void const * argument)
{	
  /* USER CODE BEGIN RollStabilizerBarSteeringEngineTask */
  /* Infinite loop */
	float DRS_Angle_1,DRS_Angle_2;
	for(;;)
  {		
//		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		Get_CAN_Analysis_Data(DRS_SteeringEngine1_Angle_n,&DRS_Angle_1);
		Get_CAN_Analysis_Data(DRS_SteeringEngine2_Angle_n,&DRS_Angle_2);
		AdjustAngle_SteeringEngine(steering_engine1_pwm,DRS_Angle_1);
		AdjustAngle_SteeringEngine(steering_engine2_pwm,DRS_Angle_2);
		osDelay(100);		
  }
  /* USER CODE END RollStabilizerBarSteeringEngineTask */
}

/* Speaker_Waterpump_TaillightTask function */
void Speaker_Waterpump_TaillightTask(void const * argument)
{
  /* USER CODE BEGIN Speaker_Waterpump_TaillightTask */
  /* Infinite loop */
	uint8_t SpeakerCtrl=0,waterPumpCtrl=1,TaillightCtrl=0;
  for(;;)
  {	
		Get_CAN_Analysis_Data(Speaker_Control_n,&SpeakerCtrl);
		Get_CAN_Analysis_Data(Taillight_Control_n,&TaillightCtrl);
//		Get_CAN_Analysis_Data(WaterPump_Control_n,&waterPumpCtrl);						//屏蔽后水泵即为上电常开
		Speaker_Taillight_WaterPumpCtrl(Speaker_Control_n,SpeakerCtrl);	
		Speaker_Taillight_WaterPumpCtrl(Taillight_Control_n,TaillightCtrl);
		Speaker_Taillight_WaterPumpCtrl(WaterPump_Control_n,waterPumpCtrl);	
		osDelay(100);
  }
  /* USER CODE END Speaker_Waterpump_TaillightTask */
}

/* Ecu2_LeakageCurrentLimitProcessTask function */
void Ecu2_LeakageCurrentLimitProcessTask(void const * argument)
{
  /* USER CODE BEGIN Ecu2_LeakageCurrentLimitProcessTask */
  /* Infinite loop */
	uint8_t IMDAndBrakeReliability_Trigger_Data=0,AMS_IMDReset_Data=0,CLEAR_AMS_IMDReset=0;
  for(;;)
  {	
		Get_CAN_Analysis_Data(IMDAndBrakeReliability_Trigger_n,&IMDAndBrakeReliability_Trigger_Data);
		Get_AMS_IMDReset_State_Data(AMS_IMDReset_n,&AMS_IMDReset_Data);	
		if(AMS_IMDReset_Data==1){	
			for(uint8_t i=0;i<5;i++)
			{
				CAN_DATA_Send(ECU2_ECU_INFO2_ID);
				osDelay(10);
			}
			Set_AMS_IMDReset_State_Data(AMS_IMDReset_n,&CLEAR_AMS_IMDReset);			
		}																															
		if(IMDAndBrakeReliability_Trigger_Data==1){
			HAL_GPIO_WritePin(ECU2_IMD_Relay_Ctrl_GPIO_Port, ECU2_IMD_Relay_Ctrl,GPIO_PIN_SET);				//发送漏电超限，断开继电器
		}
		else if(IMDAndBrakeReliability_Trigger_Data==0){
			HAL_GPIO_WritePin(ECU2_IMD_Relay_Ctrl_GPIO_Port, ECU2_IMD_Relay_Ctrl,GPIO_PIN_RESET);			//漏电超限故障清除，闭合继电器
		}
		osDelay(100);
  }
  /* USER CODE END Ecu2_LeakageCurrentLimitProcessTask */
}

/* LEDTask function */
void LEDTask(void const * argument)
{
  /* USER CODE BEGIN LEDTask */
  /* Infinite loop */	
  for(;;)
  {
		HAL_GPIO_TogglePin(ECU2_LED1_GPIO_Port, ECU2_LED1_Pin);
		HAL_GPIO_TogglePin(ECU2_LED2_GPIO_Port, ECU2_LED2_Pin);
		osDelay(200);
  }
  /* USER CODE END LEDTask */
}

/* WirelessTask function */
//void WirelessTask(void const * argument)
//{	
//  /* USER CODE BEGIN WirelessTask */
//  /* Infinite loop */
//	Wireless_AS32_Transmit(wireless_Tdata,sizeof(wireless_Tdata));	
//  for(;;)
//  {							
//    osDelay(100);
//		Wireless_AS32_Transmit(wireless_Tdata,sizeof(wireless_Tdata));	
//  }
//  /* USER CODE END WirelessTask */
//}

/* ds18b20Task function */
//void ds18b20Task(void const * argument)
//{
//  /* USER CODE BEGIN ds18b20Task */
//  /* Infinite loop */
//	if (DS18B20_Init()){  //检查器件
//		/*发送ds18b20故障信息给显示屏*/	
//	}
//  for(;;)
//  {
//		T=DS18B20_Get_Temp();
//		osDelay(100);
//  }
//  /* USER CODE END ds18b20Task */
//}
/* USER CODE BEGIN Application */

     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
