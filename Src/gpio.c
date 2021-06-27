/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
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
#include "gpio.h"
/* USER CODE BEGIN 0 */
struct{
	volatile uint8_t AMS_IMDReset;
}AMS_IMDReset_Switch_Timer_Input;

struct{
	volatile uint8_t Speaker_Stata;
	volatile uint8_t Taillight_Stata;
	volatile uint8_t Fan_Stata;
	volatile uint8_t WaterPump_Stata;
}Speaker_Taillight_WaterPumpStata;
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
osThreadId Speaker_StataGenericTaskHandle;
osThreadId AMS_IMDResetValueCheckTaskHandle;

void Speaker_StataGenericTask(void const * argument);
void AMS_IMDResetValueCheckTask(void const * argument);
static void Gpio_FREERTOS_Init(void) 
{
	/* definition and creation of Speaker_StataGenericTask */
	osThreadDef(Speaker_StataGenericTask,Speaker_StataGenericTask, osPriorityNormal, 0, 128);
	Speaker_StataGenericTaskHandle = osThreadCreate(osThread(Speaker_StataGenericTask), NULL);
	/* definition and creation of AMS_IMDResetValueCheckTask */
	osThreadDef(AMS_IMDResetValueCheckTask,AMS_IMDResetValueCheckTask, osPriorityNormal, 0, 128);
	AMS_IMDResetValueCheckTaskHandle = osThreadCreate(osThread(AMS_IMDResetValueCheckTask), NULL);
}



/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ECU2_LED2_Pin|ECU2_LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ECU2_IMD_Relay_Ctrl|ECU2_Backup_Relay1_Pin|ECU2_Optocoupler_MOS4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ECU2_Backup_Relay6_Pin|ECU2_Backup_Relay5_Pin|ECU2_Backup_Relay4_Pin|ECU2_Backup_Relay3_Pin 
                          |ECU2_Backup_Relay2_Pin|ECU2_Optocoupler_MOS2_Pin|ECU2_Optocoupler_MOS5_Pin|ECU2_Optocoupler_MOS3_Pin 
                          |ECU2_Optocoupler_MOS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PtPin */

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = ECU2_LED2_Pin|ECU2_LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = ECU2_IMD_Relay_Ctrl|ECU2_Backup_Relay1_Pin|ECU2_Optocoupler_MOS4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin 
                           PBPin PBPin PBPin PBPin 
                           PBPin */
  GPIO_InitStruct.Pin = ECU2_Backup_Relay6_Pin|ECU2_Backup_Relay5_Pin|ECU2_Backup_Relay4_Pin|ECU2_Backup_Relay3_Pin 
                          |ECU2_Backup_Relay2_Pin|ECU2_Optocoupler_MOS2_Pin|ECU2_Optocoupler_MOS5_Pin|ECU2_Optocoupler_MOS3_Pin 
                          |ECU2_Optocoupler_MOS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = ECU2_IMD_Reset_Key_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ECU2_IMD_Reset_Key_GPIO_Port, &GPIO_InitStruct);

	Gpio_FREERTOS_Init();
}

/* USER CODE BEGIN 2 */
/*
*****************************************************************************
*@brief		喇叭、尾灯、水泵开启关闭控制
*@param		Analysis_Data_n Device_num,设备编号，Speaker_num:0,Taillight_num:1,WaterPump_num 2;
*@param		uint8_t Switch_ON_OFF：0关闭,1开启
*@retval	None
*@par
*****************************************************************************
*/
void Speaker_Taillight_WaterPumpCtrl(Analysis_Data_n Device_num,uint8_t Switch_ON_OFF)
{
	if(Device_num==Speaker_Control_n){
		if(Switch_ON_OFF==0x01){
			HAL_GPIO_WritePin(GPIOB,ECU2_Optocoupler_MOS5_Pin, GPIO_PIN_SET);
		}
		else if(Switch_ON_OFF==0x00){
			HAL_GPIO_WritePin(GPIOB,ECU2_Optocoupler_MOS5_Pin, GPIO_PIN_RESET);
		}
	}
	else if(Device_num==Taillight_Control_n){
		if(Switch_ON_OFF==0x01){
			HAL_GPIO_WritePin(GPIOB,ECU2_Optocoupler_MOS2_Pin, GPIO_PIN_SET);
		}
		else if(Switch_ON_OFF==0x00){
			HAL_GPIO_WritePin(GPIOB,ECU2_Optocoupler_MOS2_Pin, GPIO_PIN_RESET);
		}
	}
	else if(Device_num==WaterPump_Control_n){
		if(Switch_ON_OFF==0x01){
			HAL_GPIO_WritePin(GPIOB,ECU2_Optocoupler_MOS3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,ECU2_Optocoupler_MOS4_Pin, GPIO_PIN_SET);
		}
		else if(Switch_ON_OFF==0x00){
			HAL_GPIO_WritePin(GPIOB,ECU2_Optocoupler_MOS3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,ECU2_Optocoupler_MOS4_Pin, GPIO_PIN_RESET);
		}	
	}
}

/* Speaker_StataGenericTask function */
void Speaker_StataGenericTask(void const * argument)
{	
  /* USER CODE BEGIN Speaker_StataGenericTask */
  /* Infinite loop */
	for(;;)
  {			
		Speaker_Taillight_WaterPumpStata.Speaker_Stata=HAL_GPIO_ReadPin(GPIOB, ECU2_Optocoupler_MOS5_Pin);
		Speaker_Taillight_WaterPumpStata.Taillight_Stata=HAL_GPIO_ReadPin(GPIOB, ECU2_Optocoupler_MOS2_Pin);
		Speaker_Taillight_WaterPumpStata.Fan_Stata=0;//风扇暂未分配IO，默认处于关闭状态
		Speaker_Taillight_WaterPumpStata.WaterPump_Stata=HAL_GPIO_ReadPin(GPIOB, ECU2_Optocoupler_MOS3_Pin);
		osDelay(50);//50ms生成一次状态，10ms发送一次
  }
  /* USER CODE END Speaker_StataGenericTask */
}

/* AMS_IMDResetValueCheckTask function */
void AMS_IMDResetValueCheckTask(void const * argument)
{	
  /* USER CODE BEGIN AMS_IMDResetValueCheckTask */
  /* Infinite loop */
	uint8_t Key_up=1;
	for(;;)
  {		
		if((Key_up==1)&&(HAL_GPIO_ReadPin(ECU2_IMD_Reset_Key_GPIO_Port,ECU2_IMD_Reset_Key_Pin)==0)){
			osDelay(10);
			Key_up=0;
			if(HAL_GPIO_ReadPin(ECU2_IMD_Reset_Key_GPIO_Port,ECU2_IMD_Reset_Key_Pin)==0){
				AMS_IMDReset_Switch_Timer_Input.AMS_IMDReset=1;				
			}
		}
		else if(HAL_GPIO_ReadPin(ECU2_IMD_Reset_Key_GPIO_Port,ECU2_IMD_Reset_Key_Pin)==1){
			Key_up=1;
		}
		osDelay(10);
  }
  /* USER CODE END AMS_IMDResetValueCheckTask */
}

/*
*****************************************************************************
*@brief		取值函数，搬运喇叭、尾灯、风扇、水泵状态
*@param		GpioGenericData_n name_num：获取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_Speaker_Taillight_Fan_Waterpump_State_Data(GpioGenericData_n name_num,void * extern_data)
{
	if(name_num==Speaker_Stata_n){
		*(uint8_t*)extern_data=Speaker_Taillight_WaterPumpStata.Speaker_Stata;		
	} 	
	else if(name_num==Taillight_Stata_n){
		*(uint8_t*)extern_data=Speaker_Taillight_WaterPumpStata.Taillight_Stata;		
	}
	else if(name_num==Fan_Stata_n){
		*(uint8_t*)extern_data=Speaker_Taillight_WaterPumpStata.Fan_Stata;		
	}
	else if(name_num==WaterPump_Stata_n){
		*(uint8_t*)extern_data=Speaker_Taillight_WaterPumpStata.WaterPump_Stata;		
	}
}

/*
*****************************************************************************
*@brief		取值函数,获取IMD复位按钮状态
*@param		GpioGenericData_n name_num：获取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_AMS_IMDReset_State_Data(GpioGenericData_n name_num,void * extern_data)
{
	if(name_num==AMS_IMDReset_n){
		*(uint8_t*)extern_data=AMS_IMDReset_Switch_Timer_Input.AMS_IMDReset;		
	} 	
}

/*
*****************************************************************************
*@brief		修改喇叭、尾灯、风扇、水泵状态
*@param		GpioGenericData_n name_num：待修改数据对应的枚举变量
*@param		void* extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_Speaker_Taillight_Fan_Waterpump_State_Data(GpioGenericData_n name_num,void * extern_data)
{
	if(name_num==Speaker_Stata_n){
		Speaker_Taillight_WaterPumpStata.Speaker_Stata=*(uint8_t*)extern_data;		
	} 	
	else if(name_num==Taillight_Stata_n){
		Speaker_Taillight_WaterPumpStata.Taillight_Stata=*(uint8_t*)extern_data;		
	}
	else if(name_num==Fan_Stata_n){
		Speaker_Taillight_WaterPumpStata.Fan_Stata=*(uint8_t*)extern_data;		
	}
	else if(name_num==WaterPump_Stata_n){
		Speaker_Taillight_WaterPumpStata.WaterPump_Stata=*(uint8_t*)extern_data;		
	}
}

/*
*****************************************************************************
*@brief		修改IMD复位按钮状态
*@param		GpioGenericData_n name_num：待修改数据对应的枚举变量
*@param		void*extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_AMS_IMDReset_State_Data(GpioGenericData_n name_num,void * extern_data)
{
	if(name_num==AMS_IMDReset_n){
		AMS_IMDReset_Switch_Timer_Input.AMS_IMDReset=*(uint8_t*)extern_data;		
	} 	
}
/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
