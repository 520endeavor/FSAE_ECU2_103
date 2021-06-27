/**
  ******************************************************************************
  * File Name          : TIM.h
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
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
#ifndef __tim_H
#define __tim_H
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

#include "can.h"
/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;

/* USER CODE BEGIN Private defines */
typedef struct{
	TIM_HandleTypeDef *timHandle;    /*!< 定时器句柄             */
	uint32_t channel;								 /*!< PWM输出通道             */
}SteeringEngine_PWM;
extern SteeringEngine_PWM steering_engine1_pwm,steering_engine2_pwm;

typedef struct{
	TIM_HandleTypeDef *timHandle;    /*!< 定时器句柄             */
	uint32_t channel;								 /*!< CAP输入通道             */
}HallSpeed_CAP;
extern HallSpeed_CAP hallspeed1_cap,hallspeed2_cap,hallspeed3_cap,hallspeed4_cap;
	
typedef enum{
	HallSpeed1_Value_n=20,
	HallSpeed2_Value_n,
	HallSpeed3_Value_n,
	HallSpeed4_Value_n
}TimGenericData_n; 

/* USER CODE END Private defines */

extern void Error_Handler(void);

void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM8_Init(void);
                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN Prototypes */
void AdjustAngle_SteeringEngine(SteeringEngine_PWM steering_engine_pwm,float angle);//调节舵机角度函数，后面跟随函数HAL_TIM_PWM_Stop关闭PWM通道，消除舵机抖动
void WheelSpeed_Sampling(HallSpeed_CAP hallspeed_cap);//霍尔测速传感器输入捕获
void Get_HallSpeed_Value_Data(TimGenericData_n name_num,void * extern_data);//取值函数，搬运四轮轮速数据@TimGenericData_n
void Set_HallSpeed_Value_Data(TimGenericData_n name_num,void * extern_data);//修改四轮轮速数据@TimGenericData_n
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
