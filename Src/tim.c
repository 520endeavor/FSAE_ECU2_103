/**
  ******************************************************************************
  * File Name          : TIM.c
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

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */
osThreadId HallSpeedGenericTaskHandle;
void HallSpeedGenericTask(void const * argument);
static void Tim_FREERTOS_Init(void) 
{
	/* definition and creation of CAN_ReceiveData_AnalysisTask */
	osThreadDef(HallSpeedGenericTask,HallSpeedGenericTask, osPriorityNormal, 0, 128);
	HallSpeedGenericTaskHandle = osThreadCreate(osThread(HallSpeedGenericTask), NULL);
}

/*计算两次上升沿脉冲时间间隔寄存器变量*/
//该结构体变量需要全局，确保初始化为0
struct{
	volatile uint16_t CAPTURE_Period_counter[4];	//用16位数据计周期溢出次数
	volatile float CAPTURE_VAL[4];								//存储两次脉冲之间的时间
	volatile uint8_t CAPTURE_Success_Flag[4];		//成功在溢出前捕捉到两个脉冲标志
	volatile uint8_t CAPTURE_STARTFlag;					//计数开启装置
	volatile uint8_t TimeoutFlag[4];							//计数溢出标志
}HallSpeedCounterINFO;

struct {
	volatile float HallSpeed1_Value;
	volatile float HallSpeed2_Value;
	volatile float HallSpeed3_Value;
	volatile float HallSpeed4_Value;
}HallSpeed_Value;


SteeringEngine_PWM steering_engine1_pwm={&htim3,TIM_CHANNEL_2},steering_engine2_pwm={&htim3,TIM_CHANNEL_1};
HallSpeed_CAP hallspeed1_cap={&htim8,TIM_CHANNEL_3},hallspeed2_cap={&htim8,TIM_CHANNEL_2},hallspeed3_cap={&htim8,TIM_CHANNEL_4},hallspeed4_cap={&htim1,TIM_CHANNEL_1};

/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim8_ch2;
DMA_HandleTypeDef hdma_tim8_ch3_up;
DMA_HandleTypeDef hdma_tim8_ch4_trig_com;	

/* HallSpeedTask function */

/* TIM1 init function */
void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 899;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
	Tim_FREERTOS_Init();
}
/* TIM3 init function */
void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);

}
/* TIM8 init function */
void MX_TIM8_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 899;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();
  
    /**TIM1 GPIO Configuration    
    PA8     ------> TIM1_CH1 
    */
    GPIO_InitStruct.Pin = ECU2_HallSpeed4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ECU2_HallSpeed4_GPIO_Port, &GPIO_InitStruct);

    /* Peripheral DMA init*/
  
    hdma_tim1_ch1.Instance = DMA1_Channel2;
    hdma_tim1_ch1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_tim1_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim1_ch1.Init.Mode = DMA_NORMAL;
    hdma_tim1_ch1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_tim1_ch1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC1],hdma_tim1_ch1);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(TIM1_UP_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM8)
  {
  /* USER CODE BEGIN TIM8_MspInit 0 */

  /* USER CODE END TIM8_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM8_CLK_ENABLE();
  
    /**TIM8 GPIO Configuration    
    PC7     ------> TIM8_CH2
    PC8     ------> TIM8_CH3
    PC9     ------> TIM8_CH4 
    */
    GPIO_InitStruct.Pin = ECU2_HallSpeed2_Pin|ECU2_HallSpeed1_Pin|ECU2_HallSpeed3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Peripheral DMA init*/
  
    hdma_tim8_ch2.Instance = DMA2_Channel5;
    hdma_tim8_ch2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_tim8_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim8_ch2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim8_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim8_ch2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim8_ch2.Init.Mode = DMA_NORMAL;
    hdma_tim8_ch2.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_tim8_ch2) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC2],hdma_tim8_ch2);

    hdma_tim8_ch3_up.Instance = DMA2_Channel1;
    hdma_tim8_ch3_up.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_tim8_ch3_up.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim8_ch3_up.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim8_ch3_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim8_ch3_up.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim8_ch3_up.Init.Mode = DMA_NORMAL;
    hdma_tim8_ch3_up.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_tim8_ch3_up) != HAL_OK)
    {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC3],hdma_tim8_ch3_up);
    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_UPDATE],hdma_tim8_ch3_up);

    hdma_tim8_ch4_trig_com.Instance = DMA2_Channel2;
    hdma_tim8_ch4_trig_com.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_tim8_ch4_trig_com.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim8_ch4_trig_com.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim8_ch4_trig_com.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim8_ch4_trig_com.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim8_ch4_trig_com.Init.Mode = DMA_NORMAL;
    hdma_tim8_ch4_trig_com.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_tim8_ch4_trig_com) != HAL_OK)
    {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC4],hdma_tim8_ch4_trig_com);
    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_TRIGGER],hdma_tim8_ch4_trig_com);
    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_COMMUTATION],hdma_tim8_ch4_trig_com);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(TIM8_UP_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(TIM8_UP_IRQn);
    HAL_NVIC_SetPriority(TIM8_CC_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);
  /* USER CODE BEGIN TIM8_MspInit 1 */

  /* USER CODE END TIM8_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(timHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */
  
    /**TIM3 GPIO Configuration    
    PA6     ------> TIM3_CH1
    PA7     ------> TIM3_CH2 
    */
    GPIO_InitStruct.Pin = ECU2_DRS_SteeringEngine2_Pin|ECU2_DRS_SteeringEngine1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();
  
    /**TIM1 GPIO Configuration    
    PA8     ------> TIM1_CH1 
    */
    HAL_GPIO_DeInit(ECU2_HallSpeed4_GPIO_Port, ECU2_HallSpeed4_Pin);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC1]);

    /* Peripheral interrupt Deinit*/
		HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);

  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM8)
  {
  /* USER CODE BEGIN TIM8_MspDeInit 0 */

  /* USER CODE END TIM8_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM8_CLK_DISABLE();
  
    /**TIM8 GPIO Configuration    
    PC7     ------> TIM8_CH2
    PC8     ------> TIM8_CH3
    PC9     ------> TIM8_CH4 
    */
    HAL_GPIO_DeInit(GPIOC, ECU2_HallSpeed2_Pin|ECU2_HallSpeed1_Pin|ECU2_HallSpeed3_Pin);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC2]);
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC3]);
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_UPDATE]);
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC4]);
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_TRIGGER]);
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_COMMUTATION]);

    /* Peripheral interrupt Deinit*/
		HAL_NVIC_DisableIRQ(TIM8_UP_IRQn);
    HAL_NVIC_DisableIRQ(TIM8_CC_IRQn);

  /* USER CODE BEGIN TIM8_MspDeInit 1 */

  /* USER CODE END TIM8_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
/*
*****************************************************************************
*@brief		调节舵机角度函数，使用该函数可在函数运行完后
					延时+关闭PWM通道函数HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel)
*@param		SteeringEngine steering_engine：包含定时器结构体句柄和PWM输出的相关通道float angle
*@param		float angle：输入调节角度
*@retval	None
*@par
*****************************************************************************
*/
void AdjustAngle_SteeringEngine(SteeringEngine_PWM steering_engine_pwm,float angle)
{
	uint16_t Pulse_SET=0;
	if(angle<0){
		angle=0;
	}
	else if(angle>90){
		angle=90;
	}
	Pulse_SET=(int)angle/90.0*1200+900;	
	if(Pulse_SET>=2100){
		Pulse_SET=2100;
	}
	else if(Pulse_SET<=900){
		Pulse_SET=900;
	}
	__HAL_TIM_SET_COMPARE(steering_engine_pwm.timHandle, steering_engine_pwm.channel, Pulse_SET);	
	HAL_TIM_PWM_Start(steering_engine_pwm.timHandle,steering_engine_pwm.channel);
}

/*
*****************************************************************************
*@brief		霍尔测速传感器输入捕获开启函数
*@param		HallSpeed_CAP hallspeed_cap：包含定时器结构体句柄和CAP输入捕获的相关通道float angle
*@retval	
*@par
*****************************************************************************
*/
void WheelSpeed_Sampling(HallSpeed_CAP hallspeed_cap)
{
	HAL_TIM_IC_Start_IT(hallspeed_cap.timHandle, hallspeed_cap.channel);	
	HAL_TIM_Base_Start_IT(hallspeed_cap.timHandle);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);		
	uint8_t Tim_Flag;
	static uint16_t Tim_Cnt=0;
	if(htim==hallspeed1_cap.timHandle&&htim->Channel==HAL_TIM_ACTIVE_CHANNEL_3){		
		Tim_Flag=0;
	}
	else if(htim==hallspeed2_cap.timHandle&&htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2){		
		Tim_Flag=1;
	}
	else if(htim==hallspeed3_cap.timHandle&&htim->Channel==HAL_TIM_ACTIVE_CHANNEL_4){		
		Tim_Flag=2;
	}
	else if(htim==hallspeed4_cap.timHandle&&htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1){		
		Tim_Flag=3;
	}
	for(uint8_t i=0;i<4;i++)
	{
		if(i==Tim_Flag){
			if(HallSpeedCounterINFO.CAPTURE_STARTFlag&(1<<i)){//该次捕获前，是否完成一次捕获
				HallSpeedCounterINFO.CAPTURE_VAL[i]=((900+__HAL_TIM_GET_COUNTER(htim)-Tim_Cnt)/900.0+HallSpeedCounterINFO.CAPTURE_Period_counter[i]-1)*25;
				Tim_Cnt=__HAL_TIM_GET_COUNTER(htim);
				HallSpeedCounterINFO.CAPTURE_Success_Flag[i]=1;
				HallSpeedCounterINFO.CAPTURE_Period_counter[i]=0;
				BaseType_t xHigherPriorityTaskWoken=pdFALSE;
				vTaskNotifyGiveFromISR(HallSpeedGenericTaskHandle, &xHigherPriorityTaskWoken);
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		
			}
			else{
				HallSpeedCounterINFO.CAPTURE_STARTFlag|=(1<<i);//第一次捕获完成标志
				Tim_Cnt=__HAL_TIM_GET_COUNTER(htim);
				HallSpeedCounterINFO.TimeoutFlag[i]=0;
			}
		}
	}		
  /* NOTE : This function Should not be modified, when the callback is needed,
            the __HAL_TIM_IC_CaptureCallback could be implemented in the user file
   */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the __HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
	if(htim==hallspeed1_cap.timHandle){
		for(uint8_t i=0;i<3;i++)
		{
			if(HallSpeedCounterINFO.CAPTURE_STARTFlag&(1<<i)){
				HallSpeedCounterINFO.CAPTURE_Period_counter[i]++;
				if(HallSpeedCounterINFO.CAPTURE_Period_counter[i]>=0x9C40){//用21位数据计数，周期为25us，溢出时间为25us;
					HallSpeedCounterINFO.CAPTURE_Period_counter[i]=0;
					HallSpeedCounterINFO.TimeoutFlag[i]=1;			//计时太长，标志位置1，在速度计算任务中将速度值0
					HallSpeedCounterINFO.CAPTURE_STARTFlag&=~(1<<i);
					BaseType_t xHigherPriorityTaskWoken=pdFALSE;
					vTaskNotifyGiveFromISR(HallSpeedGenericTaskHandle, &xHigherPriorityTaskWoken);
					portYIELD_FROM_ISR(xHigherPriorityTaskWoken);	
				}
			}		
		}
	}	
	else if(htim==hallspeed4_cap.timHandle){
		if(HallSpeedCounterINFO.CAPTURE_STARTFlag&0x08){
			HallSpeedCounterINFO.CAPTURE_Period_counter[3]++;
			if(HallSpeedCounterINFO.CAPTURE_Period_counter[3]>=0x9C40){
				HallSpeedCounterINFO.CAPTURE_Period_counter[3]=0;
				HallSpeedCounterINFO.TimeoutFlag[3]=1;//计时太长，标志位置1，在速度计算任务中将速度值0
				HallSpeedCounterINFO.CAPTURE_STARTFlag=~(1<<3);
				BaseType_t xHigherPriorityTaskWoken=pdFALSE;
				vTaskNotifyGiveFromISR(HallSpeedGenericTaskHandle, &xHigherPriorityTaskWoken);
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);	
			}
		}
	}	
}

/* HallSpeedGenericTask function */
void HallSpeedGenericTask(void const * argument)
{	
  /* USER CODE BEGIN HallSpeedGenericTask */
  /* Infinite loop */
	WheelSpeed_Sampling(hallspeed1_cap);//开启捕获
	WheelSpeed_Sampling(hallspeed2_cap);//开启捕获
	WheelSpeed_Sampling(hallspeed3_cap);//开启捕获
	WheelSpeed_Sampling(hallspeed4_cap);//开启捕获
	static float Speed_Value=0;
	for(;;)
  {			
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY);		
		for(uint8_t i=0;i<4;i++)
		{
			if(HallSpeedCounterINFO.CAPTURE_Success_Flag[i]){
				Speed_Value=1000*48.65116/HallSpeedCounterINFO.CAPTURE_VAL[i];
				HallSpeedCounterINFO.CAPTURE_Success_Flag[i]=0;		
				if (i==0){
					HallSpeed_Value.HallSpeed1_Value=Speed_Value;
				}
				else if(i==1){
					HallSpeed_Value.HallSpeed2_Value=Speed_Value;
				}
				else if(i==2){
					HallSpeed_Value.HallSpeed3_Value=Speed_Value;
				}
				else if(i==3){
					HallSpeed_Value.HallSpeed4_Value=Speed_Value;
				}			
			}
			else if(HallSpeedCounterINFO.TimeoutFlag[i]==1){
				Speed_Value=0;
				if (i==0){
					HallSpeed_Value.HallSpeed1_Value=Speed_Value;
				}
				else if(i==1){
					HallSpeed_Value.HallSpeed2_Value=Speed_Value;
				}
				else if(i==2){
					HallSpeed_Value.HallSpeed3_Value=Speed_Value;
				}
				else if(i==3){
					HallSpeed_Value.HallSpeed4_Value=Speed_Value;
				}
			}			
		}
 }
  /* USER CODE END HallSpeedGenericTask */
}

/*
*****************************************************************************
*@brief		取值函数，搬运四轮轮速数据
*@param		TimGenericData_n name_num：获取数据对应的枚举变量
*@param		const void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_HallSpeed_Value_Data(TimGenericData_n name_num,void * extern_data)
{
	if(name_num==HallSpeed1_Value_n){
		*(float*)extern_data=HallSpeed_Value.HallSpeed1_Value;
	} 	
	else if(name_num==HallSpeed2_Value_n){
		*(float*)extern_data=HallSpeed_Value.HallSpeed2_Value;
	}
	else if(name_num==HallSpeed3_Value_n){
		*(float*)extern_data=HallSpeed_Value.HallSpeed3_Value;
	}
	else if(name_num==HallSpeed4_Value_n){
		*(float*)extern_data=HallSpeed_Value.HallSpeed4_Value;
	}
}

/*
*****************************************************************************
*@brief		修改四轮轮速数据
*@param		TimGenericData_n name_num：待修改数据对应的枚举变量
*@param		void* extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_HallSpeed_Value_Data(TimGenericData_n name_num,void * extern_data)
{
	if(name_num==HallSpeed1_Value_n){
		HallSpeed_Value.HallSpeed1_Value=*(float*)extern_data;
	} 	
	else if(name_num==HallSpeed2_Value_n){
		HallSpeed_Value.HallSpeed2_Value=*(float*)extern_data;
	}
	else if(name_num==HallSpeed3_Value_n){
		HallSpeed_Value.HallSpeed3_Value=*(float*)extern_data;
	}
	else if(name_num==HallSpeed4_Value_n){
		HallSpeed_Value.HallSpeed4_Value=*(float*)extern_data;
	}
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

