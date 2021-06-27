/**
  ******************************************************************************
  * File Name          : CAN.c
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

/* Includes ------------------------------------------------------------------*/
#include "can.h"


/* USER CODE BEGIN 0 */
struct {
	volatile float	DRS_SteeringEngine1_Angle;
	volatile float	DRS_SteeringEngine2_Angle;
	volatile uint8_t FanControl;
	volatile uint8_t Speaker_Control;
	volatile uint8_t Taillight_Control;
	volatile uint8_t MotRun_Control;
	volatile uint8_t WaterPump_Control;
	volatile uint8_t IMDAndBrakeReliability_Trigger;
	volatile uint8_t Reserve0;
	volatile uint8_t Reserve1;	
}SteeringEngineAndControl;

osThreadId CAN_DataAnalysisTaskHandle;
osThreadId CAN_DataProtocolTaskHandle;
//osThreadId CAN_10PeriodicSendTaskHandle;
osThreadId CAN_TransmitTaskHandle;

TimerHandle_t CAN_100PeriodicSendTimer_Handle;//�����ʱ�����
TimerHandle_t Get_CAN_100PeriodicSendTimer_Handle(void){
	return CAN_100PeriodicSendTimer_Handle;
}
void CAN_100PeriodicSendCallback(TimerHandle_t xTimer);//��ʱ���ص�����

TimerHandle_t CAN_10PeriodicSendTimer_Handle;//�����ʱ�����
TimerHandle_t Get_CAN_10PeriodicSendTimer_Handle(void){
	return CAN_10PeriodicSendTimer_Handle;
}
void CAN_10PeriodicSendCallback(TimerHandle_t xTimer);//��ʱ���ص�����

void CAN_10PeriodicSendTask(void const * argument);
void CAN_DataAnalysisTask(void const * argument);
//void CAN_10PeriodicSendTask(void const * argument);
void CAN_TransmitTask(void const * argument);

osMessageQId CANTransmitQueueHandle;

static void CAN_FREERTOS_Init(void) 
{
	/* definition and creation of CAN_ReceiveData_AnalysisTask */
	osThreadDef(CAN_DataAnalysisTask,CAN_DataAnalysisTask, osPriorityNormal, 0, 128);
	CAN_DataAnalysisTaskHandle = osThreadCreate(osThread(CAN_DataAnalysisTask), NULL);	
	/* definition and creation of CAN_10PeriodicSendTask */
//	osThreadDef(CAN_10PeriodicSendTask,CAN_10PeriodicSendTask, osPriorityNormal, 0, 128);
//	CAN_10PeriodicSendTaskHandle = osThreadCreate(osThread(CAN_10PeriodicSendTask), NULL);	
	/* definition and creation of CAN_DataProtocolTask */
	osThreadDef(CAN_TransmitTask,CAN_TransmitTask, osPriorityNormal, 0, 128);
	CAN_TransmitTaskHandle = osThreadCreate(osThread(CAN_TransmitTask), NULL);	
	
	/* Create the queue(s) */
	/* definition and creation of CANTransmitQueue */
 	osMessageQDef(CANTransmitQueue, 20, CANTransmitMessageINFO);
  CANTransmitQueueHandle = osMessageCreate(osMessageQ(CANTransmitQueue), NULL);	
	
	/*definition and creation of CAN_100PeriodicSendTimer_Handle */ 				//���ڶ�ʱ��������100ms(100��ʱ�ӽ���)������ģʽ //ID:1		      
	CAN_100PeriodicSendTimer_Handle=xTimerCreate((const char*)"CAN_100PeriodicSend",
																			(TickType_t)100,
																			(UBaseType_t)pdTRUE,
																			(void*)1,
																			(TimerCallbackFunction_t)CAN_100PeriodicSendCallback); 
/*definition and creation of CAN_10PeriodicSendTimer_Handle */ 				//���ڶ�ʱ��������10ms(10��ʱ�ӽ���)������ģʽ //ID:2		      
	CAN_10PeriodicSendTimer_Handle=xTimerCreate((const char*)"CAN_10PeriodicSend",
																			(TickType_t)10,
																			(UBaseType_t)pdTRUE,
																			(void*)2,
																			(TimerCallbackFunction_t)CAN_10PeriodicSendCallback); 
}


/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

CanTxMsgTypeDef s1_TxMsg; //CAN1������Ϣ
 CanRxMsgTypeDef s1_RxMsg; //CAN1������Ϣ

/* CAN init function */
void MX_CAN_Init(void)
{

  hcan.Instance = CAN1;
  hcan.Init.Prescaler =9;//6;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_8TQ;//CAN_BS1_6TQ
  hcan.Init.BS2 = CAN_BS2_7TQ;//CAN_BS2_5TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;//DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
	APP_CAN_Config();
	CAN_FREERTOS_Init();
	
}



void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    /**CAN GPIO Configuration    
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX 
    */
    GPIO_InitStruct.Pin = ECU2_CAN_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ECU2_CAN_RX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ECU2_CAN_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ECU2_CAN_TX_GPIO_Port, &GPIO_InitStruct);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN GPIO Configuration    
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX 
    */
    HAL_GPIO_DeInit(GPIOA, ECU2_CAN_RX_Pin|ECU2_CAN_TX_Pin);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);

  }
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
} 

/* USER CODE BEGIN 1 */
void APP_CAN_Config(void){
	CAN_FilterConfTypeDef sFilterConfig;	
	//���ù�����	
	//configure the filter0
//	sFilterConfig.FilterIdHigh=(((uint32_t) ECU_ECU2_SteeringEngine_ID<<3)&0xffff0000)>>16;					//32λID
//	sFilterConfig.FilterIdLow=(((uint32_t) ECU_ECU2_SteeringEngine_ID<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xffff;
//	sFilterConfig.FilterMaskIdHigh=0xFFFF;
//	sFilterConfig.FilterMaskIdLow=0xFEFF<<3|0x0007;	//0xF7FF	
	sFilterConfig.FilterIdHigh=(((uint32_t) ECU_ECU2_SteeringEngine_ID<<3)&0xffff0000)>>16;					//32λID
	sFilterConfig.FilterIdLow=(((uint32_t) ECU_ECU2_SteeringEngine_ID<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xffff;
	sFilterConfig.FilterMaskIdHigh=(((uint32_t) ECU_ECU2_CONTROL_ID<<3)&0xffff0000)>>16;					//32λID;
	sFilterConfig.FilterMaskIdLow=(((uint32_t) ECU_ECU2_CONTROL_ID<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xffff;					//32λID	
	sFilterConfig.FilterNumber=0;								//������0
	sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;
	sFilterConfig.FilterMode=CAN_FILTERMODE_IDLIST;
//	sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterActivation=ENABLE;
	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);		
	//initialize the msg pointer
	hcan.pTxMsg=&s1_TxMsg;
	hcan.pRxMsg=&s1_RxMsg;
	//configuer Rx message
	hcan .pRxMsg->ExtId=0x1FFFFFFF;//Max_Data = 0x1FFFFFFF		
	hcan .pRxMsg->IDE=CAN_ID_EXT;
	hcan .pRxMsg->RTR=CAN_RTR_DATA;	
	//configuer Tx message
//	hcan .pTxMsg->StdId=0x0000;//Max_Data = x7FF	
	hcan .pTxMsg->ExtId=ECU2_ECU_INFO0_ID;//Max_Data = 0x1FFFFFFF
	hcan .pTxMsg->IDE=CAN_ID_EXT;
	hcan .pTxMsg->RTR=CAN_RTR_DATA;
	hcan .pTxMsg->DLC=8;	
}

/*
*****************************************************************************
*@brief		CAN1��CAN2���ͺ���,ʹ��CAN���������ͱ��ģ��������ñ���ID,
					���ͱ��������׵�ַ���������ݳ��ȣ�����һ�α�����������һ�����ݡ�	
*@param		CAN_HandleTypeDef* canHandle��can�ṹ�����
*@param		uint32_t ID��32��չID������0x18FF1234
*@param		uint8_t *pData�����ͱ��������׵�ַ�����緢������Ϊ
					CharBuff[8]={0,1,2,3,4,5,6,7},�����ڶ�������ΪCharBuff���ɡ�					
*@param		uint8_t Size,�������ݵĳ��ȣ�ȡֵӦΪ1~8��		
*@retval	None
*@par
*****************************************************************************
*/
void CSU_CAN_Send(CAN_HandleTypeDef* canHandle,uint32_t ID,uint8_t *pData, uint8_t Size){	//
	uint8_t a=0;
	hcan.pTxMsg->RTR = CAN_RTR_DATA;		//����֡
	hcan.pTxMsg->IDE = CAN_ID_EXT;			//��չ֡
	hcan.pTxMsg->ExtId = ID;																				//������չ֡ ID uint32_t ����:0x19ff1234
	for(a=0;a<Size;a++){
		hcan.pTxMsg->Data[a]=*(pData+a); 
	}
	hcan.pTxMsg->DLC = Size;																				//�������ݳ��� 1-8
	HAL_CAN_Transmit(&hcan,10);																		//CAN2	
}

/*
*****************************************************************************
*@brief		����SteeringEngineAndControl
*@param		uint8_t *receive_data��������յ�������
*@param		uint32_t receive_ID,�������ݵ�ID��Դ
*@retval	None
*@par
*****************************************************************************
*/
static void SteeringEngineAndControl_Analysis(uint8_t *receive_data,uint32_t receive_ID)
{
	if(receive_ID==ECU_ECU2_SteeringEngine_ID){
		SteeringEngineAndControl.DRS_SteeringEngine1_Angle=((*receive_data<<8)+*(receive_data+1))*0.1;//DRS���1�Ƕ�
		SteeringEngineAndControl.DRS_SteeringEngine2_Angle=((*(receive_data+2)<<8)+*(receive_data+3))*0.1;//DRS���2�Ƕ�
	}
	else if(receive_ID==ECU_ECU2_CONTROL_ID){	
		SteeringEngineAndControl.FanControl=*receive_data>>7;//���ȿ����ź�
		SteeringEngineAndControl.Speaker_Control=(*receive_data&0x40)>>6;//���ȿ����ź�
		SteeringEngineAndControl.Taillight_Control=(*receive_data&0x20)>>5;//β�ƿ����ź�
		SteeringEngineAndControl.MotRun_Control=(*receive_data&0x10)>>4;//���Run�źţ����ã�
		SteeringEngineAndControl.WaterPump_Control=(*receive_data&0x08)>>3;//ˮ�ÿ����ź�
		SteeringEngineAndControl.IMDAndBrakeReliability_Trigger=(*receive_data&0x04)>>2;//IMD�ƶ��ɿ��Դ���
		SteeringEngineAndControl.Reserve0=(*receive_data&0x02)>>1;//��������
		SteeringEngineAndControl.Reserve1=*receive_data&0x01;//��������
	}
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

//	HAL_CAN_Receive_IT(hcan, CAN_FILTER_FIFO0);	
//	BaseType_t xHigherPriorityTaskWoken=pdFALSE;
//	vTaskNotifyGiveFromISR(CAN_DataAnalysisTaskHandle, &xHigherPriorityTaskWoken);
//	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);	
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CAN_RxCpltCallback could be implemented in the user file
   */
}

/* CAN_ReceiveData_AnalysisTask function */
void CAN_DataAnalysisTask(void const * argument)
{
  /* USER CODE BEGIN CAN_ReceiveData_AnalysisTask */
  /* Infinite loop */
	uint8_t Receive_Data[8];
	uint32_t Receive_ID;
  for(;;)
  {
		if(HAL_CAN_Receive(&hcan, CAN_FILTER_FIFO0, 100)==HAL_OK){
			Receive_ID=hcan.pRxMsg->ExtId;	
			for(uint8_t i=0;i<8;i++){
				Receive_Data[i]=hcan.pRxMsg->Data[i];
			}		
			SteeringEngineAndControl_Analysis(Receive_Data,Receive_ID);	
		}
		osDelay(20);	
  }
  /* USER CODE END CAN_ReceiveData_AnalysisTask */
}

/*
*****************************************************************************
*@brief		ȡֵ����������CAN���ղ������õ�����
*@param		Analysis_Data_n name_num����ȡ���ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data����Ŵ��ݳ���������
*@retval	None
*@par
*****************************************************************************
*/
void Get_CAN_Analysis_Data(Analysis_Data_n name_num, void * extern_data)//*extern_data�������get����ֵ
{	
	if(name_num==DRS_SteeringEngine1_Angle_n){
		*(float*)extern_data=SteeringEngineAndControl.DRS_SteeringEngine1_Angle;
	} 	
	else if(name_num==DRS_SteeringEngine2_Angle_n){
		*(float*)extern_data=SteeringEngineAndControl.DRS_SteeringEngine2_Angle;
	}
	else if(name_num==FanControl_n){
		*(uint8_t*)extern_data=SteeringEngineAndControl.FanControl;
	}
	else if(name_num==Speaker_Control_n){
		*(uint8_t*)extern_data=SteeringEngineAndControl.Speaker_Control;
	}
	else if(name_num==Taillight_Control_n){
		*(uint8_t*)extern_data=SteeringEngineAndControl.Taillight_Control;
	}
	else if(name_num==MotRun_Control_n){
		*(uint8_t*)extern_data=SteeringEngineAndControl.MotRun_Control;
	}
	else if(name_num==WaterPump_Control_n){
		*(uint8_t*)extern_data=SteeringEngineAndControl.WaterPump_Control;
	}
	else if(name_num==IMDAndBrakeReliability_Trigger_n){
		*(uint8_t*)extern_data=SteeringEngineAndControl.IMDAndBrakeReliability_Trigger;
	}
	else if(name_num==Reserve0_n){
		*(uint8_t*)extern_data=SteeringEngineAndControl.Reserve0;
	}
	else if(name_num==Reserve1_n){
		*(uint8_t*)extern_data=SteeringEngineAndControl.Reserve1;
	}
}

/*
*****************************************************************************
*@brief		�����ٶ��󡢺����ٶ��� ����Э�黯
*@param		uint32_t *Protocol_data�������Э�黯������
*@retval	None
*@par
*****************************************************************************
*/
static void ECU2_ECU_INFO0_Protocol_Send(uint32_t Protocol_ID)
{
	CANTransmitMessageINFO CANTransmitMessage;
	CANTransmitMessage.CANTransmitID=Protocol_ID;
	float speed3_data,speed4_data;
	Get_HallSpeed_Value_Data(HallSpeed3_Value_n,&speed3_data);//ȡֵ������������������������@TimGenericData_n
	Get_HallSpeed_Value_Data(HallSpeed4_Value_n,&speed4_data);//ȡֵ�����������Ҳ������������@TimGenericData_n
	CANTransmitMessage.CANTransmitData[0]=(uint8_t) (speed3_data*3.6);										//����������٣���λKm/h
	CANTransmitMessage.CANTransmitData[1]=(uint16_t)(speed3_data*1000)>>8;								//����������٣��߾�������(��λm/h)���߰�λ
	CANTransmitMessage.CANTransmitData[2]=(uint16_t) (speed3_data*1000)&0x00FF;						//�Ͱ�λ
	CANTransmitMessage.CANTransmitData[3]=(uint8_t) (speed4_data*3.6);										//�����Ҳ����٣���λKm/h
	CANTransmitMessage.CANTransmitData[4]=(uint16_t)(speed4_data*1000)>>8;								//�����Ҳ����٣��߾�������(��λm/h)���߰�λ
	CANTransmitMessage.CANTransmitData[5]=(uint16_t) (speed4_data*1000)&0x00FF;						//�Ͱ�λ
	CANTransmitMessage.CANTransmitData[6]=0x00;				
	CANTransmitMessage.CANTransmitData[7]=0x00;					
	xQueueSendToFront(CANTransmitQueueHandle,&CANTransmitMessage,(TickType_t)0 );	
}

/*
*****************************************************************************
*@brief		ǰ���ٶ���ǰ���ٶ��� ����Э�黯
*@param		uint32_t Protocol_ID�������Э�黯������
*@retval	None
*@par
*****************************************************************************
*/
static void ECU2_ECU_INFO1_Protocol_Send(uint32_t Protocol_ID)
{
	CANTransmitMessageINFO CANTransmitMessage;
	CANTransmitMessage.CANTransmitID=Protocol_ID;
	float speed2_data,speed1_data;
	Get_HallSpeed_Value_Data(HallSpeed2_Value_n,&speed2_data);//ȡֵ�������������ǰ����������@TimGenericData_n
	Get_HallSpeed_Value_Data(HallSpeed1_Value_n,&speed1_data);//ȡֵ�����������Ҳ�ǰ����������@TimGenericData_n
	CANTransmitMessage.CANTransmitData[0]=(uint8_t) (speed2_data*3.6);										//ǰ��������٣���λKm/h
	CANTransmitMessage.CANTransmitData[1]=(uint16_t)(speed2_data*1000)>>8;								//ǰ��������٣��߾�������(��λm/h)���߰�λ
	CANTransmitMessage.CANTransmitData[2]=(uint16_t) (speed2_data*1000)&0x00FF;						//�Ͱ�λ
	CANTransmitMessage.CANTransmitData[3]=(uint8_t) (speed1_data*3.6);										//ǰ���Ҳ����٣���λKm/h
	CANTransmitMessage.CANTransmitData[4]=(uint16_t)(speed1_data*1000)>>8;								//ǰ���Ҳ����٣��߾�������(��λm/h)���߰�λ
	CANTransmitMessage.CANTransmitData[5]=(uint16_t) (speed1_data*1000)&0x00FF;						//�Ͱ�λ
	CANTransmitMessage.CANTransmitData[6]=0x00;//�¶ȴ�������δ��ͨ����ȡ0				
	CANTransmitMessage.CANTransmitData[7]=0x00;	
	xQueueSendToFront(CANTransmitQueueHandle,&CANTransmitMessage,(TickType_t)0 );	
}
  
/*
*****************************************************************************
*@brief		AMS IMD ��λ�ź� ����Э�黯
*@param		uint32_t Protocol_ID�������Э�黯������
*@retval	None
*@par
*****************************************************************************
*/
static void ECU2_ECU_INFO2_Protocol_Send(uint32_t Protocol_ID)
{
	CANTransmitMessageINFO CANTransmitMessage;
	CANTransmitMessage.CANTransmitID=Protocol_ID;
	uint8_t AMS_IMDReset_Data=0;
	Get_AMS_IMDReset_State_Data(AMS_IMDReset_n,&AMS_IMDReset_Data);
	CANTransmitMessage.CANTransmitData[0]=AMS_IMDReset_Data<<7;
	for(uint8_t i=1;i<8;i++){
		CANTransmitMessage.CANTransmitData[i]=0x00;	
	}
	xQueueSendToFront(CANTransmitQueueHandle,&CANTransmitMessage,(TickType_t)0 );	
}

/*
*****************************************************************************
*@brief		ˮ���¶�״̬�����ȣ�ˮ�ã�����״̬������Э�黯
*@param		uint32_t Protocol_ID�������Э�黯������
*@retval	None
*@par
*****************************************************************************
*/
static void ECU2_ECU_STATE_Protocol_Send(uint32_t Protocol_ID)
{
	CANTransmitMessageINFO CANTransmitMessage;
	CANTransmitMessage.CANTransmitID=Protocol_ID;
	uint8_t Speaker,Taillight,Fan,WaterPump;	
	Get_Speaker_Taillight_Fan_Waterpump_State_Data(WaterPump_Stata_n,&WaterPump);					//ȡֵ����������ˮ��״̬����@GpioGenericData_n
	Get_Speaker_Taillight_Fan_Waterpump_State_Data(Fan_Stata_n,&Fan);											//ȡֵ���������˷���״̬����@GpioGenericData_n
	Get_Speaker_Taillight_Fan_Waterpump_State_Data(Taillight_Stata_n,&Taillight);					//ȡֵ����������β��״̬����@GpioGenericData_n
	Get_Speaker_Taillight_Fan_Waterpump_State_Data(Speaker_Stata_n,&Speaker);							//ȡֵ��������������״̬����@GpioGenericData_n
	//�����¶�״̬���ڴ�����Ϊ0	
	CANTransmitMessage.CANTransmitData[0]=WaterPump<<5|Fan<<4|Taillight<<3|Speaker<<2;
	for(uint8_t i=1;i<8;i++)
	{
		CANTransmitMessage.CANTransmitData[i]=0x00;	
	}
	xQueueSendToBack(CANTransmitQueueHandle,&CANTransmitMessage,(TickType_t)0 );	
}

/*
*****************************************************************************
*@brief		CAN�������ݺ���
*@param		uint32_t Protocol_ID��Э��ID
*@retval	None
*@par
*****************************************************************************
*/
void CAN_DATA_Send(uint32_t Protocol_ID)
{	
	if(Protocol_ID==ECU2_ECU_INFO0_ID){		
		ECU2_ECU_INFO0_Protocol_Send(Protocol_ID);
	}
	else if(Protocol_ID==ECU2_ECU_INFO1_ID){		
		ECU2_ECU_INFO1_Protocol_Send(Protocol_ID);		
	}
	else if(Protocol_ID==ECU2_ECU_INFO2_ID){		
		ECU2_ECU_INFO2_Protocol_Send(Protocol_ID);	
	}
	else if(Protocol_ID==ECU2_ECU_STATE_ID){
		ECU2_ECU_STATE_Protocol_Send(Protocol_ID);
	}
}

void CAN_100PeriodicSendCallback(TimerHandle_t xTimer)
{	
	//100ms���ڷ�������
	CAN_DATA_Send(ECU2_ECU_STATE_ID);	
}

void CAN_10PeriodicSendCallback(TimerHandle_t xTimer)
{
	//10ms���ڷ�������
	CAN_DATA_Send(ECU2_ECU_INFO0_ID);
	CAN_DATA_Send(ECU2_ECU_INFO1_ID);
}

//void CAN_10PeriodicSendTask(void const * argument)
//{
//	//10ms���ڷ�������
//	CAN_DATA_Send(ECU2_ECU_INFO0_ID);
//	CAN_DATA_Send(ECU2_ECU_INFO1_ID);
//	osDelay(10);
//}

/* CAN_TransmitTask function */
void CAN_TransmitTask(void const * argument)
{
  /* USER CODE BEGIN CAN_TransmitTask */
  /* Infinite loop */
	if(Get_CAN_100PeriodicSendTimer_Handle()!=NULL){
		xTimerReset(Get_CAN_100PeriodicSendTimer_Handle(),10);																						//��λ���ڶ�ʱ��
		xTimerStart(Get_CAN_100PeriodicSendTimer_Handle(),10);			//�������ڶ�ʱ��
	}
	if(Get_CAN_10PeriodicSendTimer_Handle()!=NULL){
		xTimerReset(Get_CAN_10PeriodicSendTimer_Handle(),10);																						//��λ���ڶ�ʱ��
		xTimerStart(Get_CAN_10PeriodicSendTimer_Handle(),10);			//�������ڶ�ʱ��
	}
	CANTransmitMessageINFO CANTransmitMessage;
	for(;;)
  {
		if(xQueueReceive(CANTransmitQueueHandle,&CANTransmitMessage,(TickType_t)0)==pdTRUE){
			CSU_CAN_Send(&hcan,CANTransmitMessage.CANTransmitID,CANTransmitMessage.CANTransmitData, 8);
		}
		osDelay(2);
  }
  /* USER CODE END CAN_TransmitTask */
}

/*
*****************************************************************************
*@brief		�޸�CAN���ղ������õ�����
*@param		Analysis_Data_n name_num�����޸����ݶ�Ӧ��ö�ٱ���
*@param		void* extern_data�������޸ĵ�Ŀ��ֵ
*@retval	None
*@par
*****************************************************************************
*/
void Set_CAN_Analysis_Data(Analysis_Data_n name_num, void * extern_data)//*extern_data���޸�Ŀ��ֵ����
{	
	if(name_num==DRS_SteeringEngine1_Angle_n){
		SteeringEngineAndControl.DRS_SteeringEngine1_Angle=*(float*)extern_data;
	} 	
	else if(name_num==DRS_SteeringEngine2_Angle_n){
		SteeringEngineAndControl.DRS_SteeringEngine2_Angle=*(float*)extern_data;
	}
	else if(name_num==FanControl_n){
		SteeringEngineAndControl.FanControl=*(uint8_t*)extern_data;
	}
	else if(name_num==Speaker_Control_n){
		SteeringEngineAndControl.Speaker_Control=*(uint8_t*)extern_data;
	}
	else if(name_num==Taillight_Control_n){
		SteeringEngineAndControl.Taillight_Control=*(uint8_t*)extern_data;
	}
	else if(name_num==MotRun_Control_n){
		SteeringEngineAndControl.MotRun_Control=*(uint8_t*)extern_data;
	}
	else if(name_num==WaterPump_Control_n){
		SteeringEngineAndControl.WaterPump_Control=*(uint8_t*)extern_data;
	}
	else if(name_num==IMDAndBrakeReliability_Trigger_n){
		SteeringEngineAndControl.IMDAndBrakeReliability_Trigger=*(uint8_t*)extern_data;
	}
	else if(name_num==Reserve0_n){
		SteeringEngineAndControl.Reserve0=*(uint8_t*)extern_data;
	}
	else if(name_num==Reserve1_n){
		SteeringEngineAndControl.Reserve1=*(uint8_t*)extern_data;
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
