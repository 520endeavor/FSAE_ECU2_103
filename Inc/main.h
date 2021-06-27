/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define ECU2_TemSensor18B20DQ_Pin GPIO_PIN_0
#define ECU2_TemSensor18B20DQ_GPIO_Port GPIOA
#define ECU2_Backup_AD3_Pin GPIO_PIN_2
#define ECU2_Backup_AD3_GPIO_Port GPIOC
#define ECU2_Backup_AD2_Pin GPIO_PIN_3
#define ECU2_Backup_AD2_GPIO_Port GPIOC
#define ECU2_LED2_Pin GPIO_PIN_1
#define ECU2_LED2_GPIO_Port GPIOA
#define ECU2_USART2_TX_Wireless_Pin GPIO_PIN_2
#define ECU2_USART2_TX_Wireless_GPIO_Port GPIOA
#define ECU2_USART2_RX_Wireless_Pin GPIO_PIN_3
#define ECU2_USART2_RX_Wireless_GPIO_Port GPIOA
#define ECU2_Backup_AD0_Pin GPIO_PIN_4
#define ECU2_Backup_AD0_GPIO_Port GPIOA
#define ECU2_LED1_Pin GPIO_PIN_5
#define ECU2_LED1_GPIO_Port GPIOA
#define ECU2_DRS_SteeringEngine2_Pin GPIO_PIN_6
#define ECU2_DRS_SteeringEngine2_GPIO_Port GPIOA
#define ECU2_DRS_SteeringEngine1_Pin GPIO_PIN_7
#define ECU2_DRS_SteeringEngine1_GPIO_Port GPIOA
#define ECU2_IMD_Relay_Ctrl GPIO_PIN_5
#define ECU2_IMD_Relay_Ctrl_GPIO_Port GPIOC
#define ECU2_Backup_Relay6_Pin GPIO_PIN_0
#define ECU2_Backup_Relay6_GPIO_Port GPIOB
#define ECU2_Backup_Relay5_Pin GPIO_PIN_1
#define ECU2_Backup_Relay5_GPIO_Port GPIOB
#define ECU2_AT24C02_SCL_Pin GPIO_PIN_10
#define ECU2_AT24C02_SCL_GPIO_Port GPIOB
#define ECU2_AT24C02_SDA_Pin GPIO_PIN_11
#define ECU2_AT24C02_SDA_GPIO_Port GPIOB
#define ECU2_IMD_Reset_Key_Pin GPIO_PIN_12
#define ECU2_IMD_Reset_Key_GPIO_Port GPIOB
#define ECU2_Backup_Relay4_Pin GPIO_PIN_13
#define ECU2_Backup_Relay4_GPIO_Port GPIOB
#define ECU2_Backup_Relay3_Pin GPIO_PIN_14
#define ECU2_Backup_Relay3_GPIO_Port GPIOB
#define ECU2_Backup_Relay2_Pin GPIO_PIN_15
#define ECU2_Backup_Relay2_GPIO_Port GPIOB
#define ECU2_Backup_Relay1_Pin GPIO_PIN_6
#define ECU2_Backup_Relay1_GPIO_Port GPIOC
#define ECU2_HallSpeed2_Pin GPIO_PIN_7
#define ECU2_HallSpeed2_GPIO_Port GPIOC
#define ECU2_HallSpeed1_Pin GPIO_PIN_8
#define ECU2_HallSpeed1_GPIO_Port GPIOC
#define ECU2_HallSpeed3_Pin GPIO_PIN_9
#define ECU2_HallSpeed3_GPIO_Port GPIOC
#define ECU2_HallSpeed4_Pin GPIO_PIN_8
#define ECU2_HallSpeed4_GPIO_Port GPIOA
#define ECU2_USART1_TX_Pin GPIO_PIN_9
#define ECU2_USART1_TX_GPIO_Port GPIOA
#define ECU2_USART1_RX_Pin GPIO_PIN_10
#define ECU2_USART1_RX_GPIO_Port GPIOA
#define ECU2_CAN_RX_Pin GPIO_PIN_11
#define ECU2_CAN_RX_GPIO_Port GPIOA
#define ECU2_CAN_TX_Pin GPIO_PIN_12
#define ECU2_CAN_TX_GPIO_Port GPIOA
#define ECU2_Optocoupler_MOS4_Pin GPIO_PIN_10
#define ECU2_Optocoupler_MOS4_GPIO_Port GPIOC
#define ECU2_USART5_TX_ESP8266_Pin GPIO_PIN_12
#define ECU2_USART5_TX_ESP8266_GPIO_Port GPIOC
#define ECU2_USART5_RX_ESP8266_Pin GPIO_PIN_2
#define ECU2_USART5_RX_ESP8266_GPIO_Port GPIOD
#define ECU2_Optocoupler_MOS2_Pin GPIO_PIN_6
#define ECU2_Optocoupler_MOS2_GPIO_Port GPIOB
#define ECU2_Optocoupler_MOS5_Pin GPIO_PIN_7
#define ECU2_Optocoupler_MOS5_GPIO_Port GPIOB
#define ECU2_Optocoupler_MOS3_Pin GPIO_PIN_8
#define ECU2_Optocoupler_MOS3_GPIO_Port GPIOB
#define ECU2_Optocoupler_MOS1_Pin GPIO_PIN_9
#define ECU2_Optocoupler_MOS1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
