#include "Wireless_AS32_TTL_1W.h"

/*
*****************************************************************************
*@brief		无线串口发送数据函数
*@param		uint8_t *pData：发送数据首地址
*@param		uint16_t Size：发送数据个数
*@retval	None
*@par
*****************************************************************************
*/
void Wireless_AS32_Transmit(uint8_t *pData,uint16_t Size)
{
	HAL_UART_Transmit_DMA(&huart2, pData, Size);
}

/*
*****************************************************************************
*@brief		无线串口接收数据函数
*@param		uint8_t *pData:接收数据首地址
*@param		uint16_t Size：接收数据个数
*@retval	None
*@par
*****************************************************************************
*/
void Wireless_AS32_Receive(uint8_t *pData,uint16_t Size)
{
	HAL_UART_Receive_DMA(&huart2, pData, Size);	
}

