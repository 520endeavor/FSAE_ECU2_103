#include "Wireless_AS32_TTL_1W.h"

/*
*****************************************************************************
*@brief		���ߴ��ڷ������ݺ���
*@param		uint8_t *pData�����������׵�ַ
*@param		uint16_t Size���������ݸ���
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
*@brief		���ߴ��ڽ������ݺ���
*@param		uint8_t *pData:���������׵�ַ
*@param		uint16_t Size���������ݸ���
*@retval	None
*@par
*****************************************************************************
*/
void Wireless_AS32_Receive(uint8_t *pData,uint16_t Size)
{
	HAL_UART_Receive_DMA(&huart2, pData, Size);	
}

