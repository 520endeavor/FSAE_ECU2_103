#ifndef __WIRELESS_AS32_TTL_1W_H
#define __WIRELESS_AS32_TTL_1W_H

#include "usart.h"
#include "gpio.h"

void Wireless_AS32_Transmit(uint8_t *pData,uint16_t Size);		//无线串口发送数据函数
void Wireless_AS32_Receive(uint8_t *pData,uint16_t Size);		//无线串口接收数据函数



#endif

