#include "ds18b20.h"

  /*
*****************************************************************************
*@brief		微秒延时函数
*@param		uint32_t cnt，延时大小，延时大小，单位us
*@retval	None
*@par
*****************************************************************************
*/
static void ds18b20delay_us(uint32_t cnt)  
{  
	uint32_t i,j;  
	for(i=0;i<cnt;i++)  
	{  
		for(j=0;j<10;j++);  
	}  
}

/*
*****************************************************************************
*@brief		IO方向设置：输出
*@retval	None
*@par
*****************************************************************************
*/
void DS18B20_IO_OUT(void)
{	
	GPIO_InitTypeDef GPIO_InitStruct;	
  GPIO_InitStruct.Pin = ECU2_TemSensor18B20DQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ECU2_TemSensor18B20DQ_GPIO_Port, &GPIO_InitStruct);
}

/*
*****************************************************************************
*@brief		IO方向设置：输入
*@retval	None
*@par
*****************************************************************************
*/
void DS18B20_IO_IN(void)
{	
	GPIO_InitTypeDef GPIO_InitStruct;	
  GPIO_InitStruct.Pin = ECU2_TemSensor18B20DQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ECU2_TemSensor18B20DQ_GPIO_Port, &GPIO_InitStruct);
}

/*
*****************************************************************************
*@brief		复位DS18B20
*@retval	None
*@par
*****************************************************************************
*/
void DS18B20_Rst(void)	   
{                 
	DS18B20_IO_OUT(); 			//SET   OUTPUT
	DS18B20_DQ_OUT_LOW; 		//拉低DQ
	ds18b20delay_us(500);   //拉低750us
	DS18B20_DQ_OUT_HIGH; 		//DQ=1 
	ds18b20delay_us(15);    //15US
}

/*
*****************************************************************************
*@brief		等待DS18B20的回应
*@retval	返回1:未检测到DS18B20的存在，返回0:存在 
*@par
*****************************************************************************
*/
uint8_t DS18B20_Check(void) 	   
{   
	uint8_t retry=0;
	DS18B20_IO_IN();				//SET INPUT	 
    while (DS18B20_DQ_IN&&retry<200)
	{
		retry++;
		ds18b20delay_us(1);
	};	 
	if(retry>=200)return 1;
	else retry=0;
    while (!DS18B20_DQ_IN&&retry<240)
	{
		retry++;
		ds18b20delay_us(1);
	};
	if(retry>=240)return 1;	    
	return 0;
}

/*
*****************************************************************************
*@brief		从DS18B20读取一个位
*@retval	返回值：1/0
*@par
*****************************************************************************
*/
uint8_t DS18B20_Read_Bit(void) // read one bit
{
	uint8_t data;
	DS18B20_IO_OUT();						//SET OUTPUT
  DS18B20_DQ_OUT_LOW; 
	ds18b20delay_us(2);
  DS18B20_DQ_OUT_HIGH; 
	DS18B20_IO_IN();						//SET INPUT
	ds18b20delay_us(12);
	if(DS18B20_DQ_IN){
		data=1;
	}
	else{ 
		data=0;	
	}			
	ds18b20delay_us(50);           
	return data;
}

/*
*****************************************************************************
*@brief		从DS18B20读取一个字节
*@retval	返回值：读到的数据
*@par
*****************************************************************************
*/
uint8_t DS18B20_Read_Byte(void)    // read one byte
{        
	uint8_t i,j,dat;
	dat=0;
	for (i=1;i<=8;i++) 
	{
		j=DS18B20_Read_Bit();
		ds18b20delay_us(1);
		dat=(j<<7)|(dat>>1);
	}						    
	return dat;
}
	
/*
*****************************************************************************
*@brief		写一个字节到DS18B20
*@param		dat：要写入的字节
*@retval	None
*@par
*****************************************************************************
*/
void DS18B20_Write_Byte(uint8_t dat)     
{             
	uint8_t j;
	uint8_t testb;
	DS18B20_IO_OUT();					//SET OUTPUT;
	for (j=1;j<=8;j++) 
	{
		testb=dat&0x01;
		dat=dat>>1;
		if (testb) 
		{
			DS18B20_DQ_OUT_LOW;		// Write 1
			ds18b20delay_us(2);                            
			DS18B20_DQ_OUT_HIGH;
			ds18b20delay_us(60);             
		}
		else 
		{
			DS18B20_DQ_OUT_LOW;		// Write 0
			ds18b20delay_us(60);             
			DS18B20_DQ_OUT_HIGH;
			ds18b20delay_us(2);                          
		}
	}
}

/*
*****************************************************************************
*@brief		开始温度转换
*@retval	None
*@par
*****************************************************************************
*/
void DS18B20_Start(void)			// ds1820 start convert
{   						               
	DS18B20_Rst();	   
	DS18B20_Check();	 
	DS18B20_Write_Byte(0xcc);		// skip rom
	DS18B20_Write_Byte(0x44);		// convert
} 

/*
*****************************************************************************
*@brief		初始化DS18B20的IO口，同时检测DS的存在
*@retval	返回1:不存在，返回0:存在   
*@par
*****************************************************************************
*/
uint8_t DS18B20_Init(void)
{
 	GPIO_InitTypeDef GPIO_InitStruct;
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = ECU2_TemSensor18B20DQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  
	HAL_GPIO_Init(ECU2_TemSensor18B20DQ_GPIO_Port, &GPIO_InitStruct); 		
	HAL_GPIO_WritePin(ECU2_TemSensor18B20DQ_GPIO_Port, ECU2_TemSensor18B20DQ_Pin, GPIO_PIN_SET);
	
	DS18B20_Rst();
	return DS18B20_Check();
}  

/*
*****************************************************************************
*@brief		从ds18b20得到温度值
*@retval	返回值：温度值 （-55~125） 
*@par
*****************************************************************************
*/
float DS18B20_Get_Temp(void)
{
	float T;
	uint8_t temp;
	uint8_t TL,TH;
	uint16_t tem;
	//short tem;
	DS18B20_Start ();                 // ds1820 start convert
	DS18B20_Rst();
	DS18B20_Check();	 
	DS18B20_Write_Byte(0xcc);					// skip rom
	DS18B20_Write_Byte(0xee);					// read	    
	TL=DS18B20_Read_Byte(); 					// LSB   
	TH=DS18B20_Read_Byte(); 					// MSB   
	if(TH>7)
	{
		TH=~TH;
		TL=~TL; 
		temp=0;													//温度为负  
	}else{
		temp=1;													//温度为正	
	}		
	tem=TH; 													//获得高八位
	tem<<=8;    
	tem+=TL;													//获得低八位
	T=(double)tem*0.0625;							//转换     
	if(temp){
		return T; 											//返回温度值
	}
	else{
		return -T;
	}    
}   	 
    	  
    
