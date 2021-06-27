#include "AT24Cxx.h" 

/*
*****************************************************************************
*@brief		��AT24Cxxָ����ַ����һ������
*@param		ReadAddr:��ʼ�����ĵ�ַ  		
*@retval	����ֵ  :����������
*@par
*****************************************************************************
*/
uint8_t AT24Cxx_ReadOneByte(uint16_t ReadAddr)
{				  
	uint8_t temp=0;	
	//HAL_I2C_Mem_Read(&hi2c2, AT24C16_Address, ReadAddr, AT24CXX_MEMADD_SIZE, &temp, 1, 10);
	if(EE_TYPE>AT24C16){
		HAL_I2C_Mem_Read(&hi2c2, AT24Cxx_Address, ReadAddr, AT24CXX_MEMADD_SIZE, &temp, 1, 100);
	}
	else {		
		HAL_I2C_Mem_Read(&hi2c2, (AT24Cxx_Address+((ReadAddr/256)<<1)), (ReadAddr%256), AT24CXX_MEMADD_SIZE, &temp, 1, 100);
	}
	HAL_Delay(2);		//AT24C02��Ҫ��ʱ��FRAM16����Ҫ��ʱ
	return temp;
}

/*
*****************************************************************************
*@brief		��AT24Cxxָ����ַд��һ���ֽ�����
*@param		WriteAddr:д�����ݵ�Ŀ�ĵ�ַ 
*@param		DataToWrite:Ҫд�������
*@retval	None
*@par
*****************************************************************************
*/
void AT24Cxx_WriteOneByte(uint16_t WriteAddr,uint8_t DataToWrite)
{			
	if(EE_TYPE>AT24C16){
		HAL_I2C_Mem_Write(&hi2c2, AT24Cxx_Address, WriteAddr, AT24CXX_MEMADD_SIZE, &DataToWrite,  1, 100);		
	}
	else {		
		HAL_I2C_Mem_Write(&hi2c2, (AT24Cxx_Address+((WriteAddr/256)<<1)), (WriteAddr%256), AT24CXX_MEMADD_SIZE, &DataToWrite,  1, 100);
	}	
	HAL_Delay(2);		//AT24C02��Ҫ��ʱ��FRAM16����Ҫ��ʱ
}

/*
*****************************************************************************
*@brief		��AT24Cxx�����ָ����ַ��ʼд�볤��ΪLen������,С��ģʽ�洢����
					�ú�������д��16bit����32bit������.
*@param		WriteAddr:��ʼд��ĵ�ַ  
*@param		DataToWrite:���������׵�ַ
*@param		Len:Ҫд�����ݵĳ���2,4
*@retval	None
*@par
*****************************************************************************
*/
void AT24Cxx_WriteLenByte(uint16_t WriteAddr,uint32_t DataToWrite,uint8_t Len)
{  	
	uint8_t t;
	for(t=0;t<Len;t++)
	{
		AT24Cxx_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

/*
*****************************************************************************
*@brief		��AT24Cxx�����ָ����ַ��ʼ��������ΪLen������,С��ģʽ�洢����
					�ú������ڶ���16bit����32bit�����ݣ��ȶ����ַ�ĸ��ֽ�����
*@param		ReadAddr:��ʼ�����ĵ�ַ 
*@param		DataToWrite:���������׵�ַ
*@param		Len:Ҫ�������ݵĳ���2,4
*@retval	����ֵ������������
*@par
*****************************************************************************
*/
uint32_t AT24Cxx_ReadLenByte(uint16_t ReadAddr,uint8_t Len)
{  	
	uint8_t t;
	uint32_t temp=0;
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		temp+=AT24Cxx_ReadOneByte(ReadAddr+Len-t-1); 	 				   
	}
	return temp;												    
}

/*
*****************************************************************************
*@brief		���AT24Cxx�Ƿ���������������AT24Cxx�����һ����ַ(255)���洢��־��
					�������AT24Cxxϵ��,�����ַҪ�޸�
*@retval	����ֵ��1�����ʧ�ܣ�0�����ɹ�
*@par
*****************************************************************************
*/
uint8_t AT24Cxx_Check(void)
{
	uint8_t temp;
	temp=AT24Cxx_ReadOneByte(EE_TYPE);//����ÿ�ο�����дAT24Cxx			   
	if(temp==0X55){
		return 0;	
	}		
	else			//�ų���һ�γ�ʼ�������
	{
		AT24Cxx_WriteOneByte(EE_TYPE,0X55);
		temp=AT24Cxx_ReadOneByte(EE_TYPE);	  
		if(temp==0X55){
			return 0;
		}
	}
	return 1;											  
}

/*
*****************************************************************************
*@brief		��AT24CXX�����ָ����ַ��ʼ����ָ������������,��������
*@param		ReadAddr:��ʼ�����ĵ�ַ ��24c02Ϊ0~255
*@param		pBuffer:���������׵�ַ
*@param		NumToRead:Ҫ�������ݵĸ���
*@retval	None
*@par
*****************************************************************************
*/
void AT24Cxx_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=AT24Cxx_ReadOneByte(ReadAddr++);	
		NumToRead--;
	}
}  

/*
*****************************************************************************
*@brief		��AT24Cxx�����ָ����ַ��ʼд��ָ������������,����д��
*@param		WriteAddr:��ʼд��ĵ�ַ ��24c02Ϊ0~255
*@param		pBuffer:���������׵�ַ
*@param		NumToWrite:Ҫд�����ݵĸ���
*@retval	None
*@par
*****************************************************************************
*/
void AT24Cxx_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite)
{
	while(NumToWrite--)
	{
		AT24Cxx_WriteOneByte(WriteAddr,*pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}








