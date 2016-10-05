/*********************************HEAD FILES****************************************************/
#include"i2c.h"

/********************************************BASIC FUNCTION*************************************/

/************************************************************************************************ 
Name：I2C_Start 
Function:	
		  	start the i2c
Parameters：
		   	void
Returns：
			void 
Description:
			the function I2C_delay() is very important
************************************************************************************************/
uint8_t I2C_Start(void)
{
	SDA_OUT;
	SCL_OUT;
	SDA_H;

	SCL_H;
	I2C_delay();
	if(!SDA_read)
		return FALSE;
	SDA_L;
	I2C_delay();
	if(SDA_read)
		return FALSE;
	I2C_delay();
	return TRUE;
}

/************************************************************************************************ 
Name：I2C_SendByte 
Function:	
		  	send a byte to i2c bus
Parameters：
		   	[in]	-	SendByte:the byte need to be sent to the i2c bus
Returns：
			void 
Description:
			null
************************************************************************************************/
void I2C_SendByte(uint8_t SendByte)
{
	uint8_t i = 8;
	SDA_OUT;
	while(i--)
	{
		SCL_L;
		I2C_delay();

		if(SendByte & 0x80)
			SDA_H;
		else
			SDA_L;

		SendByte <<= 1;
		I2C_delay();
		SCL_H;
		I2C_delay();
	}
	SCL_L;
}

/************************************************************************************************ 
Name：I2C_ReadByte 
Function:	
		  	read a byte from i2c bus
Parameters：
		   	void
Returns：
			[out]	-	uint8_t:the data read from the i2c bus 
Description:
			null
************************************************************************************************/
uint8_t I2C_ReadByte(void)
{
	uint8_t i = 8;
	uint8_t ReceiveByte = 0;
	SDA_IN;
	while(i--)
	{
		ReceiveByte <<= 1;
		SCL_L;
		I2C_delay();
		SCL_H;					//read the data when scl is high
		I2C_delay();
		if(SDA_read)
		{
			ReceiveByte |= 0x01;
		}
	}
	SCL_L;
	return ReceiveByte;
}

/************************************************************************************************ 
Name：I2C_SendAck 
Function:	
		  	send ack to the i2c bus
Parameters：
		   	[in]	-	ack:ack data need to be sent to the i2c bus
Returns：
			void 
Description:
			1		noack
			0		ack
************************************************************************************************/
void I2C_SendAck(uint8_t ack)
{
	SDA_OUT;
	SCL_L;
	I2C_delay();
	if(ack)
		SDA_H;
	else
		SDA_L;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}

/************************************************************************************************ 
Name：I2C_WaitAck 
Function:	
		  	receive a ack from i2c bus
Parameters：
		   	void
Returns：
			[out]	-	uint8_t:the ack value read from the i2c bus 
Description:
			1		noack
			0		ack
************************************************************************************************/
uint8_t I2C_WaitAck(void)
{
	SDA_IN;
	SCL_L;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	if(SDA_read)
	{
		SCL_L;
		I2C_delay();
		return FALSE;
	}
	SCL_L;
	I2C_delay();
	return TRUE;
}

/************************************************************************************************ 
Name：I2C_Stop 
Function:	
		  	stop the i2c bus
Parameters：
		   	void
Returns：
			void 
Description:
			null
************************************************************************************************/
void I2C_Stop(void)
{
	SDA_OUT;
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SDA_H;
	I2C_delay();
}

/************************************************************************************************ 
Name：I2C_delay 
Function:	
		  	the time delay of i2c
Parameters：
		   	void
Returns：
			void 
Description:
			null
************************************************************************************************/
void I2C_delay(void)
{
	uint8_t i = 60;
	while(i)
	{
		i--;
	}
}


/***************************************EXTEND FUNCTION*****************************************/

/************************************************************************************************ 
Name：Single_Write 
Function:	
		  	read a byte from the i2c bus
Parameters：
		   	[in]	-	SlaveAddress:i2c slave address
						REG_Address:the slave's internal data address
						REG_data:the data which need to be wrote to the slave
Returns：
			[out]	-	uint8_t:return if the option success or fail 
Description:
			null
************************************************************************************************/
uint8_t Single_Write(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t REG_data)
{
	if(!I2C_Start())
		return FALSE;
	I2C_SendByte(SlaveAddress);
	if(!I2C_WaitAck())
	{
		I2C_Stop();
		return FALSE;
	}
	I2C_SendByte(REG_Address);
	I2C_WaitAck();
	I2C_SendByte(REG_data);
	I2C_WaitAck();
	I2C_Stop();
	delayms(5);
	return TRUE;
}

/************************************************************************************************ 
Name：Single_Read 
Function:	
		  	read the data from the mpu6050
Parameters：
		   	[in]	-	SlaveAddress:the i2c slave's address
						REG_Address:the slave's internal data address
Returns：
			[out]	-	uint8_t:the data read from the i2c bus 
Description:
			null
************************************************************************************************/
uint8_t Single_Read(uint8_t SlaveAddress,uint8_t REG_Address)
{   
	uint8_t REG_data;     	
	if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop();/*test=1; */return FALSE;}
    I2C_SendByte((uint8_t) REG_Address);   //设置低起始地址      
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(SlaveAddress+1);
    I2C_WaitAck();

	REG_data= I2C_ReadByte();
    I2C_SendAck(1);	//no ack
    I2C_Stop();
	return REG_data;
}

/************************************************************************************************ 
Name：delayms 
Function:	
		  	delay
Parameters：
		   	[in]	-	num:the time length need to delay
Returns：
			void 
Description:
			null
************************************************************************************************/
void delayms(uint32_t num)
{
		
   int i,j;  
   for(j = 0;j < num; j++)
   {
   	i = 1000;
   	while(i) 
   	{ 
    	 i--; 
   	}
   }  
}
