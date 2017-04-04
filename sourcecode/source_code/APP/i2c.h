#ifndef I2C_H
#define I2C_H
#include<stdint.h>
#include"stm32f10x.h"
#include"BSP.H"

//I2C GPIO defination
#define SCL_H         GPIOB->BSRR = GPIO_Pin_1				//HIGH
#define SCL_L         GPIOB->BRR  = GPIO_Pin_1 				//LOW
#define SCL_read      GPIOB->IDR  & GPIO_Pin_1				//READ
   
#define SDA_H         GPIOB->BSRR = GPIO_Pin_0				//HIGH
#define SDA_L         GPIOB->BRR  = GPIO_Pin_0				//LOW
#define SDA_read      (GPIOB->IDR  & GPIO_Pin_0)>>0 		//READ

#define SDA_IN		{GPIOB->CRL&=0XFFFFFFF0;GPIOB->CRL|=0X00000008;}		
#define SDA_OUT		{GPIOB->CRL&=0XFFFFFFF0;GPIOB->CRL|=0X00000007;}//change from 0X00000003 to 0X00000007		
#define SCL_OUT		{GPIOB->CRL&=0XFFFFFF0F;GPIOB->CRL|=0X00000030;}

//function declare
void Init_MPU6050(void);
void READ_MPU6050(int16_t *p);

uint8_t I2C_Start(void);
void I2C_SendByte(uint8_t SendByte);
uint8_t I2C_ReadByte(void);
void I2C_SendAck(uint8_t ack);
uint8_t I2C_WaitAck(void);
void I2C_Stop(void);

uint8_t Single_Write(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t REG_data);
uint8_t Single_Read(uint8_t SlaveAddress, uint8_t REG_Address);
void delayms(uint32_t num);

void I2C_delay(void);

#endif
