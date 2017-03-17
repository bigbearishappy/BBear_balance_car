/*************************************HEAD FILES******************************/
#include"mpu6050.h"

/******************************************************************************
Name£ºInitMPU6050 
Function:	
		  	initialize the mpu6050 
Parameters£º
		   	void
Returns£º
			void 
Description:
			wake the sensor and configuration
******************************************************************************/
void InitMPU6050(void)
{
	int i = 0;
	for(i = 0;i < 10; i++){
	Single_Write(MPU6050_Addr, PWR_MGMT_1, 0x00);	//½â³ýÐÝÃß×´Ì¬
	Single_Write(MPU6050_Addr, SMPLRT_DIV, 0x07);
	Single_Write(MPU6050_Addr, CONFIG, 0x00);
	Single_Write(MPU6050_Addr, GYRO_CONFIG, 0x18);	//set the gyro range :+-2000'/s	 
	Single_Write(MPU6050_Addr, ACCEL_CONFIG, 0x28);	//0x09 = 00001000  +-4g
	}
}

void Cal_Init_angle(void)
{
	
}

/******************************************************************************
Name£ºREAD_MPU6050 
Function:	
		  	read data from the mpu6050
Parameters£º
		   	[in]	-	*p:the pointer point to the data array
Returns£º
			void 
Description:
			null
******************************************************************************/
void READ_MPU6050(int16_t *p)
{
	uint8_t BUF[6];

   	BUF[0]=Single_Read(MPU6050_Addr,ACCEL_XOUT_L); 	//X AXIS ACC
   	BUF[1]=Single_Read(MPU6050_Addr,ACCEL_XOUT_H);
   	p[0] = (BUF[1]<<8)|BUF[0];

   	BUF[2]=Single_Read(MPU6050_Addr,ACCEL_YOUT_L);	//Z AXIS ACC
   	BUF[3]=Single_Read(MPU6050_Addr,ACCEL_YOUT_H);
   	p[1] = (BUF[3]<<8)|BUF[2];

   	BUF[4]=Single_Read(MPU6050_Addr,GYRO_YOUT_L);	//Y AXIS GYRO
   	BUF[5]=Single_Read(MPU6050_Addr,GYRO_YOUT_H);
   	p[2] = (BUF[5]<<8)|BUF[4];
}
