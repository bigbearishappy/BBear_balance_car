#ifndef BSP_H
#define BSP_H
#include<stdint.h>
#include<stdio.h>
#include"stm32f10x.h"
#include"printf.h"
#include"mpu6050.h"
#include"i2c.h"
#include"math.h"
#include"filter.h"
#include"pid.h"
#include"pwm.h"
#include"pid.h"

#define	FALSE	0
#define	TRUE	1

#define F_B  1050
#define L_R  135
#define STOP_BT_ANGLE	5

//#define gyro_y_offset	4.37040//4.37043
#define	acc_x_offset	-56.8//-156.8//-83.4
#define	acc_z_offset	-389.2
#define gyro_y_offset	-20//20//-56.36

#define REMOTE_ID 		0 

#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))
#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)
#define RDATA PAin(3)	  

extern float radian_filted;									//the angle which is filted
extern int16_t buf[3];										//the array to store the mpu6050 data
extern uint32_t distance;							   		//the car's total run distance
extern int16_t acc_x, acc_z;								//acceleration on the x,z axis
extern int16_t gyro_y ;										//angular speed ont the y axis
extern float radian_pt, radian_temp1, radian_temp2;			//radian_pt:the angle which is accumulated; radian_temp1:temporary of angle calculate
															//radian_temp2:temporary of angle calculate
extern float radian;										//the angle which is calculated by the acceleration
extern int16_t *p;											//a pointer point to the array which store the mpu6050 data
extern int16_t leftspeed,rightspeed;						//the car's left wheel and right wheel	
extern pid_s Angle_PID;										//struct to store the PID data
extern pid_s Speed_PID;

extern unsigned char control_data;
extern uint8_t heart_flag;
extern int32_t run_l,run_r;

int Systeminit(void);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void USART_Configuration(void);
void NVIC_Configuration(void);
void EXTI_Configuration(void);
uint8_t Remote_Scan(void);

#endif
