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

#define gyro_y_offset	4.37040+0.039//4.37043
#define	acc_x_offset	-143//9079//19394
#define	acc_z_offset	8232//10909//29929

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
extern pid_s sPID;											//struct to store the PID data

void RCC_Configuration(void);
void GPIO_Configuration(void);
void USART_Configuration(void);
void NVIC_Configuration(void);
void EXTI_Configuration(void);

#endif
