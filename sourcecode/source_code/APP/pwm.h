#ifndef PWM_H
#define PWM_H
#include"stm32f10x.h"

extern char speed_dir;
extern char speed_dir_l,speed_dir_r;

void PWM_Motor_Configuration(void);					//TIM4
void Heart_TIM(void);								//TIM3
void Cal_angle(void);								//TIM2
void PWM_Control(int32_t motorL, int32_t motorR);

#endif
