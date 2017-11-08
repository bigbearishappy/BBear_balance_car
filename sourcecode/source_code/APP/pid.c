/*************************************HEAD FILES******************************/
#include"pid.h"

/******************************************************************************
Name£ºPID_Init 
Function:	
		  	initialize the PID value
Parameters£º
		   	[in]	-	pid:pid structure
						Kp:P
						Ki:I
						Kd:D
Returns£º
			void 
Description:
			null
******************************************************************************/
void PID_Init(pid_t pid, float Kp, float Ki, float Kd)
{
	pid->target = 0;
	pid->integral = 0;
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
}

/******************************************************************************
Name£ºPID_Cal_Ang 
Function:	
		  	calculate the angle PID result value
Parameters£º
		   	[in]	-	*p:the pointer point to the angle PID structure
						current:current stage(angle)
						differential:the angle which is calculated every time
Returns£º
			[out]	-	int32_t:the PID value output 
Description:
			null
******************************************************************************/
int32_t PID_Cal_Ang(pid_s *p, float current, float differential, int target)
{
	float offset;
	
	p->target = target;
	
	offset = p->target - current;
	p->integral += offset;

	return (int32_t)(p->Kp*offset + p->Ki*p->integral - p->Kd*differential);
}

/******************************************************************************
Name£ºPID_Cal_Speed 
Function:	
		  	calculate the speed PID result value
Parameters£º
		   	[in]	-	*p:the pointer point to the speed PID structure
						current:current stage(speed)
Returns£º
			[out]	-	int32_t:the PID value output 
Description:
			null
******************************************************************************/
float spd_length = 0.0,spd_length_l = 0.0, spd_length_r = 0.0;
float spd_v = 0;	 //the speed after filter
int32_t last_target = 0;
int32_t PID_Cal_Speed(pid_s *p, int32_t current,int32_t target,unsigned char which_wheel)
{
	int32_t temp;
	//target = 1;
	if(target == 1)current += 2.2;
	else if(target == 2) current -= 2.0;
	temp = (int32_t)(current);
	spd_v = spd_v * 4 / 5 + (float)(current) / 5;

	if(target == 0){
		if(LEFT_WHEEL == which_wheel)
			spd_length_l += (int32_t)temp;
		else if(RIGHT_WHEEL == which_wheel)
			spd_length_r+= (int32_t)temp;
	}
#define TEMP1_F	30	 
#define TEMP1_B 3
	if(LEFT_WHEEL == which_wheel){
		if(target == 1){
			spd_length_l += ADD_VALUE_FB;
			if(spd_length_l > TEMP1_F) spd_length_l = TEMP1_F;
		}else if(target == 2){
			spd_length_l -= ADD_VALUE_FB;
			if(spd_length_l < -TEMP1_B) spd_length_l = -TEMP1_B;
		}else if(target == 3)
			spd_length_l += ADD_VALUE_LR;
		else if(target == 4)
			spd_length_l -= ADD_VALUE_LR;

		 spd_length = spd_length_l;
	}
	else if(RIGHT_WHEEL == which_wheel){
		if(target == 1){
			spd_length_r += ADD_VALUE_FB;
			if(spd_length_r > TEMP1_F) spd_length_r = TEMP1_F;
		}else if(target == 2){
			spd_length_r -= ADD_VALUE_FB;
			if(spd_length_r < -TEMP1_B) spd_length_r = -TEMP1_B;
		}else if(target == 3)
			spd_length_r -= ADD_VALUE_LR;
		else if(target == 4)
			spd_length_r += ADD_VALUE_LR;

		 spd_length = spd_length_r;
	}
		 
#define TEMP 1000
	if(spd_length > TEMP)//1000
		spd_length = TEMP;
	if(spd_length < -TEMP)
		spd_length = -TEMP;

//	printf("target:%d\r\n",target);
//	printf("spd=%.1lf\r\n",spd_v);
//	printf("l=%d\n",spd_length);
//	printf("tem=%.1lf\r\n",temp);
//	printf(",%.1lf,%.1lf\n",spd_length_l,spd_length_r);	
	return (int32_t)(p->Kp * spd_v + p->Ki * (float)spd_length + p->Kd * temp);
}
