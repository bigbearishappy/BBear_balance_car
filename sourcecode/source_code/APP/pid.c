#include"pid.h"

/************************************************************************************************ 
Name：PID_Init 
Function:	
		  	initialize the PID value
Parameters：
		   	[in]	-	pid:pid structure
						Kp:P
						Ki:I
						Kd:D
Returns：
			void 
Description:
			null
************************************************************************************************/
void PID_Init(pid_t pid, float Kp, float Ki, float Kd)
{
	pid->target = 0;
	pid->integral = 0;
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
}

/************************************************************************************************ 
Name：PID_Cal_Ang 
Function:	
		  	calculate the angle PID result value
Parameters：
		   	[in]	-	*p:the pointer point to the angle PID structure
						current:current stage(angle)
						differential:the angle which is calculated every time
Returns：
			[out]	-	int32_t:the PID value output 
Description:
			null
************************************************************************************************/
int32_t PID_Cal_Ang(pid_s *p, float current, float differential)
{
	float offset;

	offset = p->target - current;
	p->integral += offset;

	return (int32_t)(p->Kp*offset + p->Ki*p->integral - p->Kd*differential);
}

/************************************************************************************************ 
Name：PID_Cal_Speed 
Function:	
		  	calculate the speed PID result value
Parameters：
		   	[in]	-	*p:the pointer point to the speed PID structure
						current:current stage(speed)
Returns：
			[out]	-	int32_t:the PID value output 
Description:
			null
************************************************************************************************/
int32_t spd_length = 0;
float spd_v = 0;	 //滤波后的速度
int32_t PID_Cal_Speed(pid_s *p, int32_t current,int32_t target)
{
	float temp;
//	if(target > 0)
//		current = current + 0;//target speed is 0
//	else
		current = current + 10;//target speed is 0
	temp = (float)(current) * 0.2;
	spd_v = spd_v * 0.8 + (float)(current) * 0.2;
	spd_length += (int32_t)temp;
	if(spd_length > 20)
		spd_length = 20;
	if(spd_length < -20)
		spd_length = -20;	
	return (int32_t)(p->Kp * spd_v + p->Ki * spd_length);
}
