#include"pid.h"

/************************************************************************************************ 
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
Name£ºPID_Cal 
Function:	
		  	calculate the PID result value
Parameters£º
		   	[in]	-	*p:the pointer point to the PID structure
						current:current stage(angle)
						differential:the angle which is calculated every time
Returns£º
			[out]	-	int32_t:the PID value output 
Description:
			null
************************************************************************************************/
int32_t PID_Cal(pid_s *p, float current, float differential)
{
	float offset;

	offset = p->target - current;
	p->integral += offset;

	return (int32_t)(p->Kp*offset + p->Ki*p->integral - p->Kd*differential);
}
