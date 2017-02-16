/**********************************************HEAD FILES***************************************/
#include"BSP.H"

/************************************************************************************************ 
Name£ºmain 
Function:	
		  	main
Parameters£º
		   	void
Returns£º
			void 
Description:
			null
************************************************************************************************/
void SystemInit(){}
int main()
{
	RCC_Configuration();				//initialize the system clock
	USART_Configuration();				//initialize the usart
	GPIO_Configuration();
	NVIC_Configuration();
	EXTI_Configuration();
	PID_Init(&Angle_PID, 120, 0,-80); //300 0 -200 //210,0,-140	 //20170216 180 0 -120
	PID_Init(&Speed_PID, 35, 1, 0); //10,70,0//300 0 -200		 //20170216 45	1 0
	PWM_Motor_Configuration();
	InitMPU6050();
	Cal_angle();
	Heart_TIM();

	while(1)
	{
	}
}
