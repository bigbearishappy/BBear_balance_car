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
	PID_Init(&Angle_PID, 150, 0,-100); //300 0 -200 //210,0,-140	 //20170216 180 0 -120	 //20170218 120 0 -80
	PID_Init(&Speed_PID, 50, 0.5, 0); //10,70,0//300 0 -200		 //20170216 45	1 0	 //20170218 17 35 -45
	PWM_Motor_Configuration();
	InitMPU6050();
	Cal_angle();
	Heart_TIM();

	while(1)
	{
	}
}
