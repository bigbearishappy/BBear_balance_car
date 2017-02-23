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
	//RCC_Configuration();				//initialize the system clock
	USART_Configuration();				//initialize the usart
	GPIO_Configuration();
	NVIC_Configuration();
	EXTI_Configuration();
	PID_Init(&Angle_PID, 150, 0,-100); //20170219 150,0,-100
	PID_Init(&Speed_PID, 50, 0.5, 0);  //20170219 50,0.5,0
	PWM_Motor_Configuration();
	InitMPU6050();
	Cal_angle();
	Heart_TIM();

	while(1)
	{
	}
}
