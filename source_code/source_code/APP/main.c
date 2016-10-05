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
void SystemInit(){}//rewrite here
int main()
{
	RCC_Configuration();				//initialize the system clock
	USART_Configuration();				//initialize the usart
	GPIO_Configuration();
	NVIC_Configuration();
	EXTI_Configuration();
	PID_Init(&sPID, 300, 0, 0);
	PWM_Motor_Configuration();
	Cal_angle();
	Heart_TIM();
	InitMPU6050();

	while(1)
	{

	}
}
