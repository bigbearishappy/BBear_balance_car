/*************************************HEAD FILES******************************/
#include"BSP.H"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#include "queue.h"

void LED1_FUN(void *pvParameters)
{
	for(;;){
		vTaskDelay(100);
		printf("task one!\n");
	}
}

void LED2_FUN(void *pvParameters)
{
	for(;;){
		vTaskDelay(200);
		printf("task two!\n");
	}
}

void vCallbackFunctionExample( TimerHandle_t xTimer )
{
	Tim3_IRQHandler();
}


void DEMO_Task(void *pvParameters)
{
	TimerHandle_t ret = 0;
	ret = xTimerCreate("MPU6050TMR", pdMS_TO_TICKS(20), pdTRUE, NULL, vCallbackFunctionExample);
	if(ret == NULL){
		printf("timer creat error ret = %d\n", ret);
	}
	xTimerStart(ret, pdMS_TO_TICKS(0));
	for(;;){

	}
}

/******************************************************************************
Name£ºmain 
Function:	
		  	main
Parameters£º
		   	void
Returns£º
			void 
Description:
			null
******************************************************************************/
void SystemInit(){}
int main()
{	
#if 0
	RCC_Configuration();				//initialize the system clock
	USART_Configuration();				//initialize the usart
	GPIO_Configuration();
	NVIC_Configuration();
	EXTI_Configuration();
	PID_Init(&Angle_PID, 150, 0, -100); //20170219 150,0,-100
	PID_Init(&Speed_PID, 360, 37.5, 0);   //20170219 50,0.5,0   //20171019 100,25,0 //20171024 180,37.5,0
	PWM_Motor_Configuration();
	InitMPU6050();
	Cal_angle();
	Heart_TIM();
	while(1)
	{
	}
#else
	int ret = 0;
	ret = Systeminit();
	if(ret < 0)
		printf("system init error ret =%d\n", ret);

	//xTaskCreate( LED1_FUN, "LED1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
	//xTaskCreate( LED2_FUN, "LED2", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( DEMO_Task, "DEMO", 1024, NULL, tskIDLE_PRIORITY, NULL);
	
	vTaskStartScheduler();
	return 0;
#endif
}
