/*************************************HEAD FILES******************************/
#include"pwm.h"

char speed_dir = 0;//1:forward 2:back
char speed_dir_l = 0,speed_dir_r = 0; //0:stop 1:forward 2:back

/******************************************************************************
Name£ºPWM_left_Configuration 
Function:	
		  	configuration the motor's pwm
Parameters£º
		   	void
Returns£º
			void 
Description:
			null
******************************************************************************/
void PWM_Motor_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure; 
	  
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//3
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 1000; 
	TIM_TimeBaseStructure.TIM_Prescaler =0; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
	
	/* Output Compare Active Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; 

	TIM_OC1Init(TIM4, &TIM_OCInitStructure); 
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM4, ENABLE); 
	
	/* TIM4 enable counter */
	TIM_Cmd(TIM4, ENABLE);  
}

/******************************************************************************
Name£ºCal_angle 
Function:	
		  	TIM2 interrupt to calculate the angle of the car
Parameters£º
		   	void
Returns£º
			void 
Description:
			null
******************************************************************************/
void Cal_angle(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
		TIM_ICInitTypeDef  TIM_ICInitStructure;

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 	
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_SetBits(GPIOA,GPIO_Pin_3);

		TIM_DeInit(TIM2);

		TIM_TimeBaseStructure.TIM_Period = 10000-1;
		TIM_TimeBaseStructure.TIM_Prescaler = 72-1;
		TIM_TimeBaseStructure.TIM_ClockDivision = 0x0; 
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

		TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;  
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 
		TIM_ICInitStructure.TIM_ICFilter = 0x03;
		TIM_ICInit(TIM2, &TIM_ICInitStructure);

		TIM_ClearFlag(TIM2, TIM_FLAG_Update);

		TIM_ITConfig(TIM2, TIM_IT_Update|TIM_IT_CC4, ENABLE);  

		TIM_Cmd(TIM2, ENABLE);
}

/******************************************************************************
Name£ºHeart_TIM 
Function:	
		  	configuration the heart timer(TIM3 interrupt)
Parameters£º
		   	void
Returns£º
			void 
Description:
			null
******************************************************************************/
void Heart_TIM()
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStruct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_DeInit(TIM3);
	TIM_TimeBaseStructure.TIM_Period = 200 - 1;//20 - 1; //0.5s(1000)  0.1s(200) 0.05s(100)
	TIM_TimeBaseStructure.TIM_Prescaler = 36000-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x00;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
	//TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);   

	//TIM_ICInitStruct.TIM_ICMode = TIM_ICMode_ICAP;
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;  
	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Falling;  
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;  
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;  
	TIM_ICInitStruct.TIM_ICFilter = 0x0;   
	TIM_ICInit(TIM3,&TIM_ICInitStruct);      
	
	TIM_Cmd(TIM3, ENABLE);
}

/******************************************************************************
Name£ºPWM_Control 
Function:	
		  	pwm control of motor
Parameters£º
		   	[in]	-	motorL:pwm value of left motor
						motorR:pwm value of right motor
Returns£º
			void 
Description:
			GPIO defination:
			PB6			left motor forward
			PB7			left motor back
			PB8			right motor forward
			PB9			right motor back
******************************************************************************/
void PWM_Control(int32_t motorL, int32_t motorR)
{
	if(motorL >= 0 && motorR >= 0)
		speed_dir = 1;
	//if(motorL < -250 && motorR < -250)
	if(motorL < -0 && motorR < -0)
		speed_dir = 2;

	if(motorL >= 1000)motorL = 1000;
	if(motorL < -1000)motorL = -1000;
	if(motorR >= 1000)motorR = 1000;
	if(motorR < -1000)motorR = -1000;

	if(motorL >= 0){
		TIM_SetCompare1(TIM4,motorL);
		TIM_SetCompare2(TIM4,0);
	}
	if(motorL < 0){
		TIM_SetCompare1(TIM4,0);
		TIM_SetCompare2(TIM4,-motorL);
	}

	if(motorR >= 0){
		TIM_SetCompare3(TIM4,motorR);
		TIM_SetCompare4(TIM4,0);
	}
	if(motorR < 0){
		TIM_SetCompare3(TIM4,0);
		TIM_SetCompare4(TIM4,-motorR);
	}
}
