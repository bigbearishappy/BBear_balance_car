#include"pwm.h"

/************************************************************************************************ 
Name：PWM_left_Configuration 
Function:	
		  	configuration the motor's pwm
Parameters：
		   	void
Returns：
			void 
Description:
			null
************************************************************************************************/
void PWM_Motor_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure; 
	  
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//3
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 900; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 80K
	TIM_TimeBaseStructure.TIM_Prescaler =0; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位//3
	
	/* Output Compare Active Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高

	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx//3
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIMx在CCR3上的预装载寄存器	  //3
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM4, ENABLE); //使能TIMx在ARR上的预装载寄存器	  //3
	
	/* TIM4 enable counter */
	TIM_Cmd(TIM4, ENABLE);  //使能TIMx外设//3
}

/************************************************************************************************ 
Name：Cal_angle 
Function:	
		  	TIM2 interrupt to calculate the angle of the car
Parameters：
		   	void
Returns：
			void 
Description:
			null
************************************************************************************************/
void Cal_angle(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  TIM_DeInit(TIM2);//复位TIM2定时器

  /* TIM2 configuration */
  TIM_TimeBaseStructure.TIM_Period = 100 - 1; //最大计数值0xffff  0.05s      
  TIM_TimeBaseStructure.TIM_Prescaler = 36000 - 1;//分频0x36       
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0; // 时钟分割  
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //计数方向向上计数
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
 
  /* Clear TIM2 update pending flag[清除TIM2溢出中断标志] */
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);

  /* Enable TIM2 Update interrupt [TIM2溢出中断允许]*/
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);  

  /* TIM2 enable counter [允许tim2计数]*/
  TIM_Cmd(TIM2, ENABLE);
}

/************************************************************************************************ 
Name：Heart_TIM 
Function:	
		  	configuration the heart timer(TIM3 interrupt)
Parameters：
		   	void
Returns：
			void 
Description:
			null
************************************************************************************************/
void Heart_TIM()
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStruct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//open the clock of tim3

	TIM_DeInit(TIM3);
	TIM_TimeBaseStructure.TIM_Period = 100 - 1; //0.5s(1000)  0.1s(200)
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

/************************************************************************************************ 
Name：PWM_Control 
Function:	
		  	pwm control of motor
Parameters：
		   	[in]	-	motorL:pwm value of left motor
						motorR:pwm value of right motor
Returns：
			void 
Description:
			GPIO defination:
			PB6			left motor forward
			PB7			left motor back
			PB8			right motor forward
			PB9			right motor back
************************************************************************************************/
void PWM_Control(int32_t motorL, int32_t motorR)
{
	if(motorL >= 900)motorL = 900;
	if(motorL < -900)motorL = -900;

	if(motorL >= 0){TIM_SetCompare1(TIM4,motorL);TIM_SetCompare2(TIM4,0);}
	if(motorL < 0){TIM_SetCompare1(TIM4,0);TIM_SetCompare2(TIM4,-motorL);}

	if(motorR >= 0){TIM_SetCompare3(TIM4,motorR);TIM_SetCompare4(TIM4,0);}
	if(motorR < 0){TIM_SetCompare3(TIM4,0);TIM_SetCompare4(TIM4,-motorR);}

	//TIM_SetCompare1(TIM4,motorL);
}
