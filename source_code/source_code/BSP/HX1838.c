//refer to the 红外遥控解码原理 in wenku.baidu.com
#include "HX1838.h"

/*
INSUFFICIENT:
1 don't define the GPIO
2 don't open the relative clock
3 don't open the TIM4
4 don't enable the interrupt of the 4 channel
*/

unsigned int temp,timeL2H,timeL2H2;
unsigned int remote_data = 0x00000000,data = 0x00000000;
unsigned char RASE_FALL = 0x01,flag = 0x00,data_cnt = 0x00;
unsigned char repeat_flag = 0x0000;

void HX1838_Init()
{
	GPIO_InitTypeDef GPIO_InitTypeStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_ICInitTypeDef TIM_ICInitTypeStruct;
	NVIC_InitTypeDef NVIC_InitTypeStruct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitTypeStruct.GPIO_Pin = GPIO_Pin_9;		
	GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_IPD; 		
 	GPIO_InitTypeStruct.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitTypeStruct);
 	GPIO_SetBits(GPIOB,GPIO_Pin_9);	

	TIM_TimeBaseInitStruct.TIM_Period = 1 - 1;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 72 - 1;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, & TIM_TimeBaseInitStruct);

	TIM_ICInitTypeStruct.TIM_ICMode = TIM_ICMode_ICAP;
	TIM_ICInitTypeStruct.TIM_Channel = TIM_Channel_4;
	TIM_ICInitTypeStruct.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM_ICInitTypeStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitTypeStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInit(TIM4, &TIM_ICInitTypeStruct);

	TIM_Cmd(TIM4, ENABLE);

	NVIC_InitTypeStruct.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitTypeStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitTypeStruct.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitTypeStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure(&NVIC_InitTypeStruct);

	TIM_ITConfig( TIM4,TIM_IT_Update|TIM_IT_CC4,ENABLE);
}

void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET)
	{
		temp = TIM_GetCapture4(TIM4);
		//Rasing 1;Falling 0;
		if(RASE_FALL){
			timeL2H = TIM_GetCapture4(TIM4);
			RASE_FALL = 0;
		}
		//check the repeat
		if((temp - timeL2H == 11250 || 0xffffffff - temp + timeL2H == 11250) && flag == 0){
			repeat_flag = 1;
		}	
		//check the start
		if((temp - timeL2H == 13500 || 0xffffffff - temp + timeL2H == 13500) && flag == 0){
			flag = 1;
			timeL2H = temp;
		}		
		//decode the data
		if(flag && data_cnt < 32){
			timeL2H2 = TIM_GetCounter(TIM4);
			if(timeL2H2 - timeL2H == 1125){
				data = data << 1;
				timeL2H = timeL2H2;
				data_cnt++;
			}
			if(timeL2H2 - timeL2H == 2250){
				data = data << 1 | 1;
				timeL2H = timeL2H2;
				data_cnt++;
			}
		}
		if(data_cnt == 32 || repeat_flag){
			remote_data = data;
			repeat_flag = 0;
			data_cnt    = 0;
			flag        = 0;
		}
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);
	}
}
