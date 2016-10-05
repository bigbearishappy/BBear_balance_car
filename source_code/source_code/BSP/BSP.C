#include"BSP.H"

/****************************************public values*********************************************************************/
float radian_filted = 0;									//the angle which is filted
int16_t buf[3] = {0, 0, 0};									//the array to store the mpu6050 data
uint32_t distance = 0;							   			//the car's total run distance
int16_t acc_x, acc_z;										//acceleration on the x,z axis
int16_t gyro_y = 0;											//angular speed ont the y axis
float radian_pt = 0, radian_temp1 = 0, radian_temp2 = 0;	//radian_pt:the angle which is accumulated; radian_temp1:temporary of angle calculate
															//radian_temp2:temporary of angle calculate
float radian = 0;											//the angle which is calculated by the acceleration
int16_t *p;													//a pointer point to the array which store the mpu6050 data
int16_t leftspeed = 0,rightspeed = 0;						//the car's left wheel and right wheel	
pid_s sPID;													//struct to store the PID data

uint8_t flag_l = 1, flag_r = 1;
uint8_t heart_flag = 0;
int balan_pwm;
/************************************************************************************************ 
Name：RCC_Configration 
Function:	
		  	initialize the system clock
Parameters：
		   	void
Returns：
			void 
Description:
			null
************************************************************************************************/
void RCC_Configuration()
{
	ErrorStatus HSEStartUpStatus;
	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	if(HSEStartUpStatus == SUCCESS){
	    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable); //开启FLASH预读缓冲功能，加速FLASH的读取。所有程序中必须的用法.位置：RCC初始化子函数里面，时钟起振之后
    	FLASH_SetLatency(FLASH_Latency_2);                    //flash操作的延时

    	RCC_HCLKConfig(RCC_SYSCLK_Div1);  
    	RCC_PCLK2Config(RCC_HCLK_Div1);		   
		RCC_PCLK1Config(RCC_HCLK_Div2);	 	  
    	RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);	
    	RCC_PLLCmd(ENABLE); 
    	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)	  
       	{
       	}
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while(RCC_GetSYSCLKSource() != 0x08)	  
       	{ 
       	}
	}
}

/************************************************************************************************ 
Name：GPIO_Configuration 
Function:	
		  	initialize the GPIO
Parameters：
		   	void
Returns：
			void 
Description:
			null
************************************************************************************************/
void GPIO_Configuration()
{
/*****************************************I2C GPIO**********************************************/
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;						//SCL
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;						//SDA
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	  

/***********************************PWM GPIO*******************************************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;		 				//pwm-left-forward
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;		 				//pwm-left-back
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;		 				//pwm-right-forward
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		 				//pwm-right-back
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
/***********************************SPEED**********************************************************/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;					 		//left speed
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//GPIO_SetBits(GPIOB, GPIO_Pin_0);
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;							//right speed
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

/*************************************HEART LED***************************************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOC, &GPIO_InitStructure); 
	GPIO_WriteBit(GPIOC,GPIO_Pin_8,Bit_RESET);

}

/************************************************************************************************ 
Name：USART_Configuration 
Function:	
		  	initialize the usart
Parameters：
		   	void
Returns：
			void 
Description:
			null
************************************************************************************************/
void USART_Configuration()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_9;		//TX
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin  	= GPIO_Pin_10;		//RX	
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;		                           // A10 USART1_Rx  
    GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN_FLOATING;                           //浮空输入-RX
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate   = 115200;	                                //波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	                    //8位数据
	USART_InitStructure.USART_StopBits   = USART_StopBits_1;		                //1个停止位
	USART_InitStructure.USART_Parity     = USART_Parity_No;				            //无奇偶校检
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制禁止
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                //发送接收使能

	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);	
    USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);	                                                    // Enable the USARTx 
}

/************************************************************************************************ 
Name：NVIC_Configuration 
Function:	
		  	configuration the NVIC
Parameters：
		   	void
Returns：
			void 
Description:
			null
************************************************************************************************/
void NVIC_Configuration(void)
{
   NVIC_InitTypeDef NVIC_InitStructure;
  
   /* Set the Vector Table base location at 0x08000000 */
   //NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
  
   /* Configure the NVIC Preemption Priority Bits */  
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  
   /* Enable the USART1 Interrupt */
   //NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;       //通道设置为串口1中断
   //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//used to be 0	   //中断响应优先级0
   //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		   //打开中断
   //NVIC_Init(&NVIC_InitStructure); 						   //初始化
  
   NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//占优先级 
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //副优先级 
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
   NVIC_Init(&NVIC_InitStructure); 

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;		//指定中断源
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;        //指定响应优先级别2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	        //使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);
}

/************************************************************************************************ 
Name：EXTI_Configuration 
Function:	
		  	PA6,PA7 interrupt configuration
Parameters：
		   	void
Returns：
			void 
Description:
			the interrupt configuration of PA6,PA7
************************************************************************************************/
void EXTI_Configuration(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 

  EXTI_ClearITPendingBit(EXTI_Line10);//(EXTI_Line10);

  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_Line = EXTI_Line10;//EXTI_Line10;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10);

  EXTI_ClearITPendingBit(EXTI_Line11);

  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_Line = EXTI_Line11;//EXTI_Line10;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource11);//
}

/************************************************************************************************ 
Name：EXTI15_10_IRQHandler 
Function:	
		  	PB10,PB11 interrupt function
Parameters：
		   	void
Returns：
			void 
Description:
			the interrupt function of PB10,PB11
************************************************************************************************/
void EXTI15_10_IRQHandler(void)
{
  
  	if(EXTI_GetITStatus(EXTI_Line10) != RESET)  //
    {
     //添加中断处理程序
	 leftspeed++;
	 /*if(leftspeed%2)	 
	 	GPIO_ResetBits(GPIOB , GPIO_Pin_0);
	 else
		GPIO_SetBits(GPIOB , GPIO_Pin_0);*/	 
	 //printf("left = %d\r\n",left);
	 EXTI_ClearFlag(EXTI_Line10);			       //清除中断标志（必须）
     EXTI_ClearITPendingBit(EXTI_Line10);//	 
     }
	 if(EXTI_GetITStatus(EXTI_Line11) != RESET)
	 {
	 rightspeed++;
	 if(rightspeed%2)	 
	 	GPIO_ResetBits(GPIOB , GPIO_Pin_0);
	 else
		GPIO_SetBits(GPIOB , GPIO_Pin_0);
	 //printf("right = %d\r\n",right);
	 EXTI_ClearFlag(EXTI_Line11);
	 EXTI_ClearITPendingBit(EXTI_Line11);
	 }
	
}

/************************************************************************************************ 
Name：USART1_IRQHandler 
Function:	
		  	USART1 interrupt function
Parameters：
		   	void
Returns：
			void 
Description:
			receive cmd from the bluetooth module
************************************************************************************************/
void USART1_IRQHandler(void)            
{
  u8 dat;
   
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)    //若接收数据寄存器满
  {     
    dat = USART_ReceiveData(USART1);  
         
     if(dat == 0x63) //串口调试助手中发送数据时注意是十六进制发送，显示也要用十六进制显示，否则数据就会有问题                   
	 {  			
	  dat = 0;
	  printf("hello\r\n");                         			  
	 }                                          			   
  }
} 

/************************************************************************************************ 
Name：TIM3_IRQHandler 
Function:	
		  	TIM3 interrupt function
Parameters：
		   	void
Returns：
			void 
Description:
			null
************************************************************************************************/
void TIM3_IRQHandler(void)
{
	uint16_t res_l,res_r;
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_FLAG_Update);
		res_l = leftspeed * 2;
		leftspeed = 0;
		//printf("resl = %d\r\n",res_l);

		res_r = rightspeed * 2;
		rightspeed = 0;
		//printf("resr = %d\r\n",res_r);

		printf("filted = %.3lf\r\n",radian_filted);
		balan_pwm = PID_Cal(&sPID, radian_filted, radian_pt);
		//balan_pwm += (speed*6+distance*0.1);
		PWM_Control(balan_pwm, balan_pwm);
		
		if(heart_flag >= 2)
			heart_flag = 0;
		heart_flag++;
		if(heart_flag%2)
			GPIO_SetBits(GPIOC, GPIO_Pin_13);
		else
			GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	}	
}

/************************************************************************************************ 
Name：TIM2_IRQHandler 
Function:	
		  	TIM2 interrupt function to calculate the angle of the car
Parameters：
		   	void
Returns：
			void 
Description:
			null
************************************************************************************************/
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		p = buf;
		READ_MPU6050(p);
		acc_x = p[0];
		acc_z = p[1];
		gyro_y = p[2];
		//printf("gyro = %d\r\n",gyro_y);

		acc_x += acc_x_offset;
		acc_z += acc_z_offset;
		//printf("x:%d,z:%d\r\n",acc_x,acc_z);

		radian = (float)((float)acc_x / 8192);
		radian = asin(radian);
		radian = (radian * 180) / 3.1415;
		//printf("radian = %.3lf\r\n",radian);

		//radian = (float)((float)acc_x / (float)acc_z);				//radian 的单位是弧度
		//radian = atan(radian);
		//printf("radian = %.3lf\r\n",radian);

		radian_temp1 = (((float)gyro_y - 24) / 16.4) * 0.05;	//通过角速度累计计算角度

		if(radian_temp1 - radian_temp2 > 0.5 || radian_temp1 - radian_temp2 < -0.5)//之前小车平衡能力有限跟这里有很大关系
			radian_pt += radian_temp1;
		radian_temp2 = radian_temp1;

		radian_filted = Kaerman_Filter(0, radian, -radian_pt);		// 互补滤波得到小车的倾斜角度

		//radian_pt = -radian_pt;
		//radian_pt_1 = (radian_pt/180.0) * 3.14159265;				//把角度转换成弧度
		//printf("radian_pt_1 = %.2lf\r\n", radian_pt_1);
		//printf("radian_pt = %.3lf\r\n", -radian_pt);
		TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);
	} 
}
