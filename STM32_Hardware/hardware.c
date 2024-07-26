#include "hardware.h"

#ifdef IMU_IS_MPU6050 
	#include "MPU6050_Reg.h"
	#include "MPU6050.h"
#endif //IMU_IS_MPU6050

void IMU_WriteReg(uint8_t RegAddress, uint8_t Data);

void Hardware_Init()
{
//RCC periphral init****************************************************//   
        
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA |
	RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1,ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM2 |
	RCC_APB1Periph_I2C1 | RCC_APB1Periph_SPI2,ENABLE);
        
        GPIO_InitTypeDef GPIO_InitStruct;
        TIM_TimeBaseInitTypeDef TIM_TimerBaseInitStruct;
        TIM_OCInitTypeDef TIM_OCInitStruct;
        EXTI_InitTypeDef EXTI_InitStruct;
        NVIC_InitTypeDef NVIC_InitStruct;
	
//Button Init****************************************************//    
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA,&GPIO_InitStruct);
        
        EXTI_InitStruct.EXTI_Line = EXTI_Line0;
        EXTI_InitStruct.EXTI_LineCmd = ENABLE;
        EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
        EXTI_Init(&EXTI_InitStruct);
        
        NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
        NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
        NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
        NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
        NVIC_Init(&NVIC_InitStruct);
//TIM4 Init******************************************************//          
        TIM_TimerBaseInitStruct.TIM_Prescaler = 3600 - 1;
	TIM_TimerBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimerBaseInitStruct.TIM_Period = 20 - 1;                   //1ms per interrupt
	TIM_TimerBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM4,&TIM_TimerBaseInitStruct);
        
        TIM_ClearFlag(TIM4,TIM_FLAG_Update);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStruct.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 4;
	NVIC_Init(&NVIC_InitStruct);   
        
//USART init*****************************************//
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = BAUD_RATE;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1,&USART_InitStructure);
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStruct);
	
	USART_Cmd(USART1,ENABLE);
        printf("Hello!\n");
//IR transmitter init*****************************************//
        /*Infrated LED connect to GPIO A6*/
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA,&GPIO_InitStruct);
	
        /*38Khz Carrier wave of infrated singal is generate by PWM*/
	TIM_TimerBaseInitStruct.TIM_Prescaler = 3 - 1;
	TIM_TimerBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimerBaseInitStruct.TIM_Period = 633 - 1;
	TIM_TimerBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM3,&TIM_TimerBaseInitStruct);
        
        /*PWM Output*/
	TIM_OCStructInit(&TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 0;
	TIM_OC1Init(TIM3,&TIM_OCInitStruct);       
        
//IR receriver IMU EXTI init*****************************************//
        
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB,&GPIO_InitStruct);
        
        GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource8); 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource5); 
	
        EXTI_InitStruct.EXTI_Line = EXTI_Line8 ;
        EXTI_InitStruct.EXTI_LineCmd = ENABLE;
        EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
        EXTI_Init(&EXTI_InitStruct);
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line5 ;
        EXTI_InitStruct.EXTI_LineCmd = ENABLE;
        EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
        EXTI_Init(&EXTI_InitStruct);

	IMU_STOP();
        NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
        NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
        NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 7;
        NVIC_InitStruct.NVIC_IRQChannelSubPriority = 7;
        NVIC_Init(&NVIC_InitStruct);
	
//TIM2 15ms*************************************************************//
	TIM_TimerBaseInitStruct.TIM_Prescaler = TIM2_PRESCALE - 1;
	TIM_TimerBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimerBaseInitStruct.TIM_Period = TIM2_RECORD_OVERTIME_US / US_PER_TIMER2_COUNT - 1;
	TIM_TimerBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM2,&TIM_TimerBaseInitStruct);
        
        TIM_ClearFlag(TIM2,TIM_FLAG_Update);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 7;
	NVIC_Init(&NVIC_InitStruct);        
        
        TIM_Cmd(TIM2,DISABLE);
        
//FR receriver init*****************************************//
        
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA,&GPIO_InitStruct);
        
        GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource4); 

        EXTI_InitStruct.EXTI_Line = EXTI_Line4;
        EXTI_InitStruct.EXTI_LineCmd = ENABLE;
        EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
        EXTI_Init(&EXTI_InitStruct);
    
        NVIC_InitStruct.NVIC_IRQChannel = EXTI4_IRQn;
        NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
        NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 7;
        NVIC_InitStruct.NVIC_IRQChannelSubPriority = 7;
        NVIC_Init(&NVIC_InitStruct);     
	
//FR transmitter init*****************************************//
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA,&GPIO_InitStruct); 
	
//LED init*****************************************//      
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA,&GPIO_InitStruct); 
//SPI2 init*****************************************// 	

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);					
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);					
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);					
	
	SPI_InitTypeDef SPI_InitStructure;					
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;				
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;			
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;			
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;	
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;				
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;				
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;				
	SPI_InitStructure.SPI_CRCPolynomial = 7;				
	SPI_Init(SPI2, &SPI_InitStructure);					
	
	
	GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);
	SPI_Cmd(SPI2, ENABLE);					
	
 //IMU init*****************************************//    	
	
	#ifdef IMU_IS_MPU6050
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource5); 
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line5 ;
        EXTI_InitStruct.EXTI_LineCmd = ENABLE;
        EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
        EXTI_Init(&EXTI_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
        NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
        NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 6;
        NVIC_InitStruct.NVIC_IRQChannelSubPriority = 6;
        NVIC_Init(&NVIC_InitStruct);
	
	IMU_STOP();
	MPU6050_Init();
	#endif //IMU_IS_MPU6050	
}

int fputc( int ch, FILE *f )
{
        while((USART1->SR & (1 << 7) ) == 0);
        USART1->DR = ch;
	return ch;
}

void IR_38Khz_Logic(uint8_t logic)
{
	if(logic == 0){
		TIM_SetCompare1(TIM3,0);
	}
	else{
		TIM_SetCompare1(TIM3,422);
	}
}

void FR_433Mhz_Logic(uint8_t logic)
{
	if(logic == 0){
		GPIO_WriteBit(GPIOA,GPIO_Pin_5,Bit_RESET);
	}
	else{
		GPIO_WriteBit(GPIOA,GPIO_Pin_5,Bit_SET);
	}
}

void EXTI_Stop(void)
{
	EXTI->IMR &= ~(EXTI_Line0);
	EXTI->IMR &= ~(EXTI_Line4);
	EXTI->IMR &= ~(EXTI_Line8);
}

void EXTI_Restore(void)
{
	EXTI->IMR |= EXTI_Line0;
	EXTI->IMR |= EXTI_Line4;
	EXTI->IMR |= EXTI_Line8;
}
