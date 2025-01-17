#include "module1_RF433.h"

extern IR_RF_Signal_t IR_RF_Signal;

void Module1_RF433_Init(void)
{
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct;
        TIM_TimeBaseInitTypeDef TIM_TimerBaseInitStruct;
        EXTI_InitTypeDef EXTI_InitStruct;
        NVIC_InitTypeDef NVIC_InitStruct;
//RF receriver init*****************************************//
        
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA,&GPIO_InitStruct);
        
        GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource7); 

        EXTI_InitStruct.EXTI_Line = EXTI_Line7;
        EXTI_InitStruct.EXTI_LineCmd = ENABLE;
        EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
        EXTI_Init(&EXTI_InitStruct);
    
        NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
        NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
        NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 6;
        NVIC_InitStruct.NVIC_IRQChannelSubPriority = 6;
        NVIC_Init(&NVIC_InitStruct);     
	
//RF transmitter init*****************************************//
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA,&GPIO_InitStruct); 
	
//TIM2*************************************************************//
	TIM_TimerBaseInitStruct.TIM_Prescaler = TIM2_PRESCALE - 1;
	TIM_TimerBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimerBaseInitStruct.TIM_Period = TIM2_RECORD_OVERTIME_US / US_PER_TIMER2_COUNT - 1;
	TIM_TimerBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM2,&TIM_TimerBaseInitStruct);
        
        TIM_ClearFlag(TIM2,TIM_FLAG_Update);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 7;
	NVIC_Init(&NVIC_InitStruct);        
        
        TIM_Cmd(TIM2,DISABLE);     
}

void Module1_RF433_Logic(uint8_t logic)
{
	if(logic == 0){
		GPIO_WriteBit(GPIOA,GPIO_Pin_6,Bit_RESET);
	}
	else{
		GPIO_WriteBit(GPIOA,GPIO_Pin_6,Bit_SET);
	}
}

void Module1_RF433_Transmit(void)
{
	volatile uint16_t i = 0;
        uint8_t logic = 1;
        for(i = 0; i < IR_RF_Signal.length ;i++)
        {
                Module1_RF433_Logic(logic);
		Delay_us(IR_RF_Signal.duration[i] * US_PER_TIMER2_COUNT);
		logic = !logic;
		
        }
}

