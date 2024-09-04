#include "module0_IR.h"

extern IR_RF_Signal_t IR_RF_Signal;

void Module0_IR_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM2,ENABLE);
        
        GPIO_InitTypeDef GPIO_InitStruct;
        TIM_TimeBaseInitTypeDef TIM_TimerBaseInitStruct;
        TIM_OCInitTypeDef TIM_OCInitStruct;
        EXTI_InitTypeDef EXTI_InitStruct;
        NVIC_InitTypeDef NVIC_InitStruct;
	
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
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA,&GPIO_InitStruct);
        
        GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource7); 
	
        EXTI_InitStruct.EXTI_Line = EXTI_Line7 ;
        EXTI_InitStruct.EXTI_LineCmd = ENABLE;
        EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
        EXTI_Init(&EXTI_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
        NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
        NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 6;
        NVIC_InitStruct.NVIC_IRQChannelSubPriority = 6;
        NVIC_Init(&NVIC_InitStruct);
	
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

void Module0_IR_38Khz_Logic(uint8_t logic)
{
	if(logic == 0){
		TIM_SetCompare1(TIM3,0);
	}
	else{
		TIM_SetCompare1(TIM3,422);
	}
}


/**
  * @brief  Enable hardware which control the infrated LED
  * @param  IR_data:  An array which contain the infrated signal in duration form. 
           e.g. In NEC protocol a message is started by a 9ms AGC burst and followed by a 4.5ms space.
                In this case IR_data[0] * US_PER_TIMER2_COUNT = 9.5ms
                             IR_data[1] * US_PER_TIMER2_COUNT = 4.5ms
  * @param  Length: length of the 
  * @retval None
  */
void Module0_IR_Transmit(void)
{
	volatile uint16_t i = 0;
        TIM_SetCounter(TIM3,0);
        IR_PWM_ENABLE;
        uint8_t logic = 1;
        for(i = 0; i < IR_RF_Signal.length ;i++)
        {
		Module0_IR_38Khz_Logic(logic);
		Delay_us(IR_RF_Signal.duration[i] * US_PER_TIMER2_COUNT);
		logic = !logic;
                
        }
        IR_PWM_DISABLE;
}


