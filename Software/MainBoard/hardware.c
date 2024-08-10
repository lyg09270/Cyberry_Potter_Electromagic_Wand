#include "hardware.h"
#include "Delay.h"
#ifdef IMU_IS_MPU6050 
	#include "MPU6050_Reg.h"
	#include "MPU6050.h"
	#include "IIC.h"
#endif //IMU_IS_MPU6050

//void IMU_WriteReg(uint8_t RegAddress, uint8_t Data);

void Hardware_Init()
{
//RCC periphral init****************************************************//   
        
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA |
	RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1 | RCC_APB2Periph_ADC1,ENABLE);
        RCC_APB1PeriphClockCmd(RCC_TIM_FOR_BUTTON | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM2 |
	RCC_APB1Periph_I2C1 | RCC_APB1Periph_SPI2,ENABLE);
        
        GPIO_InitTypeDef GPIO_InitStruct;
        TIM_TimeBaseInitTypeDef TIM_TimerBaseInitStruct;
        TIM_OCInitTypeDef TIM_OCInitStruct;
        EXTI_InitTypeDef EXTI_InitStruct;
        NVIC_InitTypeDef NVIC_InitStruct;
		ADC_InitTypeDef ADC_InitStruct;
	
//ADC Init****************************************************//   
	//ADC worling at GPIO B1
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	ADC_InitStruct.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStruct);	
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_239Cycles5);
	
	ADC_Cmd(ADC1, ENABLE);
	
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1) == SET);
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1) == SET){}
	
//Button Init****************************************************//   
		//Button working at BUTTON_GPIO(reffer to config.h)
        GPIO_InitStruct.GPIO_Pin = BUTTON_GPIO_PIN;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(BUTTON_GPIO,&GPIO_InitStruct);
        
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

//TIM_FOR_BUTTON Init******************************************************//

    TIM_TimerBaseInitStruct.TIM_Prescaler = 3600 - 1;
	TIM_TimerBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimerBaseInitStruct.TIM_Period = 20 - 1;                   //1ms per interrupt
	TIM_TimerBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM_FOR_BUTTON,&TIM_TimerBaseInitStruct);
        
        TIM_ClearFlag(TIM_FOR_BUTTON,TIM_FLAG_Update);
	TIM_ITConfig(TIM_FOR_BUTTON,TIM_IT_Update,ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStruct.NVIC_IRQChannel = TIM_FOR_BUTTON_IRQN;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 4;
	NVIC_Init(&NVIC_InitStruct);   
        
//USART init*****************************************//
	//USART1 working at GPIOA9 and GPIOA10
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
	printf("hello\n");
//LED init*****************************************//      
	//LED working at LED_GPIO(reffer to config.h)
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStruct.GPIO_Pin = LED_GPIO_PIN;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(LED_GPIO,&GPIO_InitStruct); 
	
//SPI2 W25Q16 init*****************************************// 	

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
	printf("MPU Init \n");
	IMU_STOP();
	
	//Delay_s(5);
	uint8_t data = 0;
	LED_ON;
	IIC_read(0x69,MPU6050_RA_WHO_AM_I,1,&data);
	printf("WHOAMI:%d",data);
	IIC_read(0x68,MPU6050_RA_WHO_AM_I,1,&data);
	printf("WHOAMI:%d",data);
	MPU6050_Init();
	printf("MPU Init compelete\n");
	#endif //IMU_IS_MPU6050	
}

int fputc( int ch, FILE *f )
{
        while((USART1->SR & (1 << 7) ) == 0);
        USART1->DR = ch;
	return ch;
}

void EXTI_Stop(void)
{
	EXTI->IMR &= ~(EXTI_Line0);
	EXTI->IMR &= ~(EXTI_Line7);
}

void EXTI_Restore(void)
{
	EXTI->IMR |= EXTI_Line0;
	EXTI->IMR |= EXTI_Line7;
}

uint16_t ADC_GetValue(void)
{
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	return ADC_GetConversionValue(ADC1);
}
