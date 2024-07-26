#include "config.h"
#include "stm32f10x.h"

//********************************************************************************
#ifndef	_HARDWARE_H_
#define	_HARDWARE_H_

#define IMU_STOP() EXTI->IMR &= ~(EXTI_Line5)
#define IMU_START() EXTI->IMR |= EXTI_Line5

#define TIMER_FOR_BUTTON_ENABLE TIM_Cmd(TIM4,ENABLE)
#define TIMER_FOR_BUTTON_DISABLE TIM_Cmd(TIM4,DISABLE)

#define IR_PWM_ENABLE TIM_Cmd(TIM3,ENABLE)
#define IR_PWM_DISABLE TIM_Cmd(TIM3,DISABLE)

#define LED_ON GPIO_WriteBit(GPIOA,GPIO_Pin_7,Bit_SET)
#define LED_OFF GPIO_WriteBit(GPIOA,GPIO_Pin_7,Bit_RESET)

void Hardware_Init(void);
void IR_38Khz_Logic(uint8_t logic);
void FR_433Mhz_Logic(uint8_t logic);
void EXTI_Stop(void);
void EXTI_Restore(void);
void Signal_Copy_From_Buffer(void);
void Signal_Copy_To_Buffer(void);

#endif	//_HARDWARE_H_

