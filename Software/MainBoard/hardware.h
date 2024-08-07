#include "config.h"
#include "stm32f10x.h"

//********************************************************************************
#ifndef	_HARDWARE_H_
#define	_HARDWARE_H_

#define IMU_STOP() EXTI->IMR &= ~(EXTI_Line5)
#define IMU_START() EXTI->IMR |= EXTI_Line5

#define TIMER_FOR_BUTTON_ENABLE TIM_Cmd(TIM_FOR_BUTTON,ENABLE)
#define TIMER_FOR_BUTTON_DISABLE TIM_Cmd(TIM_FOR_BUTTON,DISABLE)

#define LED_ON GPIO_WriteBit(LED_GPIO,LED_GPIO_PIN,Bit_SET)
#define LED_OFF GPIO_WriteBit(LED_GPIO,LED_GPIO_PIN,Bit_RESET)

void Hardware_Init(void);
void EXTI_Stop(void);
void EXTI_Restore(void);
uint16_t ADC_GetValue(void);

#endif	//_HARDWARE_H_

