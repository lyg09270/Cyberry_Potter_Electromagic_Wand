#include "config.h"
#include "stm32f10x.h"

//********************************************************************************
#ifndef	_HARDWARE_H_
#define	_HARDWARE_H_


#define LED_ON GPIO_WriteBit(LED_GPIO,LED_GPIO_PIN,Bit_SET)
#define LED_OFF GPIO_WriteBit(LED_GPIO,LED_GPIO_PIN,Bit_RESET)

void Hardware_Init(void);
void EXTI_Stop(void);
void EXTI_Restore(void);
uint16_t ADC_GetValue(void);

#endif	//_HARDWARE_H_

