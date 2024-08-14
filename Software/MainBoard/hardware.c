#include "hardware.h"
#include "Delay.h"

//void IMU_WriteReg(uint8_t RegAddress, uint8_t Data);


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
