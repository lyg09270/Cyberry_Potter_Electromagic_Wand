/**
 * File Name: ADC.c
 * 
 * Copyright © 2023 Civic_Crab. All rights reserved.
 * 
 * Author: Civic_Crab
 * Creation Date: Oct 1, 2020
 * Version: 1.0.0
 * 
 * Description: This file prodides ADC functions implementation
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"                  // Device header
#include "ADC.h"
#include <math.h>
#include <stdio.h>

int32_t ADC_Sample_Avg = 0;

void ADC_PB1_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_ADC1,ENABLE);
        
        GPIO_InitTypeDef GPIO_InitStruct;
	ADC_InitTypeDef ADC_InitStruct;
	
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
}

void ADC1_Deinit(void)
{
	ADC_DeInit(ADC1);
}

static uint16_t ADC_PB1_GetValue(void)
{
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	return ADC_GetConversionValue(ADC1);
}

/// @brief Get sample average from ADC,if variance is too high
/// @param  None
/// @return -1: error variance is too high.Means that No module is installed on device.
/// @return 0: get average complete.
int8_t ADC_PB1_GetAvg(void)
{
	uint8_t i = 0 ;
	uint8_t samples_count = 0;
	int32_t sample[ADC_MAX_SAMPLES] = {0};
	
	float var = 0;
	while(var == 0 || var >= 10){
		ADC_Sample_Avg = 0;
		var = 0;
		if(samples_count >= ADC_MAX_TESTS)
		{
			return -1;
			break;
		}
		
		for(i = 0; i < ADC_MAX_SAMPLES; i++)
		{
			sample[i] = (int32_t)ADC_PB1_GetValue() ;
			#ifdef SERIAL_DEBUG
			printf("sample[%d]:%d\n",i,sample[i]);
			#endif
		}
		for(i = 0; i < ADC_MAX_SAMPLES; i++)
		{
			ADC_Sample_Avg += sample[i];
		}
		ADC_Sample_Avg = round((ADC_Sample_Avg / ADC_MAX_SAMPLES));
		
		for(i = 0; i < ADC_MAX_SAMPLES; i++)
		{
			var += (float)(ADC_Sample_Avg -sample[i]) * (float)(ADC_Sample_Avg - sample[i]);
		}
		var = (var / ADC_MAX_SAMPLES);
		#ifdef SERIAL_DEBUG
			printf("Var:%f\n",var);
			printf("Avg:%d\n",ADC_Sample_Avg);
		#endif
		samples_count++;	
	}
	#ifdef SERIAL_DEBUG
	printf("\nGet:%d %f\n",ADC_Sample_Avg,var);
	#endif
	return 0;
}
