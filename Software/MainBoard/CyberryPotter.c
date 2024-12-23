#include "CyberryPotter.h"

extern struct IMU_t IMU;
extern struct LED_t LED;
extern int32_t ADC_Sample_Avg;
Cyberry_Potter_t Cyberry_Potter;
extern IR_RF_Signal_t IR_RF_Signal;
Module_t Module;
extern uint8_t W25Q64_Buffer[4096];


void Module_None_Mode0_Handler(void)
{
	LED.Operate(BLINK_10HZ);
}

void Module_None_Mode1_Handler(void)
{
	LED.Operate(BLINK_5HZ);
}

/// @brief Detect which module is installed on device
/// @param  None
/// @return eModule_Type: module info
eModule_Type Module_Detect(void)
{
	if (ADC_Sample_Avg >= MODULE10_UPPER_LIM) {
		//ADC_Sample_Avg higher than upper limit.
		return Module_Type_None;
	} 
	else if (ADC_Sample_Avg >= MODULE10_LOWER_LIM && ADC_Sample_Avg < MODULE10_UPPER_LIM) {
		return Module_Type_10;
	} 
	else if (ADC_Sample_Avg >= MODULE9_LOWER_LIM && ADC_Sample_Avg < MODULE9_UPPER_LIM) {
		return Module_Type_9;
	} 
	else if (ADC_Sample_Avg >= MODULE8_LOWER_LIM && ADC_Sample_Avg < MODULE8_UPPER_LIM) {
		return Module_Type_8;
	} 
	else if (ADC_Sample_Avg >= MODULE7_LOWER_LIM && ADC_Sample_Avg < MODULE7_UPPER_LIM) {
		return Module_Type_7;
	} 
	else if (ADC_Sample_Avg >= MODULE6_LOWER_LIM && ADC_Sample_Avg < MODULE6_UPPER_LIM) {
		return Module_Type_6;
	} 
	else if (ADC_Sample_Avg >= MODULE5_LOWER_LIM && ADC_Sample_Avg < MODULE5_UPPER_LIM) {
		return Module_Type_5;
	} 
	else if (ADC_Sample_Avg >= MODULE4_LOWER_LIM && ADC_Sample_Avg < MODULE4_UPPER_LIM) {
		return Module_Type_4;
	} 
	else if (ADC_Sample_Avg >= MODULE3_LOWER_LIM && ADC_Sample_Avg < MODULE3_UPPER_LIM) {
		return Module_Type_3;
	} 
	else if (ADC_Sample_Avg >= MODULE2_LOWER_LIM && ADC_Sample_Avg < MODULE2_UPPER_LIM) {
		return Module_Type_2_RF_315MHZ;
	}
	else if (ADC_Sample_Avg >= MODULE1_LOWER_LIM && ADC_Sample_Avg < MODULE1_UPPER_LIM) {
		return Module_Type_1_RF_433MHZ;
	}
	else if (ADC_Sample_Avg >= MODULE0_LOWER_LIM && ADC_Sample_Avg < MODULE0_UPPER_LIM) {
		return Module_Type_0_IR;
	} 
	else {
		//ADC_Sample_Avg is below the lowest limit
		return Module_Type_None;
	}
}

/// @brief initialize reqired peripheral and function according to module type
/// @param  None
void Module_Init(void)
{
	switch (Module.Type) {
		case Module_Type_None:
			Module.Mode0_Handler = &Module_None_Mode0_Handler;
			Module.Mode1_Handler = &Module_None_Mode1_Handler;
			break;
		case Module_Type_0_IR:
			Module_IR_RF_Init();
			printf("Module %d Init\n",Module.Type);
			Module.Mode0_Handler = &Module_IR_RF_Transmit;
			Module.Mode1_Handler = &Module_IR_RF_Receive;
			break;
		case Module_Type_1_RF_433MHZ:
			Module_IR_RF_Init();
			printf("Module %d Init\n",Module.Type);
			Module.Mode0_Handler = &Module_IR_RF_Transmit;
			Module.Mode1_Handler = &Module_IR_RF_Receive;
			break;
		case Module_Type_2_RF_315MHZ:
			Module_IR_RF_Init();
			printf("Module %d Init\n",Module.Type);
			Module.Mode0_Handler = &Module_IR_RF_Transmit;
			Module.Mode1_Handler = &Module_IR_RF_Receive;
			break;
		case Module_Type_3:
			Module3_Init();
			printf("Module %d Init\n",Module.Type);
			Module.Mode0_Handler = &Module3_Mode0_Handler;
			Module.Mode1_Handler = &Module3_Mode1_Handler;
			break;
		case Module_Type_4:
			Module4_Init();
			printf("Module %d Init\n",Module.Type);
			Module.Mode0_Handler = &Module4_Mode0_Handler;
			Module.Mode1_Handler = &Module4_Mode1_Handler;
			break;
		case Module_Type_5:
			Module5_Init();
			printf("Module %d Init\n",Module.Type);
			Module.Mode0_Handler = &Module5_Mode0_Handler;
			Module.Mode1_Handler = &Module5_Mode1_Handler;
			break;
		case Module_Type_6:
			Module6_Init();
			printf("Module %d Init\n",Module.Type);
			Module.Mode0_Handler = &Module6_Mode0_Handler;
			Module.Mode1_Handler = &Module6_Mode1_Handler;
			break;
		case Module_Type_7:
			Module7_Init();
			printf("Module %d Init\n",Module.Type);
			Module.Mode0_Handler = &Module7_Mode0_Handler;
			Module.Mode1_Handler = &Module7_Mode1_Handler;
			break;
		case Module_Type_8:
			Module8_Init();
			printf("Module %d Init\n",Module.Type);
			Module.Mode0_Handler = &Module8_Mode0_Handler;
			Module.Mode1_Handler = &Module8_Mode1_Handler;
			break;
		case Module_Type_9:
			Module9_Init();
			printf("Module %d Init\n",Module.Type);
			Module.Mode0_Handler = &Module9_Mode0_Handler;
			Module.Mode1_Handler = &Module9_Mode1_Handler;
			break;
		case Module_Type_10: 
			Module10_Init();
			printf("Module %d Init\n",Module.Type);
			Module.Mode0_Handler = &Module10_Mode0_Handler;
			Module.Mode1_Handler = &Module10_Mode1_Handler;
			break;
		default:
			break;
	}
}

/// @brief System initialization
/// @param  None
void System_Init(void)
{
	USART1_Init();
	LED_Init();
    Button_Init();
	SPI2_Init();
	IMU_Init();
	Module.Mode0_Handler = &Module_None_Mode0_Handler;
	Module.Mode1_Handler = &Module_None_Mode1_Handler;
	ADC_PB1_Init();
	
	if(ADC_PB1_GetAvg() == -1){
		Module.Type = Module_Type_None;
		printf("No Module Detected\n\n");
	}
	else{
		Module.Type = Module_Detect();
		printf("Module:%d Detected\n\n",Module.Type);
	}
	ADC1_Deinit();
	Module_Init();

}

void Cyberry_Potter_System_Status_Update(void)
{
      switch(Cyberry_Potter.System_Mode){
	      case SYSTEM_MODE_0:
			Cyberry_Potter.System_Mode = SYSTEM_MODE_1;
			#ifdef SERIAL_DEBUG
			printf("SYSTEM_MODE_1\n");
			#endif //SERIAL_DEBUG
			break;
	      case SYSTEM_MODE_1:
			#ifdef LASER_ENABLE
			Cyberry_Potter.System_Mode = SYSTEM_MODE_2;
			#ifdef SERIAL_DEBUG
			printf("SYSTEM_MODE_2\n");
	      		#endif //SERIAL_DEBUG
	      
			#else 
			Cyberry_Potter.System_Mode = SYSTEM_MODE_0;
			#ifdef SERIAL_DEBUG
			printf("SYSTEM_MODE_0\n");
			#endif //SERIAL_DEBUG
			#endif  //LASER_ENABLE
			
			break;
	      #ifdef LASER_ENABLE
	      case SYSTEM_MODE_2:
			Cyberry_Potter.System_Mode = SYSTEM_MODE_0;
			#ifdef SERIAL_DEBUG
			printf("SYSTEM_MODE_0\n");
			#endif //SERIAL_DEBUG
			break;
	      #endif
      }  
}

/// @brief IMU and IR RF module interrput handler
void EXTI9_5_IRQHandler(void)
{
	
	static uint8_t i = 0;
	//IMU read
	if(EXTI_GetITStatus(EXTI_Line5)==SET){
		IMU_Get_Data(i);
		i++;
		if(i >= IMU_SEQUENCE_LENGTH_MAX){
			i = 0;
			//printf("Samlpled\n");
			IMU.Sample_Stop();
			#ifdef SYSTEM_MODE_DATA_COLLECT
			Delay_ms(200);
			IMU_Data_Print();
			#endif
		}	
	EXTI_ClearITPendingBit(EXTI_Line5);	
        }
	
	//IR and RF Receiver
        if(EXTI_GetITStatus(EXTI_Line7)==SET){
                if(IR_RF_Signal.status == SIGNAL_EMPTY){
                        Module_IR_RF_receiver_start();
                }
                else if(IR_RF_Signal.status == SIGNAL_RECORDING){
                        Module_IR_RF_Record_Duration();
                        
                }
        EXTI_ClearITPendingBit(EXTI_Line7);
        }
}

void EXTI_Stop(void)
{
	EXTI->IMR &= ~(EXTI_Line0);
}

void EXTI_Restore(void)
{
	EXTI->IMR |= EXTI_Line0;
}

uint16_t ADC_GetValue(void)
{
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	return ADC_GetConversionValue(ADC1);
}
