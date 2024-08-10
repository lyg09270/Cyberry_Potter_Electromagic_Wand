#include "CyberryPotter.h"

Cyberry_Potter_Status_Typedef Cyberry_Potter_Status;
IR_RF_Signal_Typedef IR_RF_Signal;
int32_t ADC_Sample_Avg = 0;
Module_Typedef Module;
extern uint8_t W25Q64_Buffer[4096];

/// @brief Get sample average from ADC,if variance is too high
/// @param  None
/// @return -1: error variance is too high.Means that No module is installed on device.
/// @return 0: get average complete.
int8_t ADC_GetAvg(void)
{
	uint8_t i = 0 ;
	uint8_t samples_count;
	int32_t sample[ADC_MAX_SAMPLES] = {0};
	
	float var = 0;
	while(var == 0 || var >= 1000){
		ADC_Sample_Avg = 0;
		var = 0;
		if(samples_count >= ADC_MAX_TESTS)
		{
			return -1;
			break;
		}
		
		for(i = 0; i < ADC_MAX_SAMPLES; i++)
		{
			sample[i] = (int32_t)ADC_GetValue() ;
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
/// @brief Detect which module is installed on device
/// @param  None
/// @return eModule_Type: module info
eModule_Type Module_Detect(void)
{
	if (ADC_Sample_Avg >= MODULE10_UPPER_LIM) {
		//ADC_Sample_Avg higher than upper limit.
		return Module_Type_None;
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
		// Do nothing if no module is detected.
		break;
		case Module_Type_0_IR:
			Module_IR_RF_Init();
			Module.Mode0_Handler = Module_IR_RF_Transmit;
			Module.Mode1_Handler = Module_IR_RF_Receive;
		break;
		case Module_Type_1_RF_433MHZ:
			Module_IR_RF_Init();
			Module.Mode0_Handler = Module_IR_RF_Transmit;
			Module.Mode1_Handler = Module_IR_RF_Receive;
		break;
		case Module_Type_2_RF_315MHZ:
			Module_IR_RF_Init();
			Module.Mode0_Handler = Module_IR_RF_Transmit;
			Module.Mode1_Handler = Module_IR_RF_Receive;
		break;
		case Module_Type_3:
			Module3_Init();
			Module.Mode0_Handler = Module3_Mode0_Handler;
			Module.Mode1_Handler = Module3_Mode1_Handler;
		break;
		case Module_Type_4:
			Module4_Init();
			Module.Mode0_Handler = Module4_Mode0_Handler;
			Module.Mode1_Handler = Module4_Mode1_Handler;
		break;
		case Module_Type_5:
			Module5_Init();
			Module.Mode0_Handler = Module5_Mode0_Handler;
			Module.Mode1_Handler = Module5_Mode1_Handler;
		break;
		case Module_Type_6:
			Module6_Init();
			Module.Mode0_Handler = Module6_Mode0_Handler;
			Module.Mode1_Handler = Module6_Mode1_Handler;
		break;
		case Module_Type_7:
			Module7_Init();
			Module.Mode0_Handler = Module7_Mode0_Handler;
			Module.Mode1_Handler = Module7_Mode1_Handler;
		break;
		case Module_Type_8:
			Module8_Init();
			Module.Mode0_Handler = Module8_Mode0_Handler;
			Module.Mode1_Handler = Module8_Mode1_Handler;
		break;
		case Module_Type_9:
			Module9_Init();
			Module.Mode0_Handler = Module9_Mode0_Handler;
			Module.Mode1_Handler = Module9_Mode1_Handler;
		break;
		case Module_Type_10: 
			Module10_Init();
			Module.Mode0_Handler = Module10_Mode0_Handler;
			Module.Mode1_Handler = Module10_Mode1_Handler;
		break;
		default:
		break;
	}
}

/// @brief System initialization
/// @param  None
void System_Init(void)
{
        Hardware_Init();
        Cyberry_Potter_Status.Button_Status = BUTTON_IDLE;
	Cyberry_Potter_Status.IMU_Status = IMU_Idle;
	Cyberry_Potter_Status.LED_Status = LED_ALWAYS_ON;
        Cyberry_Potter_Status.Signal_Status = SIGNAL_EMPTY;
	Cyberry_Potter_Status.System_Mode = SYSTEM_MODE_0;
	
	if(ADC_GetAvg() == 0){
		Module.Type = Module_Detect();
		Module_Init();
		#ifdef SERIAL_DEBUG
			printf("Module:%d Detected",Module.Type);
		#endif //SERIAL_DEBUG
		LED_ON;
	}
	else{
		#ifdef SERIAL_DEBUG
			printf("No Module Detected");
		#endif //SERIAL_DEBUG
		LED_OFF;
	}
	Module.Type = Module_Type_0_IR;
	Module_IR_RF_Init();
	Module.Mode0_Handler = Module_IR_RF_Print;
}

/// @brief Status LED controller
/// @param  None
void LED_Blink(void)
{
	int i = 0;
        if(Cyberry_Potter_Status.LED_Status == LED_5HZ){
		for(i = 0; i < 6; i++){
                LED_OFF;
		Delay_ms(100);
                LED_ON;    
		Delay_ms(100);		
		}
		Cyberry_Potter_Status.LED_Status = LED_ALWAYS_ON;
        }
	else if(Cyberry_Potter_Status.LED_Status == LED_10HZ){
		for(i = 0; i < 11; i++){
                LED_OFF;
		Delay_ms(50);
                LED_ON;    
		Delay_ms(50);		
		}
		Cyberry_Potter_Status.LED_Status = LED_ALWAYS_ON;
        }
	else if (Cyberry_Potter_Status.LED_Status == LED_2HZ){
		for(i = 0; i < 4; i++){
                LED_OFF;
		Delay_ms(250);
                LED_ON;    
		Delay_ms(250);		
		}
		Cyberry_Potter_Status.LED_Status = LED_ALWAYS_ON;
	}
	else if (Cyberry_Potter_Status.LED_Status == LED_ALWAYS_ON){
		LED_ON;
	}
	else{

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
			Cyberry_Potter_Status.IMU_Status = IMU_Sampled;
			i = 0;
			IMU_STOP();
		}	
	EXTI_ClearITPendingBit(EXTI_Line5);	
        }
	
	//IR and RF Receiver
        if(EXTI_GetITStatus(EXTI_Line7)==SET){
                if(Cyberry_Potter_Status.Signal_Status == SIGNAL_EMPTY){
                        Module_IR_RF_receiver_start();
                }
                else if(Cyberry_Potter_Status.Signal_Status == SIGNAL_RECORDING){
                        Module_IR_RF_Record_Duration();
                        
                }
        EXTI_ClearITPendingBit(EXTI_Line7);
        }
}

/// @brief Button EXTI handler
/// @param  None
void EXTI0_IRQHandler(void)
{
        if(EXTI_GetITStatus(EXTI_Line0)==SET){
		//If previous button status is handled.Then button status must change to IDLE.
		if(Cyberry_Potter_Status.Button_Status == BUTTON_IDLE){
			TIMER_FOR_BUTTON_ENABLE;
		}
		else{}//Else do nothing
                EXTI_ClearITPendingBit(EXTI_Line0);
        }
}

/// @brief Update system status
/// @param  None
static void Cyberry_Potter_System_Status_Update(void)
{
      switch(Cyberry_Potter_Status.System_Mode){
	      case SYSTEM_MODE_0:
			Cyberry_Potter_Status.System_Mode = SYSTEM_MODE_1;
			Cyberry_Potter_Status.LED_Status = LED_5HZ;
			#ifdef SERIAL_DEBUG
			printf("SYSTEM_MODE_1\n");
			#endif //SERIAL_DEBUG
			break;
	      case SYSTEM_MODE_1:
			Cyberry_Potter_Status.System_Mode = SYSTEM_MODE_0;
			Cyberry_Potter_Status.LED_Status = LED_10HZ;
			#ifdef SERIAL_DEBUG
			printf("SYSTEM_MODE_0\n");
			#endif //SERIAL_DEBUG
		
			break;
      }  
      LED_Blink();
}

/// @brief Button status update for a given time
/// @param  None
void TIM4_IRQHandler(void)
{
	static volatile uint16_t time_release_count_ms = 0;
	static volatile uint16_t time_hold_count_ms = 0;
	static volatile uint8_t previous_stuatus = 1;
        if(TIM_GetITStatus(TIM_FOR_BUTTON,TIM_IT_Update) == SET){
		//Read Button status on GPIO for now
		uint8_t current_status = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);
		
				//If status for now and previous is NOT same.
		if(current_status != previous_stuatus){
			time_hold_count_ms = 0;
			time_release_count_ms = 0;
		}
		
		//If status for now and previous is same,and button is held
		if(previous_stuatus == current_status && current_status == 0){
			
			time_hold_count_ms++;
			//If button status is stable for BUTTON_LONG_VERYLONG_THRESHOLD_MS
			if(time_hold_count_ms >= BUTTON_LONG_VERYLONG_THRESHOLD_MS){
					Cyberry_Potter_Status.Button_Status = BUTTON_HOLD_VERY_LONG;
					Cyberry_Potter_System_Status_Update();
					Cyberry_Potter_Status.Button_Status = BUTTON_IDLE;
					time_hold_count_ms = 0;
					time_release_count_ms = 0;
					TIMER_FOR_BUTTON_DISABLE;
				}
		}
		//If status for now and previous is same,and button is released
		else if(previous_stuatus == current_status && current_status == 1){
			time_release_count_ms++;
			//If button status is stable for BUTTON_IDLE_SHORT_THRESHOLD_MS 
			if(time_release_count_ms >= BUTTON_IDLE_SHORT_THRESHOLD_MS){
				
				//If button status is stable for BUTTON_SHORT_LONG_THRESHOLD_MS
				if(time_hold_count_ms >= BUTTON_SHORT_LONG_THRESHOLD_MS){
					Cyberry_Potter_Status.Button_Status = BUTTON_HOLD_LONG;
					#ifdef SERIAL_DEBUG
					printf("long ");
					#endif //SERIAL_DEBUG
					
				}
				//If button status is stable for BUTTON_IDLE_SHORT_THRESHOLD_MS 
				else if(time_hold_count_ms >= BUTTON_IDLE_SHORT_THRESHOLD_MS){
					Cyberry_Potter_Status.Button_Status = BUTTON_HOLD;
					#ifdef SERIAL_DEBUG
					printf("hold ");
					#endif //SERIAL_DEBUG
					
				}
				#ifdef SERIAL_DEBUG
				printf("hold:%d",time_hold_count_ms);
				#endif //SERIAL_DEBUG
				
				time_hold_count_ms = 0;
				time_release_count_ms = 0;
				TIMER_FOR_BUTTON_DISABLE;
			}
			else{}	//Else do nothing.
		}
			
		//Status update 
		previous_stuatus = current_status;
		TIM_ClearITPendingBit(TIM_FOR_BUTTON,TIM_IT_Update);
	}
}

