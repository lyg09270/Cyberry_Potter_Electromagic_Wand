#include "CyberryPotter.h"

Cyberry_Potter_Status_Typedef Cyberry_Potter_Status;
IR_RF_Signal_Typedef IR_RF_Signal;
Module_Typedef Module;
volatile uint16_t SIGNAL_Count;
extern uint8_t W25Q64_Buffer[4096];

void System_Init(void)
{
        Hardware_Init();
        Cyberry_Potter_Status.Button_Status = BUTTON_IDLE;
	Cyberry_Potter_Status.IMU_Status = IMU_Idle;
	Cyberry_Potter_Status.LED_Status = LED_ALWAYS_ON;
        Cyberry_Potter_Status.Signal_Status = SIGNAL_EMPTY;
	Cyberry_Potter_Status.System_Mode = SYSTEM_MODE_0;
	
	
	Module.Type = Module_Type_IR;
	Module.Mode0_Handler = Module0_IR_Transmit;
	//Moudle.Mode1_Handler = NULL;
	
	LED_ON;
}

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

void Signal_Transmit(void)
{
	//Data can be transmit if 
	//1.signal is loaded form ROM.
	//2.signal is recorded from receivers.
	//3.signal has sent before.
	if(Cyberry_Potter_Status.Signal_Status == SIGNAL_LOADED || 
		Cyberry_Potter_Status.Signal_Status == SIGNAL_RECORDED
		|| Cyberry_Potter_Status.Signal_Status == SIGNAL_SENT)
	{
		//Prevent singal transmition from interrupted by EXTI
		EXTI_Stop();
		Cyberry_Potter_Status.Signal_Status = SIGNAL_SENDING;
		switch(Module.Type){
			case Module_Type_IR:
				Module0_IR_Transmit();
				break;
			case Module_Type_RF_433MHZ:
				Module1_RF433_Transmit();
				break;
			case Module_Type_None:
				break;
		}
		EXTI_Restore();
		Cyberry_Potter_Status.Signal_Status = SIGNAL_SENT;
	}
}

static void Signal_receiver_start(void)
{
	TIM_Cmd(TIM2,ENABLE);
        Cyberry_Potter_Status.Signal_Status = SIGNAL_RECORDING;
        SIGNAL_Count = 0;
}

static void Signal_Record_Duration(void)
{
        IR_RF_Signal.duration[SIGNAL_Count] = TIM_GetCounter(TIM2);
        SIGNAL_Count++;
        TIM_SetCounter(TIM2,0);
              
}

static void Signal_receiver_stop(void)
{
        TIM_Cmd(TIM2, DISABLE);
        TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);
        IR_RF_Signal.length = SIGNAL_Count;
        SIGNAL_Count = 0;
        Cyberry_Potter_Status.Signal_Status = SIGNAL_RECORDED;
}

void Signal_Copy_From_Buffer(void)
{
	uint8_t temp_high, temp_low, i;
	volatile uint16_t *duration_ptr = &IR_RF_Signal.duration[0];
	uint8_t *buffer_ptr = W25Q64_Buffer;

	// Extract the length from the first two bytes of the buffer
	temp_low = *buffer_ptr++;
	temp_high = *buffer_ptr++;
	IR_RF_Signal.length = (temp_high << 8) | temp_low;

	// Copy the rest of the signal data
	for (i = 0; i < IR_RF_Signal.length; i++) {
		temp_low = *buffer_ptr++;
		temp_high = *buffer_ptr++;
		*duration_ptr++ = (temp_high << 8) | temp_low;
	}
}

void Signal_Copy_To_Buffer(void)
{
	uint8_t temp_high, temp_low, i;
	volatile uint16_t *duration_ptr = &IR_RF_Signal.duration[0];
	uint8_t *buffer_ptr = W25Q64_Buffer;
	
	// Write the length to the first two bytes of the buffer
	temp_low = IR_RF_Signal.length & 0xFF;
	temp_high = IR_RF_Signal.length >> 8;
	*buffer_ptr++ = temp_low;
	*buffer_ptr++ = temp_high;

	// Copy the rest of the signal data
	for (i = 0; i < IR_RF_Signal.length; i++, duration_ptr++) {
		temp_low = *duration_ptr & 0xFF;
		temp_high = *duration_ptr >> 8;
		*buffer_ptr++ = temp_low;
		*buffer_ptr++ = temp_high;
	}
}

//Signal recorder overtime detect*************************************************************//
void TIM2_IRQHandler()
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) == SET)
	{
                if(Cyberry_Potter_Status.Signal_Status == SIGNAL_RECORDING){
                        Signal_receiver_stop();
                }
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);        
	}
}

#ifdef SERIAL_DEBUG
void Signal_Print(void)
{
        uint16_t i = 0;
        if(Cyberry_Potter_Status.Signal_Status == SIGNAL_RECORDED || Cyberry_Potter_Status.Signal_Status == SIGNAL_SENT || 
		Cyberry_Potter_Status.Signal_Status == SIGNAL_LOADED){
			if(Module.Type == Module_Type_IR){
				printf("Signal_Type_IR:\n");
			}
			else if(Module.Type == Module_Type_RF_433MHZ){
				printf("Signal_Type_RF_433MHZ:\n");
			}
			for(i = 0; i < IR_RF_Signal.length; i++){
				printf("NO:%d  Duration: %d us Total:%d\n" , 
				       i+1, IR_RF_Signal.duration[i] * US_PER_TIMER2_COUNT,
					IR_RF_Signal.length);
			}
				
			       Signal_Transmit(); 
               }
}
	       
void Signal_Data_Reset(void)
{
        volatile uint16_t i = 0;
        SIGNAL_Count = 0;
	//Clear every data in signal sequence
        for(i = 0; i < SIGNAL_SEQUENCE_SET_LENGTH; i++){
                IR_RF_Signal.duration[i] = 0;
        }
        //Cyberry_Potter_Status.Signal_Status = SIGNAL_EMPTY;
	IR_RF_Signal.length = 0;
}

#endif //SERIAL_DEBUG

void EXTI9_5_IRQHandler()
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
                        Signal_receiver_start();
                }
                else if(Cyberry_Potter_Status.Signal_Status == SIGNAL_RECORDING){
                        Signal_Record_Duration();
                        
                }
		
        EXTI_ClearITPendingBit(EXTI_Line7);
        }
}

//Button EXTI handler*******************************************************//
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

static void Cyberry_Potter_System_Status_Update(void)
{
      switch(Cyberry_Potter_Status.System_Mode){
	      case SYSTEM_MODE_0:
			Cyberry_Potter_Status.System_Mode = SYSTEM_MODE_1;
			Cyberry_Potter_Status.LED_Status = LED_5HZ;
			printf("SYSTEM_MODE_1\n");
			break;
	      case SYSTEM_MODE_1:
			Cyberry_Potter_Status.System_Mode = SYSTEM_MODE_0;
			Cyberry_Potter_Status.LED_Status = LED_10HZ;
			printf("SYSTEM_MODE_0\n");
			break;
      }  
      LED_Blink();
}

//Button status update for a given time
void TIM4_IRQHandler(void)
{
	static volatile uint16_t time_release_count_ms = 0;
	static volatile uint16_t time_hold_count_ms = 0;
	static volatile uint8_t previous_stuatus = 1;
        if(TIM_GetITStatus(TIM_FOR_BUTTON,TIM_IT_Update) == SET){
		//Read Button status on GPIO for now
		uint8_t current_status = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);
		
		//If status for now and previous is same,and button is held
		if(previous_stuatus == current_status && current_status == 0){
			
			time_hold_count_ms++;
			//If button status is stable for BUTTON_LONG_VERYLONG_THRESHOLD_MS
			if(time_hold_count_ms >= BUTTON_LONG_VERYLONG_THRESHOLD_MS){
					Cyberry_Potter_Status.Button_Status = BUTTON_HOLD_VERY_LONG;
					Cyberry_Potter_System_Status_Update();
					Cyberry_Potter_Status.Button_Status = BUTTON_IDLE;
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
					printf("long ");
				}
				//If button status is stable for BUTTON_IDLE_SHORT_THRESHOLD_MS 
				else if(time_hold_count_ms >= BUTTON_IDLE_SHORT_THRESHOLD_MS){
					Cyberry_Potter_Status.Button_Status = BUTTON_HOLD;
					printf("hold ");
				}
				printf("hold:%d",time_hold_count_ms);
				time_hold_count_ms = 0;
				time_release_count_ms = 0;
				TIMER_FOR_BUTTON_DISABLE;
			}
			else{}	//Else do nothing.
		}
		//If status for now and previous is NOT same.
		else{}
			
		//Status update 
		previous_stuatus = current_status;
		TIM_ClearITPendingBit(TIM_FOR_BUTTON,TIM_IT_Update);
	}
}

