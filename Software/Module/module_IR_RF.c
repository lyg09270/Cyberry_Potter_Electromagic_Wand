#include "module_IR_RF.h"


IR_RF_Signal_t IR_RF_Signal;
extern struct Module_t Module;
volatile uint16_t SIGNAL_Count;
extern LED_t LED;
extern Button_t Button;
extern uint8_t W25Q64_Buffer[4096];
extern volatile ROM_Address_t ROM_Address;
extern volatile Model_Output_t model_output;

#define RECEIVER_DISABLE EXTI->IMR &= ~(EXTI_Line7)
#define RECEIVER_ENABLE EXTI->IMR |= EXTI_Line7

/// @brief Initalize IR and RF module according to module type
/// @param  None
void Module_IR_RF_Init(void)
{
	switch(Module.Type){
		case Module_Type_0_IR:
			Module0_IR_Init();
			break;
		case Module_Type_1_RF_433MHZ:
			Module1_RF433_Init();
			break;
		case Module_Type_2_RF_315MHZ:
			Module2_RF315_Init();
			break;
		default:
			break;
	}
	IR_RF_Signal.status = SIGNAL_EMPTY;
	RECEIVER_DISABLE;
}

void Module_IR_RF_receiver_start(void)
{
	TIM_Cmd(TIM2,ENABLE);
        IR_RF_Signal.status = SIGNAL_RECORDING;
        SIGNAL_Count = 0;
}

void Module_IR_RF_Record_Duration(void)
{
        IR_RF_Signal.duration[SIGNAL_Count] = TIM_GetCounter(TIM2);
        SIGNAL_Count++;
        TIM_SetCounter(TIM2,0);
              
}

void Module_IR_RF_receiver_stop(void)
{
        TIM_Cmd(TIM2, DISABLE);
        TIM_SetCounter(TIM2,0);
        IR_RF_Signal.length = SIGNAL_Count;
        SIGNAL_Count = 0;
        IR_RF_Signal.status = SIGNAL_READY;
}

void Module_IR_RF_Copy_From_Buffer(void)
{
	uint8_t temp_high, temp_low;
	uint16_t i;

	// Extract the length from the first two bytes of the buffer
	temp_low = W25Q64_Buffer[0];
	temp_high = W25Q64_Buffer[1];
	IR_RF_Signal.length = (temp_high << 8) | temp_low;
	if(IR_RF_Signal.length >= SIGNAL_SEQUENCE_SET_LENGTH){
		IR_RF_Signal.length = 0;
		IR_RF_Signal.status = SIGNAL_EMPTY;
		return;
	}
	else{
		// Copy the rest of the signal data
		for (i = 0; i < IR_RF_Signal.length; i++) {
			temp_low = W25Q64_Buffer[i * 2 + 2]; // Offset by 2 to skip the length bytes
			temp_high = W25Q64_Buffer[i * 2 + 3];
			IR_RF_Signal.duration[i] = (temp_high << 8) | temp_low;
		}
	}
	
	
}

void Module_IR_RF_Copy_To_Buffer(void)
{
	uint8_t temp_high, temp_low;
	uint16_t i;
	
	// Write the length to the first two bytes of the buffer
	if(IR_RF_Signal.length >= SIGNAL_SEQUENCE_SET_LENGTH)
	{
		return;
	}
	else{
		temp_low = IR_RF_Signal.length & 0xFF;
		temp_high = IR_RF_Signal.length >> 8;
		W25Q64_Buffer[0] = temp_low;
		W25Q64_Buffer[1] = temp_high;
	
		// Copy the rest of the signal data
		for (i = 0; i < IR_RF_Signal.length; i++) {
			temp_low = IR_RF_Signal.duration[i] & 0xFF;
			temp_high = IR_RF_Signal.duration[i] >> 8;
			W25Q64_Buffer[i * 2 + 2] = temp_low; // Offset by 2 to skip the length bytes
			W25Q64_Buffer[i * 2 + 3] = temp_high;
		}
		
	}
	
}

void Module_IR_RF_Data_Reset(void)
{
        volatile uint16_t i = 0;
        SIGNAL_Count = 0;
	//Clear every data in signal sequence
        for(i = 0; i < SIGNAL_SEQUENCE_SET_LENGTH; i++){
                IR_RF_Signal.duration[i] = 0;
        }
        IR_RF_Signal.status = SIGNAL_EMPTY;
	IR_RF_Signal.length = 0;
}

#ifdef SERIAL_DEBUG
void Module_IR_RF_Print(void)
{
        uint16_t i = 0;
        if(IR_RF_Signal.status == SIGNAL_READY){
			if(Module.Type == Module_Type_0_IR){
				printf("Signal_Type_IR:\n");
			}
			else if(Module.Type == Module_Type_1_RF_433MHZ){
				printf("Signal_Type_RF_433MHZ:\n");
			}
			for(i = 0; i < IR_RF_Signal.length; i++){
				printf("NO:%d，Duration: %d us\n" , 
				       i+1, IR_RF_Signal.duration[i] * US_PER_TIMER2_COUNT);
			}
               }
}

#endif //SERIAL_DEBUG

/// @brief Read IR RF signal from external ROM and transimit it.
/// @param  Nome
void Module_IR_RF_Transmit(void)
{
//	Cyberry_Potter_Status.LED_Status = LED_10HZ;
	ROM_Address = W25Q64_SIGNAL_TYPE_INCREMENT * Module.Type + 
			W25Q64_SECTOR_ADDRESS_INCREMENT * model_output;
	#ifdef SERIAL_DEBUG
	printf("Address:%X",ROM_Address);
	#endif //SERIAL_DEBUG
	W25Q64_Read_Sector(ROM_Address);
	Module_IR_RF_Copy_From_Buffer();
	if(IR_RF_Signal.length != 0 && IR_RF_Signal.length < SIGNAL_SEQUENCE_SET_LENGTH){
		IR_RF_Signal.status = SIGNAL_READY;
		EXTI_Stop();
		//Module_IR_RF_Print();
			switch(Module.Type){
				case Module_Type_0_IR:
					Module0_IR_Transmit();
					break;
				case Module_Type_1_RF_433MHZ:
					Module1_RF433_Transmit();
					break;
				case Module_Type_2_RF_315MHZ:
					Module2_RF315_Transmit();
					break;
				default:
					break;
			}
		LED.Operate(BLINK_10HZ);
		EXTI_Restore();
	}
	else
	{
		IR_RF_Signal.status = SIGNAL_EMPTY;
		LED.Operate(BLINK_2HZ);
	}
	
}

/// @brief Receive IR RF signal and write to external ROM.
/// @param  None
void Module_IR_RF_Receive(void)
{
	IR_RF_Signal.status = SIGNAL_EMPTY;
	RECEIVER_ENABLE;
	while(IR_RF_Signal.status != SIGNAL_READY)
	{
		LED.Operate(BLINK_2HZ);
	}
	RECEIVER_DISABLE;
	ROM_Address = W25Q64_SIGNAL_TYPE_INCREMENT * Module.Type + 
			W25Q64_SECTOR_ADDRESS_INCREMENT * model_output;
	#ifdef SERIAL_DEBUG
	printf("ROM_Address:%X",ROM_Address);
	#endif //SERIAL_DEBUG
	printf("length:%d",IR_RF_Signal.length );
	Module_IR_RF_Copy_To_Buffer();
	printf("copy\n");
	W25Q64_Write_Sector(ROM_Address);
	printf("W25Q64_Write_Sector\n");
	Module_IR_RF_Print();
	LED.Operate(BLINK_10HZ);
	Module_IR_RF_Data_Reset();
	EXTI_Restore();	
}

//Signal recorder overtime detect*************************************************************//
void TIM2_IRQHandler()
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) == SET)
	{
                if(IR_RF_Signal.status == SIGNAL_RECORDING){
                        Module_IR_RF_receiver_stop();
                }
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);        
	}
}
