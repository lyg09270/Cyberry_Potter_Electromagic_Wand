#include "module_IR_RF.h"

extern Cyberry_Potter_Status_Typedef Cyberry_Potter_Status;
extern IR_RF_Signal_Typedef IR_RF_Signal;
extern Module_Typedef Module;
volatile uint16_t SIGNAL_Count;
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
			//Module2_RF315_Init();
			break;
		default:
			break;
		}
	RECEIVER_DISABLE;
}

/// @brief Read IR RF signal from external ROM and transimit it.
/// @param  Nome
void Module_IR_RF_Transmit(void)
{
	Cyberry_Potter_Status.LED_Status = LED_10HZ;
	ROM_Address = W25Q64_SIGNAL_TYPE_INCREMENT * Module.Type + 
			W25Q64_SECTOR_ADDRESS_INCREMENT * model_output;
	printf("Address:%X",ROM_Address);
	W25Q64_Read_Sector(ROM_Address);
	Module_IR_RF_Copy_From_Buffer();
	Cyberry_Potter_Status.Signal_Status = SIGNAL_READY;
	Module_IR_RF_Print();
	LED_Blink();
	while(Cyberry_Potter_Status.Signal_Status != SIGNAL_READY);
	if(Cyberry_Potter_Status.Signal_Status == SIGNAL_READY)
	{
		printf("Ready");
		//Prevent singal transmition from interrupted by EXTI
		RECEIVER_DISABLE;
		switch(Module.Type){
			case Module_Type_0_IR:
				printf("type IR");
				Module0_IR_Transmit();
				break;
			case Module_Type_1_RF_433MHZ:
				Module1_RF433_Transmit();
				break;
			case Module_Type_2_RF_315MHZ:
				break;
			default:
				break;
		}
		RECEIVER_ENABLE;
	}
}

/// @brief Receive IR RF signal and write to external ROM.
/// @param  None
void Module_IR_RF_Receive(void)
{
	RECEIVER_ENABLE;
	while(Cyberry_Potter_Status.Signal_Status != SIGNAL_READY);
	RECEIVER_DISABLE;
	Cyberry_Potter_Status.LED_Status = LED_5HZ;
	LED_Blink();
	ROM_Address = W25Q64_SIGNAL_TYPE_INCREMENT * Module.Type + 
			W25Q64_SECTOR_ADDRESS_INCREMENT * ROM_Address;
	#ifdef SERIAL_DEBUG
	printf("ROM_Address:%X",ROM_Address);
	#endif //SERIAL_DEBUG
	Module_IR_RF_Copy_To_Buffer();
	W25Q64_Write_Sector(ROM_Address);
	Module_IR_RF_Print();
	EXTI_Restore();	
}

void Module_IR_RF_receiver_start(void)
{
	TIM_Cmd(TIM2,ENABLE);
        Cyberry_Potter_Status.Signal_Status = SIGNAL_RECORDING;
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
        TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);
        IR_RF_Signal.length = SIGNAL_Count;
        SIGNAL_Count = 0;
        Cyberry_Potter_Status.Signal_Status = SIGNAL_READY;
}

void Module_IR_RF_Copy_From_Buffer(void)
{
    uint8_t temp_high, temp_low, i;

    // Extract the length from the first two bytes of the buffer
    temp_low = W25Q64_Buffer[0];
    temp_high = W25Q64_Buffer[1];
    IR_RF_Signal.length = (temp_high << 8) | temp_low;

    // Copy the rest of the signal data
    for (i = 0; i < IR_RF_Signal.length; i++) {
        temp_low = W25Q64_Buffer[i * 2 + 2]; // Offset by 2 to skip the length bytes
        temp_high = W25Q64_Buffer[i * 2 + 3];
        IR_RF_Signal.duration[i] = (temp_high << 8) | temp_low;
    }
}

void Module_IR_RF_Copy_To_Buffer(void)
{
    uint8_t temp_high, temp_low, i;

    // Write the length to the first two bytes of the buffer
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

void Module_IR_RF_Data_Reset(void)
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

#ifdef SERIAL_DEBUG
void Module_IR_RF_Print(void)
{
        uint16_t i = 0;
        if(Cyberry_Potter_Status.Signal_Status == SIGNAL_READY){
			if(Module.Type == Module_Type_0_IR){
				printf("Signal_Type_IR:\n");
			}
			else if(Module.Type == Module_Type_1_RF_433MHZ){
				printf("Signal_Type_RF_433MHZ:\n");
			}
			for(i = 0; i < IR_RF_Signal.length; i++){
				printf("NO:%d  Duration: %d us Total:%d\n" , 
				       i+1, IR_RF_Signal.duration[i] * US_PER_TIMER2_COUNT,
					IR_RF_Signal.length);
			}
               }
}

#endif //SERIAL_DEBUG


//Signal recorder overtime detect*************************************************************//
void TIM2_IRQHandler()
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) == SET)
	{
                if(Cyberry_Potter_Status.Signal_Status == SIGNAL_RECORDING){
                        Module_IR_RF_receiver_stop();
                }
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);        
	}
}
