//********************************************************************************
#ifndef	_MODULE_IR_RF_H_
#define	_MODULE_IR_RF_H_
#include "stm32f10x.h"
#include "module0_IR.h"
#include "module1_RF433.h"
#include "module2_RF315.h"
#include "CyberryPotter.h"

#define SIGNAL_SEQUENCE_SET_LENGTH 1024

//Define TIM2 
#define TIM2_RECORD_OVERTIME_US 50000
#define TIM2_FREQUENCY 50000		//433Mhz Transmitter won't work if TIM2_FREQUENCY below 13000hz
#define US_PER_TIMER2_COUNT (int)(1*1000000/TIM2_FREQUENCY)
#define TIM2_PRESCALE SYSTEM_FREQUENCY/TIM2_FREQUENCY

typedef enum eSignal_Status{
        SIGNAL_EMPTY = 0,
        SIGNAL_RECORDING = 1,
        SIGNAL_READY = 2,
}eSignal_Status;

typedef struct IR_RF_Signal_t{
	eSignal_Status status;
        uint16_t duration[SIGNAL_SEQUENCE_SET_LENGTH];
        uint16_t length;
}IR_RF_Signal_t;

void Module_IR_RF_Init(void);
void Module_IR_RF_Transmit(void);
void Module_IR_RF_Receive(void);
void Module_IR_RF_receiver_start(void);
void Module_IR_RF_Record_Duration(void);
void Module_IR_RF_receiver_stop(void);
void Module_IR_RF_Data_Reset(void);
void Module_IR_RF_Print(void);
void Module_IR_RF_Copy_From_Buffer(void);
void Module_IR_RF_Copy_To_Buffer(void);


#endif	//_MODULE_IR_RF_H_

