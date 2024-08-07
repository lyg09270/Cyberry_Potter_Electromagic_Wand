//********************************************************************************
#ifndef	_MODULE_IR_RF_H_
#define	_MODULE_IR_RF_H_
#include "CyberryPotter.h"
#include "stm32f10x.h"
#include "module0_IR.h"
#include "module1_RF433.h"
#include "module2_RF315.h"

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

