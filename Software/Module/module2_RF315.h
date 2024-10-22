//********************************************************************************
#ifndef	_MODULE_RF315_H_
#define	_MODULE_RF315_H_
#include "stm32f10x.h"
#include "module_IR_RF.h"

void Module2_RF315_Init(void);
void Module2_RF315_Transmit(void);
void Module2_RF315_Logic(uint8_t logic);

#endif	//_MODULE_RF315_H_
