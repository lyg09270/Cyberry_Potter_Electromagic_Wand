//********************************************************************************
#ifndef	_MODULE_RF433_H_
#define	_MODULE_RF433_H_
#include "CyberryPotter.h"
#include "stm32f10x.h"

void Module1_RF433_Init(void);
void Module1_RF433_Transmit(void);
void Module1_RF433_Logic(uint8_t logic);

#endif	//_MODULE_RF433_H_
