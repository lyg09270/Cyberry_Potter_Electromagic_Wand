//********************************************************************************
#ifndef	_MODULE_IR_H_
#define	_MODULE_IR_H_
#include "stm32f10x.h"
#include "module_IR_RF.h"

#define IR_PWM_ENABLE TIM_Cmd(TIM3,ENABLE)
#define IR_PWM_DISABLE TIM_Cmd(TIM3,DISABLE)

void Module0_IR_Init(void);
void Module0_IR_Transmit(void);
void Module0_IR_38Khz_Logic(uint8_t logic);

#endif	//_MODULE_IR_H_

