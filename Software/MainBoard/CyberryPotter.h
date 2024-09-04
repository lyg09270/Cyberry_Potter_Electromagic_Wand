#include "config.h"

//********************************************************************************
#ifndef	_CYBERRY_POTTER_H_
#define	_CYBERRY_POTTER_H_
#include "Delay.h"
#include "IMU.h"
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include "W25Q64.h"
#include "button.h"
#include "LED.h"
#include "USART.h"
#include "ADC.h"

#include "module0_IR.h"
#include "module1_RF433.h"
#include "module2_RF315.h"
#include "module_IR_RF.h"
#include "module3.h"
#include "module4.h"
#include "module5.h"
#include "module6.h"
#include "module7.h"
#include "module8.h"
#include "module9.h"
#include "module10.h"

typedef enum eSystem_Mode{
        SYSTEM_MODE_0 = 0,
        SYSTEM_MODE_1 = 1,
	#ifdef LASER_ENABLE
	SYSTEM_MODE_2 = 2
	#endif
}eSystem_Mode;  

typedef enum eModule_Type{
        Module_Type_None = -1,
        Module_Type_0_IR = 0,
        Module_Type_1_RF_433MHZ = 1,
	Module_Type_2_RF_315MHZ = 2,
	Module_Type_3 = 3,
	Module_Type_4 = 4,
	Module_Type_5 = 5,
	Module_Type_6 = 6,
	Module_Type_7 = 7,
	Module_Type_8 = 8,
	Module_Type_9 = 9,
	Module_Type_10 = 10,
	
}eModule_Type;

typedef enum eModel_Output{
	Unrecognized = -1,
	RightAngle = 0,
	SharpAngle = 1,
	Lightning = 2,
	Triangle = 3,
	Letter_h = 4,
	letter_R = 5,
	letter_W = 6,
	letter_phi = 7,
	Circle = 8,
	UpAndDown = 9,
	Horn = 10,
	Wave = 11,
	NoMotion = 12
}eModel_Output;



typedef struct Cyberry_Potter_t{
        eSystem_Mode System_Mode;
	void (*System_Handler)(void);
}Cyberry_Potter_t;

typedef struct Module_t
{
	volatile eModule_Type Type;
	void (*Mode0_Handler)(void);
	void (*Mode1_Handler)(void);
	
}Module_t;

typedef int8_t Model_Output_t;
typedef uint32_t ROM_Address_t;

extern struct Module_t Module;
extern struct Cyberry_Potter_t Cyberry_Potter;

void System_Init(void);
void Cyberry_Potter_System_Status_Update(void);
void EXTI_Stop(void);
void EXTI_Restore(void);
uint16_t ADC_GetValue(void);


#ifdef Signal_DEBUG
	void Signal_Data_Reset(void);
	void Signal_Print(void);
#endif //Signal_DEBUG

#endif	//_CYBERRY_POTTER_H_
