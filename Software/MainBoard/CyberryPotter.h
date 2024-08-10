#include "config.h"

//********************************************************************************
#ifndef	_CYBERRY_POTTER_H_
#define	_CYBERRY_POTTER_H_
#include "hardware.h"
#include "Delay.h"
#include "IMU.h"
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include "W25Q64.h"

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
}eSystem_Mode;

typedef enum eButton_Status{
	BUTTON_IDLE = 0,
        BUTTON_RELEASE = 1,
        BUTTON_HOLD = 2,
	BUTTON_HOLD_LONG = 3,
	BUTTON_HOLD_VERY_LONG = 4
	
}eButton_Status;     

typedef enum eIMU_STATUS{
        IMU_Idle = 0,
        IMU_Sampling = 1,
        IMU_Sampled = 2,
}eIMU_STATUS;

typedef enum eSignal_Status{
        SIGNAL_EMPTY = 0,
        SIGNAL_RECORDING = 1,
        SIGNAL_READY = 2,
}eSignal_Status;

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

typedef enum eLED_LED{
        LED_IDLE = 0,
	LED_2HZ = 1,
        LED_5HZ = 2,
        LED_10HZ = 3,
	LED_ALWAYS_ON = 4
}eLED_STATUS;

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

typedef struct IR_RF_Signal_Typedef{
        volatile uint16_t duration[SIGNAL_SEQUENCE_SET_LENGTH];
        volatile uint16_t length;
	
}IR_RF_Signal_Typedef;

typedef struct Cyberry_Potter_Status_Typedef{
        volatile eSystem_Mode System_Mode;
        volatile eButton_Status Button_Status;
        volatile eIMU_STATUS IMU_Status;
	volatile eLED_STATUS LED_Status;
	volatile eSignal_Status Signal_Status;
}Cyberry_Potter_Status_Typedef;

typedef struct Module_Typedef
{
	volatile eModule_Type Type;
	void (*Mode0_Handler)(void);
	void (*Mode1_Handler)(void);
	
}Module_Typedef;

typedef int8_t Model_Output_t;
typedef uint32_t ROM_Address_t;

void System_Init(void);
void LED_Blink(void);


#ifdef Signal_DEBUG
	void Signal_Data_Reset(void);
	void Signal_Print(void);
#endif //Signal_DEBUG

#endif	//_CYBERRY_POTTER_H_
