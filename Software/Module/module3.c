#include "module3.h"
#include "USART.h"
#include "Delay.h"
#include <string.h>

typedef enum{
	LCTRL = 0,
	LSHIFT = 1,
	LALT = 2,
	LMETA = 3,
	ESC = 4,
	BACKSPACE = 5,
	F1 = 6,
	F2 = 7,
	F3 = 8,
	F4 = 9,
	ENTER = 10,
	F5 = 11,
	TAB = 12,
	F6 = 13,
	F7 = 14,
	F8 = 15,
	F9 = 16,
	F10 = 17,
	F11 = 18,
	F12 = 19,
	RIGHT = 20,
	LEFT = 21,
	DOWN = 22,
	UP = 23,
	UNDO = 24,		//No use
	CUT = 25,		//No use
	COPY = 26,		//No use
	PASTE = 27,		//No use
	FIND = 28,
	MUTE = 29,		//No use
	VOLUMEUP = 30, 		//No use
	VOLUMEDOWN = 31, 	//No use
	
}KEY_NAME;

#define SERIAL_CMD_MODIFIER '~'
#define SERIAL_STRING_MODIFIER '$'

#define BUF_SIZE 64
#define END_CHAR '\0'
#define KEY_ENTER '\n'

#define MODE0_MOTION1_STRING "print(\"Hello World\")\n"
#define MODE0_MOTION2_STRING "SharpAngle\n"
#define MODE0_MOTION3_STRING "Lightning\n"
#define MODE0_MOTION4_STRING "Triangle\n"
#define MODE0_MOTION5_STRING "Letter_h\n"
#define MODE0_MOTION6_STRING "Letter_R\n"
#define MODE0_MOTION7_STRING "Letter_W\n"
#define MODE0_MOTION8_STRING "Letter_phi\n"
#define MODE0_MOTION9_STRING "Circle\n"
#define MODE0_MOTION10_STRING "UpAndDown\n"
#define MODE0_MOTION11_STRING "Horn\n"
#define MODE0_MOTION12_STRING "Wave\n"

#define MODE1_MOTION1_STRING {LMETA, END_CHAR}
#define MODE1_MOTION2_STRING {RIGHT, END_CHAR}
#define MODE1_MOTION3_STRING {LEFT, END_CHAR}
#define MODE1_MOTION4_STRING {DOWN, END_CHAR}
#define MODE1_MOTION5_STRING {UP, END_CHAR}
#define MODE1_MOTION6_STRING {'5', KEY_ENTER, END_CHAR}
#define MODE1_MOTION7_STRING {'6', KEY_ENTER, END_CHAR}
#define MODE1_MOTION8_STRING {'7', KEY_ENTER, END_CHAR}
#define MODE1_MOTION9_STRING {'8', KEY_ENTER, END_CHAR}
#define MODE1_MOTION10_STRING {'9', KEY_ENTER, END_CHAR}
#define MODE1_MOTION11_STRING {'1', '0', KEY_ENTER, END_CHAR}
#define MODE1_MOTION12_STRING {'1', '1', KEY_ENTER, END_CHAR}



const char mode0_motion1[64] = MODE0_MOTION1_STRING;
const char mode0_motion2[64] = MODE0_MOTION2_STRING;
const char mode0_motion3[64] = MODE0_MOTION3_STRING;
const char mode0_motion4[64] = MODE0_MOTION4_STRING;
const char mode0_motion5[64] = MODE0_MOTION5_STRING;
const char mode0_motion6[64] = MODE0_MOTION6_STRING;
const char mode0_motion7[64] = MODE0_MOTION7_STRING;
const char mode0_motion8[64] = MODE0_MOTION8_STRING;
const char mode0_motion9[64] = MODE0_MOTION9_STRING;
const char mode0_motion10[64] = MODE0_MOTION10_STRING;
const char mode0_motion11[64] = MODE0_MOTION11_STRING;
const char mode0_motion12[64] = MODE0_MOTION12_STRING;

const char mode1_motion1[64] = MODE1_MOTION1_STRING;
const char mode1_motion2[64] = MODE1_MOTION2_STRING;
const char mode1_motion3[64] = MODE1_MOTION3_STRING;
const char mode1_motion4[64] = MODE1_MOTION4_STRING;
const char mode1_motion5[64] = MODE1_MOTION5_STRING;
const char mode1_motion6[64] = MODE1_MOTION6_STRING;
const char mode1_motion7[64] = MODE1_MOTION7_STRING;
const char mode1_motion8[64] = MODE1_MOTION8_STRING;
const char mode1_motion9[64] = MODE1_MOTION9_STRING;
const char mode1_motion10[64] = MODE1_MOTION10_STRING;
const char mode1_motion11[64] = MODE1_MOTION11_STRING;
const char mode1_motion12[64] = MODE1_MOTION12_STRING;

char Send_Buffer[128];
extern volatile Model_Output_t model_output;

void Module3_Init(void)
{
	USART3_Init();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
        
        GPIO_InitTypeDef GPIO_InitStruct;
	
//        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
//        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
//        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//        GPIO_Init(GPIOB,&GPIO_InitStruct);
//	GPIO_ResetBits(GPIOB,GPIO_Pin_0);
}

void Module3_Mode0_Handler(void)
{
	printf("Module3_Mode0_Handler\n");
	switch (model_output) {
		case RightAngle:
		    strcpy(Send_Buffer, mode0_motion1);
		    break;
		case SharpAngle:
		    strcpy(Send_Buffer, mode0_motion2);
		    break;
		case Lightning:
		    strcpy(Send_Buffer, mode0_motion3);
		    break;
		case Triangle:
		    strcpy(Send_Buffer, mode0_motion4);
		    break;
		case Letter_h:
		    strcpy(Send_Buffer, mode0_motion5);
		    break;
		case letter_R:
		    strcpy(Send_Buffer, mode0_motion6);
		    break;
		case letter_W:
		    strcpy(Send_Buffer, mode0_motion7);
		    break;
		case letter_phi:
		    strcpy(Send_Buffer, mode0_motion8);
		    break;
		case Circle:
		    strcpy(Send_Buffer, mode0_motion9);
		    break;
		case UpAndDown:
		    strcpy(Send_Buffer, mode0_motion10);
		    break;
		case Horn:
		    strcpy(Send_Buffer, mode0_motion11);
		    break;
		case Wave:
		    strcpy(Send_Buffer, mode0_motion12);
		    break;
		default:
		    Send_Buffer[0] = '\0';
		    break;
    }
	
	uint8_t i = 0;
	while(Send_Buffer[i])
	{
		USART3_WriteByte(Send_Buffer[i]);
		i++;
		Delay_ms(50);
	}
	Send_Buffer[0] = '\0';
}

void Module3_Mode1_Handler(void)
{
	printf("Module3_Mode0_Handler\n");
	switch (model_output) {
		case RightAngle:
		    strcpy(Send_Buffer, mode1_motion1);
		    break;
		case SharpAngle:
		    strcpy(Send_Buffer, mode1_motion2);
		    break;
		case Lightning:
		    strcpy(Send_Buffer, mode1_motion3);
		    break;
		case Triangle:
		    strcpy(Send_Buffer, mode1_motion4);
		    break;
		case Letter_h:
		    strcpy(Send_Buffer, mode1_motion5);
		    break;
		case letter_R:
		    strcpy(Send_Buffer, mode1_motion6);
		    break;
		case letter_W:
		    strcpy(Send_Buffer, mode1_motion7);
		    break;
		case letter_phi:
		    strcpy(Send_Buffer, mode1_motion8);
		    break;
		case Circle:
		    strcpy(Send_Buffer, mode1_motion9);
		    break;
		case UpAndDown:
		    strcpy(Send_Buffer, mode1_motion10);
		    break;
		case Horn:
		    strcpy(Send_Buffer, mode1_motion11);
		    break;
		case Wave:
		    strcpy(Send_Buffer, mode1_motion12);
		    break;
		default:
		    Send_Buffer[0] = '\0';
		    break;
    }
	
	uint8_t i = 0;
	while(Send_Buffer[i])
	{
		USART3_WriteByte(Send_Buffer[i]);
		i++;
		Delay_ms(50);
	}
	Send_Buffer[0] = '\0';
}

