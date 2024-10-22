#include "module3.h"
#include "USART.h"
#include "Delay.h"
#include <string.h>

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

#define MODE1_MOTION1_STRING  "0\n"
#define MODE1_MOTION2_STRING  "1\n"
#define MODE1_MOTION3_STRING  "2\n"
#define MODE1_MOTION4_STRING  "3\n"
#define MODE1_MOTION5_STRING  "4\n"
#define MODE1_MOTION6_STRING  "5\n"
#define MODE1_MOTION7_STRING  "6\n"
#define MODE1_MOTION8_STRING  "7\n"
#define MODE1_MOTION9_STRING  "8\n"
#define MODE1_MOTION10_STRING "9\n"
#define MODE1_MOTION11_STRING "10\n"
#define MODE1_MOTION12_STRING "11\n"

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
		    strcpy(Send_Buffer, MODE0_MOTION1_STRING);
		    break;
		case SharpAngle:
		    strcpy(Send_Buffer, MODE0_MOTION2_STRING);
		    break;
		case Lightning:
		    strcpy(Send_Buffer, MODE0_MOTION3_STRING);
		    break;
		case Triangle:
		    strcpy(Send_Buffer, MODE0_MOTION4_STRING);
		    break;
		case Letter_h:
		    strcpy(Send_Buffer, MODE0_MOTION5_STRING);
		    break;
		case letter_R:
		    strcpy(Send_Buffer, MODE0_MOTION6_STRING);
		    break;
		case letter_W:
		    strcpy(Send_Buffer, MODE0_MOTION7_STRING);
		    break;
		case letter_phi:
		    strcpy(Send_Buffer, MODE0_MOTION8_STRING);
		    break;
		case Circle:
		    strcpy(Send_Buffer, MODE0_MOTION9_STRING);
		    break;
		case UpAndDown:
		    strcpy(Send_Buffer, MODE0_MOTION10_STRING);
		    break;
		case Horn:
		    strcpy(Send_Buffer, MODE0_MOTION11_STRING);
		    break;
		case Wave:
		    strcpy(Send_Buffer, MODE0_MOTION12_STRING);
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
		switch (model_output) {
		case RightAngle:
		    strcpy(Send_Buffer, MODE1_MOTION1_STRING);
		    break;
		case SharpAngle:
		    strcpy(Send_Buffer, MODE1_MOTION2_STRING);
		    break;
		case Lightning:
		    strcpy(Send_Buffer, MODE1_MOTION3_STRING);
		    break;
		case Triangle:
		    strcpy(Send_Buffer, MODE1_MOTION4_STRING);
		    break;
		case Letter_h:
		    strcpy(Send_Buffer, MODE1_MOTION5_STRING);
		    break;
		case letter_R:
		    strcpy(Send_Buffer, MODE1_MOTION6_STRING);
		    break;
		case letter_W:
		    strcpy(Send_Buffer, MODE1_MOTION7_STRING);
		    break;
		case letter_phi:
		    strcpy(Send_Buffer, MODE1_MOTION8_STRING);
		    break;
		case Circle:
		    strcpy(Send_Buffer, MODE1_MOTION9_STRING);
		    break;
		case UpAndDown:
		    strcpy(Send_Buffer, MODE1_MOTION10_STRING);
		    break;
		case Horn:
		    strcpy(Send_Buffer, MODE1_MOTION11_STRING);
		    break;
		case Wave:
		    strcpy(Send_Buffer, MODE1_MOTION12_STRING);
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

