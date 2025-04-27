#include "module5.h"
#include "OLED.h"

extern volatile Model_Output_t model_output;

void Module5_Init(void)
{
	OLED_Init();
}

void Module5_Mode0_Handler(void)
{
    OLED_Clear();
	switch (model_output) 
    {
        case RightAngle:
            OLED_ShowString(1, 1, "Right Angle");
            break;
        case SharpAngle:
            OLED_ShowString(1, 1, "Sharp Angle");
            break;
        case Triangle:
            OLED_ShowString(1, 1, "Triangle");
            break;
        case Lightning:
            OLED_ShowString(1, 1, "Lightning");
            break;
        case Letter_h:
            OLED_ShowString(1, 1, "Letter h");
            break;
        case letter_R:
            OLED_ShowString(1, 1, "Letter R");
            break;
        case letter_W:
            OLED_ShowString(1, 1, "Letter W");
            break;
        case letter_phi:
            OLED_ShowString(1, 1, "Letter Phi");
            break;
        case UpAndDown:
            OLED_ShowString(1, 1, "Up and Down");
            break;
        case Horn:
            OLED_ShowString(1, 1, "Horn");
            break;
        case Circle:
            OLED_ShowString(1, 1, "Circle");
            break;
        case Wave:
            OLED_ShowString(1, 1, "Wave");
            break;
    }
}

void Module5_Mode1_Handler(void){
	
}

