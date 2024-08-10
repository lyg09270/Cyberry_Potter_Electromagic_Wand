#include "CyberryPotter.h"
#include "weights.h"
#include "nnom.h"

extern Cyberry_Potter_Status_Typedef Cyberry_Potter_Status;
extern IR_RF_Signal_Typedef IR_RF_Signal;
extern Module_Typedef Module;
extern float IMU_Data_mAngle[IMU_SEQUENCE_LENGTH_MAX][3];
extern float IMU_Data_mAcc[IMU_SEQUENCE_LENGTH_MAX][3];
extern uint8_t W25Q64_Buffer[4096];
volatile Model_Output_t model_output = 0;
volatile ROM_Address_t ROM_Address;
nnom_model_t* model;
#define QUANTIFICATION_SCALE (pow(2,INPUT_1_OUTPUT_DEC))

#ifdef NNOM_USING_STATIC_MEMORY
	uint8_t static_buf[1024 * 6];
#endif //NNOM_USING_STATIC_MEMORY

/// @brief quantirize IMU data and feed to CNN model
/// @param  None
void model_feed_data(void)
{
	const double scale = QUANTIFICATION_SCALE;
	uint16_t i = 0;
	for(i = 0; i < IMU_SEQUENCE_LENGTH_MAX;i++){
		nnom_input_data[i*3] = (int8_t)round(IMU_Data_mAcc[i][0] * scale);
		nnom_input_data[i*3+1] = (int8_t)round(IMU_Data_mAcc[i][1] * scale);
		nnom_input_data[i*3+2] = (int8_t)round(IMU_Data_mAcc[i][2] * scale);
	}
}

/// @brief get output of CNN model
/// @param  None
/// @return Model_Output_t: type of recognized motion
Model_Output_t model_get_output(void)
{
	volatile uint8_t i = 0;
	volatile Model_Output_t max_output = -128;
	Model_Output_t ret = 0;
	model_feed_data();
	model_run(model);
	for(i = 0; i < 13;i++){
		
		#ifdef SERIAL_DEBUG
		printf("Output[%d] = %.2f %%\n",i,(nnom_output_data[i] / 127.0)*100);
		#endif //SERIAL_DEBUG
		
		if(nnom_output_data[i] >= max_output){
			max_output = nnom_output_data[i] ;
			ret = i;
		}
	}
	if(max_output < OUPUT_THRESHOLD || ret == NoMotion){
		ret = Unrecognized;
	}
	
	#ifdef SERIAL_DEBUG
	switch(ret){
		case Unrecognized:
			printf("Unrecognized");
			break;
		case RightAngle:
			printf("RightAngle");
			break;
		case SharpAngle:
			printf("SharpAngle");
			break;
		case Lightning:
			printf("Lightning");
			break;
		case Triangle:
			printf("Triangle");
			break;
		case Letter_h:
			printf("Letter_h");
			break;
		case letter_R:
			printf("Letter_R");
			break;
		case letter_W:
			printf("Letter_W");
			break;
		case letter_phi:
			printf("Letter_phi");
			break;
		case Circle:
			printf("Circle");
			break;
		case UpAndDown:
			printf("UpAndDown");
			break;
		case Horn:
			printf("Horn");
			break;
		case Wave:
			printf("Wave");
			break;
		case NoMotion:
			printf("Unrecognized");
			break;
	}
	printf("\n");
	#endif //SERIAL_DEBUG
	
	return ret;
}

/// @brief System mode 0 handler
/// @param  None
void Cyberry_Potter_Mode0(void)
{
	//Start Sample and wait untill IMU_Sampled.
	IMU_Sample_Start();	
	while(Cyberry_Potter_Status.IMU_Status != IMU_Sampled);
	LED_ON;
	
	#ifdef SYSTEM_MODE_DATA_COLLECT
	Delay_ms(200);
	IMU_Data_Print();
	#endif //SYSTEM_MODE_DATA_COLLECT
	
	#ifndef SYSTEM_MODE_DATA_COLLECT
	
	//Inference by using sampled data
	model_output = model_get_output();
	if(model_output != Unrecognized){
		Module.Mode0_Handler();
	}
	//Fail to identify motions.
	else{
		Cyberry_Potter_Status.LED_Status = LED_2HZ;
		LED_Blink();
		EXTI_Restore();
	}
	#endif //SYSTEM_MODE_DATA_COLLECT

	Cyberry_Potter_Status.IMU_Status = IMU_Idle;
	EXTI_Restore();	
	//Cyberry_Potter_Status.Button_Status = BUTTON_IDLE;
}

/// @brief System mode 1 handler
/// @param  None
void Cyberry_Potter_Mode1(void)
{
	//Start Sample and wait till IMU_Sampled.
	IMU_Sample_Start();	
	while(Cyberry_Potter_Status.IMU_Status != IMU_Sampled);
	LED_ON;
	//Inference by using sampled data
	model_output = model_get_output();
	EXTI_Restore();
	Cyberry_Potter_Status.IMU_Status = IMU_Idle;
	
	//Inference by using sampled data
	if(model_output != Unrecognized){
		Module.Mode1_Handler();
	}	//model_output != -1
	//Fail to identify motions.
	else{
		Cyberry_Potter_Status.LED_Status = LED_2HZ;
		LED_Blink();
		EXTI_Restore();
	}
	//Cyberry_Potter_Status.Button_Status = BUTTON_IDLE;
}

extern int32_t ADC_Sample_Avg;
int8_t ADC_GetAvg(void);
/// @brief main function
int main(void)
{       
	//initalize system
    System_Init();
	//crate CNN model
	#ifdef NNOM_USING_STATIC_MEMORY
		nnom_set_static_buf(static_buf, sizeof(static_buf)); 
	#endif //NNOM_USING_STATIC_MEMORY
	model = nnom_model_create();
	
	while(1){
		//Button Status reset and wait button status change.
		Cyberry_Potter_Status.Button_Status = BUTTON_IDLE;
		while(Cyberry_Potter_Status.Button_Status == BUTTON_IDLE){
			ADC_GetAvg();
			Delay_ms(200);
		}
		
		//SYSTEM_MODE_0
		if(Cyberry_Potter_Status.System_Mode == SYSTEM_MODE_0){
			//User input
			if(Cyberry_Potter_Status.Button_Status == BUTTON_HOLD){
				Cyberry_Potter_Mode0();
			}
		}//SYSTEM_MODE_0
		
		//SYSTEM_MODE_1
		else if(Cyberry_Potter_Status.System_Mode == SYSTEM_MODE_1){
				//User input
			if(Cyberry_Potter_Status.Button_Status == BUTTON_HOLD){
				Cyberry_Potter_Mode1();
			}
		}
			
		
		
	}
}//main

