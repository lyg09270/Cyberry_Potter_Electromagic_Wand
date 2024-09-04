#include "CyberryPotter.h"
#include "weights.h"
#include "nnom.h"

#define QUANTIFICATION_SCALE (pow(2,INPUT_1_OUTPUT_DEC))
#define OUPUT_THRESHOLD 63 //The out put of model must bigger than this value unless the out put would be unrecognized.

#ifdef NNOM_USING_STATIC_MEMORY
	uint8_t static_buf[1024 * 8];
#endif //NNOM_USING_STATIC_MEMORY
nnom_model_t* model;

volatile Model_Output_t model_output = -1;
volatile ROM_Address_t ROM_Address;

/*
 * @brief Quantirize IMU data and feed to CNN model
 * @param None
 * @return None
 */
void model_feed_data(void)
{
	const double scale = QUANTIFICATION_SCALE;
	uint16_t i = 0;
	for(i = 0; i < IMU_SEQUENCE_LENGTH_MAX;i++){
		nnom_input_data[i*3] = (int8_t)round(IMU.gyro[i][Roll] * scale);
		nnom_input_data[i*3+1] = (int8_t)round(IMU.gyro[i][Pitch] * scale);
		nnom_input_data[i*3+2] = (int8_t)round(IMU.gyro[i][Yaw] * scale);
	}
}

//void model_feed_data(void)
//{
//	const double scale = QUANTIFICATION_SCALE;
//	uint16_t i = 0;
//	for(i = 0; i < IMU_SEQUENCE_LENGTH_MAX;i++){
//		nnom_input_data[i*6] = 	 (int8_t)round(IMU.acc[i][AccX] * scale);
//		nnom_input_data[i*6+1] = (int8_t)round(IMU.acc[i][AccY] * scale);
//		nnom_input_data[i*6+2] = (int8_t)round(IMU.acc[i][AccZ] * scale);
//		nnom_input_data[i*6+3] = (int8_t)round(IMU.gyro[i][Roll] * scale);
//		nnom_input_data[i*6+4] = (int8_t)round(IMU.gyro[i][Pitch] * scale);
//		nnom_input_data[i*6+5] = (int8_t)round(IMU.gyro[i][Yaw] * scale);
//	}
//}

/*
 * @brief get output of CNN model
 * @param  None
 * @return Model_Output_t: type of recognized motion
 */
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

int main(void)
{       
	//initalize system
	System_Init();
	LED.Operate(BLINK_10HZ);
	//crate CNN model
	#ifdef NNOM_USING_STATIC_MEMORY
		nnom_set_static_buf(static_buf, sizeof(static_buf)); 
	#endif //NNOM_USING_STATIC_MEMORY
	model = nnom_model_create();
	
	printf("While");
	while(1){
		//Button Status reset and wait button status change.
		if(Button.status == BUTTON_HOLD && IMU.status == IMU_Idle){
			IMU.Sample_Start();
			EXTI_Stop();
			LED.Operate(OFF);
			while(IMU.status != IMU_Sampled);
			LED.Operate(ON);
			#ifndef SYSTEM_MODE_DATA_COLLECT
			model_output = model_get_output();
			
			if(model_output != Unrecognized){
				switch(Cyberry_Potter.System_Mode){
					case SYSTEM_MODE_0:
						Module.Mode0_Handler();
						break;
					case SYSTEM_MODE_1:
						Module.Mode1_Handler();
						break;
					default:
						break;
					
				}
			}
			#endif
			IMU.status = IMU_Idle;
			Button.status_clear();
			EXTI_Restore();
		}
		else if(Button.status == BUTTON_HOLD_LONG){
			printf("BUTTON_HOLD_LONG\n");
			LED.Operate(BLINK_5HZ);
			Cyberry_Potter_System_Status_Update();
			#ifdef LASER_ENABLE
			if(Cyberry_Potter.System_Mode == SYSTEM_MODE_2){
				Laser.status = ON;
				Laser.Operate(Laser.status);
			}
			else{
				Laser.status = OFF;
				Laser.Operate(Laser.status);
			}
			#endif
			Button.status_clear();
		}
		
		
		
	}
}

