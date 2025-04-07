/**
 * File Name: IMU.c
 * 
 * Copyright © 2023 Civic_Crab. All rights reserved.
 * 
 * Author: Civic_Crab
 * Creation Date: Oct 1, 2020
 * Version: 1.0.0
 * 
 * Description: This file prodides IIC functions implementation
 */

#include "config.h"
#include "MPU6050.h"
#include "stm32f10x.h"
#include "IIC.h"
#include "MPU6050_Reg.h"
#include "IMU.h"
#include "Delay.h"
#include "math.h"
#include <stdint.h>
#include <stdio.h>

IMU_t IMU;

//measured data beginning with m, d means derivative ,mdAngle is measured angular velovity in this case.
int16_t IMU_bias[6] = {0,0,0,0,0,0};

#ifdef IMAGE_ENABLE
Canvas_t IMU_Canvas;
#endif //IMAGE_ENABLE

//This function is used to print the Acc data to your computer
//IMU_DATA_PRINT_HEADER is the header that the script use to identify which the following messages is IMU data or not.
#ifdef SYSTEM_MODE_DATA_COLLECT
void IMU_Data_Print(void){
	int16_t i = 0;
	printf(IMU_DATA_PRINT_HEADER);
	for(i = 0; i < IMU_SEQUENCE_LENGTH_MAX;i++){
	printf("%f %f %f %f %f %f\n",
		IMU.acc[i][AccX], IMU.acc[i][AccY], IMU.acc[i][AccZ],
		IMU.gyro[i][Roll], IMU.gyro[i][Pitch], IMU.gyro[i][Yaw]);
	}
}
#endif //SYSTEM_MODE_DATA_COLLECT

void IMU_Sample_Start(void)
{
	EXTI->IMR &= ~(EXTI_Line0);
	INT_START;
	IMU.status = IMU_Sampling;
}

void IMU_Sample_Stop(void)
{
	EXTI->IMR |= EXTI_Line0;
	INT_STOP;
	IMU.status = IMU_Sampled;
}

void IMU_Get_Data(uint8_t i)
{
	uint8_t temp_acc[6];
	uint8_t temp_gyro[6];
	int16_t IMU_Received[6];
	IIC1_read(0x68,MPU6050_RA_ACCEL_XOUT_H,6,temp_acc);
	IMU_Received[AccX] = (temp_acc[0] << 8) + temp_acc[1] - IMU_bias[AccX];
	IMU_Received[AccY] = (temp_acc[2] << 8) + temp_acc[3] - IMU_bias[AccY];
	IMU_Received[AccZ] = (temp_acc[4] << 8) + temp_acc[5] - IMU_bias[AccZ];
	
	IMU.acc[i][AccX] = IMU_Received[AccX] / IMU_ACC_TRANS_CONSTANT;
	IMU.acc[i][AccY] = IMU_Received[AccY] / IMU_ACC_TRANS_CONSTANT;
	IMU.acc[i][AccZ] = IMU_Received[AccZ] / IMU_ACC_TRANS_CONSTANT;
	
	IIC1_read(0x68,MPU6050_RA_GYRO_XOUT_H,6,temp_gyro);
	IMU_Received[Roll] = (temp_gyro[0] << 8) + temp_gyro[1] - IMU_bias[Roll];
	IMU_Received[Pitch] = (temp_gyro[2] << 8) + temp_gyro[3]- IMU_bias[Pitch];
	IMU_Received[Yaw] = (temp_gyro[4] << 8) + temp_gyro[5]  - IMU_bias[Yaw];
	
	IMU.gyro[i][Roll] = IMU_Received[Roll] / IMU_GYRO_TRANS_RADIAN_CONSTANT;
	IMU.gyro[i][Pitch] = IMU_Received[Pitch] / IMU_GYRO_TRANS_RADIAN_CONSTANT;
	IMU.gyro[i][Yaw] = IMU_Received[Yaw] / IMU_GYRO_TRANS_RADIAN_CONSTANT;

	#ifdef IMAGE_ENABLE
	IMU_Canvas.scale[0] += IMU.gyro[i][Roll];
	IMU_Canvas.scale[1] += IMU.gyro[i][Yaw];
	#endif //IMAGE_ENABLE
}

void Canvas_Draw_Line(Canvas_t* canvas,int8_t x1,int8_t x2,int8_t y1,int8_t y2)
{
	int8_t dx,dy,sx,sy,err,e2;
	dx = abs(x2 - x1);
	dy = abs(y2 - y1);
    sx = (x2 > x1) ? 1 : -1;
    sy = (y2 > y1) ? 1 : -1;
	err = dx - dy;

	while(1){
		canvas->data[y1][x1] = 255;
		
		if (x1 == x2 && y1 == y2)
			break;
		
		e2 = 2 * err;
		if (e2 > -dy){
			err -= dy;
			x1 += sx;
		}
		if (e2 < dx){
			err += dx;
			y1 += sy;
		}
	}
}

void Canvas_Print(Canvas_t* canvas)
{
	uint8_t i = 0;
	uint8_t j = 0;
	for(i = 0; i < IMAGE_SIZE;i++){
		for(j = 0; j < IMAGE_SIZE;j++){
			if(canvas->data[i][j] == 0){
				printf("  ");
			}
			else{
				printf("* ");
			}
		}
		printf("\n");
	}
}

void Canvas_Clear(Canvas_t* canvas)
{
	int8_t i = 0;
	int8_t j = 0;
	for(i = 0; i < IMAGE_SIZE;i++){
		for(j = 0; j < IMAGE_SIZE;j++){
			canvas->data[i][j] = 0;
		}
	}
	canvas->cursor[0] = IMAGE_SIZE / 2.0;
	canvas->cursor[1] = IMAGE_SIZE / 2.0;
}

void Canvas_Draw_From_IMU(Canvas_t* canvas)
{
	Canvas_Clear(canvas);
	canvas->scale[0] = (IMAGE_SIZE / 2.0 - 1) / canvas->scale[0];
	canvas->scale[1] = (IMAGE_SIZE / 2.0 - 1) / canvas->scale[1];

	uint8_t i = 0;
	float next_point[2] = {0,0};
	for(i = 0; i < IMU_SEQUENCE_LENGTH_MAX;i++){
		next_point[0] = canvas->cursor[0] + IMU.gyro[i][Roll] * canvas->scale[0];
		next_point[1] = canvas->cursor[1] + IMU.gyro[i][Yaw] * canvas->scale[1];
		Canvas_Draw_Line(canvas,
			roundf(canvas->cursor[0]),
			roundf(next_point[0]),
			roundf(canvas->cursor[1]),
			roundf(next_point[1])
		);
		canvas->cursor[0] = next_point[0];
		canvas->cursor[1] = next_point[1];
	}
	Canvas_Print(canvas);
	canvas->scale[0] = 0;
	canvas->scale[1] = 0;
	
}

void IMU_Init(void)
{
	MPU6050_Init();
	
	IMU.Sample_Start = &IMU_Sample_Start;
	IMU.Sample_Stop = &IMU_Sample_Stop;
}

