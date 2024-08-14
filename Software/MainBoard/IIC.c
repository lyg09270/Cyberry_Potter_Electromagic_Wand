/**
 * File Name: IIC.c
 * 
 * Copyright © 2023 Civic_Crab. All rights reserved.
 * 
 * Author: Civic_Crab
 * Creation Date: Oct 1, 2020
 * Version: 1.0.0
 * 
 * Description: This file prodides IIC functions implementation
 */


/* Includes ------------------------------------------------------------------*/
#include "IIC.h"
#include "Delay.h"


/* Macros --------------------------------------------------------------------*/
// IIC1 Macros
#define IIC1_SDA(BitVal) GPIO_WriteBit(GPIOB, GPIO_Pin_7, (BitAction)BitVal);
#define IIC1_SCL(BitVal) GPIO_WriteBit(GPIOB, GPIO_Pin_6, (BitAction)BitVal);
#define IIC1_READ_SCL_PIN() GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)

// IIC2 Macros
#define IIC2_SDA(BitVal) GPIO_WriteBit(GPIOB, GPIO_Pin_11, (BitAction)BitVal);
#define IIC2_SCL(BitVal) GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)BitVal);
#define IIC2_READ_SCL_PIN() GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)

#define IIC_Delay(us) Delay_us(us)
#define IIC_DELAY_TIME 5	//200kHz

//Fuctions implementation

/**
 * @brief Initialize GPIO that software IIC1 will uses.
 * @param 
 * @param 
 * @return 
 */
void IIC1_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	// IIC1 GPIO initialization
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	IIC1_SDA(1);
	IIC1_SCL(1);
}

/**
 * @brief Initialize GPIO that software IIC2 will uses.
 * @param 
 * @param 
 * @return 
 */
void IIC2_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // IIC2 GPIO initialization
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    IIC2_SDA(1);
    IIC2_SCL(1);
}

// IIC1 functions
static void IIC1_W_SDA(uint8_t BitVal)
{
	IIC1_SDA(BitVal);
	IIC_Delay(IIC_DELAY_TIME);
}

static void IIC1_W_SCL(uint8_t BitVal)
{
	IIC1_SCL(BitVal);
	IIC_Delay(IIC_DELAY_TIME);
}

static uint8_t IIC1_R_SDA(void)
{
	uint8_t BitVal = 0;
	BitVal = IIC1_READ_SCL_PIN();
	return BitVal;
}

static void IIC1_Start(void)
{
	IIC1_W_SDA(1);
	IIC1_W_SCL(1);
	IIC1_W_SDA(0);
	IIC1_W_SCL(0);
}

static void IIC1_Stop(void)
{
	IIC1_W_SDA(0);
	IIC1_W_SCL(1);
	IIC1_W_SDA(1);
}	

static void IIC1_Send_Byte(uint8_t SendData)
{
	int i = 0;
	for (i = 0; i < 8; i++)
	{
		IIC1_W_SDA(SendData & (0x80 >> i));
		IIC1_W_SCL(1);
		IIC1_W_SCL(0);
	}
}

static uint8_t IIC1_Receive_Byte(void)
{
	uint8_t i = 0;
	uint8_t ReceivedData = 0;

	IIC1_W_SDA(1);
	for (i = 0; i < 8; i++)
	{
		IIC1_W_SCL(1);
		if (IIC1_R_SDA() == 1)
		{
		ReceivedData |= (0x80 >> i);
		}
		IIC1_W_SCL(0);
	}
	return ReceivedData;
}

static void IIC1_Send_Ack(uint8_t Ack)
{
	IIC1_W_SDA(Ack);
	IIC1_W_SCL(1);
	IIC1_W_SCL(0);
}

static uint8_t IIC1_Receive_Ack(void)
{
	uint8_t AckBit;
	IIC1_W_SDA(1);
	IIC1_W_SCL(1);
	AckBit = IIC1_R_SDA();
	IIC1_W_SCL(0);
	return AckBit;
}

int IIC1_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t const *data)
{
	int i = 0;
	IIC1_Start();
	IIC1_Send_Byte(slave_addr << 1);
	IIC1_Receive_Ack();
	IIC1_Send_Byte(reg_addr);
	IIC1_Receive_Ack();
	for (i = 0; i < length; i++)
	{
		IIC1_Send_Byte(data[i]);
		IIC1_Receive_Ack();
	}
	IIC1_Stop();
	return 0;
}

int IIC1_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data)
{
	IIC1_Start();
	IIC1_Send_Byte(slave_addr << 1);
	IIC1_Receive_Ack();
	IIC1_Send_Byte(reg_addr);
	IIC1_Receive_Ack();

	IIC1_Start();
	IIC1_Send_Byte((slave_addr << 1) + 1);
	IIC1_Receive_Ack();
	while (length)
	{
		if (length == 1)
		{
		*data = IIC1_Receive_Byte();
		IIC1_Send_Ack(1);
		}
		else
		{
		*data = IIC1_Receive_Byte();
		IIC1_Send_Ack(0);
		}
		data++;
		length--;
	}

	IIC1_Stop();
	return 0;
}

// IIC2 functions
static void IIC2_W_SDA(uint8_t BitVal)
{
	IIC2_SDA(BitVal);
	IIC_Delay(IIC_DELAY_TIME);
}

static void IIC2_W_SCL(uint8_t BitVal)
{
	IIC2_SCL(BitVal);
	IIC_Delay(IIC_DELAY_TIME);
}

static uint8_t IIC2_R_SDA(void)
{
	uint8_t BitVal = 0;
	BitVal = IIC2_READ_SCL_PIN();
	return BitVal;
}

static void IIC2_Start(void)
{
	IIC2_W_SDA(1);
	IIC2_W_SCL(1);
	IIC2_W_SDA(0);
	IIC2_W_SCL(0);
}

static void IIC2_Stop(void)
{
	IIC2_W_SDA(0);
	IIC2_W_SCL(1);
	IIC2_W_SDA(1);
}

static void IIC2_Send_Byte(uint8_t SendData)
{
	int i = 0;
	for (i = 0; i < 8; i++)
	{
		IIC2_W_SDA(SendData & (0x80 >> i));
		IIC2_W_SCL(1);
		IIC2_W_SCL(0);
	}
}

static uint8_t IIC2_Receive_Byte(void)
{
	uint8_t i = 0;
	uint8_t ReceivedData = 0;
	
	IIC2_W_SDA(1);
	for (i = 0; i < 8; i++)
	{
		IIC2_W_SCL(1);
		if (IIC2_R_SDA() == 1)
		{
		ReceivedData |= (0x80 >> i);
		}
		IIC2_W_SCL(0);
	}
	return ReceivedData;
}

static void IIC2_Send_Ack(uint8_t Ack)
{
	IIC2_W_SDA(Ack);
	IIC2_W_SCL(1);
	IIC2_W_SCL(0);
}

static uint8_t IIC2_Receive_Ack(void)
{
	uint8_t AckBit;
	IIC2_W_SDA(1);
	IIC2_W_SCL(1);
	AckBit = IIC2_R_SDA();
	IIC2_W_SCL(0);
	return AckBit;
}

int IIC2_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t const *data)
{
	int i = 0;
	IIC2_Start();
	IIC2_Send_Byte(slave_addr << 1);
	IIC2_Receive_Ack();
	IIC2_Send_Byte(reg_addr);
	IIC2_Receive_Ack();
	for (i = 0; i < length; i++)
	{
		IIC2_Send_Byte(data[i]);
		IIC2_Receive_Ack();
	}
	IIC2_Stop();
	return 0;
}

int IIC2_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data)
{
	IIC2_Start();
	IIC2_Send_Byte(slave_addr << 1);
	IIC2_Receive_Ack();
	IIC2_Send_Byte(reg_addr);
	IIC2_Receive_Ack();

	IIC2_Start();
	IIC2_Send_Byte((slave_addr << 1) + 1);
	IIC2_Receive_Ack();
	while (length)
	{
		if (length == 1)
		{
		*data = IIC2_Receive_Byte();
		IIC2_Send_Ack(1);
		}
		else
		{
		*data = IIC2_Receive_Byte();
		IIC2_Send_Ack(0);
		}
		data++;
		length--;
	}

	IIC2_Stop();
	return 0;
}
