/**
 ***************************************************************************************
 * @file    MyI2C.c
 * @brief   软件模拟I2C主机驱动
 *          SCL→PB10, SDA→PB11，开漏输出模式
 *          用于MPU6050通信
 ***************************************************************************************
 */

#include "stm32f10x.h"                  // Device header
#include "Delay.h"

/**
 * @brief  写SCL引脚电平
 * @param  BitValue 0=低, 1=高
 */
void MyI2C_W_SCL(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)BitValue);
//	Delay_us(10);
}

/**
 * @brief  写SDA引脚电平
 * @param  BitValue 0=低, 1=高
 */
void MyI2C_W_SDA(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_11, (BitAction)BitValue);
//	Delay_us(10);
}

/**
 * @brief  读SDA引脚电平
 * @return 0=低, 1=高
 */
uint8_t MyI2C_R_SDA(void)
{
	uint8_t BitValue;
	BitValue = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
//	Delay_us(10);
	return BitValue;
}

/**
 * @brief  I2C初始化
 *         配置PB10/PB11开漏输出并释放总线
 */
void MyI2C_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);
}

/**
 * @brief  I2C起始信号
 *         SCL高电平期间SDA下降沿
 */
void MyI2C_Start(void)
{
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(0);
}

/**
 * @brief  I2C停止信号
 *         SCL高电平期间SDA上升沿
 */
void MyI2C_Stop(void)
{
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(1);
}

/**
 * @brief  I2C发送一个字节（高位先行）
 * @param  Byte 待发送字节
 */
void MyI2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i ++)
	{
		MyI2C_W_SDA(!!(Byte & (0x80 >> i)));
		MyI2C_W_SCL(1);
		MyI2C_W_SCL(0);
	}
}

/**
 * @brief  I2C接收一个字节（高位先行）
 * @return 接收到的字节
 */
uint8_t MyI2C_ReceiveByte(void)
{
	uint8_t i, Byte = 0x00;
	MyI2C_W_SDA(1);     /* 释放SDA，由从机驱动 */
	for (i = 0; i < 8; i ++)
	{
		MyI2C_W_SCL(1);
		if (MyI2C_R_SDA()){Byte |= (0x80 >> i);}
		MyI2C_W_SCL(0);
	}
	return Byte;
}

/**
 * @brief  I2C发送应答位
 * @param  AckBit 0=ACK, 1=NACK
 */
void MyI2C_SendAck(uint8_t AckBit)
{
	MyI2C_W_SDA(AckBit);
	MyI2C_W_SCL(1);
	MyI2C_W_SCL(0);
}

/**
 * @brief  I2C接收从机应答位
 * @return 0=ACK, 1=NACK
 */
uint8_t MyI2C_ReceiveAck(void)
{
	uint8_t AckBit;
	MyI2C_W_SDA(1);     /* 释放SDA */
	MyI2C_W_SCL(1);
	AckBit = MyI2C_R_SDA();
	MyI2C_W_SCL(0);
	return AckBit;
}
