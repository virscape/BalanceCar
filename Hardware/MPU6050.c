/**
 ***************************************************************************************
 * @file    MPU6050.c
 * @brief   MPU6050 六轴传感器驱动
 *          I2C通信，突发读取14字节获取全部数据
 *          陀螺仪量程±2000dps，加速度计量程±16g
 ***************************************************************************************
 */

#include "stm32f10x.h"                  // Device header
#include "MyI2C.h"
#include "MPU6050_Reg.h"

#define MPU6050_ADDRESS		0xD0           /* I2C从机地址（AD0=0） */

/**
 * @brief  写MPU6050寄存器
 * @param  RegAddress 寄存器地址
 * @param  Data       待写入数据
 */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(RegAddress);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(Data);
	MyI2C_ReceiveAck();
	MyI2C_Stop();
}

/**
 * @brief  读MPU6050单个寄存器
 * @param  RegAddress 寄存器地址
 * @return 寄存器值
 */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(RegAddress);
	MyI2C_ReceiveAck();
	
	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
	MyI2C_ReceiveAck();
	Data = MyI2C_ReceiveByte();
	MyI2C_SendAck(1);
	MyI2C_Stop();
	
	return Data;
}

/**
 * @brief  突发读取MPU6050多个连续寄存器
 * @param  RegAddress 起始寄存器地址
 * @param  DataArray  数据存放数组（输出）
 * @param  Count      读取字节数
 */
void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count)
{
	uint8_t i;
	
	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(RegAddress);
	MyI2C_ReceiveAck();
	
	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
	MyI2C_ReceiveAck();
	for (i = 0; i < Count; i ++)
	{
		DataArray[i] = MyI2C_ReceiveByte();
		if (i < Count - 1)
		{
			MyI2C_SendAck(0);
		}
		else
		{
			MyI2C_SendAck(1);
		}
	}
	MyI2C_Stop();
}

/**
 * @brief  MPU6050初始化配置
 *         PWR_MGMT_1=0x01：退出休眠，选择X轴陀螺时钟
 *         SMPLRT_DIV=0x07：采样率 8kHz/(1+7)=1kHz
 *         CONFIG=0x00：DLPF带宽256Hz
 *         GYRO_CONFIG=0x18：陀螺仪±2000dps
 *         ACCEL_CONFIG=0x18：加速度计±16g
 */
void MPU6050_Init(void)
{
	MyI2C_Init();
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);       /* 退出休眠 */
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);        /* 所有轴使能 */
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x07);        /* 1kHz采样 */
	MPU6050_WriteReg(MPU6050_CONFIG, 0x00);            /* DLPF=256Hz */
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);       /* ±2000dps */
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);      /* ±16g */
}

uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

//void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
//						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
//{
//	uint8_t DataH, DataL;
//	
//	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
//	*AccX = (DataH << 8) | DataL;
//	
//	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
//	*AccY = (DataH << 8) | DataL;
//	
//	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
//	*AccZ = (DataH << 8) | DataL;
//	
//	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
//	*GyroX = (DataH << 8) | DataL;
//	
//	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
//	*GyroY = (DataH << 8) | DataL;
//	
//	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
//	*GyroZ = (DataH << 8) | DataL;
//}

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t Data[14];
	
	MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, Data, 14);
	
	*AccX = (Data[0] << 8) | Data[1];
	*AccY = (Data[2] << 8) | Data[3];
	*AccZ = (Data[4] << 8) | Data[5];
	
	*GyroX = (Data[8] << 8) | Data[9];
	*GyroY = (Data[10] << 8) | Data[11];
	*GyroZ = (Data[12] << 8) | Data[13];
}
