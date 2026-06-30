#ifndef __MPU6050_H
#define __MPU6050_H

/**
 * @brief  MPU6050 六轴传感器驱动（I2C通信）
 *         I2C地址：0xD0
 */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

/**
 * @brief  MPU6050初始化
 *         陀螺仪±2000dps，加速度计±16g，采样率1kHz/8=125Hz
 */
void MPU6050_Init(void);

/**
 * @brief  读取设备ID
 * @return WHO_AM_I寄存器值（应为0x68）
 */
uint8_t MPU6050_GetID(void);

/**
 * @brief  读取六轴原始数据（突发读取14字节）
 * @param  AccX/Y/Z  加速度计输出值
 * @param  GyroX/Y/Z 陀螺仪输出值
 */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

#endif
