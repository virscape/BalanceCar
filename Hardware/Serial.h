#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>

/**
 * @brief  串口1驱动（USART1→PA9/TX, PA10/RX）
 *         波特率9600，8N1，接收使用中断（单字节）
 */
void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

/**
 * @brief  获取接收标志（读取后自动清零）
 * @return 1=有新数据, 0=无
 */
uint8_t Serial_GetRxFlag(void);

/**
 * @brief  获取接收到的字节数据
 * @return 接收字节
 */
uint8_t Serial_GetRxData(void);

#endif
