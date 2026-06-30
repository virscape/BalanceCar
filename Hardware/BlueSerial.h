#ifndef __BLUE_SERIAL_H
#define __BLUE_SERIAL_H

#include <stdio.h>

extern char BlueSerial_RxPacket[];      /* 接收数据包缓冲区 */
extern uint8_t BlueSerial_RxFlag;       /* 接收完成标志 */

/**
 * @brief  蓝牙串口驱动（USART2→PA2/TX, PA3/RX）
 *         波特率9600，8N1，接收格式：[数据包内容]
 */
void BlueSerial_Init(void);
void BlueSerial_SendByte(uint8_t Byte);
void BlueSerial_SendArray(uint8_t *Array, uint16_t Length);
void BlueSerial_SendString(char *String);
void BlueSerial_SendNumber(uint32_t Number, uint8_t Length);
void BlueSerial_Printf(char *format, ...);

#endif
