#ifndef __LED_H
#define __LED_H

/**
 * @brief  LED驱动模块
 *         PC13控制板载LED，低电平点亮
 */
void LED_Init(void);
void LED_ON(void);
void LED_OFF(void);

/**
 * @brief  翻转LED状态
 */
void LED_Turn(void);

#endif
