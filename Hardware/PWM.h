#ifndef __PWM_H
#define __PWM_H

/**
 * @brief  两路PWM输出初始化（TIM2 CH1/CH2 → PA0/PA1）
 *         频率：72MHz / 36 / 100 = 20kHz
 */
void PWM_Init(void);
void PWM_SetCompare1(uint16_t Compare);
void PWM_SetCompare2(uint16_t Compare);

#endif
