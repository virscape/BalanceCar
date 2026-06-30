#ifndef __MOTOR_H
#define __MOTOR_H

/**
 * @brief  电机驱动模块
 *         通过GPIO控制H桥方向 + TIM2 PWM控制转速
 *         GPIO：PB12/13→电机1方向，PB14/15→电机2方向
 */
void Motor_Init(void);

/**
 * @brief  设置电机PWM（含方向）
 * @param  n   电机编号：1或2
 * @param  PWM 有符号占空比：-100~100，正=前进，负=后退
 */
void Motor_SetPWM(uint8_t n, int8_t PWM);

#endif
