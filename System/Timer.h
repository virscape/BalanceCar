#ifndef __TIMER_H
#define __TIMER_H

/**
 * @brief  定时器初始化
 *         配置TIM1产生1ms周期中断（72MHz / 72 / 1000 = 1kHz）
 */
void Timer_Init(void);

#endif
