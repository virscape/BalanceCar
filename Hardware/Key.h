#ifndef __KEY_H
#define __KEY_H

/**
 * @brief  按键驱动模块
 *         4路按键：PB1→KEY1, PB0→KEY2, PA5→KEY3, PA4→KEY4
 *         上拉输入，按下为低电平，含软件消抖
 */
void Key_Init(void);

/**
 * @brief  获取按键编号（读取后清零）
 * @return 0=无按键, 1~4=按键编号
 */
uint8_t Key_GetNum(void);

/**
 * @brief  按键扫描状态机（需在定时中断中周期性调用）
 */
void Key_Tick(void);

#endif
