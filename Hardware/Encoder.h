#ifndef __ENCODER_H
#define __ENCODER_H

/**
 * @brief  编码器模块
 *         TIM3（PA6/PA7）→ 电机1编码器
 *         TIM4（PB6/PB7）→ 电机2编码器
 *         使用定时器编码器接口模式（TI12，4倍频）
 */
void Encoder_Init(void);

/**
 * @brief  获取编码器计数值并清零
 * @param  n 编码器编号：1=TIM3, 2=TIM4
 * @return 本次采样周期内的计数值（有符号）
 */
int16_t Encoder_Get(uint8_t n);

#endif
