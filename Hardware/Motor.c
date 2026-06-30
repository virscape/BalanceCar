#include "stm32f10x.h"                  // Device header
#include "PWM.h"

/**
 * @brief  电机初始化
 *         配置方向控制GPIO（PB12~PB15推挽输出）并初始化PWM
 */
void Motor_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	PWM_Init();
}

/**
 * @brief  设置电机方向和PWM占空比
 * @param  n   电机编号：1（PB12/13+PWM_CH1）或2（PB14/15+PWM_CH2）
 * @param  PWM 有符号占空比：正=正向转，负=反向转，取值范围 -100~100
 */
void Motor_SetPWM(uint8_t n, int8_t PWM)
{
	if (n == 1)
	{
		if (PWM >= 0)
		{
			/* PWM≥0：正转 */
			GPIO_SetBits(GPIOB, GPIO_Pin_12);
			GPIO_ResetBits(GPIOB, GPIO_Pin_13);
			PWM_SetCompare1(PWM);
		}
		else
		{
			/* PWM<0：反转 */
			GPIO_ResetBits(GPIOB, GPIO_Pin_12);
			GPIO_SetBits(GPIOB, GPIO_Pin_13);
			PWM_SetCompare1(-PWM);
		}
	}
	else if (n == 2)
	{
		if (PWM >= 0)
		{
			/* PWM≥0：正转 */
			GPIO_ResetBits(GPIOB, GPIO_Pin_14);
			GPIO_SetBits(GPIOB, GPIO_Pin_15);
			PWM_SetCompare2(PWM);
		}
		else
		{
			/* PWM<0：反转 */
			GPIO_SetBits(GPIOB, GPIO_Pin_14);
			GPIO_ResetBits(GPIOB, GPIO_Pin_15);
			PWM_SetCompare2(-PWM);
		}
	}
}
