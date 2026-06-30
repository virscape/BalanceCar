/**
 ***************************************************************************************
 * @file    LED.c
 * @brief   板载LED驱动（PC13，低电平点亮）
 ***************************************************************************************
 */

#include "stm32f10x.h"                  // Device header

/**
 * @brief  LED初始化（PC13推挽输出，默认熄灭）
 */
void LED_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOC, GPIO_Pin_13);     /* 默认熄灭 */
}

/**
 * @brief  点亮LED（PC13输出低电平）
 */
void LED_ON(void)
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}

/**
 * @brief  熄灭LED（PC13输出高电平）
 */
void LED_OFF(void)
{
	GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

/**
 * @brief  翻转LED亮灭状态
 */
void LED_Turn(void)
{
	if (GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13) == 0)
	{
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
	}
	else
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	}
}
