/**
 ***************************************************************************************
 * @file    Key.c
 * @brief   按键驱动（4路，软件消抖）
 *          PB1→KEY1, PB0→KEY2, PA5→KEY3, PA4→KEY4
 *          上拉输入，按下为低电平
 ***************************************************************************************
 */

#include "stm32f10x.h"                  // Device header
#include "Delay.h"

uint8_t Key_Num;                        /* 当前按键编号，读后清零 */

/**
 * @brief  按键GPIO初始化（PB0/1, PA4/5上拉输入）
 */
void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
 * @brief  获取按键编号（读取后自动清零）
 * @return 0=无按键按下, 1~4=对应按键编号
 */
uint8_t Key_GetNum(void)
{
	uint8_t Temp;
	if (Key_Num)
	{
		Temp = Key_Num;
		Key_Num = 0;
		return Temp;
	}
	return 0;
}

/**
 * @brief  读取当前按下的按键编号（内部使用）
 * @return 0=无按键, 1~4=按键编号
 */
uint8_t Key_GetState(void)
{
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)
	{
		return 1;
	}
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 0)
	{
		return 2;
	}
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 0)
	{
		return 3;
	}
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) == 0)
	{
		return 4;
	}
	return 0;
}

/**
 * @brief  按键扫描消抖（每1ms调用一次，消抖时间约20ms）
 *         仅当上一次按下、当前释放时才触发（上升沿检测）
 */
void Key_Tick(void)
{
	static uint8_t Count;
	static uint8_t CurrState, PrevState;
	
	Count ++;
	if (Count >= 20)                   /* 20ms消抖周期 */
	{
		Count = 0;
		
		PrevState = CurrState;
		CurrState = Key_GetState();
		
		/* 上升沿检测：上次按下，当前释放 */
		if (CurrState == 0 && PrevState != 0)
		{
			Key_Num = PrevState;
		}
	}
}
