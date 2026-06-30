#include "stm32f10x.h"                  // Device header
#include "PID.h"

/**
 * @brief  PID控制器初始化
 *         将PID结构体内所有状态变量清零
 * @param  p PID结构体指针
 * @retval 无
 */
void PID_Init(PID_t *p)
{
	p->Target = 0;
	p->Actual = 0;
	p->Actual1 = 0;
	p->Out = 0;
	p->Error0 = 0;
	p->Error1 = 0;
	p->ErrorInt = 0;
}

/**
 * @brief  PID控制器更新计算
 *         位置式PID：Out = Kp*Err0 + Ki*ΣErr + Kd*(Actual1-Actual)
 *         包含积分分离（Ki==0时清零积分）、积分限幅、
 *         微分先行、静态偏置补偿和输出限幅
 * @param  p PID结构体指针（需提前设置Target和Actual）
 * @retval 无
 */
void PID_Update(PID_t *p)
{
	/* 更新误差 */
	p->Error1 = p->Error0;
	p->Error0 = p->Target - p->Actual;
	
	/* 积分项计算（带限幅） */
	if (p->Ki != 0)
	{
		p->ErrorInt += p->Error0;
		
		if (p->ErrorInt > p->ErrorIntMax) {p->ErrorInt = p->ErrorIntMax;}
		if (p->ErrorInt < p->ErrorIntMin) {p->ErrorInt = p->ErrorIntMin;}
	}
	else
	{
		p->ErrorInt = 0;
	}
	
	/* 位置式PID：比例 + 积分 + 微分先行 */
	p->Out = p->Kp * p->Error0
		   + p->Ki * p->ErrorInt
//		   + p->Kd * (p->Error0 - p->Error1);
		   - p->Kd * (p->Actual - p->Actual1);
	
	/* 静态偏置补偿 */
	if (p->Out > 0) {p->Out += p->OutOffset;}
	if (p->Out < 0) {p->Out -= p->OutOffset;}
	
	/* 输出限幅 */
	if (p->Out > p->OutMax) {p->Out = p->OutMax;}
	if (p->Out < p->OutMin) {p->Out = p->OutMin;}
	
	/* 保存当前实际值供下一拍微分使用 */
	p->Actual1 = p->Actual;
}
