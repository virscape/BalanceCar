#ifndef __PID_H
#define __PID_H

/**
 * @brief  PID控制器结构体
 *         支持位置式PID算法，带积分限幅、输出限幅和静态偏置
 *         微分项采用"微分先行"方式（对实际值微分，避免设定值突变冲击）
 */
typedef struct {
	float Target;         /* 目标值 */
	float Actual;         /* 当前实际值 */
	float Actual1;        /* 上一拍实际值（微分先行用） */
	float Out;            /* PID输出 */
	
	float Kp;             /* 比例系数 */
	float Ki;             /* 积分系数 */
	float Kd;             /* 微分系数 */
	
	float Error0;         /* 当前误差 */
	float Error1;         /* 上一拍误差 */
	float ErrorInt;       /* 误差积分累计 */
	
	float ErrorIntMax;    /* 积分限幅上限 */
	float ErrorIntMin;    /* 积分限幅下限 */
	
	float OutMax;         /* 输出上限 */
	float OutMin;         /* 输出下限 */
	
	float OutOffset;      /* 输出静态偏置（叠加在输出上） */
} PID_t;

void PID_Init(PID_t *p);
void PID_Update(PID_t *p);

#endif
