#include "stm32f10x.h"    
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "MPU6050.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "BlueSerial.h"
#include "PID.h"
#include "NRF24L01.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* MPU6050 原始数据 */
int16_t AX, AY, AZ, GX, GY, GZ;

/* 定时器状态 */
uint8_t TimerErrorFlag;
uint16_t TimerCount;

/* 角度融合数据 */
float AngleAcc;       /* 加速度计解算角度 */
float AngleGyro;      /* 陀螺仪积分角度 */
float Angle;           /* 互补滤波后最终角度 */

uint8_t KeyNum;     /* 按键编号 */
uint8_t RunFlag;     /* 平衡运行标志：1=运行, 0=停止 */

/* 电机 PWM 输出 */
int16_t LeftPWM, RightPWM;
int16_t AvePWM;      
int16_t DifPWM;     

/* 轮速数据 */
float LeftSpeed, RightSpeed;
float AveSpeed;      /* 平均速度，作为速度环反馈 */
float DifSpeed;      /* 差速，作为转向环反馈 */

/* 串级PID控制器参数 */
/* 角度环（直立环）：输入=当前角度，输出=电机PWM */
PID_t AnglePID = {
	.Kp = 5,
	.Ki = 0.1,
	.Kd = 5,
	
	.OutMax = 100,
	.OutMin = -100,
	
	.OutOffset = 3,           /* 静态偏置补偿 */
	
	.ErrorIntMax = 600,
	.ErrorIntMin = -600,
};

/* 速度环：输入=平均轮速，输出=角度环目标值 */
PID_t SpeedPID = {
	.Kp = 2,
	.Ki = 0.05,
	.Kd = 0,
	
	.OutMax = 20,
	.OutMin = -20,
	
	.ErrorIntMax = 150,
	.ErrorIntMin = -150,
};

/* 转向环（Z轴旋转）：输入=轮速差，输出=差速PWM */
PID_t TurnPID = {
	.Kp = 4,
	.Ki = 3,
	.Kd = 0,
	
	.OutMax = 50,
	.OutMin = -50,
	
	.ErrorIntMax = 20,
	.ErrorIntMin = -20,
};

/**
  * @brief  主函数
  *         初始化所有外设后，循环处理：
  *         - OLED显示PID参数和状态
  *         - 按键启停
  *         - NRF24L01遥控数据解析
  *         - 蓝牙串口调参/遥控
  * @retval 无（永不返回）
  */
int main(void)
{
	/* 外设初始化 */
	OLED_Init();
	MPU6050_Init();
	BlueSerial_Init();
	LED_Init();
	Key_Init();
	Motor_Init();
	Encoder_Init();
	Serial_Init();
	NRF24L01_Init();
	
	Timer_Init();
	
	while (1)
	{
		if (RunFlag) {LED_ON();} else {LED_OFF();}
		
		KeyNum = Key_GetNum();
		if (KeyNum == 1)
		{
			if (RunFlag == 0)
			{
				PID_Init(&AnglePID);
				PID_Init(&SpeedPID);
				PID_Init(&TurnPID);
				RunFlag = 1;
			}
			else
			{
				RunFlag = 0;
			}
		}
		
		OLED_Clear();
		OLED_Printf(0, 0, OLED_6X8, "  Angle");
		OLED_Printf(0, 8, OLED_6X8, "P:%05.2f", AnglePID.Kp);
		OLED_Printf(0, 16, OLED_6X8, "I:%05.2f", AnglePID.Ki);
		OLED_Printf(0, 24, OLED_6X8, "D:%05.2f", AnglePID.Kd);
		OLED_Printf(0, 32, OLED_6X8, "T:%+05.1f", AnglePID.Target);
		OLED_Printf(0, 40, OLED_6X8, "A:%+05.1f", Angle);
		OLED_Printf(0, 48, OLED_6X8, "O:%+05.0f", AnglePID.Out);
		OLED_Printf(0, 56, OLED_6X8, "GY:%+05d", GY);
		OLED_Printf(56, 56, OLED_6X8, "Offset:%02.0f", AnglePID.OutOffset);
		OLED_Printf(50, 0, OLED_6X8, "Speed");
		OLED_Printf(50, 8, OLED_6X8, "%05.2f", SpeedPID.Kp);
		OLED_Printf(50, 16, OLED_6X8, "%05.2f", SpeedPID.Ki);
		OLED_Printf(50, 24, OLED_6X8, "%05.2f", SpeedPID.Kd);
		OLED_Printf(50, 32, OLED_6X8, "%+05.1f", SpeedPID.Target);
		OLED_Printf(50, 40, OLED_6X8, "%+05.1f", AveSpeed);
		OLED_Printf(50, 48, OLED_6X8, "%+05.0f", SpeedPID.Out);
		OLED_Printf(88, 0, OLED_6X8, "Turn");
		OLED_Printf(88, 8, OLED_6X8, "%05.2f", TurnPID.Kp);
		OLED_Printf(88, 16, OLED_6X8, "%05.2f", TurnPID.Ki);
		OLED_Printf(88, 24, OLED_6X8, "%05.2f", TurnPID.Kd);
		OLED_Printf(88, 32, OLED_6X8, "%+05.1f", TurnPID.Target);
		OLED_Printf(88, 40, OLED_6X8, "%+05.1f", DifSpeed);
		OLED_Printf(88, 48, OLED_6X8, "%+05.0f", TurnPID.Out);
		OLED_Update();
		
		/* NRF24L01 遥控接收与遥测回传 */
		if (NRF24L01_Receive() == 1)
		{
			uint8_t ID = NRF24L01_RxPacket[0];
			
			if (ID == 0x00 || ID == 0x01)
			{
				if (ID == 0x01)
				{
					NRF24L01_TxPacket[0] = 0x02;
					NRF24L01_TxPacket[1] = (int8_t)LeftPWM;
					NRF24L01_TxPacket[2] = (int8_t)RightPWM;
					*(float *)&NRF24L01_TxPacket[4] = Angle;			//4 5 6 7
					*(float *)&NRF24L01_TxPacket[8] = LeftSpeed;		//8 9 10 11
					*(float *)&NRF24L01_TxPacket[12] = RightSpeed;		//12 13 14 15
					
					NRF24L01_Send();
				}
				
//				int8_t LH = NRF24L01_RxPacket[1];
				int8_t LV = NRF24L01_RxPacket[2];
				int8_t RH = NRF24L01_RxPacket[3];
//				int8_t RV = NRF24L01_RxPacket[4];
				uint8_t KEY = NRF24L01_RxPacket[5];
				
				SpeedPID.Target = LV / 25.0;
				TurnPID.Target = RH / 25.0;
				
				if (KEY == 1)
				{
					if (RunFlag == 0)
					{
						PID_Init(&AnglePID);
						PID_Init(&SpeedPID);
						PID_Init(&TurnPID);
						RunFlag = 1;
					}
					else
					{
						RunFlag = 0;
					}
				}
			}
			
		}
		
		/* 蓝牙串口指令解析（调参/遥控） */
		if (BlueSerial_RxFlag == 1)
		{
			char *Tag = strtok(BlueSerial_RxPacket, ",");
			if (strcmp(Tag, "key") == 0)
			{
				char *Name = strtok(NULL, ",");
				char *Action = strtok(NULL, ",");
				
			}
			else if (strcmp(Tag, "slider") == 0)
			{
				char *Name = strtok(NULL, ",");
				char *Value = strtok(NULL, ",");
				
				if (strcmp(Name, "AngleKp") == 0)
				{
					AnglePID.Kp = atof(Value);
				}
				else if (strcmp(Name, "AngleKi") == 0)
				{
					AnglePID.Ki = atof(Value);
				}
				else if (strcmp(Name, "AngleKd") == 0)
				{
					AnglePID.Kd = atof(Value);
				}
				else if (strcmp(Name, "SpeedKp") == 0)
				{
					SpeedPID.Kp = atof(Value);
				}
				else if (strcmp(Name, "SpeedKi") == 0)
				{
					SpeedPID.Ki = atof(Value);
				}
				else if (strcmp(Name, "SpeedKd") == 0)
				{
					SpeedPID.Kd = atof(Value);
				}
				else if (strcmp(Name, "TurnKp") == 0)
				{
					TurnPID.Kp = atof(Value);
				}
				else if (strcmp(Name, "TurnKi") == 0)
				{
					TurnPID.Ki = atof(Value);
				}
				else if (strcmp(Name, "TurnKd") == 0)
				{
					TurnPID.Kd = atof(Value);
				}
				else if (strcmp(Name, "Offset") == 0)
				{
					AnglePID.OutOffset = atof(Value);
				}
			}
			else if (strcmp(Tag, "joystick") == 0)
			{
				int8_t LH = atoi(strtok(NULL, ","));
				int8_t LV = atoi(strtok(NULL, ","));
				int8_t RH = atoi(strtok(NULL, ","));
				int8_t RV = atoi(strtok(NULL, ","));
				
				SpeedPID.Target = LV / 25.0;
				TurnPID.Target = RH / 25.0;
			}
			
			BlueSerial_RxFlag = 0;
		}
		
//		BlueSerial_Printf("[plot,%f,%f,%f]", AnglePID.ErrorInt, SpeedPID.ErrorInt, TurnPID.ErrorInt);
	}
}

/**
  * @brief  TIM1更新中断服务函数（1ms周期）
  *         完成以下实时任务：
  *         - 按键定时扫描
  *         - 10ms：读取MPU6050 → 角度融合 → 角度环PID → 电机输出
  *         - 50ms：读取编码器 → 计算轮速 → 速度环+转向环PID
  *         - 检测定时器错误
  */
void TIM1_UP_IRQHandler(void)
{
	static uint16_t Count0, Count1;   /* 10ms / 50ms 分频计数器 */
	
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		
		Key_Tick();
		
		/* 10ms 任务：角度环控制 */
		Count0 ++;
		if (Count0 >= 10)
		{
			Count0 = 0;
			
			/* 读取MPU6050六轴数据 */
			MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
			
			/* 陀螺仪Y轴偏置校准 */
			GY -= 16;
			
			/* 加速度计解算角度：atan2(AX, AZ) 转换为角度 */
			AngleAcc = -atan2(AX, AZ) / 3.14159 * 180;
			
			/* 角度微调偏置 */
			AngleAcc += 0.5;
			
			/* 陀螺仪积分角度：角速率 / 满量程 * 量程 * 时间 */
			AngleGyro = Angle + GY / 32768.0 * 2000 * 0.01;
			
			/* 互补滤波：加速度计权重0.01，陀螺仪权重0.99 */
			float Alpha = 0.01;
			Angle = Alpha * AngleAcc + (1 - Alpha) * AngleGyro;
			
			/* 角度超过±50度则停止运行（防倾倒保护） */
			if (Angle > 50 || Angle < -50)
			{
				RunFlag = 0;
			}
			
			if (RunFlag)
			{
				/* 角度环PID计算 → 占空比输出 */
				AnglePID.Actual = Angle;
				PID_Update(&AnglePID);
				AvePWM = -AnglePID.Out;
				
				/* 叠加差速得到左右轮占空比 */
				LeftPWM = AvePWM + DifPWM / 2;
				RightPWM = AvePWM - DifPWM / 2;
				
				/* PWM限幅 ±100 */
				if (LeftPWM > 100) {LeftPWM = 100;} else if (LeftPWM < -100) {LeftPWM = -100;}
				if (RightPWM > 100) {RightPWM = 100;} else if (RightPWM < -100) {RightPWM = -100;}
				
				Motor_SetPWM(1, LeftPWM);
				Motor_SetPWM(2, RightPWM);
			}
			else
			{
				/* 停机：PWM置零 */
				Motor_SetPWM(1, 0);
				Motor_SetPWM(2, 0);
			}
		}
		
		/* 50ms 任务：速度环 + 转向环 */
		Count1 ++;
		if (Count1 >= 50)
		{
			Count1 = 0;
			
			/* 编码器读数转轮速（rps）：计数值 / 线数 / 采样时间 / 减速比 */
			LeftSpeed = Encoder_Get(1) / 44.0 / 0.05 / 9.27666;
			RightSpeed = Encoder_Get(2) / 44.0 / 0.05 / 9.27666;
			
			AveSpeed = (LeftSpeed + RightSpeed) / 2.0;
			DifSpeed = LeftSpeed - RightSpeed;
			
			if (RunFlag)
			{
				/* 速度环PID → 角度环目标 */
				SpeedPID.Actual = AveSpeed;
				PID_Update(&SpeedPID);
				AnglePID.Target = SpeedPID.Out;
				
				/* 转向环PID → 差速PWM */
				TurnPID.Actual = DifSpeed;
				PID_Update(&TurnPID);
				DifPWM = TurnPID.Out;
			}
		}
		
		/* 定时器错误检测 */
		if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
		{
			TimerErrorFlag = 1;
			TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		}
		TimerCount = TIM_GetCounter(TIM1);
	}
}
