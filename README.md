# 自平衡小车（Two-Wheeled Self-Balancing Robot）

基于 **STM32F103C8** 的两轮自平衡小车，采用串级 PID 控制，支持 NRF24L01 遥控和蓝牙 App 调参。

---

## 硬件架构


| 外设 | 接口 | 引脚 | 说明 |
|------|------|------|------|
| MPU6050 | I2C (软件模拟) | SCL→PB10, SDA→PB11 | 六轴姿态传感器 |
| OLED 0.96" | I2C (软件模拟) | SCL→PB8, SDA→PB9 | 128×64 参数显示 |
| 电机驱动 | GPIO + TIM2 PWM | PB12~15 (方向), PA0/1 (PWM) | TB6612 / L298N 等 H桥 |
| 编码器×2 | TIM3/4 编码器模式 | PA6/7, PB6/7 | 霍尔编码器 4倍频 |
| NRF24L01 | SPI (软件模拟) | CE→PA8, CSN→PA15, SCK→PB3, MOSI→PB5, MISO→PB4 | 2.4GHz 遥控+遥测 |
| 蓝牙 | USART2 | TX→PA2, RX→PA3 | HC-05/HC-06 调参 |
| 按键 | GPIO | PB0/1, PA4/5 | 4路，上拉输入 |
| LED | GPIO | PC13 | 运行状态指示 |

---

## 控制算法

### 串级 PID 结构

```
     遥控/蓝牙 ──▶ 速度目标 ──▶ [速度环PID] ──▶ 角度目标 ──▶ [角度环PID] ──▶ 电机PWM
                   ▲                    │                   ▲
                   │                    │                   │
              平均轮速 ◀── 编码器        └── 目标角度 ◀── 速度环输出
                                             │
     遥控/蓝牙 ──▶ 转向目标 ──▶ [转向环PID] ──▶ 差速PWM ──▶ 叠加到左右电机
                   ▲
                   │
              轮速差 ◀── 编码器
```

### PID 参数（默认值）

| 参数 | 角度环 | 速度环 | 转向环 |
|------|--------|--------|--------|
| Kp | 5.0 | 2.0 | 4.0 |
| Ki | 0.1 | 0.05 | 3.0 |
| Kd | 5.0 | 0 | 0 |
| 输出限幅 | ±100 | ±20 | ±50 |
| OutOffset | 3 | — | — |

- **角度环**（直立环）：10ms 周期，使用互补滤波（加速度计 0.01 + 陀螺仪 0.99）
- **速度环**：50ms 周期，输出作为角度环的目标偏置
- **转向环**：50ms 周期，产生左右轮差速

### 角度融合

- 加速度计角度：`atan2(AX, AZ)` 转换
- 陀螺仪积分角度：角速率 / 32768 × 2000dps × dt
- 互补滤波：`Angle = 0.01 × AngleAcc + 0.99 × AngleGyro`
- 安全保护：倾斜超过 ±50° 自动停机

---

## 遥控通信

### NRF24L01 协议

| 方向 | 格式 | 说明 |
|------|------|------|
| 遥控→小车 | `[ID, LH, LV, RH, RV, KEY]` | ID=0x00/0x01, 摇杆值 -127~127 |
| 小车→遥控 | `[0x02, LeftPWM, RightPWM, Angle, LeftSpeed, RightSpeed]` | 遥测回传（ID=0x01 时） |

- 速度目标 = LV / 25.0（范围 ±5）
- 转向目标 = RH / 25.0（范围 ±5）
- KEY=1 启停

### 蓝牙串口协议

数据包格式：`[tag,param1,param2,...]`

| Tag | 参数 | 说明 |
|-----|------|------|
| `slider` | `Name, Value` | 滑块调参，Name 支持 `AngleKp/Ki/Kd`, `SpeedKp/Ki/Kd`, `TurnKp/Ki/Kd`, `Offset` |
| `joystick` | `LH, LV, RH, RV` | 虚拟摇杆遥控 |
| `key` | `Name, Action` | 按键事件（预留） |

---

## 项目结构

```
├── User/
│   ├── main.c                  # 主程序（平衡控制、OLED显示、通信解析）
│   ├── PID.c / PID.h           # PID 控制器（位置式，微分先行）
│   ├── stm32f10x_conf.h        # 标准外设库配置
│   ├── stm32f10x_it.c/h        # 中断向量模板
│
├── Hardware/
│   ├── MPU6050.c/h              # MPU6050 六轴传感器驱动
│   ├── MPU6050_Reg.h            # MPU6050 寄存器宏定义
│   ├── MyI2C.c/h                # 软件模拟 I2C 主机
│   ├── Motor.c/h                # 电机驱动（方向+PWM）
│   ├── PWM.c/h                  # TIM2 两路 PWM 输出
│   ├── Encoder.c/h              # TIM3/4 编码器接口
│   ├── OLED.c/h                 # OLED 128×64 显示驱动
│   ├── OLED_Data.c/h            # OLED 字模+图像数据
│   ├── NRF24L01.c/h             # NRF24L01 2.4G 无线模块
│   ├── NRF24L01_Define.h        # NRF24L01 指令/寄存器宏
│   ├── Serial.c/h               # USART1 调试串口
│   ├── BlueSerial.c/h           # USART2 蓝牙串口
│   ├── Key.c/h                  # 4路按键驱动
│   ├── LED.c/h                  # 板载 LED 驱动
│
├── System/
│   ├── Timer.c/h                # TIM1 1ms 系统定时器
│   ├── Delay.c/h                # 微秒/毫秒/秒级延时
│
├── Library/                     # STM32F10x 标准外设库
├── Start/                       # 启动文件 + CMSIS
└── Project.uvprojx              # Keil MDK-ARM 工程文件
```

---

## 开发环境

| 工具 | 版本 |
|------|------|
| 开发板 | STM32F103C8（Blue Pill / 最小系统板） |
| IDE | Keil MDK-ARM V5 |
| 编译器 | ARMCC V5.06 |
| 标准库 | STM32F10x StdPeriph Library V3.5.0 |
| 调试器 | ST-Link / J-Link（SWD 接口） |

## 使用说明

1. **烧录**：用 Keil 打开 `Project.uvprojx`，编译后通过 ST-Link 下载
2. **启动**：上电后小车进入待机状态（LED 灭）
3. **按键 1**：启停平衡控制（LED 亮 = 运行中）
4. **遥控**：通过 NRF24L01 遥控器控制前进后退和转向
5. **蓝牙调参**：手机 App 连接 HC-05，通过 `[slider,...]` 指令实时调整 PID 参数

---

