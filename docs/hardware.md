# 硬件设计说明

## MCU

| 项目 | 参数 |
|------|------|
| 型号 | STM32G474VETx |
| 封装 | LQFP100 |
| 系统时钟 | HSI 16MHz → PLL (16/4×85/2) = **170MHz** |
| Flash | 512KB |
| SRAM | 128KB |

## 引脚分配

### TIM1 — PWM 输出（互补，Buck 开关控制）

| 引脚 | 功能 | 说明 |
|------|------|------|
| PA8 | TIM1_CH1 | 上管 PWM (高侧) |
| PA7 (推测) | TIM1_CH1N | 下管 PWM (低侧互补) |

PWM 频率: 170MHz / (0+1) / (1699+1) = **100kHz**
死区时间: DeadTime=24, t_dts=5.88ns → DT ≈ **141ns**

### ADC1 — 电压/电流采样

| 通道 | 引脚 | 信号 | 说明 |
|------|------|------|------|
| ADC1_IN1 | PA0 | Vout 反馈 | 经分压电阻（比例 13:1） |
| ADC1_IN2 | PA1 | Iout 反馈 | 经电流检测放大器（2.5A/V） |

ADC 触发源: TIM1 TRGO (OC1REF) — 在 PWM 上升沿触发采样

### SPI1 — LCD 显示

| 引脚 | 功能 |
|------|------|
| PA5 | SPI1_SCK |
| PA6 | SPI1_MISO |
| PA7 | SPI1_MOSI |
| PB0 | LCD_BL (背光) |
| PB1 | LCD_DC (数据/命令) |
| PB2 | LCD_CS (片选) |

### 矩阵键盘 (4x4)

| 引脚 | 功能 |
|------|------|
| PD0-PD3 | 行输出: ROW1~ROW4 |
| PD4-PD7 | 列输入: COL1~COL4 (外部中断) |

### USART1 — 串口调试

| 引脚 | 功能 |
|------|------|
| PA9 | USART1_TX |
| PA10 | USART1_RX |

波特率: **115200**, 8N1

### 其他

| 引脚 | 功能 |
|------|------|
| PC0 | USER_KEY (用户按键，调试用) |

## Buck 功率级参数

| 参数 | 值 |
|------|-----|
| 输入电压 Vin | 待测量（原理图补充） |
| 输出电压 Vout | 2.00V ~ 33.00V (CV) |
| 输出电流 Iout | 0.1A ~ 2.0A (CC) |
| 开关频率 fsw | 100kHz |
| 电压采样分压比 | 13:1 (VOLTAGE_SCALE=13.0) |
| 电流采样增益 | 2.5A/V (CURRENT_SCALE_A_PER_V=2.5) |

## 注意事项

1. 电压采样分压比 VOLTAGE_SCALE=13.0 需要实际标定确认
2. 电流采样偏置 CURRENT_OFFSET_V=0.0，可能需要实测校准
3. 死区 141ns 较保守，建议用示波器实测 V_gs 波形后调整
4. OSSR/OSSI 当前配置为 DISABLE（MOE=0 时输出浮空），存在安全风险，建议改为 ENABLE
5. Break 输入未使能（TIM_BREAK_DISABLE），缺少硬件过流保护
