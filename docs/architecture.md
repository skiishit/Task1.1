# 软件架构说明

## 控制循环

主循环基于 `HAL_GetTick()` 的 1ms 轮询（非中断触发）：

```
每 1ms:
  1. Power_HandleKeyboard()  — 键盘输入处理
  2. Power_ReadFeedback()    — ADC 采样读取 + IIR 滤波
  3. Power_ControlStep()     — PI 控制器计算 + 更新占空比

每 200ms:
  Oled_UpdateDisplay()       — LCD 显示刷新
```

## 模块划分

### 1. PI 控制器 (`main.c`)

```c
typedef struct {
  float kp;
  float ki;
  float integral;
  float out_min;
  float out_max;
} pi_ctrl_t;
```

两个独立 PI 实例：

| PI 实例 | 增益 | 用途 |
|---------|------|------|
| `g_pi_cv` | Kp=0.125, Ki=0.594 | CV 模式电压环 |
| `g_pi_cc` | Kp=0.0024, Ki=0.024 | CC 模式电流环 |

- **PI_Update()** — 条件积分（输出未饱和时积分），含输出钳位
- **PI_Reset()** — 重置积分器，设预加载值

### 2. PWM 输出 (`Power_ApplyDuty`)

- 读取 TIM1 自动重装载值计算周期
- 按占空比设置 CCR 值
- 钳位范围: `[DUTY_MIN=0.02, DUTY_MAX=0.95]`

### 3. ADC 采样 (`Power_ReadFeedback`)

- 从 DMA 缓冲区读取双通道 ADC 原始值
- 转换为实际电压/电流:
  - `Vout = V_adc × 13.0`
  - `Iout = (V_adc_i - 0.0) × 2.5`
- 一阶 IIR 低通滤波: `y[n] = y[n-1] + 0.2 × (x[n] - y[n-1])`

### 4. 键盘处理 (`Power_HandleKeyboard`)

两种交互模式：

| 模式 | 触发 | 说明 |
|------|------|------|
| 步进调整 | 按键 2/8 或 +/- | 按当前模式增加/减小设定值 |
| 编辑模式 | 按键 C | 6 位数字输入: VV.VV + I.I |
| 模式切换 | A=CV, B=CC | 切换时重置对应 PI 积分器 |

编辑模式下：
- 0-9: 输入数字
- #: 重输
- *: CANCEL
- D: CONFIRM

### 5. 显示 (`Oled_UpdateDisplay`)

LCD 显示内容（4行）：

```
Line 0: CV/CC 模式 + 电压设定值
Line 1: 电流设定值 + 电流实测值
Line 2: 电压实测值
Line 3: ADC 原始值 (CH1, CH2)
```

### 6. 串口输出 (`Uart_SendSetpoints`)

当 `g_uart_send_pending` 标志置位时，发送当前设定值：
```
Vset=05.00V Iset=1.0A
```

## 中断

- **TIM2** — 170MHz / 17000 / 10000 = **1Hz**（基础定时，当前未用于控制）
- **ADC1 DMA** — 双通道连续转换，TIM1 TRGO 触发
- **键盘列中断** — EXTI4, EXTI9_5（GPIO PD4-PD7 下降沿）

## 文件依赖关系

```
main.c
  ├── main.h              # 引脚定义、全局宏
  ├── keypad.h/.c         # 键盘驱动
  ├── lcd_driver.h/.c     # LCD SPI 驱动
  ├── lcd_fonts.h/.c      # LCD 字库
  ├── stm32g4xx_hal.h     # HAL 库
  └── stm32g4xx_it.h/.c   # 中断服务
```

## 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| CTRL_PERIOD_MS | 1 | 控制周期 1ms |
| PWM 频率 | 100kHz | TIM1 Period=1699, Prescaler=0 |
| ADC 触发 | TIM1 TRGO=OC1REF | PWM 上升沿触发采样 |
| IIR_ALPHA | 0.2 | 滤波系数 |
| DUTY_MIN | 0.02 (2%) | 最小占空比 |
| DUTY_MAX | 0.95 (95%) | 最大占空比 |
| 死区 | 141ns | DeadTime=24 |
