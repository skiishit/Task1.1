# 审查指南

## 功能清单

- [x] CV 模式恒压控制（2.00V~33.00V）
- [x] CC 模式恒流控制（0.1A~2.0A）
- [x] 矩阵键盘输入（步进 + 数字编辑）
- [x] SPI LCD 显示
- [x] USART 串口输出设定值
- [x] ADC DMA 双通道采样+IIR 滤波
- [x] PWM 100kHz 互补输出 + 死区
- [x] 软启动（100ms斜坡）
- [x] ADC电压前100次平均滤波

## 安全审查（对照 power-converter-firmware skill）

| # | 检查项 | 当前状态 | 结论 |
|---|--------|---------|------|
| 1 | 软启动 | ✅ 已实现。PWM启动前写入2%，100ms斜坡到PI目标 | **已修复** |
| 2 | 死区时间 | ⚠️ DeadTime=24 → 141ns（保守，需实测确认） | 待验证 |
| 3 | OSSR/OSSI | ❌ DISABLE — MOE=0 时输出 Hi-Z 浮空 | **需改为 ENABLE** |
| 4 | Break 硬件过流 | ❌ TIM_BREAK_DISABLE | **需使能** |
| 5 | ADC 采样时机 | ❌ TRGO=OC1REF — 在 PWM 开通边沿采样 | **建议改为 UPDATE** |
| 6 | 软件 OVP | ❌ 未实现 | **需添加** |
| 7 | PI 抗饱和 | ⚠️ 条件积分（输出饱和时冻结），大幅阶跃时积分卡死 | 建议反计算抗饱和 |
| 8 | PWM 启动前设最小占空比 | ✅ Power_ApplyDuty(DUTY_MIN)在PWM启动前执行 | **已修复** |
| 9 | ADC 电压初值滤波 | ✅ 前100次累加平均，之后切IIR | **已实现** |
| 10 | 设定值平滑变化 | ✅ VOUT_SLEW_RATE_CV=4U(0.04V/1ms=40V/s) | **已实现** |

## 待确认事项

1. 电压采样分压比 VOLTAGE_SCALE=13.0 是否需要实际标定？
2. 电流采样偏置 CURRENT_OFFSET_V=0.0 是否已校准？
3. 硬件上过流比较器输出连接到哪个 BKIN 引脚？
4. 功率级参数（输入电压、电感值、输出电容）需补充
5. 死区 141ns 是否满足实际 MOSFET+驱动器要求？

## 修改记录

| 日期 | 文件 | 修改内容 |
|------|------|---------|
| 2026-07-06 | — | 初始代码加载（commit b0cd25f） |
| 2026-07-06 | Core/Src/main.c | 软启动实现：新增#define/变量/启动序列/控制循环斜坡 |
| 2026-07-06 | Core/Src/main.c | ADC前100次平均滤波：Power_ReadFeedback增加累加平均逻辑 |
| 2026-07-06 | Core/Src/main.c | 初始占空比0.5→DUTY_MIN，PI积分0.0→DUTY_MIN |

## 实现细节说明

### 1. ADC 电压滤波 — 100点滑动平均 + IIR 叠加

**目标：** 持续对电压采样做 100 点滑动平均，再在其上叠加 IIR 滤波，双重平滑。
窗口未满时直接用已有数据计算平均值，无除零风险。

**涉及代码：**

| 位置 | 行号 | 说明 |
|------|------|------|
| 宏定义 | L72 | `#define ADC_MA_WINDOW (100U)` — 滑动窗口大小 |
| 环形缓冲 | L143-L146 | `g_vout_ma_buf[100]` 存放最近 100 个 vout 值；`g_vout_ma_idx` 当前写入位置；`g_vout_ma_sum` 运行和；`g_vout_ma_cnt` 已采集点数 |
| 函数体 | L237-L252 | `Power_ReadFeedback` 内滑动平均+IIR 逻辑 |

**流程：**
```
L239: g_vout_ma_sum += g_vout_real          // 新值加入总和
L240: if (已满100点)                          // 缓冲区已满 → 替换最旧值
L242:   sum -= buf[idx]                       // 减去最旧的值
L246: else cnt++                              // 未满 → 增加采样计数
L248: buf[idx] = g_vout_real                 // 新值写入当前位置
L249: idx = (idx + 1) % 100                   // 环形索引前进
L251: vout_ma = sum / cnt                     // 滑动平均值 (cnt≥1，无除零)
L252: g_vout_filt += 0.2 × (vout_ma - filt)  // IIR 叠加在 MA 输出上
```

**说明：** 当 cnt < 100 时，`sum/cnt` 等价于已采集样本的算术平均，从第1次开始就有效。

---

### 2. 软启动

**目标：** 上电后从最小占空比(2%)在 100ms 内线性爬升到 PI 控制器的目标值，
避免启动瞬间 50% 占空比的冲击电流烧 MOSFET。

**涉及代码：**

| 位置 | 行号 | 说明 |
|------|------|------|
| 宏定义 | L71 | `#define SOFTSTART_MS (100U)` — 斜坡持续时间 |
| 状态变量 | L140-L141 | `g_softstart_active` 激活标志；`g_softstart_tick` 启动时刻 |
| 安全启动 | L679-L683 | PWM 启动前：`g_duty_cmd=DUTY_MIN`，写入 CCR，PI 积分预载 2% |
| 激活斜坡 | L701-L702 | PWM 启动后：设 `active=1`，记录 `tick=HAL_GetTick()` |
| 斜坡逻辑 | L305-L316 | `Power_ControlStep` 末尾：PI 计算后判断 |

**流程：**
```
初始化 (L679-L683):
  g_duty_cmd = DUTY_MIN          // 2%
  Power_ApplyDuty(g_duty_cmd)    // 写入 CCR，PWM 启动后立即输出 2%
  PI_Reset(..., DUTY_MIN)        // 积分器从 2% 开始

PWM/ADC 启动后 (L701-L703):
  g_softstart_active = 1         // 激活斜坡
  g_softstart_tick = HAL_GetTick() // 记录起点

每 1ms 控制循环 (L305-L316):
  // 先正常执行 PI 计算得到 g_duty_cmd (完整 PI 输出)
  if (软启动激活中)
    elapsed = now - tick
    if (elapsed >= 100ms)  →  结束软启动 (active = 0)
    else:  ramp = elapsed / 100ms
           g_duty_cmd = 2% + ramp × (PI_duty - 2%)  // 线性插值
  Power_ApplyDuty(g_duty_cmd)    // 输出最终占空比
```

**说明：** PI 每次都完整计算，软启动只是在输出时插值到 2% 和 PI 目标之间。
100ms 后自动退出，之后直接输出 PI 原始值，对稳态控制无影响。

---

### 3. 设定值平滑变化（Setpoint Ramp）

**目标：** 用户通过键盘/编辑模式改变电压设定值时（如 10V→30V），
不让设定值瞬间跳变，而是以有限速率逐渐爬升/下降，避免输出过冲。

**涉及代码：**

| 位置 | 行号 | 说明 |
|------|------|------|
| 宏定义 | L75 | `#define VOUT_SLEW_RATE_CV (4U)` — 每周期最大变化量 (0.04V @1ms=40V/s) |
| 状态变量 | L148 | `g_vset_ramp` — 实际用于 PI 控制的设定值（初始 500U=5.00V） |
| 追赶逻辑 | L274-L291 | `Power_ControlStep` 开头：delta 追赶 |
| PI 计算 | L291 | `vset = (float)g_vset_ramp * 0.01f` — PI 使用 ramp 值 |
| LCD 显示 | L439-442 | `vset_cv = g_vset_cv` — LCD 直接显示用户设定的目标值 |
| 串口输出 | L326 | `vset_cv = g_vset_cv` — 串口发送用户设定的目标值 |

**流程：**
```
L274-L283:
  delta = g_vset_cv - g_vset_ramp          // 差值（用户目标 - 当前运行值）
  if (delta > 4U):   g_vset_ramp += 4U     // 向上追
  elif (delta < -4U): g_vset_ramp -= 4U    // 向下追
  else:               g_vset_ramp = g_vset_cv  // 差值<步长，直接对齐

L291: vset = g_vset_ramp × 0.01           // PI 控制器使用斜坡后的值
```

**效果：** 10V(1000U)→30V(3000U) 差值 2000U，每 1ms 追 4U，
需 2000/4=500 周期=500ms 完成。
LCD 每 200ms 刷新一次，显示依次为 10.00V→18.00V→26.00V→30.00V。

**小步进无感：** 按键 +/- 每次变 2U (0.02V)，小于 4U 步长，
L283 直接对齐，相当于无延迟响应用户操作。

## 修改记录补充

| 日期 | 文件 | 修改内容 |
|------|------|---------|
| 2026-07-06 | Core/Src/main.c | ADC 滤波改为持续 100 点滑动平均 + IIR 叠加（替换前100次累加） |
| 2026-07-06 | Core/Src/main.c | 设定值平滑变化：新增 g_vset_ramp + 每周期delta追赶逻辑 |
