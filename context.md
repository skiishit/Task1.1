# context.md — Power Converter Task1.1

## 项目概述

数字 Buck 变换器（电赛培训），基于 STM32G474VETx，CV/CC 控制。
代码来自 GitHub `skiishit/Task1.1` develop 分支。

---

### 第1轮 — 2026-07-06 20:58

**用户指令原文：**
新建一个嵌入式项目，做的是数字电源，先从https://github.com/skiishit/Task1.1/tree/develop上下拉最新的develop分支下代码。

**执行：**
1. 加载 embedded-keil-cli、power-converter-firmware、engineering-conventions 三个 skill
2. `git clone --depth 1 --branch develop` 到 `C:\user_program\hermes_workspace\power-converter-task1.1`
3. 确认工程为 STM32CubeIDE（非 Keil），MCU STM32G474VETx
4. 清理 Debug/ 编译产物和临时文件
5. 创建工程文档：README.md、docs/hardware.md、docs/architecture.md、docs/debug-log.md、docs/review-guide.md、context.md

**关键发现：**
- MCU: STM32G474VETx, 系统时钟 170MHz
- PWM: TIM1 CH1+CH1N 互补 @100kHz (Period=1699, Prescaler=0)
- ADC 触发: TIM1 TRGO=OC1REF（PWM 开通边沿采样，有开关噪声）
- PI: 条件积分（输出饱和时冻结），无抗饱和
- 安全配置问题：OSSR/OSSI=DISABLE, Break=DISABLE, 无软启动, ADC 采样时机不佳
- 最新 commit: b0cd25f "task2.2 PID_adjust"
- 控制周期 1ms (HAL_GetTick 轮询)

---

### 第2轮 — 2026-07-06 21:00

**用户指令原文：**
做一下软启动，然后做一下进一步的adc滤波，具体电压数值按照adc前一百次读数的平均值计算，adc没有前100个读数的时候自己做好处理不要出bug。然后再做一下设定电压平滑变化，现在在10V设定到30V的时候会有电压突变，我需要平滑的变化，比如50ms最多变1V，从lcd上看好像是10V直接变成30V，实际上是10V变成11V再变成12V这样一直慢慢变到30V。具体数值先自己计算一下，不一定要1V一变，也不一定要50ms变1V，做到平滑且安全即可

**分析：**
三件事互不依赖，可以一起改。

1. **软启动**：原始代码 `g_duty_cmd = 0.5f`，PWM 启动即 50% 占空比，有冲击电流。需要：
   - PWM 启动前设 DUTY_MIN(2%)
   - 启动后 100ms 线性斜坡到 PI 目标值

2. **ADC 滤波**：原始 IIR 从 `g_vout_filt=0.0f` 开始，启动初期值不准。需要：
   - 前 100 次采样用累加平均作为滤波值
   - 从第 1 次就有有效值（无除零风险）
   - 100 次后切回标准 IIR

3. **设定值平滑**：编辑模式或按键调整时目标瞬间跳变。需要：
   - 每控制周期(1ms)向目标逼近有限步长
   - 选 4U(0.04V/cycle) = 40V/s，10V→30V 约 500ms，平滑且不拖沓
   - LCD/串口都显示实际运行的渐变值

**改动文件：** `Core/Src/main.c`，共 7 处 patch：

| # | 位置 | 改动 |
|---|------|------|
| 1 | USER CODE BEGIN PD (after CC_KI) | 新增 SOFTSTART_MS=100U, ADC_MA_WINDOW=100U(滑动平均), VOUT_SLEW_RATE_CV=4U |
| 2 | USER CODE BEGIN PV (variables) | g_duty_cmd=0.5f→DUTY_MIN; PI积分=0.0f→DUTY_MIN; 新增软启动/ADC平均/设定值ramp变量 |
| 3 | Power_ReadFeedback | 增加前100次电压累加平均逻辑; 电流始终IIR |
| 4 | Power_ControlStep | PI计算前加设定值ramp(delta→4U/cycle); PI后加软启动斜坡 |
| 5 | Oled_UpdateDisplay | 显示 g_vset_ramp 替代 g_vset_cv |
| 6 | Uart_SendSetpoints | 串口输出 g_vset_ramp 替代 g_vset_cv |
| 7 | main() 初始化 | PWM启动前设DUTY_MIN并写入CCR; 启动后激活软启动 |

**设计决策：**
- ramp rate = 4U/cycle = 0.04V/ms = 40V/s。10V→30V 需 500ms。步进调整(0.02V)时 ramp 在1周期内追上（4U>2U，直接到达）
- ADC 初始平均：`g_vout_filt = g_vout_init_sum / (float)g_adc_init_count`，count 从1开始，无除零
- 软启动结束后自动归零，不影响后续正常控制

**下次可做：**
- [ ] ADC 触发改为 TRGO=UPDATE（避开开关噪声）
- [ ] OSSR/OSSI 改为 ENABLE + IdleState=RESET
- [ ] Break 输入使能 + 硬件过流保护
- [ ] 软件 OVP 保护阈值
- [ ] PI 反计算抗饱和（设定值前馈）

---

### 第3轮 — 2026-07-06 21:02

**用户指令原文：**
首先修改adc滤波，不是初始使用100次平均，是一直使用100次平均，并且和iir叠加。然后说明一下软启动和设定值平滑变化的实现逻辑以及对应代码在哪几行

**改动：** `Core/Src/main.c` 三处 patch

| # | 位置 | 改动 |
|---|------|------|
| 1 | USER CODE BEGIN PD | ADC_INIT_SAMPLES → ADC_MA_WINDOW(100)，去掉"初始化"语义 |
| 2 | USER CODE BEGIN PV | g_adc_init_count/g_vout_init_sum → g_vout_ma_buf[100]/idx/sum/cnt 环形缓冲 |
| 3 | Power_ReadFeedback(L234-L254) | 移除"前100次累加→切IIR"逻辑；改为100点环形滑动平均 + IIR叠加 |

**滑动平均+IIR叠加流程（L237-L252）：**
- 每次新采样 vout_real 累加到 g_vout_ma_sum（L239）
- 满100点后减去最旧值 g_vout_ma_buf[g_vout_ma_idx]（L242）
- 未满100点时 g_vout_ma_cnt++，除数减小（L246）
- 新值写入环形缓冲对应位置（L248），索引循环+1（L249）
- 计算滑动平均 vout_ma = sum/cnt（L251），cnt≥1 无除零风险
- IIR 叠加：g_vout_filt += alpha × (vout_ma - g_vout_filt)（L252）

**Round 2 ADC 滤波描述修正：** 第2轮初始实现为"前100次累加平均→切IIR"，现已修正为"持续100点滑动平均+IIR叠加"

---

### 第4轮 — 2026-07-06 21:10

**用户指令原文（上下文）：**
修一下1（软启动影子寄存器问题），然后我的ki设置为10，然后想一下adc滤波怎么调整

**改动：** `Core/Src/main.c` 三处

| # | 位置 | 改动 |
|---|------|------|
| 1 | main() 初始化 (L683) | 写入 DUTY_MIN 后加 `TIM1->EGR = TIM_EGR_UG` 强制传输影子CCR到活动寄存器 |
| 2 | USER CODE BEGIN PD | CV_KI 0.594f → 10.0f; IIR_ALPHA 0.2f → 0.3f |
| 3 | USER CODE BEGIN PD | ADC_MA_WINDOW 100U → 16U（环形缓冲对应缩小，注释同步修正） |

---

### 第5轮 — 2026-07-06 21:12

**用户反馈：**
1. 软启动仍然没有应有的效果，可能是硬件限制
2. 5s震荡已消失，但仍有约15s的稳定时间
3. 询问能否用系统时钟/ADC采样时钟直接处理ADC值（跳过1ms控制周期限制）

**分析：**

**软启动问题：** TIM1->EGR 强制更新在软件上是正确的（CEN=0时UG仍传输影子寄存器）。如果第一个PWM周期仍然出现高压尖峰，可能是硬件原因：
- 输出电容大，2%占空比的充电电流仍产生较大浪涌
- 自举电容在第一个周期未充满，高侧驱动异常
- 负载端电容反向馈电

**15s震荡：** MA(16)+IIR(0.3)的群延迟仅~10ms，15s的时间常数远超出滤波器范围。更像是 PI 积分器(Ki=10)与硬件时间常数(输出电容充放电、负载响应)的相互作用。调节过程长可能是因为 Ki 过大导致积分器反复 windup/unwind。

**ADC高速处理的可行性：** ADC硬件采样率100kHz(10µs)，DMA连续更新g_adc_dma[]。目前软件每1ms只读1次，浪费99%的ADC数据。可以用以下方式利用高速ADC数据：

- **方案A：硬件过采样** — STM32G4 ADC内置硬件过采样单元，可配置16/32/64/128/256倍过采样，硬件自动累加→移位→输出，零软件开销、零延迟。在 USER CODE 中使能即可。
- **方案B：TIM2中断累加** — TIM2当前1Hz，改为100µs周期，ISR中累加ADC样本。每1ms(10次中断)传递一次平均值到MA+IIR。AD采样利用率从1%提升到10%。
- **方案C：DMA双缓冲** — 配置DMA循环双缓冲，填满一个缓冲区(100样本/1ms)后触发中断处理，100%利用ADC数据。

**TODO（新增）：**
- [ ] ADC硬件过采样或TIM2加速ADC处理
- [ ] OSSR/OSSI ENABLE
- [ ] Break使能
- [ ] PI抗饱和优化

---

### 第6轮 — 2026-07-06 21:15

**用户指令原文：**
之前在没有加adc ma的时候没有发生过震荡。采用方案a（硬件过采样）

**改动：** `Core/Src/main.c` 四处

| # | 位置 | 改动 |
|---|------|------|
| 1 | USER CODE BEGIN PD | 删除 ADC_MA_WINDOW 宏定义 |
| 2 | USER CODE BEGIN PV | 删除 MA 环形缓冲变量(g_vout_ma_buf/idx/sum/cnt) |
| 3 | Power_ReadFeedback (L230-L231) | 电压回归标准IIR滤波，删除MA+IIR级联逻辑 |
| 4 | main() ADC校准后 (L653-L656) | 增加硬件过采样配置: ADC1->CFGR2 OVSE=1, OVSR=3(16x), OVSS=4(shift4) |

**当前ADC滤波链路：**
```
ADC硬件(100kHz采样) → 硬件16x过采样(累加16次→右移4→每160µs输出)
→ 软件IIR α=0.3 (每1ms读一次最新过采样值)
```

**噪声抑制：** 硬件16x = √16=4x, IIR α=0.3 ≈ 2.7x, 合计 ≈ 10x (0.05V → ~0.005V)
**总延迟：** 仅IIR的~1.7ms（无MA的50ms/10ms/8ms延迟）
