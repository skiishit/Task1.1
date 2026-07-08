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
