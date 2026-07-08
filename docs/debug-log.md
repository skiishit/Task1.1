# 调试日志

## 2026-07-06 — 初始化

### 从 GitHub 加载代码

从 https://github.com/skiishit/Task1.1 的 develop 分支克隆了本项目。
最新 commit: `b0cd25f task2.2 PID_adjust`
工程类型: STM32CubeIDE（非 Keil），MCU: STM32G474VETx

### 当前状态

- 代码已加载，清理了 Debug/ 编译产物和临时文件
- 工程为 STM32CubeIDE 工程（.cproject + .project + .ioc）
- 功能：Buck 变换器 CV/CC 控制
- 控制周期 1ms（主循环轮询）
- PI 控制带条件积分（anti-windup 简单实现）
- ADC 采样在 PWM 上升沿触发（TRGO=OC1REF）
- OSSR/OSSI=DISABLE，Break=DISABLE（安全配置待优化）
- 死区 141ns（保守值，需实测）
- PWM 启动前 duty 初始化为 0.5（50%），非软启动

### 待解决

- [ ] ADC 触发改为 TRGO=UPDATE（避开开关噪声）
- [ ] OSSR/OSSI 改为 ENABLE + IdleState=RESET
- [ ] Break 输入使能
- [ ] 软件 OVP 保护
- [ ] PI 反计算抗饱和

---

## 2026-07-06 — 第2轮：软启动 + ADC前100次平均滤波 + 设定值平滑

### 修改内容

**1. 软启动（Soft-Start）**
- `SOFTSTART_MS` (100ms) 宏定义
- PWM启动前先调用 `Power_ApplyDuty(DUTY_MIN)` + `PI_Reset(DUTY_MIN)`
- PI计算后若软启动激活则线性插值：`duty = DUTY_MIN + ramp×(PI_duty - DUTY_MIN)`
- `g_duty_cmd` 初始值 0.5f → DUTY_MIN(0.02f)
- PI积分器初始值 0.0f → DUTY_MIN

**2. ADC 前100次平均滤波**
- 前 ADC_INIT_SAMPLES(100) 次：电压用累加平均作为滤波值
- 从第1次开始就有有效值（运行平均值），无除零风险
- 100次后自动切换回标准 IIR 滤波
- 电流始终用 IIR 不受影响

**3. 设定值平滑变化**
- `VOUT_SLEW_RATE_CV=4U`(0.04V/cycle@1ms=40V/s)
- 每控制周期向 g_vset_cv 逼近最多 4U，LCD/串口都显示 ramp 值
- 10V→30V 约 500ms 渐变完成
- 步进调整(0.02V)时 ramp 1周期追上，无感

