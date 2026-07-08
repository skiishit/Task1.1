# Power Converter Task1.1 — 数字 Buck 变换器

STM32G474VETx 实现的同步 Buck 变换器，支持 CV（恒压）/ CC（恒流）双模式数字控制，通过 4x4 矩阵键盘和 SPI LCD 进行人机交互。

## 功能

- **CV 模式（恒压）** — 输出电压 2.00V~33.00V，步进 0.02V
- **CC 模式（恒流）** — 输出电流 0.1A~2.0A，步进 0.1A
- **模式切换** — 按键 A=CV，B=CC，C 进入编辑模式
- **4 位电压 + 2 位电流数字键盘输入**，带 CONFIRM/CANCEL 提示
- **SPI LCD 显示** — 设定值、实测值、ADC 原始值
- **USART 串口输出** — 设定值实时上报
- **ADC+IIR 滤波** — 电压/电流反馈采集
- **PWM 100kHz** — 互补输出，带死区时间

## 硬件平台

- MCU: STM32G474VETx
- 拓扑: 同步 Buck 变换器
- PWM: TIM1 CH1 + CH1N 互补输出 @ 100kHz
- ADC: ADC1 CH1(电压)/CH2(电流)，TIM1 TRGO 触发采样
- 显示: SPI LCD（ILI9341 兼容，SPI1）
- 输入: 4x4 矩阵键盘
- 通信: USART1（115200 baud）

## 工程结构

```
power-converter-task1.1/
├── Core/
│   ├── Inc/          # 头文件
│   │   ├── main.h           # 引脚定义、全局宏
│   │   ├── keypad.h         # 键盘驱动
│   │   ├── lcd_driver.h     # LCD 驱动
│   │   ├── lcd_fonts.h      # LCD 字库
│   │   ├── oled_ssd1306.h   # OLED 驱动（保留未用）
│   │   ├── stm32g4xx_hal_conf.h
│   │   └── stm32g4xx_it.h
│   ├── Src/          # 源文件
│   │   ├── main.c           # 主程序：初始化、控制循环、显示
│   │   ├── keypad.c         # 键盘驱动实现
│   │   ├── lcd_driver.c     # LCD 驱动实现
│   │   ├── lcd_fonts.c      # LCD 字库数据
│   │   ├── oled_ssd1306.c   # OLED 驱动（保留未用）
│   │   ├── stm32g4xx_it.c   # 中断服务
│   │   ├── syscalls.c / sysmem.c / system_stm32g4xx.c
│   └── Startup/
│       └── startup_stm32g474vetx.s
├── Drivers/          # STM32G4 HAL 库
├── Task1.1.ioc       # STM32CubeMX 配置
├── docs/             # 工程文档
└── README.md
```

## 源码来源

GitHub: [skiishit/Task1.1](https://github.com/skiishit/Task1.1) — develop 分支
最新 commit: `b0cd25f task2.2 PID_adjust`

## 编译

使用 STM32CubeIDE（STM32CubeIDE 1.14.0+）打开工程根目录即可编译。
