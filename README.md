# RJ_SKILLS — Embedded Control Contest Copilot

面向**全国大学生电子设计竞赛（电赛 NUEDC）**与**全国大学生嵌入式芯片与系统设计竞赛（嵌入式大赛）**控制类赛道的专业级 Antigravity Agent Skill。

## 核心覆盖能力

- **四大主控与工具链深度适配**
  - **STM32**（F1 / F4 / G4 / H7）+ Keil MDK (AC5/AC6) + STM32CubeMX (`.ioc`)
  - **TI MSPM0**（MSPM0G3507 等）+ TI CCS / Theia + SysConfig (`.syscfg`)
  - **ESP32**（ESP32 / ESP32-S3 等）+ ESP-IDF (`CMakeLists.txt` / `sdkconfig` / 双核 SMP FreeRTOS)
  - **瑞萨 Renesas RA**（RA4 / RA6 / RA8）+ RASC / FSP (`configuration.xml`) + Keil MDK
- **控制类四大核心载体算法与工程架构**
  - **智能小车**：差速/麦轮/全向轮/阿克曼正逆运动学、速度+位置+陀螺仪航向角（Yaw）锁定串级闭环、S型/梯形加减速
  - **二维/三维云台**：微分先行 PID、目标速度前馈、死区补偿、视觉像素转视场角（`atan2f`）、1kHz 陀螺仪内环 + 视觉外环多速率融合
  - **无人机飞控**：Mahony 四元数互补滤波、二阶巴特沃斯低通滤波抗机架震动、姿态/定高定点串级闭环、倾角与断连失控保护
  - **机械臂控制**：解析法正逆运动学（含奇异点与工作空间越界保护）、五次多项式/梯形多轴同步轨迹插补、静态重力前馈
- **三大核心工作流**
  1. **代码审查与深度改进（Audit & Optimize）**：结合静态扫描脚本进行四层漏斗排错（并发实时性 $\rightarrow$ 硬件配置匹配 $\rightarrow$ 浮点与内存效率 $\rightarrow$ 架构与安全护栏）。
  2. **从零构建高效程序（Zero-to-One Firmware）**：时序预算规划 + 四层解耦架构（裸机时间片轮询 / FreeRTOS / RT-Thread）+ 赛题分问状态机 + VOFA+ 实时调参。
  3. **软硬联调现象级排障（HIL Debug Triage）**：开环极性自检、HardFault 栈回溯、电源/电磁干扰与控制参数震荡二分定位。

## 目录结构

```text
embedded-control-copilot/
├── SKILL.md                                           # Skill 主控路由与执行协议
├── scripts/                                           # Python 标准库自动化体检工具（零第三方依赖）
│   ├── embedded_static_scan.py                        # 嵌入式 C/C++ 反模式与平台专坑静态扫描器
│   ├── hw_config_inspector.py                         # .ioc / .syscfg / .uvprojx / configuration.xml / sdkconfig 解析器
│   └── map_memory_analyzer.py                         # Keil / GCC .map 内存占用与软浮点污染分析器
├── references/                                        # 按需渐进式加载的深度技术手册
│   ├── embedded-c-pitfalls.md                         # 嵌入式 C 四层致命缺陷审查清单
│   ├── mcu-platforms-guide.md                         # STM32 / TI MSPM0 / ESP32 / 瑞萨 RA 专坑与最佳实践
│   ├── contest-architectures.md                       # 裸机时间片+赛题状态机 vs FreeRTOS/RT-Thread 架构规范
│   ├── motion-control-algorithms.md                   # 车/台/机/臂运动学、滤波与高阶控制算法详解
│   └── comm-motor-and-debug.md                        # 零拷贝串口/CAN电机协议、VOFA+ 调参与现象级联调手册
└── resources/                                         # 零动态内存、ISR 安全的 C99 黄金代码骨架库
    ├── bsp_motor_imu_interface/motion_hal_interface.h # 跨芯片硬件抽象接口（结构体函数指针解耦）
    ├── pid_controller/                                # 竞赛级全功能 PID（串级/抗积分饱和/微分先行/前馈/死区）
    ├── ringbuf_protocol/                              # SPSC 无锁环形缓冲区 + 状态机协议解包 + VOFA+ JustFloat
    └── scheduler_and_fsm/                             # 确定性非阻塞调度器 + 电赛分问隔离表驱动状态机 + 失控看门狗
```

## 全局生效与安装说明

- **Antigravity（反重力）**：由于 Windows 下 Antigravity 扫描全局技能要求真实目录（不识别 Junction 联接），修改本仓库后请运行仓库根目录下的 `sync-to-antigravity.ps1`，将 `embedded-control-copilot/` 同步至 `~/.gemini/config/skills/embedded-control-copilot`；在任意工程中可通过自然语言点名 `embedded-control-copilot` 或由 Agent 自动触发。
- **Codex**：可通过目录联接将 `embedded-control-copilot/` 挂载至 `~/.codex/skills/embedded-control-copilot`，支持 `$embedded-control-copilot` 显式调用或按上下文自动激活。
