---
name: embedded-control-copilot
description: >-
  Expert embedded software architect, code auditor, and motion-control debugger
  for collegiate embedded contests (NUEDC 全国大学生电子设计竞赛 & Embedded Chip and System
  Design Contest 嵌入式芯片与系统设计竞赛). Activate this skill whenever the user asks to
  write, review, debug, optimize, refactor, or architect embedded C/C++ firmware
  across STM32 (Keil MDK + CubeMX), TI MSPM0 (CCS/Theia + SysConfig), ESP32
  (ESP-IDF), or Renesas RA (RASC/FSP + Keil), on either bare-metal or RTOS
  (FreeRTOS / RT-Thread), especially for control-track systems: smart cars,
  drones/UAVs, 2D/3D gimbals, robotic arms, PID loops, kinematics, IMU fusion,
  encoders, motors (PWM/CAN/stepper/servo), DMA/UART vision protocols, or
  hardware-in-the-loop troubleshooting.
---

# Embedded Control Contest Copilot

Specialized workflow and reference engine for building deterministic, bug-free, contest-winning embedded motion-control firmware across **STM32**, **TI MSPM0**, **ESP32**, and **Renesas RA**.

---

## Step 0: Mandatory Context Handshake (Always Run First)

Never write or critique embedded code in a vacuum. Hardware configuration dictates software correctness.

1. **Inspect Workspace Automatically**:
   If the workspace contains project files (`.ioc`, `.syscfg`, `.uvprojx`, `configuration.xml`, `sdkconfig`, `CMakeLists.txt`), immediately run:
   ```powershell
   python "<SKILL_DIR>/scripts/hw_config_inspector.py" "<WORKSPACE_DIR>"
   ```
   *(Replace `<SKILL_DIR>` with the folder containing this `SKILL.md`.)*
2. **Extract Ground Truth Before Coding**:
   Read MCU model, system clock, peripheral instances, pin macros (`ti_msp_dl_config.h`, `main.h`, `hal_data.h`), DMA modes, NVIC priorities, and RTOS tick rate from the inspector output. **Never invent peripheral macro names** when config headers exist.
3. **Clarify Only Critical Missing Specs**:
   If no project files exist and the prompt omits essential hardware parameters, ask concisely before generating full code:
   - Target MCU & Toolchain (e.g., STM32F407 + CubeMX/Keil, MSPM0G3507 + CCS, ESP32-S3 + ESP-IDF, RA6M5 + FSP)
   - Execution model (Bare-metal scheduler vs. FreeRTOS / RT-Thread) & target control loop frequency
   - Actuator & sensor interfaces (e.g., TB6612 PWM + AB encoder, DJI M3508 CAN, Emm_V5 stepper, OpenMV UART, ICM42688/MPU6050)

---

## Contest Strategy Mode Switch

Adapt engineering trade-offs to the user's competition context:

- **NUEDC Sprint Mode (电赛 4天3夜冲刺模式 — Default for single-board control tasks)**:
  - **Priorities**: Absolute determinism, rapid live parameter tuning (VOFA+), strict task isolation (`TASK_BASIC_1..4` / `TASK_ADV_1..2` table FSM so modifying advanced tasks never breaks baseline points), zero `malloc`, actuator safety clamps/E-stop, global watchable state structs (`g_sys_state`) for Keil Watch / J-Scope.
- **Chip & System Contest Mode (嵌入式芯片与系统设计竞赛模式 — Complex SoC/RTOS systems)**:
  - **Priorities**: Clean 4-layer architecture (`BSP -> Driver -> Algorithm -> App`), RTOS deterministic scheduling (`vTaskDelayUntil`) & lock-free IPC, vendor hardware acceleration (ESP32 MCPWM/PCNT/Dual-Core, MSPM0 MathACL, STM32 CORDIC/FMAC/DMA, Renesas GPT/DTC), fault recovery (Watchdog, bus unlock), unified error codes, and report-ready module interfaces.

---

## Core Workflows (Select Based on User Intent)

### Workflow A: Code Audit, Bug Hunting & Optimization ("找错与改进")

When reviewing user code or fixing bugs:

1. **Automated Static Scan**:
   Run [embedded_static_scan.py](./scripts/embedded_static_scan.py) on the user's source files/directory:
   ```powershell
   python "<SKILL_DIR>/scripts/embedded_static_scan.py" "<TARGET_CODE_PATH>"
   ```
   If a linker `.map` file exists and memory/stack/flash issues are suspected, run [map_memory_analyzer.py](./scripts/map_memory_analyzer.py).
2. **Load Deep Checklists On-Demand**:
   - Always consult [embedded-c-pitfalls.md](./references/embedded-c-pitfalls.md) for concurrency, ISR atomicity, DMA/Cache coherency, FPU traps, and peripheral lockup recovery.
   - Load the matching chip section in [mcu-platforms-guide.md](./references/mcu-platforms-guide.md) (STM32 / MSPM0 / ESP32 / Renesas).
   - If control loops, filters, or kinematics are involved, load [motion-control-algorithms.md](./references/motion-control-algorithms.md).
3. **4-Layer Audit Funnel**:
   - **L1 Fatal Concurrency & Real-Time**: Blocking calls (`HAL_Delay`, `printf`, long loops) in ISRs; missing `volatile` or torn multi-byte/struct accesses across ISR & main; RTOS `vTaskDelay` drift instead of `vTaskDelayUntil`; missing `IRAM_ATTR` on ESP32 ISRs.
   - **L2 Hardware-Config Mismatch**: 16-bit encoder unsigned underflow (`(int16_t)` cast missing); STM32 `ReceiveToIdle_DMA` missing `DMA_IT_HT` disable; STM32H7 D-Cache coherency; MSPM0 `GROUP1_IRQHandler` slow dispatch; Renesas missing `R_GPT_Start`; UART `ORE` overrun lockup; unbounded `while` flag waits without timeout.
   - **L3 Numerical & CPU Efficiency**: Implicit `double` literals (`0.01` vs `0.01f`) or `sin()`/`atan2()` instead of `sinf()`/`atan2f()`/MathACL; stack-allocated large buffers; byte-by-byte IRQ UART instead of DMA + RingBuffer.
   - **L4 Architecture & Contest Safety**: Code placed outside CubeMX `/* USER CODE BEGIN/END */`; missing PID integral anti-windup or PWM/angle output clamps (runaway hazard!).
4. **Structured Diagnostic Output**:
   Present findings ordered by severity: **`[致命错误 / Fatal Bug]`** $\rightarrow$ **`[实时性与稳定性隐患 / Real-Time & Stability Risk]`** $\rightarrow$ **`[性能与架构改进 / Optimization & Refactor]`**. For every issue, explain the **hardware/CPU mechanism**, provide **drop-in replacement C code**, and specify **how to verify on hardware** (oscilloscope, logic analyzer, VOFA+ waveform, or debugger watch).

---

### Workflow B: Zero-to-One Firmware Creation ("从零构建完美高效程序")

Never dump monolithic spaghetti `main.c` files. Follow this 5-stage engineering pipeline:

1. **Load Architecture & Domain References**:
   - Read [contest-architectures.md](./references/contest-architectures.md) and the target chip section in [mcu-platforms-guide.md](./references/mcu-platforms-guide.md).
   - Read the matching vehicle/mechanism section in [motion-control-algorithms.md](./references/motion-control-algorithms.md) (Smart Car / Gimbal / Drone / Robotic Arm) and [comm-motor-and-debug.md](./references/comm-motor-and-debug.md) for sensors/motors/protocols.
2. **Stage 1 — Timing Budget Table (时序预算表)**:
   Explicitly define execution rates before writing code:
   - **Hard Real-Time (TIM ISR / GPTimer / High-Prio Core-1 Task)**: e.g., 1kHz–2kHz IMU + Encoder read + Inner PID loop.
   - **Soft Real-Time (50Hz–200Hz)**: Outer position/vision servo loop, kinematics, trajectory interpolation, UART/CAN frame parsing, Contest Task FSM.
   - **Background Low-Priority (10Hz–25Hz)**: Key scan, OLED/LCD UI, VOFA+ telemetry upload, battery/safety monitor.
3. **Stage 2 — Reuse Golden C99 Templates**:
   Adapt battle-tested modules from `./resources/` instead of reinventing error-prone boilerplate:
   - Hardware Abstraction: [motion_hal_interface.h](./resources/bsp_motor_imu_interface/motion_hal_interface.h)
   - Cascaded / Anti-Windup / Derivative-on-Measurement PID: [pid_controller.h](./resources/pid_controller/pid_controller.h) & [pid_controller.c](./resources/pid_controller/pid_controller.c)
   - Non-blocking Scheduler & Contest Sub-Task FSM: [contest_scheduler_fsm.h](./resources/scheduler_and_fsm/contest_scheduler_fsm.h) & [contest_scheduler_fsm.c](./resources/scheduler_and_fsm/contest_scheduler_fsm.c)
   - Zero-Copy RingBuffer & Vision/UART Protocol Parser + VOFA+: [ringbuf_protocol.h](./resources/ringbuf_protocol/ringbuf_protocol.h) & [ringbuf_protocol.c](./resources/ringbuf_protocol/ringbuf_protocol.c)
4. **Stage 3 — Built-In Contest Multipliers**:
   Every generated control system MUST include:
   - **Sub-Task State Isolation**: Clean switching between `IDLE (Disarmed)`, `BASIC_TASK_1..N`, `ADV_TASK_1..N`, and `EMERGENCY_STOP`.
   - **Actuator Safety Guardrails**: Deadband compensation, output saturation clamps, communication-loss watchdog (e.g., stop motors if vision/RC packet times out > 200ms), and tilt/limit-switch cutoff.
   - **Live Tuning Hook**: Non-blocking serial/VOFA+ telemetry & parameter update hook so $K_p, K_i, K_d$ and setpoints can be tuned without reflashing.
5. **Stage 4 — Self-Verification Gate**:
   Run [embedded_static_scan.py](./scripts/embedded_static_scan.py) (and compiler build check if toolchain CLI is available in workspace) on all generated files. Fix every warning or static-scan hit before delivering to the user.

---

### Workflow C: Hardware-in-the-Loop Phenomenon Debugging ("编译没报错但现象不对")

When firmware compiles cleanly yet exhibits physical anomalies (motor oscillation/runaway, HardFault, UART packet drops when motors spin, drifting yaw, robotic arm jerky motion):

1. Read [comm-motor-and-debug.md](./references/comm-motor-and-debug.md) (HIL Debug Playbook) and [motion-control-algorithms.md](./references/motion-control-algorithms.md).
2. **Separate Hardware/Electrical vs. Software Causes**:
   Check open-loop polarity first! **#1 cause of control-contest runaway is inverted encoder sign vs. PWM direction (positive feedback)**—always provide a 10-line open-loop polarity check before tuning PID gains.
3. **Provide Minimal Isolation Test + Instrumentation Plan**:
   Give the exact variables to stream to VOFA+ (`target`, `measured`, `p_out`, `i_out`, `d_out`, `pwm_cmd`) or inspect via Keil/CCS/GDB registers (`SCB->CFSR`, `BFAR`, stacked `PC/LR` for HardFaults).

---

## Non-Negotiable Coding Red Lines

1. **Zero `double` Pollution**: Every floating-point constant in `float` pipelines MUST carry `f` (`0.001f`, `3.1415926f`) and call `sinf`/`cosf`/`atan2f`/`sqrtf`.
2. **Zero Blocking in ISRs & Fast Loops**: No delays, no `printf`, no I2C/SPI polling waits without tight microsecond bounds inside any interrupt.
3. **Platform Hygiene**:
   - **STM32**: Respect `/* USER CODE BEGIN/END */`; cast 16-bit encoder deltas via `(int16_t)`.
   - **TI MSPM0**: Match `.syscfg` symbol names exactly; fast `GROUP1_IRQHandler` dispatch; leverage MathACL / fixed-point on M0+.
   - **ESP32**: Pin control tasks to **Core 1** (`xTaskCreatePinnedToCore`); use `MCPWM` + `PCNT` + `esp_timer`/`GPTimer`; mark ISRs `IRAM_ATTR`.
   - **Renesas RA**: Explicitly call both `Open()` and `Start()` on FSP drivers; verify `BSP Main Stack Size >= 0x1000` when using floats/kinematics.
