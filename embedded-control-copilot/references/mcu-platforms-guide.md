# 四大竞赛主流 MCU 平台与工具链实战避坑指南 (MCU Platforms & Toolchains Guide)

> **覆盖平台**：STM32 (CubeMX + Keil) · TI MSPM0 (SysConfig + CCS/Keil) · ESP32/S3 (ESP-IDF) · Renesas RA (RASC/FSP + Keil)  
> **定位**：聚焦电赛（NUEDC）与嵌入式大赛中最容易卡住队伍数小时的底层外设配置、编译器差异与硬件专属深坑。

---

## 1. STM32 (F1 / F4 / G4 / H7) + STM32CubeMX (`.ioc`) + Keil MDK (AC5 / AC6)

### 1.1 CubeMX 重生成无损隔离架构（3-Hook 法则）
* **核心痛点**：比赛现场频繁调整引脚或定时器参数并点击 `GENERATE CODE`，一旦代码写在 `/* USER CODE BEGIN ... */` 注释块之外，心血瞬间被清空；若把几百行控制代码全塞在 `main.c` 注释块内，又极难阅读与多人协作。
* **3-Hook 标准工程结构**：在工程根目录自建 `App/`、`Alg/`、`Bsp/` 文件夹，CubeMX 生成的文件中**永远只保留 3 处单行钩子调用**：
  1. `main.c` 的 `/* USER CODE BEGIN Includes */`：`#include "app_entry.h"`
  2. `main.c` 的 `/* USER CODE BEGIN 2 */`：`App_Init();`（统一启动编码器、PWM、DMA 串口与控制定时器）
  3. `main.c` 的 `/* USER CODE BEGIN WHILE */`：`App_Loop();`（HAL 中断回调如 `HAL_TIM_PeriodElapsedCallback` 同样在 `Bsp/bsp_irq_bridge.c` 中重写覆盖 `__weak` 定义，彻底不动 `stm32f4xx_it.c`）。

### 1.2 Keil AC5 vs AC6 (`armclang`) 半主机死机陷阱 (`BKPT 0xAB`) 与优化避坑
* **经典赛场惨案**：插着 ST-Link / DAP-Link 仿真一切正常，**拔掉调试器独立上电整车一动不动**！接回调试器 Attach 暂停，发现 PC 停在汇编 `0xBEAB` (`BKPT 0xAB`)。
* **根因**：调用了 `printf` 等标准 I/O 函数但未勾选 `Use MicroLIB`，ARM C 库默认启用**半主机模式 (Semihosting)**，通过软断点请求仿真器代为输出，脱机运行时无人响应断点即永久挂起！此外 MicroLIB 在 AC6 下存在 64 位格式化不全、无 C++ 支持等缺陷。
* **AC5 & AC6 双兼容免 MicroLIB 去半主机 `printf` 标准模板**：

```c
/* ✅ 竞赛标准写法 (Contest-Grade)：放入 bsp_retarget.c，兼容 AC5/AC6 且无需开启 MicroLIB */
#include <stdio.h>
#include "usart.h"

#if !defined(__MICROLIB)
  #if defined(__CC_ARM)                          /* ARM Compiler 5 (armcc) */
    #pragma import(__use_no_semihosting)
    struct __FILE { int handle; };
  #elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050) /* ARM Compiler 6 (armclang) */
    __asm(".global __use_no_semihosting\n\t");
    struct __FILE { int handle; };
  #endif

  FILE __stdout;
  FILE __stdin;

  void _sys_exit(int x) { (void)x; while (1) {} }
  void _ttywrch(int ch) { (void)ch; }            /* 防止部分 C 库符号顺带拉入半主机 */
#endif

int fputc(int ch, FILE *f) {
    (void)f;
    uint8_t c = (uint8_t)ch;
    /* 注意：仅限低频初始化/主循环调试使用；高频波形请走独立 DMA 发送缓冲（如 VOFA+ JustFloat） */
    HAL_UART_Transmit(&huart1, &c, 1, 2);
    return ch;
}
```

* **AC6 (`-O2` / `-Oz`) 优化铁律**：AC6 基于 LLVM，若代码中存在缺 `volatile` 的标志位、未初始化局部变量或未写 `return` 的非 `void` 函数，`-O2`/`-Oz` 会直接裁掉整块分支甚至生成 `UDF` 指令。竞赛推荐开启 **`-O1` 或 `-O2`（配合勾选 `Plain char is signed` 及 C99 模式）**，并开启 `-Wall -Wdouble-promotion`。

### 1.3 STM32 运动控制外设四大高频深坑

#### (1) 正交编码器：严禁“读后清零 (`CNT = 0`)”，必须自由回绕差分
* **原因**：在执行 `cnt = TIM3->CNT;` 到 `TIM3->CNT = 0;` 的数个时钟周期（尤其若其间被更高优先级中断抢占数微秒）内到达的编码器脉冲会被 `CNT = 0` 直接抹除，车速越高丢脉冲越严重，里程计闭环必然跑偏！

```c
/* ❌ 错误写法 (Buggy)：每次读取后清零硬件 CNT，高速旋转与中断抢占下必丢脉冲！ */
int16_t Encoder_Read_Buggy(void) {
    int16_t delta = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0); // <--- 两句之间新来的脉冲被直接抹杀！
    return delta;
}

/* ✅ 竞赛标准写法 (Contest-Grade)：CubeMX 设 Counter Period (ARR) = 65535，自由回绕无损模差分 */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint16_t prev_cnt;
    int64_t  total_cnt;
} STM32_Encoder_t;

void Encoder_Init(STM32_Encoder_t *enc, TIM_HandleTypeDef *htim) {
    enc->htim = htim;
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
    enc->prev_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(htim);
    enc->total_cnt = 0;
}

int16_t Encoder_SampleDelta(STM32_Encoder_t *enc) {
    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(enc->htim);
    int16_t delta = (int16_t)(now - enc->prev_cnt); // 16 位补码自动处理 65535<->0 正反向跨界
    enc->prev_cnt = now;
    enc->total_cnt += delta;
    return delta;
}
```

#### (2) 高级定时器 `TIM1` / `TIM8` 互补输出与主输出使能 (`MOE`)
* 使用 `TIM1_CH1N` 等互补通道驱动半桥/三相桥时，仅调用 `HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1)` 不会开启 `N` 通道！**必须成对调用 `HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1)`**；若寄存器级直接操作，切记置位 `TIM1->BDTR |= TIM_BDTR_MOE;`。

#### (3) 视觉/激光雷达不定长 DMA 接收：`DMA_IT_HT` 截断与 `ORE` 掉线自愈
* `HAL_UARTEx_ReceiveToIdle_DMA()` 默认同时开启了 DMA **半传输中断 (`HT`)** 与传输完成中断 (`TC`)。若不手动关掉 `HT`，当一帧协议恰好跨过缓冲区中点时，`HAL_UARTEx_RxEventCallback` 会提前触发吐出半截残帧！同时必须接管 `HAL_UART_ErrorCallback` 防止热插拔串口触发 `ORE` 永久停收：

```c
/* ✅ 竞赛标准写法 (Contest-Grade)：ReceiveToIdle_DMA 关闭 HT 半满中断 + 错误自动重启闭环 */
static uint8_t s_vision_dma_buf[64];

void Vision_UART_StartRx(void) {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_vision_dma_buf, sizeof(s_vision_dma_buf));
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT); // 关键！禁止半满中断把完整帧切成两半
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART2) {
        Vision_Protocol_FeedBytes(s_vision_dma_buf, Size);
        Vision_UART_StartRx(); // 重新挂载下一轮接收
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        __HAL_UART_CLEAR_OREFLAG(huart); // 清除 ORE/FE/NE 标志并重启 DMA
        HAL_UART_AbortReceive(huart);
        Vision_UART_StartRx();
    }
}
```

#### (4) NVIC 优先级分组与 FreeRTOS 铁律
* 全局必须为 **`NVIC_PRIORITYGROUP_4`**（4 位全抢占 `0..15`，子优先级 `0`）。
* 在 FreeRTOS 工程中，凡调用 `FromISR` API 的中断（如 `UART2_IRQn`、`DMA`），其抢占优先级数值**必须 `>= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`（CubeMX 默认 `5`，因此只能设 `5 ~ 15`）**；误设为 `0 ~ 4` 会立即触发 `vPortValidateInterruptPriority` 断言死循环！

---

## 2. TI MSPM0 (MSPM0G3507 / L1306) + CCS/Theia + SysConfig (`DL_*`)

### 2.1 SysConfig 宏映射与 `NVIC_EnableIRQ` 手动使能铁律
* **工作流规范**：一切引脚、定时器、串口句柄名称均以 `.syscfg` 编译后自动生成的 **`ti_msp_dl_config.h`** 为唯一准绳（如 `PWM_MOTOR_INST`、`TIMER_CTRL_INST_INT_IRQN`、`UART_0_INST_IRQHandler`），切勿手猜硬编码。
* **头号天坑（95% 初学者必踩）**：在 `.syscfg` 图形界面勾选了定时器中断、串口接收中断或外部引脚中断后，生成的 **`SYSCFG_DL_init()` 只开启了外设内部 `IMASK`，绝对不会自动开启 Cortex-M0+ 的 NVIC 中断通道！** 必须在 `SYSCFG_DL_init()` 后显式调用 `NVIC_EnableIRQ(...)`：

```c
/* ❌ 错误写法 (Buggy)：只调 SYSCFG_DL_init()，定时器与串口中断永远进不去！ */
int main(void) {
    SYSCFG_DL_init();
    DL_TimerG_startCounter(TIMER_CTRL_INST); // 计数器虽然在跑，但 NVIC 未放行，ISR 永远不触发！
    while (1) {}
}

/* ✅ 竞赛标准写法 (Contest-Grade)：按序清挂起 + 开启 NVIC 通道 + 启动硬件计数器 */
int main(void) {
    SYSCFG_DL_init();

    NVIC_ClearPendingIRQ(UART_VISION_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_VISION_INST_INT_IRQN);

    NVIC_EnableIRQ(INT_GROUP1_IRQN);          // GPIOA / GPIOB 外部编码器中断均汇聚于 GROUP1！

    NVIC_ClearPendingIRQ(TIMER_CTRL_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_CTRL_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_CTRL_INST);  // 最后启动定频控制环定时器

    while (1) { App_BackgroundLoop(); }
}
```

### 2.2 MSPM0G3507 多轮编码器终极方案（`GROUP1_IRQHandler` 极速解码）
* **硬件限制**：**MSPM0G3507 全片仅有 `TIMG8` 一个定时器具备硬件 QEI 正交解码功能**（`MSPM0L1306` 则一个都没有）！双轮或四轮麦轮/全向轮智能车必须依赖 GPIO 外部中断完成多路正交测速。
* **SysConfig 配置要点**：
  1. 将各轮编码器 A 相配置为 **Rising and Falling Edge（双边沿触发，即 2 倍频解码，兼顾分辨率与 CPU 中断负载）**，B 相配置为普通数字输入（仅在 ISR 中读电平判向）；
  2. 为所有编码器引脚开启硬件数字毛刺滤波（`Input Filter: Filter 3 / 8 ULPCLK Cycles`），滤除电机碳刷火花干扰。
* **极速 `GROUP1_IRQHandler` 实现**（单次读取整组 `MIS` 标志并一次性处理同一端口同时到达的多路边沿，绝不漏脉冲）：

```c
/* ✅ 竞赛标准写法 (Contest-Grade)：MSPM0G3507 四轮编码器 GROUP1_IRQHandler 零漏脉极速解码 */
#include "ti_msp_dl_config.h"

volatile int32_t g_enc_cnt[4] = {0}; // [0]=LF, [1]=LR, [2]=RF, [3]=RR

void GROUP1_IRQHandler(void) {
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        case DL_INTERRUPT_GROUP1_IIDX_GPIOA: {
            // 一次性取出 GPIOA 上所有已触发的编码器 A 相中断位（防止两轮同时跳变漏判！）
            const uint32_t mask_a = GPIO_ENC_LF_A_PIN | GPIO_ENC_LR_A_PIN;
            uint32_t mis = DL_GPIO_getEnabledInterruptStatus(GPIOA, mask_a);
            uint32_t din = GPIOA->DIN31_0; // 单指令直读整组输入寄存器电平

            if (mis & GPIO_ENC_LF_A_PIN) {
                bool a_hi = (din & GPIO_ENC_LF_A_PIN) != 0U;
                bool b_hi = (din & GPIO_ENC_LF_B_PIN) != 0U;
                g_enc_cnt[0] += (a_hi == b_hi) ? +1 : -1; // A、B 同相/异相判正反转
            }
            if (mis & GPIO_ENC_LR_A_PIN) {
                bool a_hi = (din & GPIO_ENC_LR_A_PIN) != 0U;
                bool b_hi = (din & GPIO_ENC_LR_B_PIN) != 0U;
                g_enc_cnt[1] += (a_hi == b_hi) ? +1 : -1;
            }
            DL_GPIO_clearInterruptStatus(GPIOA, mis);
            break;
        }
        case DL_INTERRUPT_GROUP1_IIDX_GPIOB: {
            const uint32_t mask_b = GPIO_ENC_RF_A_PIN | GPIO_ENC_RR_A_PIN;
            uint32_t mis = DL_GPIO_getEnabledInterruptStatus(GPIOB, mask_b);
            uint32_t din = GPIOB->DIN31_0;

            if (mis & GPIO_ENC_RF_A_PIN) {
                bool a_hi = (din & GPIO_ENC_RF_A_PIN) != 0U;
                bool b_hi = (din & GPIO_ENC_RF_B_PIN) != 0U;
                g_enc_cnt[2] += (a_hi == b_hi) ? +1 : -1;
            }
            if (mis & GPIO_ENC_RR_A_PIN) {
                bool a_hi = (din & GPIO_ENC_RR_A_PIN) != 0U;
                bool b_hi = (din & GPIO_ENC_RR_B_PIN) != 0U;
                g_enc_cnt[3] += (a_hi == b_hi) ? +1 : -1;
            }
            DL_GPIO_clearInterruptStatus(GPIOB, mis);
            break;
        }
        default:
            break;
    }
}
```

### 2.3 MSPM0 PWM 向下计数占空比反转大坑 (`DOWN` vs `UP` Counting)
* **90% STM32 迁移者必踩雷**：SysConfig 中新建 `PWM` 外设时，默认计数模式为 **`Edge-aligned Down Counting`（向下计数，从 `Period` 递减到 `0`）**！默认动作是在 `LOAD` 置高（或过 `CC` 置高、到 `ZERO` 拉低），导致直接调用 `DL_TimerG_setCaptureCompareValue(PWM_0_INST, val, DL_TIMER_CC_0_INDEX)` 时，**`val = 0` 输出 100% 满占空比，`val = Period` 才是 0% 停转**！一上电设 `0` 电机直接满速飞车！

```c
/* ❌ 错误写法 (Buggy)：在默认 Down Counting 模式下照搬 STM32 习惯，写 0 瞬间满速暴走！ */
DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_0_INDEX); // 实际 100% 占空比！

/* ✅ 竞赛标准写法 (Contest-Grade)：驱动层统一按 Period - duty 换算并严格限幅（TIMA/TIMG 通用） */
#define MOTOR_PWM_PERIOD  (4000U) // 对应 SysConfig 中配置的 PWM Period Count

void MSPM0_Motor_SetDuty(GPTIMER_Regs *timer, DL_TIMER_CC_INDEX ch, uint32_t duty_counts) {
    if (duty_counts > MOTOR_PWM_PERIOD) duty_counts = MOTOR_PWM_PERIOD;
    // 默认 Edge-aligned Down Counting 下：CC = Period - DutyCounts（0 -> CC=Period 即 0% 高电平）
    uint32_t cc_val = MOTOR_PWM_PERIOD - duty_counts;
    DL_Timer_setCaptureCompareValue(timer, cc_val, ch);
}
```

### 2.4 Cortex-M0+ 无硬件 FPU 算力加速：`MathACL` 协处理器与 Q16.16 定点化
* **瓶颈**：MSPM0G3507 (80MHz M0+) **无硬件 FPU**，软件 `atan2f`/`sinf`/`cosf` 极慢。但 G3507 配备了硬件 **`MATHACL`（数学加速器，需在 SysConfig 中添加 `MATHACL` 模块以使能电源域）**，可在约 20~30 个时钟周期内硬件算完 CORDIC `sincos`、`atan2`、`sqrt` 和除法！对于高频 PID，推荐直接采用 **Q16.16 定点数** 或 `MathACL`：

```c
/* ✅ 竞赛标准写法 A (Contest-Grade)：调用 MSPM0G3507 硬件 MathACL 极速计算同步 sin/cos */
#include <ti/driverlib/dl_mathacl.h>

// 输入 angle_rad 范围 [-pi, pi)，输出 sin/cos（利用 Q31 硬件 CORDIC，耗时 < 0.5us @80MHz）
void MSPM0_FastSinCos(float angle_rad, float *sin_out, float *cos_out) {
    static const DL_MathACL_operationConfig s_sincos_cfg = {
        .opType      = DL_MATHACL_OP_TYPE_SINCOS,
        .opSign      = DL_MATHACL_OPSIGN_SIGNED,
        .iterations  = 24U,                       // 24 次 CORDIC 迭代达最高精度
        .scaleFactor = 0U,
        .qType       = DL_MATHACL_Q_TYPE_Q31
    };
    // 将 [-pi, pi) 归一化映射到 Q31 [-1.0, +1.0)
    int32_t angle_q31 = (int32_t)(angle_rad * (2147483648.0f / 3.14159265f));
    DL_MathACL_startSinCosOperation(MATHACL, &s_sincos_cfg, (uint32_t)angle_q31);
    DL_MathACL_waitForOperation(MATHACL);
    int32_t cos_q31 = (int32_t)DL_MathACL_getResultOne(MATHACL); // RES1 = cos
    int32_t sin_q31 = (int32_t)DL_MathACL_getResultTwo(MATHACL); // RES2 = sin
    *cos_out = (float)cos_q31 * (1.0f / 2147483648.0f);
    *sin_out = (float)sin_q31 * (1.0f / 2147483648.0f);
}

/* ✅ 竞赛标准写法 B (Contest-Grade)：MSPM0L1306 / G3507 通用 Q16.16 纯整数增量式 PID (>2kHz 零浮点) */
typedef int32_t q16_t; // 1.0 表示为 (1 << 16) = 65536
#define Q16_FROM_FLOAT(x) ((q16_t)((x) * 65536.0f))
#define Q16_MUL(a, b)     ((q16_t)(((int64_t)(a) * (int64_t)(b)) >> 16))

int32_t PID_Incremental_Q16(q16_t kp, q16_t ki, q16_t kd, int32_t err, int32_t *e1, int32_t *e2, int32_t *out) {
    q16_t du = Q16_MUL(kp, (err - *e1) << 16)
             + Q16_MUL(ki, err << 16)
             + Q16_MUL(kd, (err - 2 * (*e1) + (*e2)) << 16);
    *e2 = *e1;
    *e1 = err;
    *out += (du >> 16);
    return *out;
}
```

---

## 3. ESP32 (ESP32 / ESP32-S3) + ESP-IDF (CMake + SMP FreeRTOS)

### 3.1 双核物理隔离：Core 0 协议栈 vs Core 1 硬实时运动控制
* **SMP 调度抖动根源**：ESP-IDF 默认将 Wi-Fi、BLE、LwIP TCP/IP、HTTP 图传与系统事件循环全部跑在 **Core 0 (`PRO_CPU`)** 上，且 Wi-Fi 底层高优先级任务与中断会频繁抢占 Core 0 数百微秒至数毫秒。
* **双核绑定准则**：严禁使用不绑核的 `xTaskCreate()` 运行控制环！必须用 **`xTaskCreatePinnedToCore(..., 1)` 将 IMU 采样、运动学解算、串级 PID 独占固定到 Core 1 (`APP_CPU`)**，把 Wi-Fi 调参/图传/MQTT 全部留在 Core 0。

### 3.2 高精度定频控制：抛弃粗糙的 `vTaskDelay(1)`，改用 `gptimer` ISR 唤醒
* **痛点**：ESP-IDF 默认 `CONFIG_FREERTOS_HZ = 100`（1 个 Tick 高达 `10ms`！），即便改为 `1000Hz`，`vTaskDelay(1)` 也只是相对延时且受调度影响抖动巨大。
* **1kHz+ 硬实时标准范式**：使用硬件通用定时器 `driver/gptimer.h` 在报警中断中通过 `vTaskNotifyGiveFromISR` 直接唤醒绑在 Core 1 的最高优先级控制任务（抖动 `< 5μs`），或使用 `esp_timer_start_periodic()` 高优先级软定时回调：

```c
/* ❌ 错误写法 (Buggy)：未绑核 + 默认 100Hz Tick 下 vTaskDelay(1) 实际休眠 10ms（或变为 0 饿死看门狗） */
void ctrl_task_buggy(void *arg) {
    while (1) {
        PID_Step();
        vTaskDelay(pdMS_TO_TICKS(1)); // 在 100Hz 默认配置下 pdMS_TO_TICKS(1) == 0，死循环触发 Task WDT！
    }
}

/* ✅ 竞赛标准写法 (Contest-Grade)：gptimer 1kHz 硬件中断 + Core 1 独占最高优先级任务通知 */
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static TaskHandle_t s_ctrl_task_hdl = NULL;

static bool IRAM_ATTR on_ctrl_timer_alarm_cb(gptimer_handle_t timer,
                                             const gptimer_alarm_event_data_t *edata,
                                             void *user_ctx) {
    BaseType_t high_task_awoken = pdFALSE;
    vTaskNotifyGiveFromISR(s_ctrl_task_hdl, &high_task_awoken);
    return (high_task_awoken == pdTRUE); // 立即触发上下文切换
}

static void Control_Task_Core1(void *arg) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // 精确 1.000ms 硬件中断唤醒
        MotionControl_Update_1kHz();
    }
}

void Control_System_Start(void) {
    // 1. 先创建绑定在 Core 1 的高优先级控制任务
    xTaskCreatePinnedToCore(Control_Task_Core1, "ctrl_1k", 4096, NULL, configMAX_PRIORITIES - 2, &s_ctrl_task_hdl, 1);

    // 2. 配置 1MHz 分辨率 gptimer，每 1000 ticks (1ms) 周期触发报警
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1 tick = 1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_cfg, &gptimer));
    gptimer_event_callbacks_t cbs = { .on_alarm = on_ctrl_timer_alarm_cb };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
    gptimer_alarm_config_t alarm_cfg = {
        .reload_count = 0,
        .alarm_count = 1000,      // 1000us = 1kHz
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_cfg));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}
```

### 3.3 新版硬件外设：`esp_driver_pcnt` 四倍频正交解码与 `mcpwm_prelude.h` 电机驱动
* **严禁使用 ESP-IDF v4 废弃的旧版 `driver/pcnt.h` / `driver/mcpwm.h`**！ESP-IDF v5.x 必须使用句柄式新驱动：
  * **硬件正交编码器 (`driver/pulse_cnt.h`)**：单 Unit 挂双 Channel 实现零 CPU 开销硬件 **4 倍频正交计数**，开启 `pcnt_unit_set_glitch_filter`（如 `1000ns`）硬件滤除毛刺，并配置高低限观察点（Watchpoint）加 `flags.accum_count = true` 自动扩展突破 16 位硬件计数器上限！
  * **电机专用 PWM (`driver/mcpwm_prelude.h`)**：通过 `mcpwm_new_timer` → `mcpwm_new_operator` → `mcpwm_new_comparator` → `mcpwm_new_generator` 级联构建，原生支持互补死区 (`mcpwm_generator_set_dead_time`) 与过流故障硬件刹车 (`mcpwm_new_gpio_fault`)。

```c
/* ✅ 竞赛标准写法 (Contest-Grade)：ESP-IDF v5.x PCNT 硬件 4 倍频正交编码器 + 毛刺滤波 + 自动溢出累加 */
#include "driver/pulse_cnt.h"

pcnt_unit_handle_t ESP32_Encoder_Init_4x(int gpio_a, int gpio_b) {
    pcnt_unit_config_t unit_cfg = {
        .high_limit = 10000,
        .low_limit  = -10000,
        .flags.accum_count = true, // 开启硬件溢出观察点自动软件累加，突破 16 位限制！
    };
    pcnt_unit_handle_t unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &unit));

    pcnt_glitch_filter_config_t filter_cfg = { .max_glitch_ns = 1000 }; // 滤除 <1us 电机电磁毛刺
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter_cfg));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, unit_cfg.high_limit));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, unit_cfg.low_limit));

    // Channel A + Channel B 交叉互锁实现完整 4 倍频解码
    pcnt_chan_config_t chan_a_cfg = { .edge_gpio_num = gpio_a, .level_gpio_num = gpio_b };
    pcnt_channel_handle_t chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_a_cfg, &chan_a));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    pcnt_chan_config_t chan_b_cfg = { .edge_gpio_num = gpio_b, .level_gpio_num = gpio_a };
    pcnt_channel_handle_t chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_b_cfg, &chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    ESP_ERROR_CHECK(pcnt_unit_start(unit));
    return unit; // 运行期随时调用 pcnt_unit_get_count(unit, &total_pulses) 读取累计脉冲
}
```

### 3.4 Flash Cache 挂起期间的 `IRAM_ATTR` 与 `DRAM_ATTR` 生死红线
* 当主循环执行 **NVS 调参保存、SPI Flash 擦写、OTA 或部分 Wi-Fi 校准动作**时，ESP32 会临时挂起外部 SPI Flash 的指令/数据 Cache。
* 若一个注册为 `ESP_INTR_FLAG_IRAM` 的硬件中断（如 `gptimer` 回调或急停外部中断）：
  1. ISR 函数本体或它调用的子函数未标 **`IRAM_ATTR`**（代码落在 Flash `.flash.text`）；
  2. 或 ISR 内读取了 `const float lut[]` / `switch` 跳转表而未标 **`DRAM_ATTR`**（常量默认落在 Flash `.flash.rodata`）——
  一旦与 NVS 保存撞车，CPU 立即触发 **`Guru Meditation Error: Core 1 panic'ed (Cache disabled but cached memory region accessed)`** 重启！

---

## 4. Renesas RA (RA4M2 / RA6M5 / RA8M1) + RASC/FSP + Keil MDK

### 4.1 FSP 驱动显式生命周期（配置器生成结构体 ≠ 自动打开外设！）
* **头号误区**：从 STM32CubeMX 转到 Renesas Smart Configurator (`RASC`) 的选手常以为生成代码后 PWM、UART、ADC 就已初始化完毕。**FSP 的 `hal_entry()` 进入时所有外设均处于关闭态**！必须按严格生命周期调用 API：
  * **GPT 定时器 PWM**：`R_GPT_Open(&g_timer0_ctrl, &g_timer0_cfg)` → `R_GPT_Start(&g_timer0_ctrl)` → 动态调占空比调用 `R_GPT_DutyCycleSet(&g_timer0_ctrl, duty_raw_counts, GPT_IO_PIN_GTIOCA)`（注意第三个参数指定引脚 `GPT_IO_PIN_GTIOCA` / `GPT_IO_PIN_GTIOCB`，第二个参数是**原始计数值而非百分比**！）。
  * **GPT 硬件正交编码器（Phase Counting Mode）**：在 RASC 中添加 GPT Stack，将 Mode 设为 `Timer, Periodic` 并在 `Input -> Phase Counting Mode` 选择 `Mode 1`（4 倍频正交解码，将 `GTIOCA`/`GTIOCB` 绑定编码器 A/B 相）。代码中 `R_GPT_Open` + `R_GPT_Start` 后，通过 `timer_status_t status; R_GPT_StatusGet(&g_enc_gpt_ctrl, &status);` 读取 `status.counter` 做模差分即可。
  * **SCI UART 异步通信（`r_sci_uart` / RA8M1 `r_sci_b_uart`）**：调用 `R_SCI_UART_Open(&g_uart0_ctrl, &g_uart0_cfg)` 后，无需挂载单字节接收缓冲即可在 Callback 中直接通过 **`p_args->event == UART_EVENT_RX_CHAR`** 拿到硬件接收到的字节 `(uint8_t)p_args->data` 压入 RingBuffer，极度轻量可靠！

```c
/* ❌ 错误写法 (Buggy)：未调 R_GPT_Open/Start 直接设占空比，或把百分比 50 当作计数值传入 */
void RA_Motor_Buggy(void) {
    R_GPT_DutyCycleSet(&g_pwm_motor_ctrl, 50, GPT_IO_PIN_GTIOCA); // 未 Open 返回 FSP_ERR_NOT_OPEN！
}

/* ✅ 竞赛标准写法 (Contest-Grade)：完整 FSP Open/Start 生命周期 + UART_EVENT_RX_CHAR 零丢包回调 */
#include "hal_data.h"

static uint32_t s_pwm_period_counts = 0;

void RA_BSP_Init(void) {
    fsp_err_t err = R_GPT_Open(&g_pwm_motor_ctrl, &g_pwm_motor_cfg);
    assert(FSP_SUCCESS == err);

    timer_info_t info;
    R_GPT_InfoGet(&g_pwm_motor_ctrl, &info);
    s_pwm_period_counts = info.period_counts; // 动态读取 RASC 配置的周期总计数值

    R_GPT_DutyCycleSet(&g_pwm_motor_ctrl, 0U, GPT_IO_PIN_GTIOCA);
    R_GPT_Start(&g_pwm_motor_ctrl);

    err = R_SCI_UART_Open(&g_uart_vision_ctrl, &g_uart_vision_cfg);
    assert(FSP_SUCCESS == err);
}

/* RASC 中配置的 UART Callback 函数 */
void user_vision_uart_callback(uart_callback_args_t *p_args) {
    if (p_args->event == UART_EVENT_RX_CHAR) {
        RingBuf_PushByte(&g_vision_rb, (uint8_t)p_args->data); // 单字节极速入环形队列
    } else if (p_args->event == UART_EVENT_TX_COMPLETE) {
        g_uart_tx_busy = false;
    }
}
```

### 4.2 RASC 默认栈 (`0x400`) 与堆 (`0`) 瞬间 `NMI` / `HardFault` 陷阱
* **致命默认配置**：RASC 新建裸机工程时，在 `BSP -> Properties -> RA Common` 下：
  * **`Main Stack Size (bytes)` 默认仅为 `0x400` (1024 字节)！**
  * **`Heap Size (bytes)` 默认为 `0`！**
* **爆雷现象**：Cortex-M33 (RA4M2 / RA6M5) 和 Cortex-M85 (RA8M1) 硬件启用了栈指针监控器（MSP Stack Monitor / `PSPLIM` / `MSPLIM`）。只要调用一次 `snprintf(buf, sizeof(buf), "%.2f", angle)` 或跑一个 3×3 矩阵运算，栈深瞬间突破 `1024` 字节，硬件立即触发 **`NMI_Handler` (`BSP_GRP_IRQ_MPU_STACK`) 或 `UsageFault`（栈溢出异常）**；而 `printf` 内部若尝试申请堆缓冲区，因 `Heap = 0` 直接返回失败或死机！
* **必检动作**：建工程第一步打开 RASC `BSP -> Properties`，将 **`Main Stack Size` 改为 `0x2000` (8KB)**，若使用标准库格式化将 **`Heap Size` 改为 `0x1000` (4KB)**，生成代码后打开 `ra_cfg/fsp_cfg/bsp/bsp_cfg.h` 核验宏 `BSP_CFG_STACK_MAIN_BYTES (0x2000)`！

### 4.3 Cortex-M33 / M85 微秒级精确计时 (`DWT->CYCCNT`)
* `R_BSP_SoftwareDelay()` 仅供阻塞等待，无法测量控制周期 `dt`。在 RA4M2 / RA6M5 / RA8M1 上应开启内核 DWT 硬件周期计数器实现纳秒级零中断开销计时：

```c
/* ✅ 竞赛标准写法 (Contest-Grade)：Cortex-M33 / M85 开启 DWT 硬件周期计数器精确测量真实 dt */
void RA_DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 使能跟踪调试模块
#if (__CORTEX_M == 85U) || defined(RENESAS_CORTEX_M85)
    DWT->LAR = 0xC5ACCE55U;                         // 部分 M7/M85 内核需解锁 DWT 写保护
#endif
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline float RA_DWT_GetDeltaSeconds(uint32_t *last_cyccnt) {
    uint32_t now = DWT->CYCCNT;
    uint32_t diff = now - *last_cyccnt;             // 32 位无符号减法自动处理溢出回绕
    *last_cyccnt = now;
    return (float)diff / (float)SystemCoreClock;    // 真实物理 dt（秒）
}
```
