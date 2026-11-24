# 嵌入式控制固件 C99 四层代码审计清单 (4-Layer Code Audit Checklist)

> **适用场景**：全国大学生电子设计竞赛（NUEDC / 电赛）、嵌入式芯片与系统设计竞赛、智能车竞赛等硬实时运动控制固件审查。  
> **审计原则**：凡涉及中断（ISR）、DMA、浮点控制律、外设总线轮询及执行器输出的代码，必须逐层通过以下四道防线审查。

---

## Layer 1: 并发、中断安全与原子性 (Concurrency, ISR Safety & Atomicity)

### 1.1 ISR 内阻塞延时、重型格式化与执行时间预算超标
* **故障机理**：
  1. 在高频控制中断（如 1kHz 编码器/PID 环、10kHz 电流环）中调用 `HAL_Delay()`：若当前外设中断抢占优先级高于或等于 `SysTick_IRQn`，`uwTick` 永远无法自增，CPU **当场永久死锁**。
  2. 在 ISR 内调用 `printf` / `sprintf`（尤其是含 `%f` 浮点格式化）：单次耗时可达数百微秒至数毫秒，且占用大量栈空间，直接击穿控制周期，导致下一拍中断丢失（Overrun）和微分项 `Kd` 剧烈震荡。
* **硬实时预算铁律**：**ISR 实测最坏执行时间（WCET）必须 `< 中断周期的 10% ~ 20%`**（例如 1kHz / 1ms 控制中断，ISR 耗时须 `< 150μs`）。必须在 ISR 首尾翻转测试 GPIO（或读取 `DWT->CYCCNT`）实测验证。

```c
/* ❌ 错误写法 (Buggy)：ISR 内死延时 + 阻塞串口打印，必死锁或撑爆控制周期 */
void TIM6_DAC_IRQHandler(void) {
    __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
    PID_Step_1kHz();
    printf("vel=%.2f, pwm=%d\r\n", g_vel, g_pwm); // 阻塞数毫秒 + 非重入栈爆炸！
    HAL_Delay(2);                                 // 若 TIM6 优先级 >= SysTick，永久卡死！
}

/* ✅ 竞赛标准写法 (Contest-Grade)：ISR 仅做定频采样/控制闭环 + 置标志，慢速遥测下放主循环 */
volatile uint8_t g_ctrl_tick_1khz = 0;

void TIM6_DAC_IRQHandler(void) {
    if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
        uint32_t t0 = DWT->CYCCNT;       // DWT 周期计数监控 WCET
        ControlLoop_FastStep_1kHz();     // 纯定点/单精度浮点算术 + 写寄存器 (< 80us)
        g_isr_cycles = DWT->CYCCNT - t0;
        g_ctrl_tick_1khz = 1;
    }
}
```

### 1.2 缺失 `volatile` 与“伪原子性”数据撕裂 (Data Tearing)
* **故障机理**：
  1. **漏写 `volatile`**：跨 ISR 与主循环（或 DMA）共享的标志位/变量未加 `volatile`，在 Keil AC6 `-O2` / GCC `-O2` 下被编译器缓存进通用寄存器，`while (!rx_done)` 编译成死循环 `B .`。
  2. **误把 `volatile` 当原子锁**：在 32 位 Cortex-M 内核上，单条 `LDR`/`STR` 仅能保证**对齐的 8/16/32 位标量**读写原子性。对于 **`uint64_t`、`double` 或多字段结构体**（如 `imu.pitch, imu.roll, imu.yaw`），主循环读完 `pitch` 后若被 DMA/TIM ISR 抢占并更新整包姿态，随后主循环读到的 `roll/yaw` 来自新帧——**新旧帧拼接撕裂**会导致姿态解算或串级 PID 瞬间输出尖峰脉冲！

```c
typedef struct {
    float pitch, roll, yaw;
    float gyro_z;
    uint32_t timestamp_us;
} IMU_State_t;

/* ❌ 错误写法 (Buggy)：以为加了 volatile 就安全，多字段结构体读取发生撕裂 (Torn Read) */
volatile IMU_State_t g_imu; // ISR 随时写入，主循环分多次 LDR 读取，新旧数据混杂！

void Balance_Task(void) {
    float p = g_imu.pitch;
    // <--- 若此处恰好触发 UART/SPI DMA 完成中断更新了 g_imu，下面读到的 gyro_z 与 p 不同步！
    float gz = g_imu.gyro_z;
    Motor_SetOut(kp * p + kd * gz);
}

/* ✅ 竞赛标准写法 A (Contest-Grade)：保存恢复式极短临界区（禁止无脑 __enable_irq 破坏嵌套！） */
static IMU_State_t g_imu_shared;

static inline void IMU_ReadSnapshot_Critical(IMU_State_t *out) {
    uint32_t primask = __get_PRIMASK(); // 记录进入前的中断屏蔽状态（FreeRTOS 请用 taskENTER_CRITICAL）
    __disable_irq();
    *out = g_imu_shared;                // 仅做几十字节内存拷贝 (< 1us)，严禁在临界区内算 PID
    __set_PRIMASK(primask);             // 精确恢复原中断状态
}

/* ✅ 竞赛标准写法 B (Contest-Grade)：无锁顺序锁 (Seqlock)，零关中断抖动，适合高频单写多读 */
typedef struct {
    volatile uint32_t seq;              // 偶数=稳定可读，奇数=ISR 正在写入
    IMU_State_t data;
} IMU_Seqlock_t;

static IMU_Seqlock_t g_imu_seq;

// [写端：仅在单一 ISR 中调用]
void IMU_ISR_Publish(const IMU_State_t *new_sample) {
    g_imu_seq.seq++;                    // 变为奇数，标记写入中
    __DMB();                            // 内存屏障，防止编译器/CPU 乱序重排
    g_imu_seq.data = *new_sample;
    __DMB();
    g_imu_seq.seq++;                    // 变为偶数，标记写入完成
}

// [读端：主循环或低优先级任务调用]
void IMU_ReadSnapshot_LockFree(IMU_State_t *out) {
    uint32_t s1, s2;
    do {
        s1 = g_imu_seq.seq;
        if (s1 & 1U) continue;          // 若正处于奇数写入态，原地重试
        __DMB();
        *out = g_imu_seq.data;
        __DMB();
        s2 = g_imu_seq.seq;
    } while (s1 != s2);                 // 若读取期间被 ISR 插队修改，自动重读一轮
}
```

### 1.3 不可重入函数隐患 (Reentrancy Hazards)
* **故障机理**：C 标准库中 `strtok()`（内部维护隐藏 `static char *` 指针）、`malloc()/free()`（破坏堆链表结构）、非线程安全版 `snprintf()/printf()` 在主循环与 ISR（或多个 RTOS 任务）并发调用时，静态状态互相踩踏，引发极难复现的偶发 `HardFault`。
* **修复准则**：CSV/NMEA 文本协议解析一律使用可重入版 `strtok_r(buf, ",", &saveptr)` 或手写有限状态机；严禁在初始化完成后（尤其是 ISR 内）调用 `malloc`/`free`。

---

## Layer 2: 内存、DMA、Cache 一致性与硬件故障恢复 (Memory, DMA, Cache & Hardware Faults)

### 2.1 DMA 缓冲区生命周期、内存域隔离与 D-Cache 一致性
* **故障机理**：
  1. **栈变量喂给异步 DMA**：函数内定义局部数组 `uint8_t cmd[16]` 传给 `HAL_UART_Transmit_DMA()` 后函数立即返回，栈空间被后续中断或函数调用覆写，串口实际发出一串乱码；若是 `Receive_DMA` 指向栈帧，DMA 稍后回写数据直接改写函数返回地址 `LR`，当场 `HardFault`！
  2. **跨总线矩阵访问失败（STM32H7 典型坑）**：STM32H7 的 `DTCM RAM (0x20000000)` 虽与内核零等待直连，但**普通 `DMA1`/`DMA2` 根本无权访问 DTCM**（传输静默失败或置位 `TEIF`）。必须通过链接脚本段属性放到 D1 `AXI SRAM (0x24000000)` 或 D2 `SRAM1/2 (0x30000000)`。
  3. **Cortex-M7 (STM32H7) 开启 D-Cache 后的数据陈旧**：DMA 直接读写物理 SRAM 而绕过 CPU Cache。发送前不 Clean Cache 会发出旧数据；接收后不 Invalidate Cache 会让 CPU 永远读到 Cache 里的旧值（且缓冲区首尾未按 **32 字节 Cache Line 对齐**时，`Invalidate` 会把相邻全局变量顺手抹掉！）。

```c
/* ❌ 错误写法 (Buggy)：栈上局部数组传给非阻塞 DMA + H7 下未对齐未刷 Cache */
void Vision_SendAck(uint8_t task_id) {
    uint8_t frame[4] = {0xAA, task_id, (uint8_t)(0xAA + task_id), 0x55};
    HAL_UART_Transmit_DMA(&huart1, frame, 4); // 致命错误！函数返回瞬间 frame 内存已失效
}

/* ✅ 竞赛标准写法 (Contest-Grade)：静态生命周期 + 内存域指定 + 32 字节对齐与 Cache 维护 */
#if defined(STM32H7xx)
  // H7：放入 D2 SRAM 段（或 MPU 配置为 Non-Cacheable 的专用段），严格 32 字节对齐
  #define DMA_BUF_ATTR __attribute__((section(".RAM_D2"), aligned(32)))
#else
  #define DMA_BUF_ATTR __attribute__((aligned(4)))
#endif

static DMA_BUF_ATTR uint8_t s_tx_dma_buf[32]; // 长度亦凑整为 32 字节整数倍以防污染邻近变量

bool Vision_SendAck_Safe(uint8_t task_id) {
    if (huart1.gState != HAL_UART_STATE_READY) return false; // 防止上次发送未完强行覆写缓冲区
    s_tx_dma_buf[0] = 0xAA;
    s_tx_dma_buf[1] = task_id;
    s_tx_dma_buf[2] = (uint8_t)(0xAAU + task_id);
    s_tx_dma_buf[3] = 0x55;
#if defined(STM32H7xx)
    SCB_CleanDCache_by_Addr((uint32_t *)s_tx_dma_buf, sizeof(s_tx_dma_buf));
#endif
    return (HAL_UART_Transmit_DMA(&huart1, s_tx_dma_buf, 4) == HAL_OK);
}
```

### 2.2 栈溢出三大杀手 (Stack Overflow Killers)
* **审计红线**：MCU 启动文件（`startup_*.s`）默认栈大小往往仅有 **`0x400` (1KB，如 STM32CubeMX / Renesas RASC 默认值)**。以下三种写法必爆栈：
  1. 函数内声明大局部缓冲区（如 `float fft_buf[512];` 瞬间吃掉 2KB 栈）；
  2. 变长数组 **VLA (`float mat[n][n]`)** 或 `alloca()`（`n` 一旦受异常输入污染变大，栈指针 `SP` 直接击穿 `.bss` 段）；
  3. 递归调用（树搜索、迷宫 DFS 未改写为显式静态栈迭代）。

### 2.3 无界硬件标志死等 (`while(FLAG)` 电磁干扰死机)
* **故障机理**：电赛现场大功率直流有刷电机换向、电磁铁/继电器通断会在地线上耦合数十伏尖峰毛刺，导致 I2C/SPI 外设状态机偶发卡死。裸写 `while (I2C_BUSY)` 无超时机制，整台机器人会在赛程中途瞬间“植物人”（PWM 锁死在最后占空比疯跑）。

```c
/* ❌ 错误写法 (Buggy)：死等硬件标志位，受一发 EMI 毛刺干扰即永久死机 */
void AS5600_ReadRaw(uint16_t *angle) {
    while (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY)) { /* 死循环无退路 */ }
    // ...
}

/* ✅ 竞赛标准写法 (Contest-Grade)：有界超时退出 + 错误计数 + 硬件外设自愈复位 */
bool AS5600_ReadRaw_Safe(uint16_t *angle) {
    uint32_t timeout = 2000U; // 约数十微秒上限，绝不拖垮控制周期
    while (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY)) {
        if (--timeout == 0U) {
            I2C1_BusRecovery_And_Reinit(); // 触发 9 时钟总线恢复 + 外设软复位
            return false;                  // 向上层报告本帧采样无效，改用上一拍预测值
        }
    }
    uint8_t buf[2];
    if (HAL_I2C_Mem_Read(&hi2c1, (0x36 << 1), 0x0C, I2C_MEMADD_SIZE_8BIT, buf, 2, 2) != HAL_OK) {
        I2C1_BusRecovery_And_Reinit();
        return false;
    }
    *angle = ((uint16_t)(buf[0] & 0x0FU) << 8) | buf[1];
    return true;
}
```

### 2.4 UART 过载锁死 (`ORE`/`FE`) 与 I2C `SDA` 死锁自愈
* **故障 1：高速视觉/IMU 串口静默“失聪” (`ORE` Overrun Error)**  
  OpenMV / K230 / 外置陀螺仪以 `460800` / `921600` 波特率上电即狂发字节流。当 MCU 刚完成 `UART_Init` 但尚未挂载 DMA/RX 中断，或某段关中断略长导致 RDR 寄存器未及时取走，硬件置位 **`ORE`（溢出错误）或 `FE`（帧错误）**。在 STM32 / MSPM0 等芯片上，`ORE` 未清除前将不再正常触发后续字节接收，或死循环重入错误中断！
* **故障 2：MCU 按复位键后 I2C 永远报 `BUSY` (`SDA` 被从机拉死)**  
  当 MPU6050 / OLED 正在往 `SDA` 输出低电平 `0` 时，若按下 MCU 复位键，MCU 丢失上下文而从机仍在死等 `SCL` 时钟走完当前字节，`SDA` 被死死拉低，硬件 I2C 初始化后寸步难行。**必须在上电初始化及超时恢复中执行 9 个 `SCL` 脉冲解锁序列**：

```c
/* ✅ 竞赛标准写法 (Contest-Grade)：I2C 总线死锁 9-SCL 脉冲解锁序列（通用平台原理） */
void I2C1_BusRecovery_And_Reinit(void) {
    HAL_I2C_DeInit(&hi2c1);

    // 1. 将 SCL/SDA 临时切为开漏输出 (Open-Drain + Pull-Up)
    GPIO_InitTypeDef gpio = {
        .Pin = GPIO_PIN_6 | GPIO_PIN_7, // PB6=SCL, PB7=SDA
        .Mode = GPIO_MODE_OUTPUT_OD,
        .Pull = GPIO_PULLUP,
        .Speed = GPIO_SPEED_FREQ_HIGH
    };
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); // 释放 SDA

    // 2. 发送最多 9 个 SCL 时钟脉冲，让卡在半途的从机吐完剩余 bit 并释放 SDA
    for (int i = 0; i < 9; i++) {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) break; // SDA 已释放则提前结束
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        DWT_Delay_us(5);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        DWT_Delay_us(5);
    }

    // 3. 手动构造一个标准 STOP 条件 (SCL 高电平期间 SDA 由低拉高)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    DWT_Delay_us(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    DWT_Delay_us(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    DWT_Delay_us(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    DWT_Delay_us(5);

    // 4. 恢复硬件 I2C 控制器
    MX_I2C1_Init();
}
```

---

## Layer 3: FPU、定点数与数值计算陷阱 (FPU, Fixed-Point & Numerical Pitfalls)

### 3.1 单精度 FPU 的双精度隐式提升陷阱 (`double` Promotion Trap)
* **故障机理**：STM32F4 / STM32G4 (Cortex-M4F)、RA4M2 / RA6M5 (Cortex-M33)、ESP32-S3 搭载的均为**单精度硬件 FPU（仅硬件加速 32-bit `float`）**。C99 语法规定：不带 `f` 后缀的浮点常量（如 `0.05`、`3.1415926`）类型为 64-bit `double`！表达式中只要混入一个 `double` 字面量或误调 `sin()`/`cos()`/`atan2()`/`sqrt()`/`fabs()`，整条算式全部被迫走**纯软件双精度仿真库**，单条三角函数或乘除耗时暴增 **15 ~ 40 倍**！
* **编译防线**：GCC / Clang / Keil AC6 编译选项强制加上 `-Wdouble-promotion`。

```c
/* ❌ 错误写法 (Buggy)：漏写 'f' 后缀 + 误调双精度数学库，单精度 FPU 完全闲置！ */
float Complementary_Filter(float acc_y, float acc_z, float gyro_x, float prev_angle) {
    float acc_angle = atan2(acc_y, acc_z) * 57.2957795;         // 软双精度 atan2 + 软双精度乘法！
    return 0.98 * (prev_angle + gyro_x * 0.001) + 0.02 * acc_angle; // 全程软件 double 运算！
}

/* ✅ 竞赛标准写法 (Contest-Grade)：全链路 'f' 字面量 + 单精度数学函数族 (*f) */
float Complementary_Filter_Fast(float acc_y, float acc_z, float gyro_x, float prev_angle) {
    float acc_angle = atan2f(acc_y, acc_z) * 57.2957795f;       // 硬件 FPU + 快速单精度库
    return 0.98f * (prev_angle + gyro_x * 0.001f) + 0.02f * acc_angle;
}
```

### 3.2 `float` 累加器“大数吃小数”精度枯竭
* **故障机理**：IEEE-754 `float` 仅有 23 位尾数（加上隐含位共 24 bit，约 7 位十进制有效精度）。当用 `float t += 0.0001f`（10kHz 积分）或里程计 `dist_mm += ds` 长时间累加：一旦累加和达到 `2048.0f`，最低有效位间距 `ULP = 2048 * 2^-23 ≈ 0.000244f > 0.0001f`——此后执行 `t += 0.0001f` 结果**恒等于原来的 `t`，时间与积分彻底停滞**！

```c
/* ❌ 错误写法 (Buggy)：用 float 直接累加微小步长 dt 或微米位移，几十分钟后积分彻底卡死 */
static float s_run_time_s = 0.0f;
void Tick_10kHz(void) {
    s_run_time_s += 0.0001f; // 当 s_run_time_s 涨到 2048.0f 后，每次 +0.0001f 直接被舍入为 +0！
}

/* ✅ 竞赛标准写法 A (Contest-Grade)：离散节拍/编码器脉冲一律用 int64_t 精确累加，用时再转 float */
static int64_t s_total_ticks_10khz = 0;
static int64_t s_total_enc_pulses  = 0;

float Get_TotalDistance_Meters(void) {
    return (float)s_total_enc_pulses * METERS_PER_PULSE_F; // 永无舍入漂移
}

/* ✅ 竞赛标准写法 B (Contest-Grade)：连续物理量积分（如长时航向/PID 积分器）使用 Kahan 补偿求和 */
typedef struct {
    float sum;
    float comp; // 捕获上一轮被吞掉的低位尾数误差
} KahanAcc_t;

static inline float Kahan_Add(KahanAcc_t *acc, float delta) {
    float y = delta - acc->comp;
    float t = acc->sum + y;
    acc->comp = (t - acc->sum) - y;
    acc->sum = t;
    return acc->sum;
}
```

### 3.3 整数除法截断、有/无符号比较灾难与 16 位定时器回绕
* **三大经典数值惨案**：
  1. `float kp = 1 / 2;` → 先按整数除得 `0`，再转 `0.0f`。
  2. `int32_t pwm = -200; uint32_t max_pwm = 1000; if (pwm > max_pwm)` → `pwm` 被隐式提升为 `uint32_t` (`4294967096U`)，判断为真，反转瞬间变正向满油门！
  3. 16 位硬件编码器/定时器从 `65530` 正转越过 `65535` 变为 `10`：手写 `now - last = -65520` 算出巨型负速度。必须利用补码同余性质强转为 `int16_t`。

```c
/* ❌ 错误写法 (Buggy)：整数截断 + 有无符号混比 + 编码器溢出跳变 */
uint32_t limit = 800;
int32_t out = (1 / 2) * err;     // 恒为 0！
if (out > limit) out = limit;    // 当 out < 0 时条件竟然成立，负输出全被篡改为 +800！
int32_t vel = enc_now - enc_old; // 越过 65535 回绕时瞬间跳变为 -65530，车轮猛踹一脚！

/* ✅ 竞赛标准写法 (Contest-Grade)：显式有符号域比较 + 16 位补码无分支差分 */
const int32_t limit_s32 = 800;
float out_f = 0.5f * (float)err;
int32_t out_s32 = (int32_t)out_f;
if (out_s32 >  limit_s32) out_s32 =  limit_s32;
if (out_s32 < -limit_s32) out_s32 = -limit_s32;

// 只要两次采样间脉冲变化量在 [-32768, +32767] 内，正反转过零回绕 100% 精确无分支！
int16_t delta_enc = (int16_t)((uint16_t)enc_now - (uint16_t)enc_old);
enc_old = enc_now;
```

### 3.4 `NaN` / `Inf` 传染链防护与反三角函数定义域钳位
* **故障机理**：机械臂逆运动学余弦定理 `cos_theta = (L1*L1 + L2*L2 - r*r) / (2*L1*L2)` 中，哪怕目标点恰在工作空间边界，单精度浮点舍入也常算出 `1.0000002f`。直接喂给 `acosf(1.0000002f)` 返回 **`NaN`**！一旦 `NaN` 混入 PID 积分器或 IIR 滤波器状态变量，所有比较运算（`x > max`、`x < min`）对 `NaN` **全部返回 `false`**，常规限幅器瞬间被穿透，随后 `(int32_t)NaN` 触发 C 语言未定义行为写入 PWM 寄存器！

```c
/* ❌ 错误写法 (Buggy)：裸调 acosf/sqrtf + 常规 if 限幅挡不住 NaN 穿透 */
float theta = acosf(cos_val);         // cos_val = 1.0000001f 时产出 NaN！
if (pwm_f >  1000.0f) pwm_f =  1000.0f; // NaN > 1000.0f 为 false！限幅失效！
if (pwm_f < -1000.0f) pwm_f = -1000.0f; // NaN < -1000.0f 为 false！限幅失效！
TIM1->CCR1 = (uint32_t)pwm_f;           // 未定义行为，电机失控！

/* ✅ 竞赛标准写法 (Contest-Grade)：定义域硬钳位 + 写硬件寄存器前强制 isfinite 熔断保护 */
#include <math.h>

static inline float Clamp_F32(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline float Safe_Acosf(float x) {
    return acosf(Clamp_F32(x, -1.0f, 1.0f)); // 彻底根除定义域越界 NaN
}

static inline float Safe_Sqrtf(float x) {
    return (x <= 0.0f) ? 0.0f : sqrtf(x);
}

void Motor_ApplyPWM_Guarded(PID_Ctrl_t *pid, float pwm_cmd) {
    if (!isfinite(pwm_cmd)) {                // 拦截 NaN / +Inf / -Inf
        PID_Reset(pid);                      // 立即净化被传染的积分器与历史状态
        Hardware_PWM_StopAll();              // 紧急关断输出
        g_sys_fault_flags |= FAULT_MATH_NAN;
        return;
    }
    pwm_cmd = Clamp_F32(pwm_cmd, -1000.0f, 1000.0f);
    Hardware_PWM_SetSigned((int32_t)pwm_cmd);
}
```

---

## Layer 4: 状态污染、未对齐访问与执行器失控防线 (State Isolation, Alignment & Fail-Safe)

### 4.1 控制器函数内隐藏 `static` 变量导致的多通道串扰
* **故障机理**：在通用函数 `PID_Compute()` 或 `LowPassFilter()` 内使用 `static float prev_err` 保存历史值。单轮/单轴调试完全正常；一旦左轮和右轮（或云台 Pitch 与 Yaw）先后调用同一个函数，左轮把右轮的 `prev_err` 覆盖，产生莫名其妙的高频耦合震荡。
* **准则**：算法模块内部**零 `static` 可变状态**，所有历史变量必须收纳于 `xxx_ctx_t *ctx` 实例结构体。

### 4.2 串口协议字节流强转引发 Cortex-M0+ 未对齐 `HardFault`
* **故障机理**：在 STM32F4 (M4) 上勉强能跑通的 `float target_x = *(float *)&rx_buf[1];`（偏移地址 `1` 非 4 字节对齐），移植到 **TI MSPM0G3507 / L1306 (Cortex-M0+)** 时，因 ARMv6-M 架构**硬件不支持任何非对齐 `LDR`/`STR` 指令**，执行瞬间百分百触发 **`HardFault`**！
* **准则**：二进制协议序列化/反序列化一律使用 `memcpy()`（现代编译器在 `-O1`/`-O2` 下会自动内联优化，零函数调用开销且全架构绝对安全）。

```c
/* ❌ 错误写法 (Buggy)：奇偶偏移指针强转 float/int32，在 MSPM0 (Cortex-M0+) 上当场 HardFault！ */
// 帧格式：[0]=0xA5, [1..4]=float dx, [5..8]=float dy, [9]=checksum
void Parse_VisionFrame_Buggy(const uint8_t *buf) {
    g_vision_dx = *(const float *)(&buf[1]); // 地址 &buf[1] 模 4 余 1 -> M0+ 必死机！
    g_vision_dy = *(const float *)(&buf[5]);
}

/* ✅ 竞赛标准写法 (Contest-Grade)：帧头 + 校验和 + memcpy 安全解包 + 刷新心跳时间戳 */
bool Parse_VisionFrame_Safe(const uint8_t *buf, uint32_t now_ms) {
    if (buf[0] != 0xA5U) return false;
    uint8_t sum = 0;
    for (int i = 0; i < 9; i++) sum += buf[i];
    if (sum != buf[9]) return false;

    float dx, dy;
    memcpy(&dx, &buf[1], sizeof(float)); // 适配 M0+/M3/M4/M33/M7/RISC-V 全平台零异常
    memcpy(&dy, &buf[5], sizeof(float));
    if (!isfinite(dx) || !isfinite(dy)) return false;

    g_vision.dx = dx;
    g_vision.dy = dy;
    g_vision.last_update_ms = now_ms;    // 喂狗：刷新视觉在线心跳
    return true;
}
```

### 4.3 传感器/上位机掉线看门狗缺失（断线飞车隐患）
* **故障机理**：震动导致 OpenMV 杜邦线虚接松脱时，串口不再进入中断，`g_vision.dx` 永远停留在断线前最后一帧的偏差值（如 `+120.0f`），云台打死撞限位或小车全速冲出赛道。
* **强制规则**：在控制环每一拍检查 `(uint32_t)(now_ms - g_vision.last_update_ms) > VISION_TIMEOUT_MS`（如 `100ms`），一旦超时立即将期望速度平滑斜坡归零或切入 `STATE_FAULT_ESTOP`。
