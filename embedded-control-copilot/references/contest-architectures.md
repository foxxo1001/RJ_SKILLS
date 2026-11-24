# 电赛与嵌入式竞赛高可靠软件架构手册 (Contest Firmware Architectures)

> **设计目标**：解决电赛四天三夜高压开发中最常见的三大架构灾难——**改硬件重写全套算法**、**现场切换基础题/发挥题时历史状态残留疯车**、以及 **OLED 刷屏/串口打波形拖垮 PID 控制周期**。

---

## 1. 架构选型决策矩阵：裸机确定性时间片调度 vs RTOS (FreeRTOS / RT-Thread)

并非所有赛题都适合上 RTOS！盲目在 32KB RAM 的小资源芯片上堆砌多任务，往往因栈溢出与互斥锁死锁葬送比赛。请按以下矩阵决策：

| 评估维度 | 裸机前后台 + 硬件定时器确定性调度 (Bare-Metal) | 实时操作系统 (FreeRTOS / RT-Thread / ESP-IDF SMP) |
| :--- | :--- | :--- |
| **典型适用赛题** | 送药/巡线小车、风力摆、倒立摆、双轴激光云台追踪、开关电源/逆变器数字控制 | 四旋翼无人机、视觉+机械臂复合协同作业、带 Wi-Fi/BLE/LwIP/FATFS/LVGL 复杂交互系统 |
| **推荐目标芯片** | MSPM0L1306 / G3507、STM32F103 / F407 / G431、RA4M2 | ESP32-S3（原生必选）、STM32F407 / H743、RA6M5 / RA8M1 |
| **控制环抖动 (Jitter)** | **极低 (`< 1μs`)**：核心闭环直接跑在定频硬件 Timer ISR 中 | **低 (`5 ~ 20μs`)**：依赖最高优先级任务 + 信号量/任务通知唤醒（超高频环仍须放 ISR） |
| **RAM 与栈开销** | 极省（共享单一主栈 `MSP`，4KB~8KB 绰绰有余） | 较高（每个任务独立分配 `TCB + Task Stack`，总需求通常 `>= 24KB`） |
| **阻塞库容忍度** | **差**：主循环一旦调阻塞式 OLED 全屏刷新 (`25ms`) 会卡住按键与非中断任务 | **极强**：低优先级 UI/网络任务可自由阻塞等待，毫秒不伤高优先级控制任务 |
| **现场排障难度** | 直观线性，逻辑分析仪拉引脚即可定位全部时序 | 需防范优先级反转、队列满阻塞、单任务隐蔽爆栈 (`StackHighWaterMark`) |

> **黄金折中法则**：即便选用 FreeRTOS，对于 **`>= 2kHz` 的超高频内环**（如 FOC 电流环、2kHz 陀螺仪角速度环），依然应直接放在高优先级硬件定时器/ADC 注入中断中执行，仅将 `100Hz ~ 500Hz` 位置/轨迹规划、通信协议解析与人机交互交给 RTOS 任务。

---

## 2. 四层解耦固件目录架构 (4-Layer Decoupled Firmware Layout)

赛前准备的算法库之所以能在比赛公布指定芯片（如临时从 STM32 换成 TI MSPM0 或瑞萨 RA）后 **30 分钟内无缝移植**，全靠严格的单向依赖四层架构：

```text
Project_Root/
├── App/                        # [第 4 层：赛题业务层] —— 只认识赛题逻辑，不认识芯片寄存器
│   ├── contest_fsm.c/.h        # 基础题 1~4 / 发挥题 1~2 表驱动状态机与安全急停切换
│   ├── trajectory_planner.c/.h # 梯形/S 型速度规划、圆弧/矩形轨迹插补、自动校准流程
│   └── telemetry_ui.c/.h       # OLED 选关菜单、发车倒计时、VOFA+ 调参波形快照打包
│
├── Algorithm/                  # [第 3 层：纯数学算法层] —— ★绝对严禁包含任何 MCU 头文件★
│   ├── pid_controller.c/.h     # 位置式/增量式 PID、积分抗饱和、微分先行、不完全微分
│   ├── kinematics.c/.h         # 差速/麦轮/全向轮正逆运动学、二连杆/三连杆平面逆解
│   └── filters.c/.h            # 一阶 IIR 低通、二阶 Butterworth、互补滤波/Mahony 姿态解算
│
├── Module_Driver/              # [第 2 层：板级外设模块层] —— 依赖抽象函数指针/结构体接口
│   ├── drv_motor.c/.h          # TB6612 / DRV8870 / 无刷电调正反转死区补偿与限幅封装
│   ├── drv_imu_icm42688.c/.h   # IMU 寄存器配置、零偏校准（通过抽象 SPI_ReadWrite 回调通信）
│   └── drv_vision_proto.c/.h   # OpenMV / K230 / 激光雷达 RingBuffer + 逐字节帧状态机解析
│
└── BSP/                        # [第 1 层：芯片硬件胶水层] —— 唯一允许出现 HAL_* / DL_* / R_* 的目录
    ├── bsp_pwm_enc.c/.h        # 封装底层定时器 PWM 输出与编码器原始脉冲差读取
    ├── bsp_uart_dma.c/.h       # 封装串口 DMA/中断收发与 ORE 错误自动恢复
    └── bsp_time_gpio.c/.h      # DWT 微秒计时、蜂鸣器声光提示（电赛题目必考项！）、按键消抖
```

### 跨层硬约束：`Algorithm/` 纯净度自检与 `Module_Driver/` 依赖注入
* **`Algorithm/` 编译铁律**：文件顶部**只能 `#include <stdint.h>`, `<stdbool.h>`, `<stddef.h>`, `<math.h>`, `<string.h>`**。如果某份 `pid.c` 里出现了 `#include "stm32f4xx_hal.h"` 或直接写了 `TIM1->CCR1 = out;`，审查直接判定不合格！
* **`Module_Driver/` 硬件抽象接口示例**：

```c
/* ✅ 竞赛标准写法 (Contest-Grade)：Module_Driver 通过轻量函数指针解耦 STM32 / MSPM0 / ESP32 / RA */
typedef struct {
    void (*set_pwm_permille)(int16_t duty_permille); // 输入 [-1000, +1000] 代表 -100.0% ~ +100.0%
    int16_t (*read_enc_delta)(void);                 // 返回上一控制拍至今的编码器脉冲增量
    int16_t deadzone_permille;                       // 电机静摩擦死区前馈补偿值（如 35 = 3.5%）
    bool    invert_dir;                              // 硬件接线反相一键软件翻转，免现场重焊电机线！
} Motor_Port_t;

static inline void Motor_WriteNormalized(const Motor_Port_t *m, float cmd_norm) {
    if (!isfinite(cmd_norm)) { m->set_pwm_permille(0); return; }
    if (cmd_norm >  1.0f) cmd_norm =  1.0f;
    if (cmd_norm < -1.0f) cmd_norm = -1.0f;

    int16_t permille = (int16_t)(cmd_norm * 1000.0f);
    if (m->invert_dir) permille = -permille;

    // 静摩擦死区补偿（小误差精细停位时不抖、起步瞬间不卡死）
    if (permille > 0 && permille <  m->deadzone_permille) permille =  m->deadzone_permille;
    if (permille < 0 && permille > -m->deadzone_permille) permille = -m->deadzone_permille;
    m->set_pwm_permille(permille);
}
```

---

## 3. 电赛多小问零串扰模式：表驱动有限状态机 (NUEDC Task Isolation FSM)

### 3.1 为什么现场评委测评时最容易翻车？
电赛控制题通常包含 **基础部分第 (1)~(4) 问 + 发挥部分第 (1)~(2) 问**，规则要求通过车载按键/拨码开关 + OLED 菜单现场选关一键启动，且**两次测试之间往往不允许接电脑重烧程序甚至不允许断电重启**（防止陀螺仪重新温漂校准耗时）！
* **常见翻车写法**：在 `while(1)` 或中断里用一团乱麻的全局变量 `if (mode == 1) ... else if (mode == 2) ...`。测完第 (1) 问切第 (2) 问时，**上一问残留的 PID 积分项 `integral`、里程计累计坐标 `(x, y)`、子步骤序号 `sub_step`、定时器计数 `cnt` 忘记清零**，按下发车键瞬间小车带着旧积分猛冲撞毁道具！

### 3.2 标准 `on_enter` / `on_step` / `on_exit` 三段生命周期状态机
强制规定：**任何题目模式切换必须经过 `STATE_IDLE_SAFE` 安全待机态**；每次切入新状态自动触发一次 `on_enter()` 洗净所有控制器、里程计与子步骤；任何状态退出或触发倾角/越界保护时，立即执行 `on_exit()` 关断电机与激光笔！

```c
/* ❌ 错误写法 (Buggy)：满天飞的全局标志与残留静态变量，测完第1问切第2问直接暴走！ */
void Control_ISR_Buggy(void) {
    static int step = 0;
    static float dist = 0.0f;
    if (g_key_mode == 1) {
        dist += g_vel * 0.001f;
        if (dist > 1.0f) { Motor_Stop(); step = 1; } // 切到 mode 2 再切回 mode 1 时 dist 和 step 全是脏数据！
    } else if (g_key_mode == 2) {
        // PID 积分器还留着 mode 1 结束时的饱和值！
    }
}

/* ✅ 竞赛标准写法 (Contest-Grade)：表驱动任务隔离状态机，保证任意顺序反复切换 100% 零状态泄漏 */
typedef enum {
    STATE_IDLE_SAFE = 0,   // 安全待机：PWM 全关断，允许按键切换题号与 OLED 菜单预览
    STATE_CALIBRATE,       // 传感器零偏/场地基准点一键标定（如陀螺仪静止采零偏、云台标四角）
    STATE_BASIC_1,         // 基础第 1 问
    STATE_BASIC_2,         // 基础第 2 问
    STATE_BASIC_3,         // 基础第 3 问
    STATE_BASIC_4,         // 基础第 4 问
    STATE_ADV_1,           // 发挥第 1 问
    STATE_ADV_2,           // 发挥第 2 问
    STATE_FAULT_ESTOP,     // 故障急停锁死（倾角超限/视觉断线/堵转过流，须人工按键确认解除）
    STATE_COUNT
} ContestState_e;

typedef struct {
    const char *name;                          // 用于 OLED 屏幕实时显示当前状态名
    void (*on_enter)(void);                    // 切入瞬间执行且仅执行 1 次：复位 PID、清零里程计与子状态
    ContestState_e (*on_step)(float dt_s);     // 定频周期调用：返回目标下一状态（保持当前态则返回自身）
    void (*on_exit)(void);                     // 切出瞬间执行且仅执行 1 次：刹停电机、关激光/电磁铁/声光复位
} State_Desc_t;

typedef struct {
    ContestState_e curr;
    uint32_t       state_elapsed_ms;           // 当前赛题已运行毫秒数（天然支持题目限时保护！）
    uint8_t        selected_quest;             // OLED 菜单预选的题号 (STATE_BASIC_1 .. STATE_ADV_2)
} Contest_FSM_t;

/* 统一清理底座：任何赛题进入前调用，彻底根除跨题状态污染 */
static void Reset_All_Controllers_And_Odometry(void) {
    PID_Reset(&g_pid_vel_l);
    PID_Reset(&g_pid_vel_r);
    PID_Reset(&g_pid_yaw);
    Odometry_Zero(&g_odom);
}

static void Basic1_OnEnter(void) {
    Reset_All_Controllers_And_Odometry();
    Trajectory_SetLine(&g_traj, 1.00f, 0.40f); // 目标直行 1.00m，限速 0.40m/s
}

static ContestState_e Basic1_OnStep(float dt_s) {
    if (fabsf(g_imu.roll) > 35.0f || fabsf(g_imu.pitch) > 35.0f) {
        return STATE_FAULT_ESTOP;              // 侧翻/冲撞保护，毫秒级瞬切急停
    }
    if (Trajectory_UpdateAndTrack(&g_traj, dt_s)) {
        Buzzer_Beep_NonBlocking(200);          // 电赛必备：到达终点触发非阻塞声光提示！
        return STATE_IDLE_SAFE;                // 完美完赛，自动回安全态等待评委下一题指令
    }
    return STATE_BASIC_1;
}

static void Common_Actuator_SafeExit(void) {
    Motor_StopAll();
    Laser_SetEnable(false);
}

static const State_Desc_t k_state_table[STATE_COUNT] = {
    [STATE_IDLE_SAFE]   = { "IDLE_SAFE",  Common_Actuator_SafeExit, Idle_OnStep,      NULL                     },
    [STATE_CALIBRATE]   = { "CALIBRATE",  Calib_OnEnter,            Calib_OnStep,     Common_Actuator_SafeExit },
    [STATE_BASIC_1]     = { "BASIC_1",    Basic1_OnEnter,           Basic1_OnStep,    Common_Actuator_SafeExit },
    [STATE_BASIC_2]     = { "BASIC_2",    Basic2_OnEnter,           Basic2_OnStep,    Common_Actuator_SafeExit },
    [STATE_BASIC_3]     = { "BASIC_3",    Basic3_OnEnter,           Basic3_OnStep,    Common_Actuator_SafeExit },
    [STATE_BASIC_4]     = { "BASIC_4",    Basic4_OnEnter,           Basic4_OnStep,    Common_Actuator_SafeExit },
    [STATE_ADV_1]       = { "ADV_1",      Adv1_OnEnter,             Adv1_OnStep,      Common_Actuator_SafeExit },
    [STATE_ADV_2]       = { "ADV_2",      Adv2_OnEnter,             Adv2_OnStep,      Common_Actuator_SafeExit },
    [STATE_FAULT_ESTOP] = { "ESTOP!",     Common_Actuator_SafeExit, Estop_OnStep,     NULL                     },
};

/* 状态机核心调度引擎（在定频控制环中调用） */
void ContestFSM_Tick(Contest_FSM_t *fsm, float dt_s) {
    const State_Desc_t *desc = &k_state_table[fsm->curr];
    ContestState_e next = desc->on_step(dt_s);
    fsm->state_elapsed_ms += (uint32_t)(dt_s * 1000.0f + 0.5f);

    if (next != fsm->curr && next < STATE_COUNT) {
        if (desc->on_exit) desc->on_exit();                // 1. 先执行旧状态安全清理
        fsm->curr = next;
        fsm->state_elapsed_ms = 0U;
        if (k_state_table[next].on_enter) {
            k_state_table[next].on_enter();                // 2. 再执行新状态纯净初始化
        }
    }
}
```

---

## 4. RTOS 确定性实时控制模式 (RTOS Deterministic Control Pattern)

在 FreeRTOS / RT-Thread 下构建复杂控制系统时，必须严格遵守以下四条硬实时架构准则：

### 4.1 四级严格抢占优先级金字塔
（以 FreeRTOS 优先级数值越大越高为例，`configMAX_PRIORITIES = 8`）：

| 优先级层级 | FreeRTOS 优先级 | 典型任务名称与职责 | 唤醒与周期机制 | 铁律约束 |
| :--- | :---: | :--- | :--- | :--- |
| **L1：安全急停保护** | `7` (最高) | `Task_SafetyGuard`（堵转过流、倾角保护、急停中断响应） | 故障事件组 / `vTaskNotifyGiveFromISR` | 零阻塞等待，拥有无条件关断硬件 MOE/使能脚最高权限 |
| **L2：硬实时运动控制** | `6` | `Task_MotionCtrl`（500Hz/1kHz 姿态解算 + 串级 PID + 运动学） | **`vTaskDelayUntil`** 或硬件 Timer ISR 通知 | **严禁等互斥锁 (`xSemaphoreTake`)、严禁调 `printf`/OLED I2C** |
| **L3：异步传感与视觉解析** | `4` | `Task_VisionParser`（解包 OpenMV / 激光雷达字节流） | UART Idle DMA ISR 投递 **`StreamBuffer` / `Queue`** 唤醒 | 校验通过后通过原子快照/Seqlock 更新目标量，不直接写 PWM |
| **L4：遥测波形与人机交互** | `1 ~ 2` (最低) | `Task_TelemetryUI`（20Hz OLED 刷新、按键扫描、100Hz VOFA+ 波形） | `vTaskDelay(pdMS_TO_TICKS(50))` | 只读快照结构体，允许慢速 I2C/SPI 刷屏而不影响控制环一丝一毫 |

### 4.2 `vTaskDelayUntil` 绝对相位锁定 vs `vTaskDelay` 累积相位漂移
* `vTaskDelay(1)` 是**从调用时刻算起的相对睡眠**，若某拍解算多花了 `0.3ms`，采样周期就会拉长并产生永久累计漂移；**定频控制任务必须使用 `vTaskDelayUntil(&xLastWakeTime, xPeriod)`**！

### 4.3 零锁遥测快照 (`g_debug_snapshot`) 与杜绝优先级反转
* **优先级反转惨案**：低优先级 `Task_TelemetryUI` 拿住一把互斥锁 `xMutex` 读取 PID 内部状态并顺手往慢速串口发包（耗时 `8ms`），此时高优先级 `Task_MotionCtrl` 醒来试图获取同一把 `xMutex`，被低优先级任务活活阻塞 `8ms`，控制环瞬间崩溃！
* **零锁快照模式**：控制任务在每拍末尾用 `< 1μs` 将关键调参观测变量单向拷贝至全局快照 `g_debug_snapshot`；低优先级 UI/VOFA+ 任务无锁读取快照打包 DMA 发送，彻底消灭互斥锁争用！

```c
/* ❌ 错误写法 (Buggy)：控制任务与慢速打印任务共用互斥锁 + 相对延时漂移 */
void Task_MotionCtrl_Buggy(void *arg) {
    while (1) {
        xSemaphoreTake(g_state_mutex, portMAX_DELAY); // 被低优先级串口打印任务卡死数毫秒！
        Control_Step();
        xSemaphoreGive(g_state_mutex);
        vTaskDelay(pdMS_TO_TICKS(2));                 // 相对延时，周期随负载波动漂移
    }
}

/* ✅ 竞赛标准写法 (Contest-Grade)：vTaskDelayUntil 定频锁相 + 零锁快照 VOFA+ 高速调参 + 栈水位自检 */
typedef struct {
    float target_vel, actual_vel;
    float target_yaw, actual_yaw;
    float pid_p_out, pid_i_out, pid_d_out;
    uint16_t ctrl_exec_us;
    uint16_t min_free_stack_words;
} DebugSnapshot_t;

static volatile uint32_t s_snap_seq = 0;
static DebugSnapshot_t   s_debug_snap;

void Task_MotionCtrl_500Hz(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(2); // 500Hz (要求 configTICK_RATE_HZ == 1000)

    /* 注意：last_wake 必须在紧贴循环入口前初始化，防止初始化耗时导致开机连跑多拍追赶！ */
    while (1) {
        vTaskDelayUntil(&last_wake, period);

        uint32_t t0 = DWT->CYCCNT;
        ContestFSM_Tick(&g_fsm, 0.002f);
        uint32_t elapsed_us = (DWT->CYCCNT - t0) / (SystemCoreClock / 1000000U);

        // 单写端零锁发布调参快照 (< 1us)，绝不等待任何低优先级任务！
        s_snap_seq++;
        __DMB();
        s_debug_snap.target_vel           = g_pid_vel_l.target;
        s_debug_snap.actual_vel           = g_pid_vel_l.measure;
        s_debug_snap.pid_i_out            = g_pid_vel_l.i_term;
        s_debug_snap.ctrl_exec_us         = (uint16_t)elapsed_us;
        s_debug_snap.min_free_stack_words = (uint16_t)uxTaskGetStackHighWaterMark(NULL);
        __DMB();
        s_snap_seq++;
    }
}
```

### 4.4 上场前必做的栈水位线审查 (`uxTaskGetStackHighWaterMark`)
* 自由落体式的随机死机 80% 源于某个 RTOS 任务栈悄悄耗尽。
* **赛前强制检查表**：
  1. 在 `FreeRTOSConfig.h` 中开启 **`#define configCHECK_FOR_STACK_OVERFLOW 2`**（硬件涂鸦魔数检测法），并实现 `vApplicationStackOverflowHook` 在爆栈瞬间关断全部电机 PWM 并点亮红灯报警；
  2. 跑完一遍最复杂的发挥题全流程后，通过 OLED 或调试器查看各任务的 `uxTaskGetStackHighWaterMark(task_hdl)`（注意返回值单位是 **Word = 4 Bytes**，ESP-IDF 下为 **Byte**）：任何任务历史最低剩余栈空间若 **`< 总栈深的 25%`（或 `< 128 Words`）**，立即将该任务栈扩容一倍！
