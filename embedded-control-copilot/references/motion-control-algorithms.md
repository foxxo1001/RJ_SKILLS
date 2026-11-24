# 嵌入式运动控制算法与信号调理实战手册 (`motion-control-algorithms.md`)

本手册面向全国大学生电子设计竞赛（NUEDC）、智能汽车竞赛、RoboMaster 及工程训练综合能力竞赛，提供经过真机实战验证的控制环路架构、信号调理滤波与四大典型赛题机构（智能小车、二维/三维云台、四旋翼无人机、多自由度机械臂）的精确数学建模、C99 单精度浮点实现及调参启发式准则。

> **Cortex-M 单精度 FPU 铁律**：`STM32F4/G4/H7`、`GD32F4`、`MSPM0`（软浮点）等主控的硬件 FPU（如 `FPv4-SP-D16`）仅硬件加速 32 位 `float`。代码中**严禁**出现无 `f` 后缀的浮点字面量（如 `0.001`、`3.1415926`）或双精度数学库函数（`sin`、`cos`、`atan2`、`sqrt`），否则编译器将隐式提升为 64 位 `double` 软浮点运算，单次三角函数耗时从 `~15` 周期暴增至 `200+` 周期，直接击穿高频中断时序！本手册所有代码严格遵循 `float` + `f` 后缀与 `<math.h>` 单精度函数（`sinf`、`cosf`、`tanf`、`atan2f`、`acosf`、`asinf`、`sqrtf`、`fabsf`）。

---

## 1. 控制环路架构与信号调理基础 (Control Loop & Signal Conditioning Foundations)

### 1.1 多速率串级控制黄金法则 (Multi-Rate Cascaded Control Golden Rule)

在机电控制系统中，不同物理量的时间常数相差数个数量级（电枢电流 $\ll$ 角速度 $\ll$ 空间角度/位移 $\ll$ 视觉帧率）。串级控制（Cascaded Control）必须满足**带宽与采样频率分离准则**：

$$f_{\text{inner}} \ge (4 \sim 10)\,f_{\text{outer}}, \qquad \omega_{c,\text{inner}} \ge (4 \sim 10)\,\omega_{c,\text{outer}}$$

若内外环同频或内环响应过慢，外环下发的指令在内环尚未收敛时即发生剧烈变化，内外环产生相位耦合，表现为难以通过调参消除的 `1–5 Hz` 持续极限环振荡。

| 控制层级 | 典型反馈传感器 | 推荐执行频率 $f_s$ | 周期 $\Delta t$ | 触发源要求 |
| :--- | :--- | :--- | :--- | :--- |
| **FOC 电流 / 力矩最内环** | 三相采样电阻 + ADC | `10 kHz – 20 kHz` | `50 – 100 us` | PWM 中心对齐定时器触发 ADC 注入组中断 |
| **角速度内环 / 轮速内环** | 6轴 IMU 陀螺仪 / 正交编码器 | `500 Hz – 1000 Hz` | `1 – 2 ms` | 独立硬件基本定时器（`TIM6/TIM7`）或 `SysTick` |
| **姿态角度环 / 里程位置环 / 航向环** | 姿态解算欧拉角 / 里程计积分 | `100 Hz – 200 Hz` | `5 – 10 ms` | `1 kHz` 硬件定时器内整数分频（`tick % 5 == 0`） |
| **视觉伺服外环 / 激光雷达定位环** | OpenMV / MaixCam / K210 / RPi | `30 Hz – 60 Hz` | `16.7 – 33.3 ms` | 串口帧到达事件更新目标，或带速度前馈外推 |

**确定性时基准则**：严禁在带 `HAL_Delay()` 的主循环或受低优先级阻塞的 RTOS 任务中直接假定固定 `dt` 计算微分与积分。推荐在 `1 kHz` 高优先级硬件定时器 ISR（或由信号量精准唤醒的最高优先级控制线程）中采用整数分频调度多速率环路：

```c
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

static inline float clampf(float val, float min_v, float max_v) {
    if (val < min_v) return min_v;
    if (val > max_v) return max_v;
    return val;
}

/* 1kHz 确定性多速率串级调度骨架 (置于硬件定时器中断或最高优先级同步任务) */
void control_scheduler_1khz_isr(void) {
    static uint32_t tick = 0;
    const float dt_inner = 0.001f; /* 1000 Hz */
    const float dt_outer = 0.005f; /* 200 Hz  */

    tick++;
    imu_and_encoder_sample(dt_inner);

    /* 外环: 200 Hz (每 5 次内环执行 1 次，满足 5:1 黄金频率比) */
    if ((tick % 5U) == 0U) {
        outer_position_attitude_loop_update(dt_outer);
    }

    /* 内环: 1000 Hz 每次必执行，紧跟 PWM 输出更新 */
    inner_rate_speed_loop_update(dt_inner);
    actuator_pwm_or_can_commit();
}
```

---

### 1.2 竞赛级高级 PID 变体与必选工程机制

教科书式位置 PID $u_k = K_p e_k + K_i \sum e_j \Delta t + K_d \frac{e_k - e_{k-1}}{\Delta t}$ 在真实电机与视觉闭环中几乎必定失效。以下三项改进在赛题中属于**强制标配**：

#### (1) 抗积分饱和 (Anti-Windup)：积分限幅 + 变速积分分离 + 反算跟踪
* **物理致灾机理**：当目标发生大阶跃（如小车起步冲刺 `2.0 m` 或云台大幅甩头 `60°`）时，电机输出迅速顶满物理上限 $u_{\max}$（如 `PWM = 100%` 或最大母线电流），系统受限于物理最大加速度无法更快响应，误差 $e$ 在数百毫秒内持续为较大正值。若不加限制，积分项 $\int e\,dt$ 会疯狂累积到远超 $u_{\max}$ 的巨量电荷（**Integral Windup**）。当机构终于到达目标点（$e = 0$）时，庞大的残余正积分仍死死维持满占空比输出，迫使机构严重冲过头，直到产生面积相等的反向负误差才能把积分“泄放”完毕，表现为**起步/大角度切换时的巨幅超调与低频来回拍振**。
* **三重工程防护**：
  1. **积分独立限幅 (Integral Clamping)**：$I_k = \text{clamp}(I_k, -I_{\max}, +I_{\max})$。通常取 $I_{\max} = (0.25 \sim 0.50)\,u_{\max}$，仅留足克服静摩擦、稳态风阻或偏心重力的裕度。
  2. **梯形变速积分分离 (Trapezoidal Conditional Integration)**：设全速积分阈值为 $A$，分离过渡带宽为 $B$。当 $|e| \ge A + B$ 时彻底冻结或清零积分（大误差段纯靠 PD/Bang-Bang 快速逼近）；在 $A < |e| < A + B$ 区间平滑衰减积分速率系数 $\beta(e)$，避免硬阈值切换引起控制量跳变：
     $$\beta(e) = \begin{cases} 1, & |e| \le A \\ \dfrac{A + B - |e|}{B}, & A < |e| < A + B \\ 0, & |e| \ge A + B \end{cases}$$
  3. **饱和反算跟踪 (Back-Calculation)**：当未限幅总输出 $u_{\text{raw}}$ 超出 $[u_{\min}, u_{\max}]$ 时，将饱和差值 $(u_{\text{sat}} - u_{\text{raw}})$ 乘以反算增益 $K_b$（常取 $K_b \approx 1.0\text{f} \sim 3.0\text{f}$ 对应每步修正权重 $K_b \Delta t$）动态退回积分器，一旦执行器触顶立即停止积分膨胀。

#### (2) 微分先行 + 不完全微分 (Derivative-on-Measurement + 1st-Order LPF)
* **为什么必选**：当视觉模块以 `30–60 fps` 更新目标坐标时，外环设定值 $r_k$ 呈阶梯波（每 `16.7–33.3 ms` 突变一次），而内环/位置环运行在 `200–1000 Hz`。若对误差 $e_k = r_k - y_k$ 求导，$\frac{r_k - r_{k-1}}{0.001\,\text{s}}$ 会在每次相机新帧到达的 `1 ms` 内产生巨幅脉冲尖峰（**Derivative Kick / 微分冲击**），剩余 `32 ms` 又突变为 `0`，导致舵机/电机每帧剧烈顿挫敲齿、驱动芯片严重发热！
* **精确公式**：仅对连续变化的**被控量测量值 $y_k$** 求负导数，并串联一阶低通滤波器抑制编码器量化噪声与陀螺仪高频毛刺：
  $$D_{\text{raw}, k} = -K_d \frac{y_k - y_{k-1}}{\Delta t}, \qquad D_k = \alpha_d D_{\text{raw}, k} + (1 - \alpha_d) D_{k-1}$$
  其中 $\alpha_d = \frac{2\pi f_{c,d} \Delta t}{1 + 2\pi f_{c,d} \Delta t}$，推荐微分滤波截止频率 $f_{c,d} = 20\text{f} \sim 50\text{f}\,\text{Hz}$。

#### (3) 速度 / 加速度 / 死区前馈控制 (Feedforward Control)
* **为什么必选**：纯反馈 PID 本质是“事后纠偏”——必须先落后于目标产生误差 $e \neq 0$，才有控制力输出，跟踪运动靶或 S 曲线轨迹时必然存在稳态相位滞后。前馈根据参考轨迹的解析导数 $(\dot{r}, \ddot{r})$ 直接提供 `80%–90%` 的标称驱动量，实现**近零相位滞后跟踪**，让反馈 PID 仅需修正剩余 `10%–20%` 的模型扰动：
  $$u_{\text{ff}} = K_v \dot{r} + K_a \ddot{r} + u_{\text{dz}} \cdot \text{sgn}_{\epsilon}(\dot{r}), \qquad \text{sgn}_{\epsilon}(\dot{r}) = \begin{cases} +1, & \dot{r} > \epsilon_v \\ -1, & \dot{r} < -\epsilon_v \\ 0, & |\dot{r}| \le \epsilon_v \end{cases}$$

#### 竞赛级全功能 PID 控制器 C99 实现

```c
typedef struct {
    /* 核心增益 */
    float kp;
    float ki;
    float kd;

    /* 前馈增益 */
    float kv_ff;          /* 速度前馈系数 */
    float ka_ff;          /* 加速度前馈系数 */
    float u_deadzone;     /* 库仑静摩擦/死区前馈补偿量 */
    float ff_vel_eps;     /* 触发死区前馈的最小参考速度阈值 */

    /* 抗饱和与限制参数 */
    float i_max;          /* 积分项独立限幅 (>= 0) */
    float out_min;        /* 总输出下限 */
    float out_max;        /* 总输出上限 */
    float err_deadband;   /* 误差死区 (消除到位后微幅抖动) */
    float err_sep_a;      /* 变速积分全速区上限 A */
    float err_sep_b;      /* 变速积分过渡带宽 B (B <= 0 表示仅硬阈值 A) */
    float kb_aw;          /* 反算抗饱和系数 (如 0.0f~50.0f, 乘 dt 生效) */

    /* 微分先行低通滤波系数 */
    float d_lpf_alpha;    /* 0.0f < alpha <= 1.0f */

    /* 内部运行状态 */
    float integral;
    float prev_meas;
    float d_filt;
    float last_out;
    bool  initialized;
} pid_ctrl_t;

void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd,
              float i_max, float out_min, float out_max,
              float d_cutoff_hz, float sample_dt) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kv_ff = 0.0f;
    pid->ka_ff = 0.0f;
    pid->u_deadzone = 0.0f;
    pid->ff_vel_eps = 1e-3f;

    pid->i_max = fabsf(i_max);
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->err_deadband = 0.0f;
    pid->err_sep_a = 1e9f; /* 默认不分离，按需配置 */
    pid->err_sep_b = 0.0f;
    pid->kb_aw = 0.0f;

    if (d_cutoff_hz > 0.0f && sample_dt > 0.0f) {
        float rc = 1.0f / (2.0f * 3.14159265f * d_cutoff_hz);
        pid->d_lpf_alpha = sample_dt / (rc + sample_dt);
    } else {
        pid->d_lpf_alpha = 1.0f;
    }

    pid->integral = 0.0f;
    pid->prev_meas = 0.0f;
    pid->d_filt = 0.0f;
    pid->last_out = 0.0f;
    pid->initialized = false;
}

void pid_reset(pid_ctrl_t *pid) {
    pid->integral = 0.0f;
    pid->d_filt = 0.0f;
    pid->last_out = 0.0f;
    pid->initialized = false;
}

/* 标准更新接口：支持外部直接传入自定义误差 err (例如经过 wrap_deg180 处理的航向误差) */
float pid_update_ext(pid_ctrl_t *pid, float err, float meas,
                     float ref_vel, float ref_acc, float dt) {
    if (dt <= 1e-6f || dt > 0.2f) {
        return pid->last_out;
    }

    /* 首拍保护：对齐历史测量值，防止上电首拍 (meas - 0)/dt 产生巨幅微分冲击 */
    if (!pid->initialized) {
        pid->prev_meas = meas;
        pid->d_filt = 0.0f;
        pid->initialized = true;
    }

    /* 1. 误差死区处理 */
    float abs_err = fabsf(err);
    float eff_err = (abs_err <= pid->err_deadband) ? 0.0f : err;

    /* 2. 比例项 */
    float p_term = pid->kp * eff_err;

    /* 3. 微分先行 + 一阶低通滤波 (对测量值 -meas 求导，彻底免疫设定值阶跃) */
    float d_raw = -pid->kd * (meas - pid->prev_meas) / dt;
    pid->prev_meas = meas;
    pid->d_filt += pid->d_lpf_alpha * (d_raw - pid->d_filt);

    /* 4. 梯形变速积分系数 beta 计算 */
    float beta = 1.0f;
    if (abs_err > pid->err_sep_a) {
        if (pid->err_sep_b > 1e-6f && abs_err < (pid->err_sep_a + pid->err_sep_b)) {
            beta = (pid->err_sep_a + pid->err_sep_b - abs_err) / pid->err_sep_b;
        } else {
            beta = 0.0f;
        }
    }

    /* 5. 前馈项计算 */
    float ff_term = pid->kv_ff * ref_vel + pid->ka_ff * ref_acc;
    if (ref_vel > pid->ff_vel_eps) {
        ff_term += pid->u_deadzone;
    } else if (ref_vel < -pid->ff_vel_eps) {
        ff_term -= pid->u_deadzone;
    }

    /* 6. 预计算未限幅输出并执行条件积分 + 反算抗饱和 */
    float i_candidate = pid->integral + beta * pid->ki * eff_err * dt;
    i_candidate = clampf(i_candidate, -pid->i_max, pid->i_max);

    float u_unsat = p_term + i_candidate + pid->d_filt + ff_term;
    float u_sat = clampf(u_unsat, pid->out_min, pid->out_max);

    if (pid->kb_aw > 0.0f) {
        /* 反算跟踪抗饱和 */
        i_candidate += pid->kb_aw * (u_sat - u_unsat) * dt;
    } else {
        /* 若无反算增益，采用条件钳位法：已饱和且误差继续同向恶化时禁止累加 */
        bool sat_hi = (u_unsat > pid->out_max) && (eff_err > 0.0f);
        bool sat_lo = (u_unsat < pid->out_min) && (eff_err < 0.0f);
        if (sat_hi || sat_lo) {
            i_candidate = pid->integral;
        }
    }
    pid->integral = clampf(i_candidate, -pid->i_max, pid->i_max);

    /* 重新合成最终限幅输出 */
    float out = clampf(p_term + pid->integral + pid->d_filt + ff_term,
                       pid->out_min, pid->out_max);
    pid->last_out = out;
    return out;
}

static inline float pid_update(pid_ctrl_t *pid, float ref, float meas, float dt) {
    return pid_update_ext(pid, ref - meas, meas, 0.0f, 0.0f, dt);
}
```

---

### 1.3 核心数字滤波器库 (Essential Digital Filters)

#### (1) 一阶 RC 低通滤波器 (1st-Order LPF)
由模拟截止频率 $f_c$（单位 `Hz`）与采样周期 $\Delta t$（单位 `s`）精确计算平滑因子 $\alpha$：
$$\tau = \frac{1}{2\pi f_c}, \qquad \alpha = \frac{\Delta t}{\tau + \Delta t} = \frac{2\pi f_c \Delta t}{1 + 2\pi f_c \Delta t}, \qquad y_k = y_{k-1} + \alpha (x_k - y_{k-1})$$
* **调参准则**：编码器测速推荐 $f_c = 15\text{f} \sim 35\text{f}\,\text{Hz}$（`@ 500–1000 Hz`）；注意在 $f = f_c$ 处会产生 $-45^\circ$ 相位滞后，$f_c$ 严禁低于外环带宽的 $3$ 倍。

#### (2) 二阶巴特沃斯低通滤波器 (2nd-Order Butterworth Biquad LPF)
四旋翼螺旋桨高频机械振动（`80–250 Hz`）或高速智能车底盘颠簸传入 IMU 时，一阶滤波滚降仅 `-20 dB/dec`，压制高频噪声会损失过多低频相位裕度；二阶 Butterworth 提供平坦通带与 `-40 dB/dec` 陡峭滚降。采用带频率预畸变的双线性变换（Direct Form II Transposed，单精度浮点数值稳定性最佳）：
$$K = \tan\!\left(\frac{\pi f_c}{f_s}\right), \qquad N = \frac{1}{1 + \sqrt{2}\,K + K^2}$$
$$b_0 = K^2 N, \quad b_1 = 2 b_0, \quad b_2 = b_0, \quad a_1 = 2(K^2 - 1)N, \quad a_2 = (1 - \sqrt{2}\,K + K^2)N$$

#### (3) 中值 + 滑动平均复合滤波 (Median-of-5 + Moving Average)
专治激光测距（`VL53L1X` / `TFmini`）、超声波、红外测距偶发的单点飞值（高反光/多径跳变）。先用 5 点中值剔除野值脉冲，再送入环形滑动均值平滑量化台阶。

#### (4) 一维标量卡尔曼滤波器 (1D Kalman Filter)
状态方程 $x_k = x_{k-1} + u_k + w_k$（$w_k \sim \mathcal{N}(0, Q)$），观测方程 $z_k = x_k + v_k$（$v_k \sim \mathcal{N}(0, R)$）。稳态收敛速度与平滑度仅取决于信噪方差比 $Q/R$：$Q$ 越大越信任动态变化（响应快、噪声大），$R$ 越大越压制传感器噪声（平滑强、滞后增）。

```c
/* ==================== 1. 一阶低通滤波器 ==================== */
typedef struct {
    float alpha;
    float y;
    bool  init;
} lpf1_t;

static inline void lpf1_init(lpf1_t *f, float fc_hz, float dt) {
    float w = 2.0f * 3.14159265f * fc_hz * dt;
    f->alpha = clampf(w / (1.0f + w), 0.0f, 1.0f);
    f->y = 0.0f;
    f->init = false;
}

static inline float lpf1_apply(lpf1_t *f, float x) {
    if (!f->init) {
        f->y = x;
        f->init = true;
        return x;
    }
    f->y += f->alpha * (x - f->y);
    return f->y;
}

/* ==================== 2. 二阶 Butterworth 低通滤波器 ==================== */
typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float s1, s2;
    bool  init;
} biquad_lpf_t;

void biquad_lpf_init(biquad_lpf_t *bq, float fc_hz, float fs_hz) {
    /* 限制截止频率严格低于奈奎斯特频率 0.45 * fs */
    float ratio = clampf(fc_hz / fs_hz, 0.001f, 0.45f);
    float k = tanf(3.14159265f * ratio);
    float k2 = k * k;
    float norm = 1.0f / (1.0f + 1.41421356f * k + k2);

    bq->b0 = k2 * norm;
    bq->b1 = 2.0f * bq->b0;
    bq->b2 = bq->b0;
    bq->a1 = 2.0f * (k2 - 1.0f) * norm;
    bq->a2 = (1.0f - 1.41421356f * k + k2) * norm;
    bq->s1 = 0.0f;
    bq->s2 = 0.0f;
    bq->init = false;
}

float biquad_lpf_apply(biquad_lpf_t *bq, float x) {
    if (!bq->init) {
        /* 稳态无冲击预充电: 令直流增益为 1 的内部状态匹配初值 x */
        bq->s1 = x * (1.0f - bq->b0);
        bq->s2 = x * (bq->b2 - bq->a2);
        bq->init = true;
        return x;
    }
    float y = bq->b0 * x + bq->s1;
    bq->s1 = bq->b1 * x - bq->a1 * y + bq->s2;
    bq->s2 = bq->b2 * x - bq->a2 * y;
    return y;
}

/* ==================== 3. 5点中值 + 8点滑动平均复合滤波 ==================== */
#define MED_WIN 5
#define AVG_WIN 8

typedef struct {
    float med_buf[MED_WIN];
    float avg_buf[AVG_WIN];
    uint8_t med_idx;
    uint8_t avg_idx;
    uint8_t count;
    bool    primed;
} med_avg_filter_t;

static inline void swapf_val(float *a, float *b) {
    float t = *a; *a = *b; *b = t;
}

float med_avg_apply(med_avg_filter_t *f, float x) {
    if (!f->primed) {
        for (int i = 0; i < MED_WIN; i++) f->med_buf[i] = x;
        for (int i = 0; i < AVG_WIN; i++) f->avg_buf[i] = x;
        f->med_idx = 0U;
        f->avg_idx = 0U;
        f->primed = true;
        return x;
    }

    f->med_buf[f->med_idx] = x;
    f->med_idx = (uint8_t)((f->med_idx + 1U) % MED_WIN);

    /* 5 元素排序网络求中位数 (无分支膨胀，固定 9 次比较) */
    float v[MED_WIN];
    for (int i = 0; i < MED_WIN; i++) v[i] = f->med_buf[i];
    if (v[0] > v[1]) swapf_val(&v[0], &v[1]);
    if (v[3] > v[4]) swapf_val(&v[3], &v[4]);
    if (v[0] > v[3]) swapf_val(&v[0], &v[3]);
    if (v[1] > v[4]) swapf_val(&v[1], &v[4]);
    if (v[1] > v[2]) swapf_val(&v[1], &v[2]);
    if (v[2] > v[3]) swapf_val(&v[2], &v[3]);
    if (v[1] > v[2]) swapf_val(&v[1], &v[2]);
    float median = v[2];

    /* 送入滑动窗口均值 (每次直接求和 8 点，彻底杜绝浮点递推累积漂移) */
    f->avg_buf[f->avg_idx] = median;
    f->avg_idx = (uint8_t)((f->avg_idx + 1U) % AVG_WIN);
    float sum = 0.0f;
    for (int i = 0; i < AVG_WIN; i++) sum += f->avg_buf[i];
    return sum * (1.0f / (float)AVG_WIN);
}

/* ==================== 4. 一维标量卡尔曼滤波器 ==================== */
typedef struct {
    float x;  /* 状态估计值 */
    float p;  /* 估计误差协方差 */
    float q;  /* 过程噪声方差 (推荐起步: 0.005f ~ 0.05f) */
    float r;  /* 观测噪声方差 (实测静止传感器标准差的平方 sigma^2) */
    bool  init;
} kalman1d_t;

static inline void kalman1d_init(kalman1d_t *kf, float q, float r) {
    kf->x = 0.0f;
    kf->p = 1.0f;
    kf->q = q;
    kf->r = fmaxf(r, 1e-6f);
    kf->init = false;
}

static inline float kalman1d_update(kalman1d_t *kf, float z_meas, float u_delta) {
    if (!kf->init) {
        kf->x = z_meas;
        kf->p = kf->r;
        kf->init = true;
        return z_meas;
    }
    /* 预测步 */
    float x_pred = kf->x + u_delta;
    float p_pred = kf->p + kf->q;

    /* 更新步 */
    float k_gain = p_pred / (p_pred + kf->r);
    kf->x = x_pred + k_gain * (z_meas - x_pred);
    kf->p = (1.0f - k_gain) * p_pred;
    return kf->x;
}
```

---

## 2. 智能小车运动控制 (Smart Car: 差速 / 麦克纳姆轮 / 全向轮 / 阿克曼)

### 2.1 车体坐标系与四大底盘精确正逆运动学

统一采用右手车体坐标系 **FLU**：车头前进为 $+x$（线速度 $v_x$，$\text{m/s}$），车体正左平移为 $+y$（线速度 $v_y$，$\text{m/s}$），俯视逆时针旋转为 $+\omega_z$（角速度，$\text{rad/s}$）。设车轮半径为 $R_w$，第 $i$ 轮线速度 $v_i = \omega_i R_w$（轮子使车体向 $+x$ 前进的旋转方向定义为正，全向轮按切向单位向量定义）。

#### (1) 两轮差速 / 四轮滑移差速底盘 (Differential 2WD / Skid-Steer 4WD)
设左右轮距为 $W$（四轮差速原地转向存在轮胎横向搓地滑移，里程计反解等效轮距 $W_{\text{eff}} = \kappa W$，橡胶轮在赛道上实测 $\kappa \in [1.08, 1.30]$）：
$$\text{逆解:}\quad v_L = v_x - \frac{W}{2}\omega_z, \qquad v_R = v_x + \frac{W}{2}\omega_z$$
$$\text{正解:}\quad v_x = \frac{v_L + v_R}{2}, \qquad \omega_z = \frac{v_R - v_L}{W_{\text{eff}}}$$

#### (2) 四轮麦克纳姆轮底盘 (4-Wheel Mecanum，标准 `X-Pattern` / 底面接触辊呈 `O` 形)
设前后轴距的一半为 $L_x = L/2$，左右轮距的一半为 $L_y = W/2$，几何力臂和 $K = L_x + L_y$（俯视轮顶斜辊延长线交于车心呈 `X` 形，即左前 `FL` 右斜、右前 `FR` 左斜、左后 `RL` 左斜、右后 `RR` 右斜）：
$$\text{逆解:}\quad \begin{bmatrix} v_{\text{FL}} \\ v_{\text{FR}} \\ v_{\text{RL}} \\ v_{\text{RR}} \end{bmatrix} = \begin{bmatrix} 1 & -1 & -(L_x + L_y) \\ 1 & +1 & +(L_x + L_y) \\ 1 & +1 & -(L_x + L_y) \\ 1 & -1 & +(L_x + L_y) \end{bmatrix} \begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix}$$
$$\text{正解:}\quad v_x = \frac{v_{\text{FL}} + v_{\text{FR}} + v_{\text{RL}} + v_{\text{RR}}}{4}, \quad v_y = \frac{-v_{\text{FL}} + v_{\text{FR}} + v_{\text{RL}} - v_{\text{RR}}}{4}, \quad \omega_z = \frac{-v_{\text{FL}} + v_{\text{FR}} - v_{\text{RL}} + v_{\text{RR}}}{4(L_x + L_y)}$$

> **麦轮/全向轮等比例抗饱和缩放铁律**：当合成 $(v_x, v_y, \omega_z)$ 导致某轮 $|v_i| > v_{\text{limit}}$ 时，**严禁**对单轮独立 `clampf`！独立截断会破坏四轮速度矢量比例，导致小车边自旋边横向跑偏。必须以 $s = v_{\text{limit}} / \max_i |v_i|$ 对四轮同步等比缩放。

#### (3) 三轮 $120^\circ$ 全向轮与四轮 $45^\circ$ 交叉全向轮底盘 (3/4-Wheel Omni)
* **三轮全向（车轮中心距底盘中心半径 $R$）**：设 `W1` 位于正前方 $+x$ 轴上（正转驱动力沿 $-y$，使车逆时针旋转方向切向定义为：$v_i = -\sin\theta_i\,v_x + \cos\theta_i\,v_y + R\omega_z$；取 $\theta_1 = 0^\circ$ 前轮正转沿 $+y$ 左向，$\theta_2 = 120^\circ$ 左后轮，$\theta_3 = 240^\circ$ 右后轮）：
  $$\text{逆解:}\quad v_1 = v_y + R\omega_z, \qquad v_2 = -\frac{\sqrt{3}}{2}v_x - \frac{1}{2}v_y + R\omega_z, \qquad v_3 = +\frac{\sqrt{3}}{2}v_x - \frac{1}{2}v_y + R\omega_z$$
  $$\text{正解:}\quad v_x = \frac{v_3 - v_2}{\sqrt{3}}, \qquad v_y = \frac{2v_1 - v_2 - v_3}{3}, \qquad \omega_z = \frac{v_1 + v_2 + v_3}{3R}$$
* **四轮 $45^\circ$ X 型全向轮（安装方位角 $\theta \in \{+45^\circ\text{(FL)}, -45^\circ\text{(FR)}, +135^\circ\text{(RL)}, -135^\circ\text{(RR)}\}$，定义各轮使车头 $+x$ 分量为正的转向为正，中心距轮半径 $R$，$c = \frac{\sqrt{2}}{2} = 0.70710678\text{f}$）**：
  $$\text{逆解:}\quad v_{\text{FL}} = c(v_x - v_y) - R\omega_z,\quad v_{\text{FR}} = c(v_x + v_y) + R\omega_z,\quad v_{\text{RL}} = c(v_x + v_y) - R\omega_z,\quad v_{\text{RR}} = c(v_x - v_y) + R\omega_z$$
  $$\text{正解:}\quad v_x = \frac{v_{\text{FL}} + v_{\text{FR}} + v_{\text{RL}} + v_{\text{RR}}}{4c},\quad v_y = \frac{-v_{\text{FL}} + v_{\text{FR}} + v_{\text{RL}} - v_{\text{RR}}}{4c},\quad \omega_z = \frac{-v_{\text{FL}} + v_{\text{FR}} - v_{\text{RL}} + v_{\text{RR}}}{4R}$$

#### (4) 阿克曼转向几何与自行车运动学模型 (Ackermann Steering)
设前后轴距为 $L$，前桥主销轮距为 $W$，后桥中心纵向速度为 $v_x$，等效前轮中心转角为 $\delta$（左转为正）：
$$\delta = \text{atan2f}(L \cdot \omega_z,\; v_x)\;\;(\text{当 } v_x < 0 \text{ 倒车时取 } \delta = \text{atan2f}(L \cdot \omega_z,\; |v_x|) \cdot \text{sgn}(v_x)), \qquad \omega_z = \frac{v_x \tan\delta}{L}$$
内外前轮阿克曼差角与后轮电子差速公式（转弯瞬心半径 $R_c = L / \tan\delta$）：
$$\tan\delta_L = \frac{2 L \tan\delta}{2 L - W \tan\delta}, \qquad \tan\delta_R = \frac{2 L \tan\delta}{2 L + W \tan\delta}$$
$$v_{\text{RL}} = v_x \left(1 - \frac{W \tan\delta}{2 L}\right), \qquad v_{\text{RR}} = v_x \left(1 + \frac{W \tan\delta}{2 L}\right)$$

#### 四大底盘正逆运动学完整 C99 实现

```c
/* 通用多轮等比例缩放限速，保持底盘合成速度矢量方向不变 */
static inline void desaturate_wheels(float *w, int n, float max_speed) {
    float peak = 0.0f;
    for (int i = 0; i < n; i++) {
        float a = fabsf(w[i]);
        if (a > peak) peak = a;
    }
    if (peak > max_speed && peak > 1e-6f) {
        float scale = max_speed / peak;
        for (int i = 0; i < n; i++) w[i] *= scale;
    }
}

/* 1. 差速底盘正逆解 */
static inline void diff_ik(float vx, float wz, float track_w, float max_v,
                           float *v_left, float *v_right) {
    float w[2] = { vx - 0.5f * track_w * wz, vx + 0.5f * track_w * wz };
    desaturate_wheels(w, 2, max_v);
    *v_left = w[0];
    *v_right = w[1];
}

static inline void diff_fk(float v_left, float v_right, float eff_track_w,
                           float *vx, float *wz) {
    *vx = 0.5f * (v_left + v_right);
    *wz = (v_right - v_left) / fmaxf(eff_track_w, 1e-4f);
}

/* 2. 四轮麦克纳姆轮 (X-Pattern) 正逆解: wheels[0..3] = {FL, FR, RL, RR} */
void mecanum_ik(float vx, float vy, float wz, float lx_plus_ly,
                float max_v, float wheels[4]) {
    float rot = lx_plus_ly * wz;
    wheels[0] = vx - vy - rot; /* FL */
    wheels[1] = vx + vy + rot; /* FR */
    wheels[2] = vx + vy - rot; /* RL */
    wheels[3] = vx - vy + rot; /* RR */
    desaturate_wheels(wheels, 4, max_v);
}

void mecanum_fk(const float wheels[4], float lx_plus_ly,
                float *vx, float *vy, float *wz) {
    *vx = 0.25f * ( wheels[0] + wheels[1] + wheels[2] + wheels[3]);
    *vy = 0.25f * (-wheels[0] + wheels[1] + wheels[2] - wheels[3]);
    *wz = 0.25f * (-wheels[0] + wheels[1] - wheels[2] + wheels[3])
          / fmaxf(lx_plus_ly, 1e-4f);
}

/* 3a. 三轮 120 度全向轮正逆解: wheels[0..2] = {Front(+y), RearLeft, RearRight} */
void omni3_ik(float vx, float vy, float wz, float radius, float max_v, float w[3]) {
    const float k_sqrt3_2 = 0.86602540f;
    float rot = radius * wz;
    w[0] = vy + rot;
    w[1] = -k_sqrt3_2 * vx - 0.5f * vy + rot;
    w[2] =  k_sqrt3_2 * vx - 0.5f * vy + rot;
    desaturate_wheels(w, 3, max_v);
}

void omni3_fk(const float w[3], float radius, float *vx, float *vy, float *wz) {
    const float k_inv_sqrt3 = 0.57735027f;
    *vx = (w[2] - w[1]) * k_inv_sqrt3;
    *vy = (2.0f * w[0] - w[1] - w[2]) * (1.0f / 3.0f);
    *wz = (w[0] + w[1] + w[2]) / fmaxf(3.0f * radius, 1e-4f);
}

/* 3b. 四轮 45 度交叉全向轮正逆解: wheels[0..3] = {FL, FR, RL, RR} */
void omni4_ik(float vx, float vy, float wz, float radius, float max_v, float w[4]) {
    const float c = 0.70710678f;
    float rot = radius * wz;
    w[0] = c * (vx - vy) - rot; /* FL */
    w[1] = c * (vx + vy) + rot; /* FR */
    w[2] = c * (vx + vy) - rot; /* RL */
    w[3] = c * (vx - vy) + rot; /* RR */
    desaturate_wheels(w, 4, max_v);
}

void omni4_fk(const float w[4], float radius, float *vx, float *vy, float *wz) {
    const float inv_4c = 0.35355339f; /* 1 / (4 * sqrt(2)/2) */
    *vx = ( w[0] + w[1] + w[2] + w[3]) * inv_4c;
    *vy = (-w[0] + w[1] + w[2] - w[3]) * inv_4c;
    *wz = 0.25f * (-w[0] + w[1] - w[2] + w[3]) / fmaxf(radius, 1e-4f);
}

/* 4. 阿克曼底盘逆解 (自行车模型 -> 双前轮阿克曼转角 + 双后轮差速) */
void ackermann_ik(float vx, float wz, float wheelbase_l, float track_w,
                  float max_steer_rad, float *delta_l, float *delta_r,
                  float *v_rl, float *v_rr) {
    float delta_center = 0.0f;
    if (fabsf(vx) > 0.03f) {
        /* 兼容正反转车速下的转向角曲率一致性 */
        float dir = (vx >= 0.0f) ? 1.0f : -1.0f;
        delta_center = atan2f(wheelbase_l * wz * dir, fabsf(vx));
    }
    delta_center = clampf(delta_center, -max_steer_rad, max_steer_rad);

    float t = tanf(delta_center);
    float two_l = 2.0f * wheelbase_l;
    *delta_l = atan2f(two_l * t, two_l - track_w * t);
    *delta_r = atan2f(two_l * t, two_l + track_w * t);

    float diff_ratio = (track_w * t) / fmaxf(two_l, 1e-4f);
    *v_rl = vx * (1.0f - diff_ratio);
    *v_rr = vx * (1.0f + diff_ratio);
}
```

---

### 2.2 直线航向锁定与弯道巡线多环架构

电赛小车（如送药小车、2024 H题自动行驶小车）在无黑线空白区盲走直线与黑线弯道切换时，推荐采用**“底层四轮独立编码器速度内环 + 纵向里程规划环 + 横向巡线/IMU航向锁定切换外环”**架构：

```
[里程目标 s_goal] -> [S曲线速度规划器] --------------------------> vx_ref ──┐
                                                                           ├──> [底盘逆运动学] -> v1..v4_ref -> [500-1000Hz 轮速PI内环] -> PWM
[模式选择] ──┬─ 盲走直线态: wrap_deg180(yaw_ref - imu_yaw) -> [Yaw PD] ──> wz_ref ──┘
             └─ 弯道巡线态: K_lat * err_lat + K_ang * err_ang - K_d * gz ─┘ (同时按曲率降速 vx_ref)
```

* **$\pm 180^\circ$ 航向环绕误差陷阱（生死红线）**：IMU 航向角通常输出范围 $(-180^\circ, +180^\circ]$ 或 $[0^\circ, 360^\circ)$。当目标锁定在 $179^\circ$ 而车头受扰动偏过 $+180^\circ$ 跳变为 $-179^\circ$ 时，直接相减 `179.0f - (-179.0f) = +358.0f`（实际仅偏 $2^\circ$！），会导致小车瞬间原地暴走甩尾自旋！**任何角度相减必须强制包裹 `wrap_deg180()`**。
* **陀螺仪原始角速度直代微分项**：航向锁定环与巡线环的阻尼项无需对噪声较大的欧拉角或相机偏差做差分，直接取 `IMU Z轴角速度 gz_deg_s` 作为阻尼项 $-K_d \cdot g_z$，相位超前且零差分毛刺。

```c
/* 将任意角度差折算至 [-180.0f, +180.0f] 最短劣弧 */
static inline float wrap_deg180(float deg) {
    deg = fmodf(deg + 180.0f, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg - 180.0f;
}

static inline float wrap_rad_pi(float rad) {
    const float pi = 3.14159265f;
    const float two_pi = 6.28318531f;
    rad = fmodf(rad + pi, two_pi);
    if (rad < 0.0f) rad += two_pi;
    return rad - pi;
}

/* 直线航向锁定控制器 (100-200 Hz, 输出角速度指令 wz_ref [rad/s])
 * 典型参数: kp_yaw = 0.08f ~ 0.18f (rad/s per deg), kd_gyro = 0.003f ~ 0.015f */
float chassis_heading_lock_update(float target_yaw_deg, float current_yaw_deg,
                                  float gyro_z_deg_s, float kp_yaw, float kd_gyro,
                                  float max_wz_rad_s) {
    float err_deg = wrap_deg180(target_yaw_deg - current_yaw_deg);
    float wz_cmd = kp_yaw * err_deg - kd_gyro * gyro_z_deg_s;
    return clampf(wz_cmd, -max_wz_rad_s, max_wz_rad_s);
}

/* 弯道视觉/灰度巡线横向控制 + 曲率自适应降速 (100-200 Hz) */
void chassis_line_track_update(float lat_err_mm, float line_angle_err_deg,
                               float gyro_z_deg_s, float base_vx, float min_vx,
                               float kp_lat, float kp_ang, float kd_gyro,
                               float k_curve_slowdown, float max_wz,
                               float *vx_out, float *wz_out) {
    /* lat_err_mm > 0 表示赛道中心在车头左侧 (+y)，需产生正逆时针角速度 +wz */
    float wz = kp_lat * lat_err_mm + kp_ang * line_angle_err_deg - kd_gyro * gyro_z_deg_s;
    wz = clampf(wz, -max_wz, max_wz);

    /* 大曲率急弯自动削减纵向车速，防止离心侧滑冲出赛道 */
    float vx = base_vx - k_curve_slowdown * fabsf(wz);
    *vx_out = clampf(vx, min_vx, base_vx);
    *wz_out = wz;
}
```

---

### 2.3 防打滑梯形与七段式 S 曲线速度规划器 ($v_{\max}, a_{\max}, j_{\max}$)

若直接给里程环下发阶跃目标（如 `s_ref` 从 `0` 突变为 `2.0 m`），起步与急刹瞬间轮胎突破地面静摩擦极限发生空转打滑，**编码器积分里程在 0.2 秒内即永久性虚增/虚减 `5–20 cm`**，后续所有定点停车全部报废！

* **赛场实测防滑约束值**：
  * 硅胶/软橡胶轮在哑光 PVC 赛道：$v_{\max} = 0.8\text{f} \sim 1.8\text{f}\,\text{m/s}$，$a_{\max} = 1.5\text{f} \sim 2.5\text{f}\,\text{m/s}^2$，$j_{\max} = 8.0\text{f} \sim 20.0\text{f}\,\text{m/s}^3$。
  * 麦克纳姆轮/全向轮（点接触摩擦小）：$v_{\max} = 0.5\text{f} \sim 1.2\text{f}\,\text{m/s}$，$a_{\max} = 0.8\text{f} \sim 1.4\text{f}\,\text{m/s}^2$，$j_{\max} = 5.0\text{f} \sim 12.0\text{f}\,\text{m/s}^3$。

以下在线离散加加速度受限（Jerk-Limited S-Curve）规划器每周期实时递推输出平滑连续的 `(pos, vel, acc)`，天然支持中途动态修改目标位置，且 `vel, acc` 可直接喂给里程环的前馈输入：

```c
typedef struct {
    float v_max;
    float a_max;
    float j_max;
    float pos;      /* 当前规划参考位置 [m] */
    float vel;      /* 当前规划参考速度 [m/s] */
    float acc;      /* 当前规划参考加速度 [m/s^2] */
} scurve_profiler_t;

void scurve_init(scurve_profiler_t *sp, float v_max, float a_max, float j_max, float init_pos) {
    sp->v_max = fmaxf(v_max, 1e-3f);
    sp->a_max = fmaxf(a_max, 1e-3f);
    sp->j_max = fmaxf(j_max, 1e-3f);
    sp->pos = init_pos;
    sp->vel = 0.0f;
    sp->acc = 0.0f;
}

/* 在线 S 曲线步进生成 (推荐在 200-1000 Hz 周期调用) */
void scurve_step(scurve_profiler_t *sp, float target_pos, float dt) {
    float dist_err = target_pos - sp->pos;
    float dir = (dist_err >= 0.0f) ? 1.0f : -1.0f;
    float abs_dist = fabsf(dist_err);

    /* 到位死区收敛钳位 */
    if (abs_dist < 1e-4f && fabsf(sp->vel) < 1e-3f && fabsf(sp->acc) < 1e-2f) {
        sp->pos = target_pos;
        sp->vel = 0.0f;
        sp->acc = 0.0f;
        return;
    }

    /* 考虑把当前加速度先泄放到 0 所需的附加制动裕度: dv_jerk = a^2 / (2 * j_max) */
    float v_stop_margin = (sp->acc * sp->acc) / (2.0f * sp->j_max);
    /* 平滑安全制动包络速度: v_allow = sqrt(2 * a_eff * s) */
    float a_eff = 0.85f * sp->a_max; /* 留 15% 急动度收尾裕度防末端微超调 */
    float v_brake = sqrtf(2.0f * a_eff * abs_dist);
    float v_des = dir * fminf(sp->v_max, fmaxf(0.0f, v_brake - 0.5f * v_stop_margin));

    /* 由速度误差计算期望加速度，并考虑加速度归零滑坡 */
    float v_err = v_des - sp->vel;
    float dir_v = (v_err >= 0.0f) ? 1.0f : -1.0f;
    float a_des_mag = sqrtf(2.0f * sp->j_max * fabsf(v_err));
    float a_des = clampf(dir_v * a_des_mag, -sp->a_max, sp->a_max);

    /* 限制加加速度 Jerk 步进更新加速度、速度与位移 */
    float da_max = sp->j_max * dt;
    sp->acc += clampf(a_des - sp->acc, -da_max, da_max);
    sp->acc = clampf(sp->acc, -sp->a_max, sp->a_max);

    float prev_vel = sp->vel;
    sp->vel = clampf(sp->vel + sp->acc * dt, -sp->v_max, sp->v_max);
    sp->pos += 0.5f * (prev_vel + sp->vel) * dt;
}
```

---

## 3. 二维/三维云台控制 (2D/3D Gimbal：激光打靶与运动目标追踪)

### 3.1 像素误差转空间角度误差 (`atan2f`) 与相机-激光视差补偿

#### (1) 针孔投影精确反解（为什么线性近似 $K \cdot \Delta u$ 在广角边缘必挂）
设图像像素坐标系左上角为原点，$+u$ 向右、$+v$ 向下，光心坐标为 $(c_x, c_y)$，等效像素焦距为 $(f_x, f_y)$。目标像素 $(u, v)$ 相对相机光轴的水平偏角与俯仰偏角精确解为（云台遵循 FLU：左转 $+\psi_{\text{yaw}}$，低头/抬头按机构定义，此处设抬头为 $+\theta_{\text{pitch}}$）：
$$\Delta\psi_{\text{cam}} = -\text{atan2f}(u - c_x,\; f_x), \qquad \Delta\theta_{\text{cam}} = -\text{atan2f}(v - c_y,\; f_y)$$

* **线性公式 $K \cdot \Delta u$ 在广角边缘失效的数学根源**：对 $\arctan(\Delta u / f_x)$ 求导得局部角灵敏度 $\frac{d\theta}{du} = \frac{f_x}{f_x^2 + \Delta u^2}$。在电赛常用的 `90°–120°` 广角无畸变镜头（如追踪 `1m x 1m` 大靶纸四角）中，画面边缘 $|\Delta u| \approx (1.0 \sim 1.7)\,f_x$，边缘处的真实像素-角度比例衰减至中心区域的 **$25\% \sim 50\%$**！
  * 若用开环单次解算指哪打哪（电赛盲打复位题型），线性模型在靶纸四个角直接产生 **`3°–8°`（对应 `1 m` 处脱靶 `5–14 cm`）** 的几何桶形误差；
  * 若在闭环反馈中使用中心整定的线性增益，边缘等效环路增益翻倍，引发大角度切入时的明显超调。
* **2 分钟桌面实测标定 $f_x$**：将云台固定在距墙面 $D = 1.000\,\text{m}$ 处，墙上贴间距 $\Delta X = 0.500\,\text{m}$ 的两个标记点，读取图像中两标记像素横坐标差 $\Delta u$，则 $f_x = \frac{\Delta u \cdot D}{\Delta X}$（无正方形畸变传感器一般 $f_y \approx f_x$）。
* **大俯仰角球面耦合修正**：对于“底座 Yaw + 上层 Pitch”二轴正交云台，当相机随 Pitch 轴俯仰角为 $\theta_p$ 时，相机水平偏差需除以 $\cos\theta_p$ 才能还原底座 Yaw 轴所需转角：$\Delta\psi_{\text{base}} = \frac{\Delta\psi_{\text{cam}}}{\text{clampf}(\cos\theta_p, 0.25\text{f}, 1.0\text{f})}$。

#### (2) 相机光心与激光发射器安装视差补偿 (Parallax Compensation)
激光笔光轴与相机镜头光心不可能物理重合。设在相机参考系下（$+X_c$ 右，$+Y_c$ 下，$+Z_c$ 沿光轴向前），激光发射孔相对镜头光心的安装偏移为 $(\delta_x, \delta_y, \delta_z)$（单位 `m`，例如激光笔装在镜头正上方 `35 mm` 处，则 $\delta_x = 0, \delta_y = -0.035\text{f}$），且激光笔与相机光轴物理平行安装（无穷远平行校准）：
* 当靶面距离为 $Z_t$ 时，激光光斑在靶面上始终位于相机光轴交点的 $(\delta_x, \delta_y)$ 偏移处。若直接让目标重合于光心 $(c_x, c_y)$，激光必然偏离目标 $\sqrt{\delta_x^2 + \delta_y^2}$！
* **精确动态补偿法**：只要激光笔与光轴平行（或在已知基准距离 $Z_{\text{cal}}$ 标定过零点偏角 $(\psi_0, \theta_0)$），任意靶距 $Z_t$ 下的靶心期望像素坐标 $(u_{\text{aim}}, v_{\text{aim}})$ 或补偿角为：
  $$u_{\text{aim}}(Z_t) = c_x + f_x \frac{\delta_x}{Z_t - \delta_z}, \qquad v_{\text{aim}}(Z_t) = c_y + f_y \frac{\delta_y}{Z_t - \delta_z}$$
  $$\Delta\psi_{\text{err}} = -\text{atan2f}(u - c_x, f_x) + \text{atan2f}(\delta_x, Z_t - \delta_z) + \psi_{\text{trim}}$$
  $$\Delta\theta_{\text{err}} = -\text{atan2f}(v - c_y, f_y) + \text{atan2f}(\delta_y, Z_t - \delta_z) + \theta_{\text{trim}}$$
  *注：若曾在 $Z_{\text{cal}} = 1.0\,\text{m}$ 处把激光拧歪对准了光心 $(c_x, c_y)$，则安装倾角 $\theta_{\text{trim}} = -\text{atan2f}(\delta_y, Z_{\text{cal}})$；当小车开到 $Z_t = 0.5\,\text{m}$ 或 $2.0\,\text{m}$ 移动打靶时，带入上式即可全程毫米级命中靶心！靶距 $Z_t$ 可由同轴 ToF 测距模块或靶框已知物理宽度 $W_{\text{real}}$ 透视测距 $Z_t = \frac{f_x W_{\text{real}}}{w_{\text{pix}}}$ 实时获取。*

---

### 3.2 运动目标追踪双环架构与延迟对齐前馈

追踪匀速/变速运动光斑或小车边跑边打靶（如电赛 2023 E题运动目标控制与自动追踪、2025 移动打靶题型）的核心架构：

1. **500–1000 Hz 云台末端 IMU 角速度内环**：将 6 轴 IMU **刚性贴装在云台最末端相机/激光支架上**（而非底座上！）。内环直接控制空间绝对角速度 $\omega_{\text{gimbal}}$，使云台天然具备**载体运动解耦自稳能力**——底盘过减速带或转向时，内环在 `1 ms` 内反向抵消扰动，无需等 `33 ms` 慢速视觉发现脱靶再补救。
2. **世界系目标绝对角重构（消除视觉延迟自激振荡的关键！）**：视觉曝光+传输耗时 $\tau_{\text{delay}} \approx 20 \sim 45\,\text{ms}$。若直接将滞后的相对误差 $\Delta\theta_{\text{cam}}$ 送入高增益位置环，云台已转到位而旧图像仍显示有误差，云台必然产生 `2–4 Hz` 追尾自激振荡！正确做法是用长度为 `32` 的环形历史队列缓存云台历史姿态角 $\theta_{\text{imu}}(t - \tau_{\text{delay}})$，在新视觉帧到达瞬间重构**目标在世界系下的真实角度**，叠加目标角速度低通估计器 $\hat{\omega}_{\text{target}}$ 进行相位超前预测与速度前馈：

$$\theta_{\text{target}}(t_{\text{cap}}) = \theta_{\text{imu}}(t - \tau_{\text{delay}}) + \Delta\theta_{\text{err}}, \qquad \hat{\theta}_{\text{target}}(t_{\text{now}}) = \theta_{\text{target}}(t_{\text{cap}}) + \hat{\omega}_{\text{target}} \cdot \tau_{\text{delay}}$$
$$\omega_{\text{ref}} = K_{p,\text{pos}} \cdot \text{wrap\_rad\_pi}\!\left(\hat{\theta}_{\text{target}}(t_{\text{now}}) - \theta_{\text{imu}}(t_{\text{now}})\right) + K_{\text{ff}} \cdot \hat{\omega}_{\text{target}}$$

```c
#define GIMBAL_HIST_LEN 64 /* 200Hz 采样下可回溯 320ms 历史姿态 */

typedef struct {
    /* 相机内参与视差参数 */
    float fx, fy, cx, cy;
    float laser_dx_m, laser_dy_m, laser_dz_m;
    float yaw_trim_rad, pitch_trim_rad;

    /* 历史云台世界角队列 (200Hz 推入) */
    float hist_yaw[GIMBAL_HIST_LEN];
    float hist_pitch[GIMBAL_HIST_LEN];
    uint8_t hist_head;

    /* 目标世界系估计状态 */
    float target_yaw_cap;
    float target_pitch_cap;
    float target_vyaw_filt;
    float target_vpitch_filt;
    float time_since_frame;
    float vision_delay_s;     /* 典型 0.025f ~ 0.040f (25~40ms) */
    float vel_lpf_alpha;      /* 目标测速滤波系数, 典型 0.25f ~ 0.45f */
    float kp_pos;             /* 位置外环比例增益, 典型 8.0f ~ 18.0f [1/s] */
    float k_ff;               /* 目标速度前馈增益, 典型 0.85f ~ 1.0f */
    bool  has_lock;
} gimbal_tracker_t;

/* 200Hz 定时推入当前云台 IMU 世界角 [rad] */
void gimbal_tracker_push_pose_200hz(gimbal_tracker_t *gt, float imu_yaw_rad, float imu_pitch_rad) {
    gt->hist_head = (uint8_t)((gt->hist_head + 1U) % GIMBAL_HIST_LEN);
    gt->hist_yaw[gt->hist_head] = imu_yaw_rad;
    gt->hist_pitch[gt->hist_head] = imu_pitch_rad;
    gt->time_since_frame += 0.005f;
}

/* 视觉新帧到达回调 (30-100Hz 异步调用) */
void gimbal_tracker_on_vision_frame(gimbal_tracker_t *gt, float u_px, float v_px,
                                    float target_dist_m, float frame_dt) {
    float z_eff = fmaxf(target_dist_m - gt->laser_dz_m, 0.15f);

    /* 1. 像素误差转非线性角度 + 激光视差动态补偿 */
    float dyaw = -atan2f(u_px - gt->cx, gt->fx)
                 + atan2f(gt->laser_dx_m, z_eff) + gt->yaw_trim_rad;
    float dpitch = -atan2f(v_px - gt->cy, gt->fy)
                   + atan2f(gt->laser_dy_m, z_eff) + gt->pitch_trim_rad;

    /* 2. 回溯曝光时刻 t - vision_delay_s 的云台世界角 */
    int delay_slots = (int)(gt->vision_delay_s * 200.0f + 0.5f);
    if (delay_slots >= GIMBAL_HIST_LEN) delay_slots = GIMBAL_HIST_LEN - 1;
    uint8_t idx = (uint8_t)((gt->hist_head + GIMBAL_HIST_LEN - delay_slots) % GIMBAL_HIST_LEN);

    float pitch_at_cap = gt->hist_pitch[idx];
    float cos_p = clampf(cosf(pitch_at_cap), 0.25f, 1.0f);
    float new_yaw = wrap_rad_pi(gt->hist_yaw[idx] + dyaw / cos_p);
    float new_pitch = pitch_at_cap + dpitch;

    /* 3. 估计目标在世界系下的真实运动角速度 (完全剥离云台自身转动!) */
    if (gt->has_lock && frame_dt > 0.005f && frame_dt < 0.25f) {
        float vy_raw = wrap_rad_pi(new_yaw - gt->target_yaw_cap) / frame_dt;
        float vp_raw = (new_pitch - gt->target_pitch_cap) / frame_dt;
        gt->target_vyaw_filt += gt->vel_lpf_alpha * (vy_raw - gt->target_vyaw_filt);
        gt->target_vpitch_filt += gt->vel_lpf_alpha * (vp_raw - gt->target_vpitch_filt);
    } else {
        gt->target_vyaw_filt = 0.0f;
        gt->target_vpitch_filt = 0.0f;
        gt->has_lock = true;
    }

    gt->target_yaw_cap = new_yaw;
    gt->target_pitch_cap = new_pitch;
    gt->time_since_frame = 0.0f;
}

/* 200Hz 外环解算输出期望角速度指令 -> 送入 1000Hz 陀螺仪角速度 PID 内环 */
void gimbal_tracker_compute_rate_cmd(gimbal_tracker_t *gt, float cur_yaw_rad, float cur_pitch_rad,
                                     float max_rate_rad_s, float *wyaw_ref, float *wpitch_ref) {
    if (!gt->has_lock || gt->time_since_frame > 0.30f) {
        gt->has_lock = false;
        *wyaw_ref = 0.0f;
        *wpitch_ref = 0.0f;
        return;
    }
    /* 总超前预测时间 = 视觉链路固定延迟 + 距上一帧已流逝时间 */
    float lead_t = clampf(gt->vision_delay_s + gt->time_since_frame, 0.0f, 0.12f);
    float pred_yaw = wrap_rad_pi(gt->target_yaw_cap + gt->target_vyaw_filt * lead_t);
    float pred_pitch = gt->target_pitch_cap + gt->target_vpitch_filt * lead_t;

    float err_yaw = wrap_rad_pi(pred_yaw - cur_yaw_rad);
    float err_pitch = pred_pitch - cur_pitch_rad;

    *wyaw_ref = clampf(gt->kp_pos * err_yaw + gt->k_ff * gt->target_vyaw_filt,
                       -max_rate_rad_s, max_rate_rad_s);
    *wpitch_ref = clampf(gt->kp_pos * err_pitch + gt->k_ff * gt->target_vpitch_filt,
                         -max_rate_rad_s, max_rate_rad_s);
}
```

---

### 3.3 云台三类执行器工程特性与专项补偿

| 执行器类型 | 最小分辨率与背隙 | 核心痛点 | 必做工程补偿手段 |
| :--- | :--- | :--- | :--- |
| **PWM / 串行总线舵机**<br>(`MG996R` / `FEETECH ST3215` / `LX-16A`) | PWM 舵机死区 `2–4 us`（`~0.3°`），齿轮背隙 `0.2°–0.8°`；磁编码总线舵机 `4096` 线（`0.088°`） | 内部自带黑盒位置环，外部再套高 $K_i$ 会在背隙内发生 `1–2 Hz` 持续点头狩猎（Limit Cycle） | 1. 严禁 `30 Hz` 阶跃直发角度，用 `200 Hz` 线性/S曲线插值下发；<br>2. 视觉误差 `< 1.5 px` 时开启积分死区冻结；<br>3. 反向换向瞬间叠加半背隙偏置 `+0.5 * backlash * sgn(vel)`（带滞回防抖）。 |
| **闭环步进电机**<br>(`张大头 Emm_V5` / `TMC2209 STEP+DIR`) | 16/32 细分下分辨率极高（`0.056°`），直连同步带几乎零背隙 | 低速共振区（整步 `120–300 PPS` 约 `0.6–1.5 rps`）剧烈抖动丢步；串口发位置帧过快堵塞 | 1. 强制开启 **16 或 32 细分**（禁用 1/2/4 低细分）与 `TMC2209 StealthChop`；<br>2. 把步进电机当作**纯角速度源**控制（定时调节 `TIM->ARR` 脉冲频率），规避频繁加减速启停；<br>3. 限制加速度斜率快速跨过中频机械共振带。 |
| **无刷直驱云台电机**<br>(`DJI GM6020` / `SimpleFOC + AS5600/MT6701`) | `14-bit` 磁编（`0.022°`），**零减速器背隙**，力矩带宽 `> 50 Hz` | 齿槽转矩脉动（Cogging Torque）、轴承静摩擦死区、Pitch 轴相机重心不平衡掉头 | 1. Pitch 轴叠加余弦重力平衡前馈 $u_g = K_g \cos(\theta_p - \theta_0)$；<br>2. 角速度内环输出叠加平滑库仑摩擦前馈 $u_{\text{fric}} = u_c \tanh(\omega_{\text{ref}} / 0.05\text{f})$；<br>3. `GM6020` 优先使用电流模式（固件支持时）或带母线电压前馈的电压模式。 |

---

## 4. 四旋翼无人机飞控与空地协同自主巡航 (Drone / UAV)

### 4.1 机体坐标系致命符号约定警告 (`FLU` vs `FRD`)

> **飞控第一铁律：炸机翻扣（100ms Flip-of-Death）90% 源于 `FLU` 与 `FRD` 坐标系混用！**
> * **本手册以下代码全部严格统一采用右手系 `FLU`（前 `+X`、左 `+Y`、上 `+Z`，与 ROS / 室内视觉SLAM / 激光雷达一致）**：
>   * **Roll ($\phi$，绕 `+X`)**：右翼下沉、左翼抬起为正（$+\phi \implies$ 产生向 `-Y` 右侧的水平推力分量，因此向右飞需给 $+\phi_{\text{ref}}$，向左 `+Y` 飞需给 $-\phi_{\text{ref}}$！）。
>   * **Pitch ($\theta$，绕 `+Y`)**：由右手定则，大拇指指向左 `+Y`，四指从 `+Z`（上）卷向 `+X`（前），故**机头下俯为正 $+\theta$**（$+\theta \implies$ 产生向 `+X` 前方的水平推力分量！）。
>   * **Yaw ($\psi$，绕 `+Z`)**：俯视**逆时针左转为正 $+\psi$**。
>   * **静止水平放置时的加速度计读数**：加速度计测量的是支撑面抵抗重力的比力（Proper Acceleration），在 `FLU` 下水平静止读数为 **`(0, 0, +1g)`**！若你的底层驱动按航空系 `FRD`（前右地）把静止 Z 轴转换成了 `-1g`，直接喂入下方 `FLU` Mahony 滤波器会导致解算姿态瞬间倒置 $180^\circ$！

---

### 4.2 Mahony 互补滤波四元数姿态解算 (`q0, q1, q2, q3`)

采用机体到大地（Body-to-World `ENU/NWU` 水平面）单位四元数 $\mathbf{q} = [q_0, q_1, q_2, q_3]^T$，融合三轴陀螺仪角速度 $\boldsymbol{\omega}$ 与三轴加速度计比力 $\mathbf{a}$，在线估计陀螺仪慢变温漂零偏 $\mathbf{b}_g$。当线性机动过载使加速度模长偏离 `[0.75g, 1.25g]` 窗口时自动切断重力修正，防止水平加减速污染姿态角：

```c
typedef struct {
    float q0, q1, q2, q3;         /* 单位四元数 (初始化为 1, 0, 0, 0) */
    float int_ex, int_ey, int_ez; /* 陀螺零偏积分项 */
    float two_kp;                 /* 2 * Kp, 推荐 2.0f * 0.8f = 1.6f */
    float two_ki;                 /* 2 * Ki, 推荐 2.0f * 0.005f = 0.01f */
    float roll_rad, pitch_rad, yaw_rad;
} mahony_ahrs_t;

void mahony_init(mahony_ahrs_t *m, float kp, float ki) {
    m->q0 = 1.0f; m->q1 = 0.0f; m->q2 = 0.0f; m->q3 = 0.0f;
    m->int_ex = 0.0f; m->int_ey = 0.0f; m->int_ez = 0.0f;
    m->two_kp = 2.0f * kp;
    m->two_ki = 2.0f * ki;
    m->roll_rad = 0.0f; m->pitch_rad = 0.0f; m->yaw_rad = 0.0f;
}

/* 输入: gx,gy,gz [rad/s] (FLU), ax,ay,az [m/s^2 或 g 均可, 静止水平时 az > 0] */
void mahony_update_imu(mahony_ahrs_t *m, float gx, float gy, float gz,
                       float ax, float ay, float az, float g_ref, float dt) {
    float q0 = m->q0, q1 = m->q1, q2 = m->q2, q3 = m->q3;

    /* 1. 检验加速度模长是否在合理重力窗口 [0.75g, 1.25g] 内，剔除剧烈机动与撞击加速度 */
    float a_norm_sq = ax * ax + ay * ay + az * az;
    float a_min_sq = (0.75f * g_ref) * (0.75f * g_ref);
    float a_max_sq = (1.25f * g_ref) * (1.25f * g_ref);

    if (a_norm_sq > a_min_sq && a_norm_sq < a_max_sq) {
        float inv_a = 1.0f / sqrtf(a_norm_sq);
        float ax_n = ax * inv_a;
        float ay_n = ay * inv_a;
        float az_n = az * inv_a;

        /* 估计大地 +Z 轴单位向量在机体 FLU 下的投影 half_v = 0.5 * R^T * [0, 0, 1]^T */
        float half_vx = q1 * q3 - q0 * q2;
        float half_vy = q0 * q1 + q2 * q3;
        float half_vz = q0 * q0 - 0.5f + q3 * q3;

        /* 叉乘误差 half_e = a_norm x half_v */
        float half_ex = (ay_n * half_vz - az_n * half_vy);
        float half_ey = (az_n * half_vx - ax_n * half_vz);
        float half_ez = (ax_n * half_vy - ay_n * half_vx);

        if (m->two_ki > 0.0f) {
            m->int_ex = clampf(m->int_ex + m->two_ki * half_ex * dt, -0.2f, 0.2f);
            m->int_ey = clampf(m->int_ey + m->two_ki * half_ey * dt, -0.2f, 0.2f);
            m->int_ez = clampf(m->int_ez + m->two_ki * half_ez * dt, -0.2f, 0.2f);
            gx += m->int_ex;
            gy += m->int_ey;
            gz += m->int_ez;
        }
        gx += m->two_kp * half_ex;
        gy += m->two_kp * half_ey;
        gz += m->two_kp * half_ez;
    }

    /* 2. 一阶四元数微分方程积分 dq/dt = 0.5 * q (*) omega */
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    float nq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz);
    float nq1 = q1 + ( q0 * gx + q2 * gz - q3 * gy);
    float nq2 = q2 + ( q0 * gy - q1 * gz + q3 * gx);
    float nq3 = q3 + ( q0 * gz + q1 * gy - q2 * gx);

    /* 3. 安全快速归一化 (利用 Cortex-M4F 硬件 VSQRT.F32，杜绝 0x5f3759df 近似误差) */
    float q_norm_sq = nq0 * nq0 + nq1 * nq1 + nq2 * nq2 + nq3 * nq3;
    float inv_q = 1.0f / sqrtf(fmaxf(q_norm_sq, 1e-12f));
    m->q0 = nq0 * inv_q;
    m->q1 = nq1 * inv_q;
    m->q2 = nq2 * inv_q;
    m->q3 = nq3 * inv_q;

    /* 4. 提取 ZYX 欧拉角 (带 asinf 防 NaN 钳位) */
    float sinp = clampf(2.0f * (m->q0 * m->q2 - m->q3 * m->q1), -1.0f, 1.0f);
    m->roll_rad  = atan2f(2.0f * (m->q0 * m->q1 + m->q2 * m->q3),
                          1.0f - 2.0f * (m->q1 * m->q1 + m->q2 * m->q2));
    m->pitch_rad = asinf(sinp);
    m->yaw_rad   = atan2f(2.0f * (m->q0 * m->q3 + m->q1 * m->q2),
                          1.0f - 2.0f * (m->q2 * m->q2 + m->q3 * m->q3));
}
```

---

### 4.3 串级姿态控制与姿态优先 X 型混控 (Attitude-Priority Mixing)

采用标准 **X 型四旋翼布局（FLU 俯视图）**：
* `M1`：右前（Front-Right），桨叶 **逆时针 CCW** 旋转（产生顺时针反扭力矩 $-\tau_z$）
* `M2`：左后（Rear-Left），桨叶 **逆时针 CCW** 旋转（产生顺时针反扭力矩 $-\tau_z$）
* `M3`：左前（Front-Left），桨叶 **顺时针 CW** 旋转（产生逆时针反扭力矩 $+\tau_z$）
* `M4`：右后（Rear-Right），桨叶 **顺时针 CW** 旋转（产生逆时针反扭力矩 $+\tau_z$）

在 `FLU` 符号定义下（$+R$ 右翼下沉需左桨加力、右桨减力；$+P$ 机头下俯需后桨加力、前桨减力；$+Y$ 机体逆时针转需 CW 桨加力、CCW 桨减力）：
$$\begin{bmatrix} M_1 \\ M_2 \\ M_3 \\ M_4 \end{bmatrix} = T + \begin{bmatrix} -R & -P & -Y \\ +R & +P & -Y \\ +R & -P & +Y \\ -R & +P & +Y \end{bmatrix}$$

> **为什么必须实现“姿态优先抗饱和混控（AirMode 思想）”**：若简单对每个电机做 `clampf(m[i], 0.05f, 1.0f)`，当飞机大油门爬升（$T = 0.90$）同时遭遇阵风需要大姿态修正（$R = 0.25$）时，高侧电机被削平在 `1.0`，实际输出差动滚转力矩瞬间丢失一半，且偏航反扭力失衡，导致飞机直接侧翻！必须**优先砍掉偏航 $Y$ 饱和量，再整体平移总油门 $T$ 保全 100% 的横滚与俯仰差动力矩**。

```c
/* 串级外环角度 P 控制器 (250-500Hz): 角度环严禁加积分，防止地面微倾累积翻机!
 * 典型参数: kp_rp = 4.5f ~ 7.0f [1/s], kp_yaw = 3.0f ~ 5.0f [1/s] */
void uav_attitude_outer_p(float roll_ref, float pitch_ref, float yaw_ref,
                          float roll, float pitch, float yaw,
                          float kp_rp, float kp_yaw, float max_rate_rp, float max_rate_yaw,
                          float *rate_r_ref, float *rate_p_ref, float *rate_y_ref) {
    *rate_r_ref = clampf(kp_rp * (roll_ref - roll), -max_rate_rp, max_rate_rp);
    *rate_p_ref = clampf(kp_rp * (pitch_ref - pitch), -max_rate_rp, max_rate_rp);
    *rate_y_ref = clampf(kp_yaw * wrap_rad_pi(yaw_ref - yaw), -max_rate_yaw, max_rate_yaw);
}

/* 姿态优先抗饱和四电机混控 (归一化推力范围 [idle_min, 1.0f], 如 idle_min = 0.06f) */
void uav_mixer_x_attitude_priority(float thrust, float roll_u, float pitch_u, float yaw_u,
                                   float idle_min, float m_out[4]) {
    float span_avail = 1.0f - idle_min;

    /* 1. 最高优先级: 计算纯 Roll + Pitch 差动量，仅在超出全量程物理极限时等比缩放 */
    float rp[4] = {
        -roll_u - pitch_u, /* M1: FR */
        +roll_u + pitch_u, /* M2: RL */
        +roll_u - pitch_u, /* M3: FL */
        -roll_u + pitch_u  /* M4: RR */
    };
    float rp_min = rp[0], rp_max = rp[0];
    for (int i = 1; i < 4; i++) {
        if (rp[i] < rp_min) rp_min = rp[i];
        if (rp[i] > rp_max) rp_max = rp[i];
    }
    float rp_span = rp_max - rp_min;
    if (rp_span > span_avail && rp_span > 1e-6f) {
        float scale = span_avail / rp_span;
        for (int i = 0; i < 4; i++) rp[i] *= scale;
        rp_min *= scale;
        rp_max *= scale;
    }

    /* 2. 次优先级: 平移集体油门 T，确保 T + rp[i] 完全落在 [idle_min, 1.0f] 之内 */
    float t_adj = clampf(thrust, idle_min - rp_min, 1.0f - rp_max);

    /* 3. 最低优先级: 在剩余上下裕度内叠加 Yaw 力矩，绝不挤占 Roll/Pitch 控制权 */
    float yaw_vec[4] = { -yaw_u, -yaw_u, +yaw_u, +yaw_u };
    float y_scale = 1.0f;
    for (int i = 0; i < 4; i++) {
        float base = t_adj + rp[i];
        if (base + yaw_vec[i] > 1.0f && yaw_vec[i] > 1e-6f) {
            float s = (1.0f - base) / yaw_vec[i];
            if (s < y_scale) y_scale = s;
        } else if (base + yaw_vec[i] < idle_min && yaw_vec[i] < -1e-6f) {
            float s = (idle_min - base) / yaw_vec[i];
            if (s < y_scale) y_scale = s;
        }
    }
    y_scale = clampf(y_scale, 0.0f, 1.0f);

    for (int i = 0; i < 4; i++) {
        m_out[i] = clampf(t_adj + rp[i] + yaw_vec[i] * y_scale, idle_min, 1.0f);
    }
}
```

---

### 4.4 定高融合、光流定点解耦与强制安全保护 (Failsafe)

#### (1) 激光 ToF 倾角补偿与 Z 轴加速度融合定高
* **倾角余弦修正**：机腹下视激光（`VL53L1X` / `TFmini-S` / `STP-23L`）测得的是斜距 $d_{\text{tof}}$，真实垂直高度为：
  $$h_{\text{tof}} = d_{\text{tof}} \cdot \cos\phi \cdot \cos\theta$$
  （当 $|\phi| > 35^\circ$ 或 $|\theta| > 35^\circ$ 时斜射易打到远端地面导致高度暴增，应暂停观测更新）。
* **机体倾角推力损失补偿**：当飞机前倾/侧倾运动时，总升力在垂直方向仅剩 $\cos\phi\cos\theta$ 分量，油门输出必须除以余弦因子防止边飞边掉高：
  $$T_{\text{cmd}} = \frac{T_{\text{hover}} + u_{z,\text{pid}}}{\text{clampf}(\cos\phi \cdot \cos\theta,\; 0.5\text{f},\; 1.0\text{f})}$$

#### (2) 光流角速度旋转解耦 (Gyro-Compensated Optical Flow)
光流传感器（`PMW3901` / `LC307`）输出的像素角速度 $(\omega_{\text{flow},x}, \omega_{\text{flow},y})$ 耦合了飞机平移速度与自身俯仰/横滚角速度！**必须先减去相位对齐（相同低通截止频率）的陀螺仪角速度，再乘以真实高度 $h$**：
$$v_{x,\text{body}} = h \cdot (\omega_{\text{flow},x} - \omega_{y,\text{gyro}}), \qquad v_{y,\text{body}} = h \cdot (\omega_{\text{flow},y} + \omega_{x,\text{gyro}})$$
* **手摇零速度核对铁律**：手持无人机悬空在离地 `0.5 m` 处，保持机心空间位置不动，纯手腕前后左右摇晃机身（`±15°`），VOFA+ 波形中解耦后的 `vx_body, vy_body` 必须紧贴 `0.0 m/s`！若随摇晃同幅波动，说明符号反了或光流尺度标定系数 `flow_scale` 不对，切勿直接起飞！

#### (3) 赛场强制安全保护状态机 (Mandatory Contest Failsafe)
任何自主室内无人机必须将以下代码跑在主控最高优先级环路：
1. **倾角超 $40^\circ$ 瞬时锁桨 (`> 40 deg` Instant Kill)**：擦碰杆子或参数震荡一旦倾角越界，在 `1 ms` 内强制四路电调输出 `0` 并锁死 `DISARMED` 状态，保住螺旋桨、电调与碳板机架。
2. **ToF / 光流断联超时自动斜坡盲降**：看门狗检测 ToF 超过 `200 ms` 无新帧或光流/下视超过 `300 ms` 失效时，立即将目标姿态归零水平（`roll_ref = pitch_ref = 0.0f`），切断高度积分，以悬停油门的 `92%` 按 `-0.04f/s` 斜率缓降着陆，杜绝传感器掉线后满油门撞天花板（顶棚杀手）！
3. **起飞前陀螺仪静止方差门限自检**：上电采集 `500` 点计算零偏时，同时检查三轴极差 `max - min < 0.025 rad/s`（约 `1.5°/s`）；若队员碰了飞机或桌面震动，自动清空重采，校准未通过严禁解锁。

```c
typedef enum {
    UAV_DISARMED = 0,
    UAV_CALIBRATING,
    UAV_ARMED_FLYING,
    UAV_FAILSAFE_LANDING,
    UAV_CRASH_LOCKED
} uav_safety_state_t;

uav_safety_state_t uav_failsafe_guard_1khz(uav_safety_state_t state,
                                           float roll_rad, float pitch_rad,
                                           uint32_t tof_age_ms, uint32_t flow_age_ms,
                                           float hover_thrust, float dt,
                                           float *override_roll_ref,
                                           float *override_pitch_ref,
                                           float *override_thrust,
                                           bool  *motors_kill) {
    const float max_tilt_rad = 40.0f * (3.14159265f / 180.0f);
    static float land_thrust = 0.0f;
    static float land_timer = 0.0f;

    if (state == UAV_CRASH_LOCKED || state == UAV_DISARMED) {
        *motors_kill = true;
        return state;
    }

    /* 1. 倾角 > 40 度无条件瞬时急停锁死 */
    if (fabsf(roll_rad) > max_tilt_rad || fabsf(pitch_rad) > max_tilt_rad) {
        *motors_kill = true;
        return UAV_CRASH_LOCKED;
    }

    /* 2. 传感器超时触发水平自稳斜坡盲降 */
    if (state == UAV_ARMED_FLYING && (tof_age_ms > 200U || flow_age_ms > 300U)) {
        state = UAV_FAILSAFE_LANDING;
        land_thrust = hover_thrust * 0.93f;
        land_timer = 0.0f;
    }

    if (state == UAV_FAILSAFE_LANDING) {
        *motors_kill = false;
        *override_roll_ref = 0.0f;
        *override_pitch_ref = 0.0f;
        land_thrust = fmaxf(0.10f, land_thrust - 0.04f * dt);
        *override_thrust = land_thrust;
        land_timer += dt;
        /* 盲降超过 4.0 秒或油门降至怠速底线则自动上锁停桨 */
        if (land_timer > 4.0f || land_thrust <= 0.12f) {
            *motors_kill = true;
            return UAV_DISARMED;
        }
        return state;
    }

    *motors_kill = false;
    return state;
}
```

---

## 5. 多自由度机械臂控制 (Robotic Arm：2/3/4-DOF 串联臂与 SCARA)

### 5.1 几何解析逆运动学与防 `NaN` 边界钳位

以电赛最经典的 **3-DOF / 4-DOF 旋转底座垂直串联机械臂**（如 2024 E题三子棋对弈机械臂、果蔬分拣/码垛臂）为例：
* `q0`：腰部云台旋转角（绕竖直 `+Z` 轴），$q_0 = \text{atan2f}(y, x)$。
* 设肩关节中心高出基座平面 $d_1$，相对回转轴前偏径向偏置 $L_0$；大臂长 $L_1$，小臂长 $L_2$；若带第 4 自由度腕部俯仰关节（保持吸盘竖直朝下 $\phi_{\text{tool}} = -\frac{\pi}{2}$ 或水平抓取 $\phi_{\text{tool}} = 0$，腕部至吸盘中心长 $L_3$），则先反推腕关节中心在柱面平面 $(r, z)$ 内的坐标：
  $$r = \sqrt{x^2 + y^2} - L_0, \quad r_w = r - L_3 \cos\phi_{\text{tool}}, \quad z_w = (z - d_1) - L_3 \sin\phi_{\text{tool}}$$
* 二连杆余弦定理（定义 $q_1$ 为大臂相对水平径向 $+r$ 向上仰角，$q_2$ 为小臂相对大臂延长线的弯折角）：
  $$D^2 = r_w^2 + z_w^2, \qquad \cos q_2 = \frac{D^2 - L_1^2 - L_2^2}{2 L_1 L_2}$$

> **两大生死级数值防护**：
> 1. **工作空间可达球壳钳位**：当视觉标定微偏使目标点超出最大臂展 $D > L_1 + L_2 - \epsilon$（留 $\epsilon = 1.0\,\text{mm}$ 避开全伸直雅可比奇异边界）或缩入内死区 $D < |L_1 - L_2| + \epsilon$ 时，必须将 $(r_w, z_w)$ 沿射线等比例拉回可达球壳表面，**绝不能直接放弃解算或任由余弦值越界**！
> 2. **强制 `clampf(cos_q2, -1.0f, 1.0f)` 防 `NaN` 炸舵机**：即便做了球壳钳位，单精度浮点舍入仍极易算出 `cos_q2 = 1.0000002f`。一旦直接传入 `acosf()`，返回 `NaN` 将污染整个轨迹与 PWM 比较寄存器，导致机械臂瞬间满速砸向限位块！
> 3. **上肘位 (`elbow_up = true`, $q_2 \le 0$) 优先准则**：桌面抓棋子/分拣场景一律选**上肘位**（大臂高抬、小臂下探），避免肘关节横扫撞飞桌面其他棋子或物料盒；对于水平面运动的 **SCARA 臂**（将上述 $(r_w, z_w)$ 直接替换为水平坐标 $(x, y)$ 即可复用同一套二连杆公式！），选与上一时刻 $|q_2 - q_{2,\text{prev}}|$ 更近的一支以防中途翻肘。

```c
typedef struct {
    float d1;      /* 肩关节离地高度 [mm] */
    float l0;      /* 肩关节径向偏置 [mm] */
    float l1;      /* 大臂长度 [mm] */
    float l2;      /* 小臂长度 [mm] */
    float l3;      /* 腕关节到末端吸盘长度 [mm] (3-DOF 无腕关节时设为 0.0f) */
    float q_min[4];/* 各关节软下限 [rad] */
    float q_max[4];/* 各关节软上限 [rad] */
} arm_geom_t;

/* 4-DOF 串联臂解析逆解 (返回 true 表示原生可达, false 表示已安全钳位至工作空间边界或软限位) */
bool arm_ik_4dof(const arm_geom_t *g, float x_mm, float y_mm, float z_mm,
                 float tool_pitch_rad, bool elbow_up, float q_out[4]) {
    bool exact_ok = true;

    /* 1. 底座回转角 q0 (当目标正处回转中心 r_xy ~ 0 时保持 0 防 atan2f 抖动) */
    float r_xy = sqrtf(x_mm * x_mm + y_mm * y_mm);
    q_out[0] = (r_xy > 1e-3f) ? atan2f(y_mm, x_mm) : 0.0f;

    /* 2. 折算腕关节中心在垂直平面内的坐标 (rw, zw) */
    float rw = (r_xy - g->l0) - g->l3 * cosf(tool_pitch_rad);
    float zw = (z_mm - g->d1) - g->l3 * sinf(tool_pitch_rad);

    /* 3. 工作空间球壳可达性检查与径向投影钳位 */
    const float eps = 1.0f; /* 1.0 mm 奇异安全裕度 */
    float r_max = fmaxf(g->l1 + g->l2 - eps, eps);
    float r_min = fabsf(g->l1 - g->l2) + eps;
    float d = sqrtf(rw * rw + zw * zw);

    if (d > r_max) {
        float scale = r_max / d;
        rw *= scale;
        zw *= scale;
        exact_ok = false;
    } else if (d < r_min) {
        if (d > 1e-4f) {
            float scale = r_min / d;
            rw *= scale;
            zw *= scale;
        } else {
            rw = r_min;
            zw = 0.0f;
        }
        exact_ok = false;
    }

    /* 4. 余弦定理 + 强制 [-1.0f, 1.0f] 防 NaN 保护 */
    float d_sq = rw * rw + zw * zw;
    float denom = fmaxf(2.0f * g->l1 * g->l2, 1e-6f);
    float cos_q2 = (d_sq - g->l1 * g->l1 - g->l2 * g->l2) / denom;
    cos_q2 = clampf(cos_q2, -1.0f, 1.0f);

    float sin_q2_mag = sqrtf(fmaxf(0.0f, 1.0f - cos_q2 * cos_q2));
    float sin_q2 = elbow_up ? -sin_q2_mag : sin_q2_mag;
    q_out[2] = atan2f(sin_q2, cos_q2);

    /* 5. 求解大臂仰角 q1 与腕部补偿角 q3 */
    float k1 = g->l1 + g->l2 * cos_q2;
    float k2 = g->l2 * sin_q2;
    q_out[1] = atan2f(zw, rw) - atan2f(k2, k1);
    q_out[3] = tool_pitch_rad - (q_out[1] + q_out[2]);

    /* 6. 关节物理软限位强制钳位 */
    for (int i = 0; i < 4; i++) {
        if (q_out[i] < g->q_min[i] || q_out[i] > g->q_max[i]) {
            q_out[i] = clampf(q_out[i], g->q_min[i], g->q_max[i]);
            exact_ok = false;
        }
    }
    return exact_ok;
}
```

---

### 5.2 多轴时间同步五次多项式轨迹规划 (Quintic Polynomial Trajectory)

若直接把逆解出的新关节角 `[q0, q1, q2, q3]` 一次性丢给舵机/电机，转角小的轴提前停下、转角大的轴还在猛冲，且起止瞬间速度与加速度阶跃会导致轻量化亚克力/碳管底座剧烈晃动（吸盘甩飞棋子！）。

采用归一化时间 $\tau = t / T \in [0, 1]$ 的**五次多项式插值**：
$$s(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5, \quad \dot{s}(\tau) = \frac{30\tau^2 - 60\tau^3 + 30\tau^4}{T}, \quad \ddot{s}(\tau) = \frac{60\tau - 180\tau^2 + 120\tau^3}{T^2}$$
* **完美边界特性**：$s(0)=0, s(1)=1$，且起止两端**速度为零 $\dot{s}(0)=\dot{s}(1)=0$、加速度也为零 $\ddot{s}(0)=\ddot{s}(1)=0$**！
* **自动多轴同步时长计算**：五次曲线峰值速度系数为 $1.875$（位于 $\tau=0.5$），峰值加速度系数为 $\frac{10}{\sqrt{3}} \approx 5.7735$。根据所有关节中最慢的那一轴计算出统一总时长 $T_{\text{sync}}$，即可保证四轴**同起同停、丝滑落位**：

```c
#define ARM_DOF 4

typedef struct {
    float q_start[ARM_DOF];
    float q_delta[ARM_DOF];
    float duration;
    float elapsed;
    bool  active;
} arm_quintic_traj_t;

void arm_traj_plan_sync(arm_quintic_traj_t *tr, const float q_now[ARM_DOF],
                        const float q_goal[ARM_DOF], const float v_max[ARM_DOF],
                        const float a_max[ARM_DOF], float min_duration_s) {
    float t_req = fmaxf(min_duration_s, 0.05f);

    for (int i = 0; i < ARM_DOF; i++) {
        tr->q_start[i] = q_now[i];
        tr->q_delta[i] = q_goal[i] - q_now[i];
        float dq = fabsf(tr->q_delta[i]);

        float tv = (1.875f * dq) / fmaxf(v_max[i], 1e-3f);
        float ta = sqrtf((5.77350269f * dq) / fmaxf(a_max[i], 1e-3f));
        if (tv > t_req) t_req = tv;
        if (ta > t_req) t_req = ta;
    }
    tr->duration = t_req;
    tr->elapsed = 0.0f;
    tr->active = true;
}

/* 100-200Hz 周期采样插值，同步输出各关节期望角度、角速度与角加速度(可直接供 MIT 模式前馈) */
bool arm_traj_eval(arm_quintic_traj_t *tr, float dt,
                   float q_ref[ARM_DOF], float dq_ref[ARM_DOF], float ddq_ref[ARM_DOF]) {
    if (!tr->active) return false;

    tr->elapsed += dt;
    float tau = clampf(tr->elapsed / tr->duration, 0.0f, 1.0f);
    float tau2 = tau * tau;
    float tau3 = tau2 * tau;
    float tau4 = tau3 * tau;
    float tau5 = tau4 * tau;

    float s   = 10.0f * tau3 - 15.0f * tau4 + 6.0f * tau5;
    float ds  = (30.0f * tau2 - 60.0f * tau3 + 30.0f * tau4) / tr->duration;
    float dds = (60.0f * tau - 180.0f * tau2 + 120.0f * tau3) / (tr->duration * tr->duration);

    for (int i = 0; i < ARM_DOF; i++) {
        q_ref[i]   = tr->q_start[i] + tr->q_delta[i] * s;
        dq_ref[i]  = tr->q_delta[i] * ds;
        ddq_ref[i] = tr->q_delta[i] * dds;
    }

    if (tau >= 1.0f) {
        tr->active = false;
    }
    return tr->active;
}
```

> **垂直取放防刮擦准则**：取放桌面物品时，严禁直接从起点斜插向物料点！务必拆分为三段关键帧：`[目标正上方 Z + 40mm 悬停] -> [笛卡尔竖直插值下降抓吸] -> [竖直抬升 Z + 40mm]`。

---

### 5.3 关节软限位与静态重力矩前馈 (Static Gravity Torque Feedforward)

当大臂、小臂前伸处于水平姿态（$\theta_1 \approx 0^\circ$）时，悬臂梁静态重力矩达到峰值；对于达妙 `DM-J4310` MIT 力控电机、无刷 FOC 关节或柔顺大负载舵机，无重力前馈会产生明显的末端下垂静差（`3–12 mm`）。

设 $q_1$ 为大臂相对水平面的绝对仰角，$q_1 + q_2$ 为小臂相对水平面的绝对角，$q_1 + q_2 + q_3$ 为腕部相对水平面的绝对角，各关节所需的静态重力平衡前馈力矩（或等效电流/角度挠度补偿量）为：
$$\tau_{g,1} = K_{g1} \cos q_1 + K_{g2} \cos(q_1 + q_2) + K_{g3} \cos(q_1 + q_2 + q_3)$$
$$\tau_{g,2} = K_{g2} \cos(q_1 + q_2) + K_{g3} \cos(q_1 + q_2 + q_3), \qquad \tau_{g,3} = K_{g3} \cos(q_1 + q_2 + q_3)$$

* **60 秒免称重实测标定法**：无需查阅三维 CAD 质量与质心表！
  1. 先令大臂竖直朝上（$q_1 = +90^\circ, \cos q_1 = 0$），小臂水平前伸（$q_1 + q_2 = 0^\circ$），保持腕部竖直（$q_1 + q_2 + q_3 = -90^\circ$），微调或读取肘关节维持水平所需的稳态电流/力矩，即为 **$K_{g2}$**；
  2. 再令大臂与小臂同时叠合或竖直折叠小臂（使 $q_1 = 0^\circ, q_1 + q_2 = +90^\circ$），读取肩关节维持水平所需的稳态力矩，即为 **$K_{g1}$**。

```c
static inline void arm_gravity_feedforward(const float q[4], float kg1, float kg2, float kg3,
                                           float tau_ff[4]) {
    float c1   = cosf(q[1]);
    float c12  = cosf(q[1] + q[2]);
    float c123 = cosf(q[1] + q[2] + q[3]);

    tau_ff[0] = 0.0f; /* 竖直底座回转轴不受重力矩影响 */
    tau_ff[1] = kg1 * c1 + kg2 * c12 + kg3 * c123;
    tau_ff[2] = kg2 * c12 + kg3 * c123;
    tau_ff[3] = kg3 * c123;
}
```
