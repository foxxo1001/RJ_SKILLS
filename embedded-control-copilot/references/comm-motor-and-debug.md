# 嵌入式串口通信协议、竞赛电机驱动生态与硬件在环排障手册 (`comm-motor-and-debug.md`)

本手册面向全国大学生电子设计竞赛（NUEDC）、智能汽车竞赛、RoboMaster 与工程训练赛，系统梳理防丢包防错位的视觉/传感器二进制串口协议、四大主流竞赛执行器驱动生态（直流有刷 H 桥、大疆 CAN 电机、张大头闭环步进、达妙 MIT 关节与总线舵机）、VOFA+ 实时波形与免重刷调参引擎，以及硬件在环（HIL）现象级故障定位规程。所有 C99 代码严格遵循单精度 `float` + `f` 字面量规范。

---

## 1. 传感器与视觉模块防弹级串口通信 (`OpenMV / MaixCam / K210 / RPi / Lidar / JY901`)

### 1.1 为什么 `sscanf` 与无状态机的定长 DMA 在赛场必崩？

在实验室空载静态桌面看似正常的裸串口写法，一旦上赛道启动电机，必然触发以下四类灾难：
1. **上电异步错位滑窗（Byte Slip）**：视觉模块（如 MaixCam / OpenMV）启动耗时 `2–5 s`，或 MCU 单独按下复位键重启时，串口总线上正处于一帧数据的中间字节。定长接收 `HAL_UART_Receive_DMA(&huart1, buf, 10)` 会把“上一帧后 4 字节 + 下一帧前 6 字节”永久拼成错位帧，除非双端断电重开否则永远无法自恢复！
2. **粘包与断包（Sticky / Split Packets）**：依赖 `UART IDLE` 空闲中断直接按 `buf[0] == 0xAA` 解析单包时，若视觉端连续两包紧挨发送（中间无 `>1 Byte` 空闲间隔），一次 `IDLE` 会收到 `20` 字节粘包，第二包直接被丢弃；若 Linux（树莓派/RDK X3）内核调度使一包中间停顿 `0.2 ms`，则一帧被切成两半报废。
3. **电机换向电磁干扰（EMI Glitch Byte）**：有刷电机电刷火花或 H 桥高频斩波通过并行杜邦线耦合进 `RX` 引脚，凭空插入 `1` 个 `0xFF` 或 `0x00` 噪点字节。`sscanf(buf, "%d,%d\n", &dx, &dy)` 遇到乱码不仅耗时数十微秒阻塞中断，更可能解析出 `dx = 28500` 的荒谬像素误差，令云台或小车瞬间满速猛冲！
4. **大小端与结构体对齐填充（Struct Padding）**：32 位 ARM 默认按 4 字节自然对齐，若未加 `#pragma pack(1)` 或在 `Cortex-M0/M0+`（如 `MSPM0G3507`）上对非对齐缓冲区强转 `*(int16_t *)&buf[3]`，直接触发硬件 `HardFault`！

---

### 1.2 二进制协议帧设计与双端字节级对齐实现 (`Python` $\leftrightarrow$ `MCU C99`)

#### 标准竞赛二进制帧结构（小端序 Little-Endian + 缩放定点数）
避免跨平台传输可变长 ASCII 字符串，推荐采用双帧头 + 功能码 + 载荷长度 + 定点整数载荷 + `CRC-8` 校验帧：

| 字节偏移 | 字段名 | 类型 | 取值 / 含义说明 |
| :---: | :--- | :---: | :--- |
| `0` | `SOF1` | `uint8_t` | 固定帧头 1：`0xAA` |
| `1` | `SOF2` | `uint8_t` | 固定帧头 2：`0x55` |
| `2` | `CMD` | `uint8_t` | 消息类型（如 `0x01`=激光靶偏差，`0x02`=巡线偏差与路口标志） |
| `3` | `LEN` | `uint8_t` | 载荷字节数（如 `5`，状态机强制校验 `LEN <= MAX_PAYLOAD` 防溢出） |
| `4` | `status` | `int8_t` | 目标有效标志（`1`=锁定目标，`0`=丢失目标，`-1`=识别到终点线） |
| `5..6` | `dx_x10` | `int16_t` | 小端序，水平像素误差 $\times 10$（如 `-124.5 px` 传 `-1245`，精度 `0.1 px`） |
| `7..8` | `dy_x10` | `int16_t` | 小端序，垂直像素误差 $\times 10$（或巡线航向偏角 $\times 100$） |
| `9` | `CRC8` | `uint8_t` | 对字节 `0..(4+LEN-1)` 计算的 `CRC-8`（多项式 `0x07`）或累加和校验 |

> **Python `struct.pack` 格式符精确对照**：`<` 强制小端序且**零对齐填充（No Padding）**；`B`=`uint8_t`，`b`=`int8_t`，`h`=`int16_t`，`H`=`uint16_t`，`i`=`int32_t`，`f`=`float`。
> * 带显式 `LEN` 字段的完整 10 字节帧格式为 `'<BBBBbhhB'`；
> * 若采用精简定长帧（省略 `LEN`，由 `CMD` 隐含固定 5 字节载荷），9 字节帧格式即为 **`'<BBBbhhB'`**。下方同时给出可直接运行的双端字节级完全匹配实现（采用标准带 `LEN` 通用帧 `'<BBBBbhhB'`，并在注释标明 `'<BBBbhhB'` 紧凑变体）。

#### 发送端：OpenMV / MaixCam (MaixPy) / Raspberry Pi Python 代码

```python
import struct

def calc_crc8(data: bytes) -> int:
    """CRC-8 (poly=0x07, init=0x00)，与 MCU C99 逐位完全一致"""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def clamp_i16(val: int) -> int:
    return max(-32768, min(32767, int(val)))

def pack_vision_target_frame(cmd: int, status: int, dx_px: float, dy_px: float) -> bytes:
    """
    打包视觉偏差帧 (10 字节: 0xAA 0x55 | cmd | len=5 | status(b) | dx_x10(h) | dy_x10(h) | crc8(B))
    注: 若使用无 len 字段的 9 字节紧凑格式 '<BBBbhhB'，仅需去掉下方 payload_len=5 及一个 'B' 即可。
    """
    dx_i16 = clamp_i16(round(dx_px * 10.0))
    dy_i16 = clamp_i16(round(dy_px * 10.0))
    payload_len = 5

    # 先打包前 9 字节计算 CRC8，再拼装完整 10 字节 '<BBBBbhhB'
    header_and_body = struct.pack('<BBBBbhh', 0xAA, 0x55, cmd & 0xFF, payload_len,
                                  status, dx_i16, dy_i16)
    crc = calc_crc8(header_and_body)
    return struct.pack('<BBBBbhhB', 0xAA, 0x55, cmd & 0xFF, payload_len,
                       status, dx_i16, dy_i16, crc)

# OpenMV / MaixPy 主循环调用示例:
# uart.write(pack_vision_target_frame(cmd=0x01, status=1, dx_px=-34.6, dy_px=12.8))
```

#### 接收端：MCU C99 无锁环形队列 + 逐字节抗噪状态机

架构铁律：**`DMA Circular / UART IRQ` 只负责往无锁环形缓冲区塞原始字节，严禁在中断里做复杂业务处理**；控制主循环或高优先级解析任务以非阻塞方式抽干环形队列，逐字节推入有限状态机（FSM）。即便中途插入任意乱码，状态机也能在下一帧 `0xAA 0x55` 到达时 `100%` 自同步恢复！

```c
#include <stdint.h>
#include <stdbool.h>

/* ==================== 1. 无锁单生产者单消费者 (SPSC) 环形缓冲区 ==================== */
#define UART_RB_SIZE 256U /* 必须为 2 的幂次，利用位与替代取模加速 */
#define UART_RB_MASK (UART_RB_SIZE - 1U)

typedef struct {
    volatile uint16_t head;
    volatile uint16_t tail;
    uint8_t buf[UART_RB_SIZE];
} uart_ringbuf_t;

static inline void rb_push_byte_isr(uart_ringbuf_t *rb, uint8_t b) {
    uint16_t next = (uint16_t)((rb->head + 1U) & UART_RB_MASK);
    if (next != rb->tail) { /* 满则丢弃最新字节，绝不踩坏未读旧数据 */
        rb->buf[rb->head] = b;
        rb->head = next;
    }
}

static inline bool rb_pop_byte(uart_ringbuf_t *rb, uint8_t *out) {
    if (rb->head == rb->tail) return false;
    *out = rb->buf[rb->tail];
    rb->tail = (uint16_t)((rb->tail + 1U) & UART_RB_MASK);
    return true;
}

/* ==================== 2. CRC-8 校验与防非对齐小端解包工具 ==================== */
static uint8_t calc_crc8_c(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00U;
    for (uint8_t i = 0U; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1U) ^ 0x07U) : (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

/* 兼容 Cortex-M0/M0+/M4/M7 的绝对安全小端读取 (零未对齐内存访问异常风险) */
static inline int16_t unpack_le_i16(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8U));
}

/* ==================== 3. 逐字节协议解析状态机 ==================== */
#define PROTO_MAX_PAYLOAD 32U

typedef enum {
    PARSE_WAIT_AA = 0,
    PARSE_WAIT_55,
    PARSE_WAIT_CMD,
    PARSE_WAIT_LEN,
    PARSE_READ_PAYLOAD,
    PARSE_WAIT_CRC
} proto_state_t;

typedef struct {
    proto_state_t state;
    uint8_t cmd;
    uint8_t len;
    uint8_t idx;
    uint8_t raw_frame[4U + PROTO_MAX_PAYLOAD]; /* 缓存 AA 55 CMD LEN PAYLOAD 供 CRC 校验 */
    uint32_t rx_ok_cnt;
    uint32_t crc_err_cnt;
} vision_parser_t;

typedef struct {
    int8_t   status;
    float    dx_px;
    float    dy_px;
    uint32_t update_tick_ms;
} vision_target_msg_t;

/* 推入单字节驱动状态机，解析出完整合法帧时返回 true */
bool vision_parser_feed_byte(vision_parser_t *vp, uint8_t byte,
                             uint32_t now_ms, vision_target_msg_t *out_msg) {
    switch (vp->state) {
    case PARSE_WAIT_AA:
        if (byte == 0xAAU) {
            vp->raw_frame[0] = 0xAAU;
            vp->state = PARSE_WAIT_55;
        }
        break;

    case PARSE_WAIT_55:
        if (byte == 0x55U) {
            vp->raw_frame[1] = 0x55U;
            vp->state = PARSE_WAIT_CMD;
        } else if (byte == 0xAAU) {
            /* 关键边界修复! 若连续收到 0xAA 0xAA 0x55，第二个 0xAA 必须停留在 WAIT_55 而非退回起点! */
            vp->state = PARSE_WAIT_55;
        } else {
            vp->state = PARSE_WAIT_AA;
        }
        break;

    case PARSE_WAIT_CMD:
        vp->cmd = byte;
        vp->raw_frame[2] = byte;
        vp->state = PARSE_WAIT_LEN;
        break;

    case PARSE_WAIT_LEN:
        /* 长度合法性硬门限: 若噪声伪造 LEN=250，立即丢弃重置，防止吞掉后续数帧 */
        if (byte > 0U && byte <= PROTO_MAX_PAYLOAD) {
            vp->len = byte;
            vp->raw_frame[3] = byte;
            vp->idx = 0U;
            vp->state = PARSE_READ_PAYLOAD;
        } else {
            vp->state = PARSE_WAIT_AA;
        }
        break;

    case PARSE_READ_PAYLOAD:
        vp->raw_frame[4U + vp->idx] = byte;
        vp->idx++;
        if (vp->idx >= vp->len) {
            vp->state = PARSE_WAIT_CRC;
        }
        break;

    case PARSE_WAIT_CRC: {
        vp->state = PARSE_WAIT_AA;
        uint8_t expected_crc = calc_crc8_c(vp->raw_frame, (uint8_t)(4U + vp->len));
        if (byte == expected_crc) {
            vp->rx_ok_cnt++;
            const uint8_t *pld = &vp->raw_frame[4];
            if (vp->cmd == 0x01U && vp->len == 5U) {
                out_msg->status = (int8_t)pld[0];
                out_msg->dx_px  = (float)unpack_le_i16(&pld[1]) * 0.1f;
                out_msg->dy_px  = (float)unpack_le_i16(&pld[3]) * 0.1f;
                out_msg->update_tick_ms = now_ms;
                return true;
            }
        } else {
            vp->crc_err_cnt++;
        }
        break;
    }

    default:
        vp->state = PARSE_WAIT_AA;
        break;
    }
    return false;
}
```

#### 附：电赛高频外设——维特智能 `WIT JY901 / HWT101` 串口协议速查
* **帧格式（定长 `11` 字节，小端，校验和 `SUM8 = sum(b[0..9]) & 0xFF`）**：
  * `0x55 0x51`: 加速度包，`ax_g = unpack_le_i16(&b[2]) * (16.0f / 32768.0f)`
  * `0x55 0x52`: 角速度包，`gz_deg_s = unpack_le_i16(&b[6]) * (2000.0f / 32768.0f)`
  * `0x55 0x53`: 欧拉角包，`yaw_deg = unpack_le_i16(&b[6]) * (180.0f / 32768.0f)`
* **避坑要点**：默认回传速率仅 `10 Hz`（`9600 bps`），上车闭环前**必须用上位机或串口指令改为 `115200 / 230400 bps`、`100 Hz` 或 `200 Hz` 回传并保存配置**！`HWT101` 单轴高精度陀螺仪上电零偏极小，非常适合小车航向锁定。

---

## 2. 竞赛电机与执行器驱动生态全解 (Contest Motor & Actuator Ecosystems)

### 2.1 直流有刷减速电机 + H 桥驱动 (`TB6612FNG` / `DRV8833` / `AT8236` / `BTN7971`)

#### (1) 快衰减（滑行 Coast）与慢衰减（电磁制动 Brake）模式选型铁律
H 桥在 PWM 关断半周期内对电机电感续流回路的处理方式直接决定低速线性度：
* **快衰减模式（Fast Decay / Sign-Magnitude Coast）**：PWM 关断期间 H 桥下管/上管关闭，电机绕组经体二极管向电源反灌或呈高阻态滑行。**致命缺点**：负载轻重不同时滑行降速极不一致，`PWM < 25%` 区间电机转速严重非线性，下坡或减速时毫无制动力！
* **慢衰减模式（Slow Decay / Brake Decay，强烈推荐用于小车速度闭环！）**：PWM 导通期加正向母线电压，PWM 关断期令**两上管同时导通或两下管同时导通（电枢短接续流制动）**。电机电感电流平缓衰减，占空比与空载/带载转速严格成正比，且目标减速时自动产生强大的电磁再生制动力矩！
  * **`TB6612FNG` 接法**：别忘了 **`STBY` 引脚必须接 `3.3V` 拉高**（悬空即高阻罢工！）。`AIN1=1, AIN2=0` 正转，`PWMA` 输入占空比（`TB6612` 内部逻辑在 `AIN1 != AIN2` 且 `PWMA=0` 时进入 Short-Brake 短接慢衰减态，天然线性度优良；紧急停车置 `AIN1=1, AIN2=1` 强刹）。
  * **`DRV8833 / AT8236` 双 PWM 引脚慢衰减驱动公式**：设期望归一化控制量 $u \in [-1.0\text{f}, +1.0\text{f}]$：
    * 正转（$u \ge 0$）：`IN1_duty = 1.0f`，`IN2_duty = 1.0f - u`（关断期 `IN1=1, IN2=1` 上桥臂短接慢衰减！）
    * 反转（$u < 0$）：`IN1_duty = 1.0f - (-u)`，`IN2_duty = 1.0f`

#### (2) 电机死区实测与四倍频正交编码器无损测速
* **死区实测规程**：将小车轮子贴地静止，以 `0.005f`（`0.5%`）步进缓慢上调开环占空比，记录车轮刚能持续平稳克服齿轮静摩擦转动的最小占空比 `u_deadband`（`JGA25-370 / MG513` 典型值 `0.04f ~ 0.10f`）。
* **编码器 16 位定时器无清零差分法**：启用 `TIMx Encoder Mode TI1 and TI2`（AB 相上下沿**四倍频**，轴转一圈总脉冲数 $C_{\text{rev}} = 4 \times \text{PPR}_{\text{motor}} \times N_{\text{gear}}$，如 `13线霍尔 x 4 x 30减速比 = 1560`，`500线光电 GMR x 4 x 30 = 60000`）。**严禁在中断里写 `TIMx->CNT = 0`**（清零瞬间到达的脉冲边沿会永久丢失导致里程漂移！）。直接利用 C 语言 `int16_t` 补码自然回绕减法，即使跨越 `65535 -> 0` 也能算出精确无误的有符号增量：

```c
#include <math.h>

static inline float clampf(float val, float min_v, float max_v) {
    if (val < min_v) return min_v;
    if (val > max_v) return max_v;
    return val;
}

typedef struct {
    uint16_t prev_cnt;
    int64_t  total_ticks;
    float    counts_per_rev; /* 4.0f * PPR * gear_ratio */
    float    wheel_radius_m;
    float    vel_m_s;
    float    lpf_alpha;
    bool     init;
} encoder_odom_t;

void encoder_odom_update(encoder_odom_t *enc, uint16_t hw_tim_cnt, float dt) {
    if (!enc->init) {
        enc->prev_cnt = hw_tim_cnt;
        enc->init = true;
        return;
    }
    /* 16位无符号相减强转 int16_t: 完美跨越 0 <-> 65535 溢出边界!
     * (前提: 单周期脉冲数 |delta| < 32767，1kHz 下对应极高转速完全满足) */
    int16_t delta = (int16_t)(hw_tim_cnt - enc->prev_cnt);
    enc->prev_cnt = hw_tim_cnt;
    enc->total_ticks += (int64_t)delta;

    float rev_per_sec = ((float)delta / enc->counts_per_rev) / fmaxf(dt, 1e-5f);
    float raw_vel = rev_per_sec * (2.0f * 3.14159265f * enc->wheel_radius_m);
    enc->vel_m_s += enc->lpf_alpha * (raw_vel - enc->vel_m_s);
}

/* 带死区前馈映射的 DRV8833/AT8236 慢衰减双路 PWM 占空比生成 */
void hbridge_apply_slow_decay(float u_norm, float deadband,
                              float *duty_in1, float *duty_in2) {
    u_norm = clampf(u_norm, -1.0f, 1.0f);
    if (fabsf(u_norm) < 1e-3f) {
        /* 零速驻车电磁锁死 */
        *duty_in1 = 1.0f;
        *duty_in2 = 1.0f;
        return;
    }
    /* 将 (0, 1] 线性重映射至 [deadband, 1.0] */
    float mag = deadband + (1.0f - deadband) * fabsf(u_norm);
    mag = clampf(mag, 0.0f, 1.0f);
    if (u_norm > 0.0f) {
        *duty_in1 = 1.0f;
        *duty_in2 = 1.0f - mag;
    } else {
        *duty_in1 = 1.0f - mag;
        *duty_in2 = 1.0f;
    }
}
```

---

### 2.2 大疆 RoboMaster CAN 电机生态 (`M2006+C610` / `M3508+C620` / `GM6020`)

* **总线物理层铁律**：波特率固定 **`1 Mbps`**（采样点建议 `80%–87.5%`），总线首尾两端必须并联 **`120 Ω` 终端电阻**（断电万用表量 `CAN_H` 与 `CAN_L` 阻值必须为 **`60 Ω` 左右**；若为 `120 Ω` 或无穷大，电机一上大电流必定丢帧疯转！）。
* **标准帧 ID 与控制量程全表（注意大疆 CAN 全部为大端序 Big-Endian！）**：

| 电机 / 电调型号 | 控制发送 `StdId` | 反馈接收 `StdId` | 控制量范围 (`int16_t`) | 减速比 |
| :--- | :---: | :---: | :---: | :---: |
| **`M3508` + `C620`** | ID `1..4`: `0x200`<br>ID `5..8`: `0x1FF` | `0x200 + ID`<br>(`0x201 .. 0x208`) | `-16384 .. +16384`<br>（对应 `-20A .. +20A`） | `3591.0f / 187.0f`<br>($\approx 19.2032\text{f}$) |
| **`M2006` + `C610`** | ID `1..4`: `0x200`<br>ID `5..8`: `0x1FF` | `0x200 + ID`<br>(`0x201 .. 0x208`) | `-10000 .. +10000`<br>（对应 `-10A .. +10A`） | `36.0f : 1.0f` |
| **`GM6020` 云台电机** | 电压控制 ID `1..4`: `0x1FF`<br>电压控制 ID `5..7`: `0x2FF`<br>*(电流模式: `0x1FE` / `0x2FE`)* | **`0x204 + ID`**<br>(`0x205 .. 0x20B`) | 电压: `-25000 .. +25000`<br>电流: `-16384 .. +16384` | `1.0f : 1.0f`（直驱） |

> **CAN ID 冲突经典巨坑**：`GM6020` 拨码设为 `ID = 1` 时，其反馈帧 `0x205` 与同一条 CAN 总线上 `ID = 5` 的 `M3508/M2006` 反馈帧 `0x205` **完全撞车**，且控制帧 `0x1FF` 也会互相覆盖！同总线混接时，`M3508` 用 `ID 1..4`（控 `0x200`，回 `0x201..0x204`），`GM6020` 用 `ID 1..4`（控 `0x1FF`，回 `0x205..0x208`）则完美互不冲突！

#### 大端解包与跨 `8191 <-> 0` 多圈连续角度解卷绕 C99 实现

```c
typedef struct {
    uint16_t ecd;            /* 单圈机械角编码值 0..8191 */
    uint16_t last_ecd;
    uint16_t offset_ecd;     /* 上电首帧零点基准 */
    int32_t  round_cnt;      /* 转子累计整圈数 */
    int64_t  total_ecd;      /* 相对上电零点的连续总编码值 */
    int16_t  speed_rpm;      /* 转子转速 [RPM] */
    int16_t  given_current;  /* 实际转矩电流反馈 */
    uint8_t  temperature;    /* 电调/电机温度 [degC] */

    float    inv_gear_ratio; /* 输出轴减速比倒数: M3508 为 187.0f/3591.0f, M2006 为 1.0f/36.0f */
    float    out_angle_deg;  /* 减速器输出轴连续多圈角度 [deg] */
    float    out_vel_rad_s;  /* 减速器输出轴角速度 [rad/s] */
    bool     offset_inited;
} dji_motor_t;

/* 在 CAN 接收中断回调中调用: rx_data 长度固定 8 字节 (Big-Endian) */
void dji_motor_unpack_feedback(dji_motor_t *m, const uint8_t rx_data[8]) {
    m->last_ecd      = m->ecd;
    m->ecd           = (uint16_t)(((uint16_t)rx_data[0] << 8U) | (uint16_t)rx_data[1]);
    m->speed_rpm     = (int16_t)(((uint16_t)rx_data[2] << 8U) | (uint16_t)rx_data[3]);
    m->given_current = (int16_t)(((uint16_t)rx_data[4] << 8U) | (uint16_t)rx_data[5]);
    m->temperature   = rx_data[6];

    if (!m->offset_inited) {
        m->offset_ecd = m->ecd;
        m->last_ecd = m->ecd;
        m->round_cnt = 0;
        m->offset_inited = true;
    }

    /* 过零点解卷绕: 1kHz 反馈下转子单毫秒不可能转过半圈 (4096 ticks, 对应 30000 RPM) */
    int32_t diff = (int32_t)m->ecd - (int32_t)m->last_ecd;
    if (diff > 4096) {
        m->round_cnt--;
    } else if (diff < -4096) {
        m->round_cnt++;
    }

    m->total_ecd = (int64_t)m->round_cnt * 8192LL
                   + (int64_t)m->ecd - (int64_t)m->offset_ecd;
    m->out_angle_deg = (float)m->total_ecd * (360.0f / 8192.0f) * m->inv_gear_ratio;
    m->out_vel_rad_s = (float)m->speed_rpm * (2.0f * 3.14159265f / 60.0f) * m->inv_gear_ratio;
}

/* 打包 4 个电机的 16 位控制电流/电压至 8 字节大端 CAN 发送载荷 (用于 0x200 / 0x1FF / 0x2FF) */
void dji_motor_pack_cmd(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4, uint8_t tx_data[8]) {
    tx_data[0] = (uint8_t)((uint16_t)iq1 >> 8U);
    tx_data[1] = (uint8_t)((uint16_t)iq1 & 0xFFU);
    tx_data[2] = (uint8_t)((uint16_t)iq2 >> 8U);
    tx_data[3] = (uint8_t)((uint16_t)iq2 & 0xFFU);
    tx_data[4] = (uint8_t)((uint16_t)iq3 >> 8U);
    tx_data[5] = (uint8_t)((uint16_t)iq3 & 0xFFU);
    tx_data[6] = (uint8_t)((uint16_t)iq4 >> 8U);
    tx_data[7] = (uint8_t)((uint16_t)iq4 & 0xFFU);
}
```

---

### 2.3 闭环步进电机驱动器 (`张大头 Emm_V5` UART/CAN 协议 & `STEP/DIR`)

电赛云台与麦轮高精度定位小车广泛采用 `张大头 Emm_V5.0` 一体化磁编闭环步进电机。

#### (1) `Emm_V5` 核心协议帧与多机同步触发 (`CheckByte = 0x6B`)
* **使能控制帧（6B）**：`[Addr] [0xF3] [0xAB] [Enable: 01/00] [Sync: 00] [0x6B]`
* **速度模式帧（8B）**：`[Addr] [0xF6] [Dir: 00(CW)/01(CCW)] [RPM_Hi] [RPM_Lo] [Acc: 0..255] [Sync] [0x6B]`
* **位置模式帧（13B）**：`[Addr] [0xFD] [Dir] [RPM_Hi] [RPM_Lo] [Acc] [Pulses_B3..B0 (uint32 BE)] [AbsFlag: 00相对/01绝对] [Sync] [0x6B]`（16 细分下 `3200` 脉冲 = `360°`）。
* **多轴精准同步启动机制**：四轮小车或双轴云台若逐个直发指令（`Sync = 0x00`），第 1 号电机与第 4 号电机启动时间相差数毫秒会导致起步歪头！正确流程：依次向地址 `1..4` 发送带 **`Sync = 0x01`** 的缓存指令（电机收到后暂不动作），最后广播一条 `4` 字节同步触发帧 **`0x00 0xFF 0x66 0x6B`**，所有轴在同一时刻精准同步起动！

#### (2) 严防串口指令洪泛死锁（Avoiding Command Flooding）
* **带宽算账铁律**：在默认 `115200 bps` 下，传输 `1` 字节耗时 `86.8 us`，发送一条 `13` 字节位置指令耗时 `1.13 ms`（四轴轮询一圈需 `4.5 ms`）。若你在 `1 kHz` 定时器里每毫秒往串口塞指令，DMA 发送队列瞬间溢出覆写，且步进驱动器内部单片机忙于解析残缺串口中断导致电机一卡一顿！
* **实战准则**：
  1. 串口模式下，先在驱动板菜单把波特率调至 **`460800` 或 `921600 bps`**（多轴共线更推荐直接走 **`CAN 500kbps/1Mbps` 分包模式**），并在菜单关闭“命令应答返回（Response = Off）”防止半双工/共线回包撞车；
  2. 云台高频连续速度追踪（`> 50 Hz`）若走串口，严格限频在 **`50–100 Hz`** 发送速度帧（`acc = 0` 直接跟随外部斜坡）；若需 `500 Hz` 零延迟视觉伺服，直接用主控硬件定时器 `TIMx PWM (Duty=50%)` 动态改写 `TIMx->ARR` 驱动 **`STEP/DIR` 引脚**！

```c
/* 张大头 Emm_V5 速度模式打包 (8 字节) */
void emm_v5_pack_speed(uint8_t addr, float speed_rpm, uint8_t acc,
                       bool multi_sync, uint8_t tx_buf[8]) {
    uint8_t dir = (speed_rpm >= 0.0f) ? 0U : 1U;
    float abs_rpm = clampf(fabsf(speed_rpm), 0.0f, 3000.0f);
    uint16_t rpm_u16 = (uint16_t)(abs_rpm + 0.5f);

    tx_buf[0] = addr;
    tx_buf[1] = 0xF6U;
    tx_buf[2] = dir;
    tx_buf[3] = (uint8_t)(rpm_u16 >> 8U);
    tx_buf[4] = (uint8_t)(rpm_u16 & 0xFFU);
    tx_buf[5] = acc; /* 0 = 不启用内部梯形加减速，直接响应外部连续调速 */
    tx_buf[6] = multi_sync ? 0x01U : 0x00U;
    tx_buf[7] = 0x6BU;
}

/* 张大头 Emm_V5 位置模式打包 (13 字节, 默认 16 细分 3200 脉冲/圈) */
void emm_v5_pack_pos_deg(uint8_t addr, float angle_deg, float max_rpm, uint8_t acc,
                         bool is_absolute, bool multi_sync, uint8_t tx_buf[13]) {
    uint8_t dir = (angle_deg >= 0.0f) ? 0U : 1U;
    uint16_t rpm_u16 = (uint16_t)(clampf(fabsf(max_rpm), 1.0f, 3000.0f) + 0.5f);
    uint32_t pulses = (uint32_t)(fabsf(angle_deg) * (3200.0f / 360.0f) + 0.5f);

    tx_buf[0]  = addr;
    tx_buf[1]  = 0xFDU;
    tx_buf[2]  = dir;
    tx_buf[3]  = (uint8_t)(rpm_u16 >> 8U);
    tx_buf[4]  = (uint8_t)(rpm_u16 & 0xFFU);
    tx_buf[5]  = acc;
    tx_buf[6]  = (uint8_t)((pulses >> 24U) & 0xFFU);
    tx_buf[7]  = (uint8_t)((pulses >> 16U) & 0xFFU);
    tx_buf[8]  = (uint8_t)((pulses >> 8U)  & 0xFFU);
    tx_buf[9]  = (uint8_t)(pulses & 0xFFU);
    tx_buf[10] = is_absolute ? 0x01U : 0x00U;
    tx_buf[11] = multi_sync  ? 0x01U : 0x00U;
    tx_buf[12] = 0x6BU;
}

/* 广播多机同步运动触发指令 (4 字节): 0x00 0xFF 0x66 0x6B */
static inline void emm_v5_pack_sync_trigger(uint8_t tx_buf[4]) {
    tx_buf[0] = 0x00U; tx_buf[1] = 0xFFU; tx_buf[2] = 0x66U; tx_buf[3] = 0x6BU;
}
```

---

### 2.4 达妙关节电机 (`DM-J4310`) MIT 阻抗模式与串行总线舵机 (`FEETECH` / `LX-16A`)

#### (1) 达妙 `DM-J4310` CAN MIT 阻抗控制打包与解包
电调内部高频执行阻抗控制律：$\tau = K_p (p_{\text{des}} - p) + K_d (v_{\text{des}} - v) + \tau_{\text{ff}}$。
* **上电使能特权帧（8B）**：向电机 `CAN_ID` 发送 `0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC`（失能尾字节为 `0xFD`，保存当前位置为零点尾字节为 `0xFE`）。
* **参数量程对齐红线**：代码中 `P_MAX, V_MAX, T_MAX`（默认 `12.5 rad, 30.0 rad/s, 10.0 N·m`）**必须与达妙上位机写入驱动器 Flash 的设定值一字不差**，否则解算比例完全错乱！纯力矩控制必须令 `kp = 0.0f, kd = 0.0f`；位置阻抗跟踪时 **`kd` 严禁设为 `0`**（推荐 `kp = 15.0f ~ 45.0f`, `kd = 0.8f ~ 2.5f`），否则无阻尼剧烈震荡！

```c
static inline uint16_t dm_float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
    float span = x_max - x_min;
    x = clampf(x, x_min, x_max);
    uint32_t max_int = (1UL << bits) - 1UL;
    return (uint16_t)(((x - x_min) * (float)max_int) / span + 0.5f);
}

static inline float dm_uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits) {
    float span = x_max - x_min;
    uint32_t max_int = (1UL << bits) - 1UL;
    return ((float)x_int * span) / (float)max_int + x_min;
}

/* 打包 DM-J4310 MIT 8字节控制帧: p(16b) | v(12b) | kp(12b) | kd(12b) | t_ff(12b) */
void dm4310_mit_pack(float p_des, float v_des, float kp, float kd, float t_ff,
                     uint8_t tx[8]) {
    const float p_max = 12.5f, v_max = 30.0f, t_max = 10.0f;
    uint16_t p_u  = dm_float_to_uint(p_des, -p_max, p_max, 16U);
    uint16_t v_u  = dm_float_to_uint(v_des, -v_max, v_max, 12U);
    uint16_t kp_u = dm_float_to_uint(kp,    0.0f,   500.0f, 12U);
    uint16_t kd_u = dm_float_to_uint(kd,    0.0f,   5.0f,   12U);
    uint16_t t_u  = dm_float_to_uint(t_ff,  -t_max, t_max,  12U);

    tx[0] = (uint8_t)(p_u >> 8U);
    tx[1] = (uint8_t)(p_u & 0xFFU);
    tx[2] = (uint8_t)(v_u >> 4U);
    tx[3] = (uint8_t)(((v_u & 0x0FU) << 4U) | (kp_u >> 8U));
    tx[4] = (uint8_t)(kp_u & 0xFFU);
    tx[5] = (uint8_t)(kd_u >> 4U);
    tx[6] = (uint8_t)(((kd_u & 0x0FU) << 4U) | (t_u >> 8U));
    tx[7] = (uint8_t)(t_u & 0xFFU);
}

/* 解包 DM-J4310 8字节反馈帧 */
void dm4310_mit_unpack(const uint8_t rx[8], uint8_t *err_state,
                       float *pos_rad, float *vel_rad_s, float *torque_nm) {
    const float p_max = 12.5f, v_max = 30.0f, t_max = 10.0f;
    *err_state = (uint8_t)(rx[0] >> 4U); /* 1=已使能正常, 8=超压, 9=欠压, A=过流, B=MOS过温 */
    uint16_t p_int = (uint16_t)(((uint16_t)rx[1] << 8U) | rx[2]);
    uint16_t v_int = (uint16_t)(((uint16_t)rx[3] << 4U) | (rx[4] >> 4U));
    uint16_t t_int = (uint16_t)(((uint16_t)(rx[4] & 0x0FU) << 8U) | rx[5]);

    *pos_rad   = dm_uint_to_float(p_int, -p_max, p_max, 16U);
    *vel_rad_s = dm_uint_to_float(v_int, -v_max, v_max, 12U);
    *torque_nm = dm_uint_to_float(t_int, -t_max, t_max, 12U);
}
```

#### (2) PWM 舵机与串行总线舵机 (`飞特 FEETECH ST3215` / `幻尔 LX-16A`)
* **PWM 舵机**：标准周期 `20 ms`（`50 Hz`，数字舵机可开 `200 Hz`），高电平脉宽 `500 us .. 2500 us` 线性对应 `0° .. 180°`（或 `270°` 规格）。在 `1 MHz` 计数频率定时器（`PSC = TimerClk_MHz - 1, ARR = 20000 - 1`）下，`CCR = (uint32_t)(500.0f + angle_deg * (2000.0f / 180.0f) + 0.5f)`。
* **串行总线舵机同步写 (`SyncWrite`)**：机械臂多关节同步插值控制时，使用飞特 `STS/ST3215`（默认 `1 Mbps`，小端，`0..4095` 对应 `0..360°`）的 `0x83 SyncWrite` 广播单帧同时更新所有舵机目标位置，零总线回包冲突：

```c
/* 飞特 FEETECH STS 系列 (如 ST3215) 单舵机写目标位置+运行时间+限速 (13字节, 小端) */
void feetech_sts_pack_write_pos(uint8_t id, uint16_t pos_0_4095,
                                uint16_t time_ms, uint16_t speed_steps_s,
                                uint8_t tx[13]) {
    pos_0_4095 &= 0x0FFFU;
    tx[0]  = 0xFFU;
    tx[1]  = 0xFFU;
    tx[2]  = id;
    tx[3]  = 0x09U; /* Length = 参数个数(7) + 2 */
    tx[4]  = 0x03U; /* INST_WRITE */
    tx[5]  = 0x2AU; /* SMS_STS_GOAL_POSITION_L 首地址 42 */
    tx[6]  = (uint8_t)(pos_0_4095 & 0xFFU);
    tx[7]  = (uint8_t)(pos_0_4095 >> 8U);
    tx[8]  = (uint8_t)(time_ms & 0xFFU);
    tx[9]  = (uint8_t)(time_ms >> 8U);
    tx[10] = (uint8_t)(speed_steps_s & 0xFFU);
    tx[11] = (uint8_t)(speed_steps_s >> 8U);

    uint8_t sum = 0U;
    for (int i = 2; i <= 11; i++) sum = (uint8_t)(sum + tx[i]);
    tx[12] = (uint8_t)(~sum);
}
```

---

## 3. VOFA+ 实时波形透视与免重刷在线调参引擎

> **盲调 PID 是电赛最大的时间黑洞！** 花 10 分钟接通无线串口（ESP8266 透传 / 蓝牙高速串口 / 无线 DAP 虚拟串口，`460800` 或 `921600 bps`）配合 **VOFA+ `JustFloat`** 引擎，将调参效率提升 10 倍以上。

### 3.1 `JustFloat` 二进制极速波形发送器 (`0x00, 0x00, 0x80, 0x7F`)

相比 `printf("%f,%f\n")`（单次格式化消耗 `80–250 us` CPU 时间且极易撑爆串口带宽），`JustFloat` 直接将内存中的 `float` 数组连同 4 字节 IEEE-754 正无穷帧尾 `{0x00, 0x00, 0x80, 0x7F}` 一次性交给 `UART DMA` 发出，CPU 拷贝开销 **不足 `1.5 us`**！
* **黄金 8 通道标准观测布局**：`ch[0]=Target, ch[1]=Actual, ch[2]=Error, ch[3]=P_out, ch[4]=I_out, ch[5]=D_out, ch[6]=FF_out, ch[7]=Total_PWM`。一眼即可看清是 $P$ 不够、$I$ 饱和还是 $D$ 放大了高频毛刺！

```c
#include <string.h>

#define VOFA_CH_COUNT 8U

#pragma pack(push, 1)
typedef struct {
    float   ch[VOFA_CH_COUNT];
    uint8_t tail[4]; /* 固定帧尾: 0x00, 0x00, 0x80, 0x7F */
} vofa_justfloat_pkt_t;
#pragma pack(pop)

static vofa_justfloat_pkt_t g_vofa_tx_dma_buf;

/* 在 200-500Hz 周期调用 (传入硬件 UART DMA 非阻塞发送钩子) */
void vofa_send_justfloat(const float channels[VOFA_CH_COUNT],
                         bool (*uart_dma_is_busy)(void),
                         void (*uart_dma_transmit)(const uint8_t *buf, uint16_t len)) {
    /* 若上一帧 DMA 尚未发完，直接跳过本帧，绝不阻塞控制环路! */
    if (uart_dma_is_busy()) {
        return;
    }
    memcpy(g_vofa_tx_dma_buf.ch, channels, sizeof(float) * VOFA_CH_COUNT);
    g_vofa_tx_dma_buf.tail[0] = 0x00U;
    g_vofa_tx_dma_buf.tail[1] = 0x00U;
    g_vofa_tx_dma_buf.tail[2] = 0x80U;
    g_vofa_tx_dma_buf.tail[3] = 0x7FU;

    uart_dma_transmit((const uint8_t *)&g_vofa_tx_dma_buf,
                      (uint16_t)sizeof(vofa_justfloat_pkt_t));
}
```

---

### 3.2 零 `sscanf` 轻量级 ASCII 在线调参解析器 (`P1=1.25\n` / `STOP\n`)

在 VOFA+ 控件面板绑定滑动条发送文本指令（如 `P1=1.25\n`、`I1=0.05\n`、`D1=0.02\n`、`V1=0.80\n`、`STOP=1\n`），以下代码无需引入庞大的 `<stdio.h> sscanf` 或 `<stdlib.h> strtof`，栈占用不到 `32` 字节，支持正负小数安全解析与紧急停机：

```c
/* 极简无依赖单精度浮点字符串解析器 */
static bool parse_float_lite(const char *s, float *out_val) {
    if (!s || *s == '\0') return false;
    float sign = 1.0f;
    if (*s == '-') { sign = -1.0f; s++; }
    else if (*s == '+') { s++; }

    float int_part = 0.0f;
    float frac_part = 0.0f;
    float frac_div = 1.0f;
    bool has_digit = false;

    while (*s >= '0' && *s <= '9') {
        int_part = int_part * 10.0f + (float)(*s - '0');
        has_digit = true;
        s++;
    }
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') {
            frac_part = frac_part * 10.0f + (float)(*s - '0');
            frac_div *= 10.0f;
            has_digit = true;
            s++;
        }
    }
    if (!has_digit) return false;
    *out_val = sign * (int_part + frac_part / frac_div);
    return true;
}

typedef struct {
    char    line_buf[32];
    uint8_t pos;
} tune_cmd_parser_t;

/* 在后台主循环逐字节喂入串口接收字符，支持 P1=1.25\n, I1=..., D1=..., STOP\n */
void tune_cmd_feed_char(tune_cmd_parser_t *tp, char c,
                        float *kp1, float *ki1, float *kd1,
                        float *target_ref, volatile bool *emergency_stop) {
    if (c == '\r' || c == '\n') {
        if (tp->pos == 0U) return;
        tp->line_buf[tp->pos] = '\0';
        tp->pos = 0U;

        /* 最高优先级急停指令: 发送 "STOP" 立即切断所有电机输出 */
        if (tp->line_buf[0] == 'S' && tp->line_buf[1] == 'T' &&
            tp->line_buf[2] == 'O' && tp->line_buf[3] == 'P') {
            *emergency_stop = true;
            return;
        }

        /* 键值格式: <KeyChar><ChannelDigit>=<FloatVal> 例如 P1=2.45 */
        if (tp->line_buf[2] == '=') {
            float val = 0.0f;
            if (parse_float_lite(&tp->line_buf[3], &val)) {
                char key = tp->line_buf[0];
                char idx = tp->line_buf[1];
                if (idx == '1') {
                    if (key == 'P')      *kp1 = clampf(val, 0.0f, 500.0f);
                    else if (key == 'I') *ki1 = clampf(val, 0.0f, 500.0f);
                    else if (key == 'D') *kd1 = clampf(val, 0.0f, 100.0f);
                    else if (key == 'T') *target_ref = val;
                }
            }
        }
        return;
    }

    if (tp->pos < (sizeof(tp->line_buf) - 1U)) {
        tp->line_buf[tp->pos++] = c;
    } else {
        tp->pos = 0U; /* 超长垃圾行自动清零防溢出 */
    }
}
```

---

## 4. 硬件在环 (HIL) 现象级故障排查作战手册

### 4.1 闭环前强制 3 分钟开环极性与符号核对规程 (Pre-Loop Polarity Check)

> **铁律：任何新装机底盘、云台、机械臂或无人机，在未完成以下三步核对前，严禁直接开启闭环 PID！80% 的炸机与打齿都发生在第一次上电闭环的 `0.5 秒` 内。**

1. **Step 0 — 物理架空 / 拆桨保护**：小车用胶带卷垫高底盘使**四轮悬空**；无人机**必须拆掉全部螺旋桨**；机械臂手扶在工作空间正中，随时准备拍急停开关。
2. **Step 1 — 纯传感器正方向手推核验（电机断电失能）**：打开 VOFA+ 观察传感器原始物理量：
   * **小车**：手拨左前轮向车头 `+X` 正向转动 $\implies$ `v_FL` 必须为 **`+`** 且拨一圈里程增量约等于 $2\pi R_w$；手拿车体逆时针旋转 `+Z` $\implies$ IMU `yaw` 与 `gz` 必须同时为 **`+`**！
   * **云台**：手将靶标移向画面右侧（$u > c_x$）$\implies$ 确认计算出的偏航角增量方向恰好能让云台往右追（`FLU` 下右转为 `-yaw`）。
3. **Step 2 — 执行器 `10%` 微占空比开环点动核验**：代码强制输出固定小控制量 `u = +0.10f` 持续 `300 ms`：
   * 确认车轮旋转方向确实使车向前 `+X` 走，**且同一时刻编码器反馈速度 `vel` 也为 `+`**！若轮子往前转而编码器读数为负，立即对调编码器 A/B 相引脚或在底层驱动取反编码器计数（切勿去把 PID 增益改成负数凑合！）。
4. **Step 3 — 极小 $K_p$ 柔顺负反馈拨动测试**：令 $K_i = 0, K_d = 0$，给一个极小的纯比例 $K_p$（最大输出限幅锁在 `15%`），上电将当前位置锁为目标 `target = current`。**用手轻轻拨偏车轮或云台 `10°`**：
   * **正确负反馈**：电机产生柔和的**抵抗力试图把你的手推回原位** $\implies$ 极性 100% 正确，可放心逐步放开限幅整定 $K_p \to K_d \to K_i$！
   * **致命正反馈**：手刚微碰一下，机构顺着你推的方向瞬间加速逃逸顶死限位 $\implies$ 反馈或输出极性反了！

---

### 4.2 赛场五大高频故障现象矩阵与根因定位表

| 典型故障现象 | 核心物理/代码根因 | 1 分钟现场验证与根治处方 |
| :--- | :--- | :--- |
| **1. 一使能瞬间全速疯转 / 猛砸机械限位** | ① **正反馈**：编码器 AB 相反接、IMU 轴向反装或混控矩阵符号写反；<br>② **角度过零点跳变**：当前姿态恰在 $\pm 180^\circ$ 或编码器 `0 / 8191` 交界处，未做 `wrap_deg180` / 多圈解卷绕；<br>③ **目标初值未对齐**：上电 `yaw = 135°` 而 `target_yaw` 硬编码为 `0°`，产生 `135°` 巨幅阶跃。 | 1. 立即退回 **4.1 节三步法** 查符号；<br>2. 全局搜索所有角度相减处是否漏写 `wrap_deg180()`；<br>3. 在 `IDLE -> ARMED` 状态切换瞬间强制执行一次 `target = current_meas; pid_reset(&pid);`。 |
| **2. 电机原地高频“滋滋”啸叫、剧烈抖振且外壳迅速发烫** | ① **$K_d$ 过大且无低通滤波**：编码器每毫秒跳变 $\pm 1$ 个脉冲被 `/ 0.001f` 放大 1000 倍，导致 PWM 在 `+Max` 与 `-Max` 间高频满幅打摆；<br>② **采样周期抖动（Jitter）**：在 `while(1)` 或低优先级任务里算 PID，$\Delta t$ 忽长忽短导致微分项爆炸。 | 1. 在 VOFA+ 看 `D_out` 通道：若毛刺幅度占总输出 `> 30%`，立即把 $K_d$ 减半并串联 `20–40 Hz` 一阶/二阶 Butterworth 低通滤波；<br>2. 将内环移入硬件定时器确定性中断。 |
| **3. 围绕目标点慢速 `1–3 Hz` 正弦等幅摆动（画龙 / 点头狩猎）** | ① **$K_p$ 或 $K_i$ 过猛 / 积分饱和**：系统相位裕度耗尽；<br>② **过度滤波或视觉长延迟**：传感器加了 20 点滑动平均或 `fc < 3 Hz` 重度滤波，引入巨大相位滞后；<br>③ **齿轮背隙 + 静摩擦极限环**：舵机/减速箱存在空回死区，$K_i$ 缓慢蓄力冲破静摩擦后越过背隙超调，周而复始。 | 1. 先令 $K_i = 0$：若振荡消失，说明是积分过强或背隙极限环 $\implies$ 开启**积分限幅 + 到位误差死区（`|err| < eps` 停积分）**；<br>2. 若 $K_i=0$ 仍慢晃，检查滤波截止频率是否过低，云台启用 **历史姿态对齐补偿视觉延迟**，适当加入陀螺仪角速度阻尼 $-K_d g_z$。 |
| **4. 平时正常，唯独电机急加速 / 换向瞬间 MCU 重启、屏幕白屏或串口狂出乱码** | ① **共电源轨欠压复位（Brown-Out Reset, `BOR`）**：舵机或驱动共用给单片机供电的 `5V DCDC/LDO`，堵转尖峰电流（`3–8 A`）把 `5V` 瞬间拉塌至 `2.6 V` 以下；<br>② **母线缺大容量储能电解电容**：再生制动反灌尖峰或电池内阻压降无缓冲；<br>③ **地线环路（Ground Bounce）**：电机 `5 A` 回流电流流过与 MCU/OpenMV 共用的细杜邦 `GND` 线，抬高地电位 `1 V+` 淹没 `3.3 V` 串口电平！ | 1. **电源彻底分轨**：MCU 与视觉走独立低压差 LDO/DCDC，舵机与电机走独立大电流航模 BEC/DCDC；<br>2. 在电机驱动板 `VM` 与 `GND` 焊盘根部就近并联 **`470 uF ~ 2200 uF` 低阻电解电容 + `100 nF` 瓷片电容**；有刷电机两极焊盘间加焊 `104 (0.1 uF)` 消火花电容；<br>3. **单点星型接地（Star Ground）**：功率地与信号地仅在锂电池总负极一点汇合，绝不串接细杜邦线！ |
| **5. 运行中随机卡死进入 `HardFault_Handler`** | 数组/DMA 越界踩栈、RTOS 任务栈溢出、空函数指针调用、除零或 `Cortex-M0+` 非对齐内存访问、FPU 未使能执行浮点指令。 | 严禁盲猜删代码！按下方 **4.3 节寄存器与栈回溯规程** 30 秒精确锁定崩溃源代码行号。 |

---

### 4.3 `HardFault` 精确案发现场还原规程 (`Cortex-M` / `Keil` / `CCS` / `ESP-IDF`)

#### (1) `ARM Cortex-M3 / M4F / M7` 故障状态寄存器速查
连接仿真器暂停在 `HardFault_Handler` 时，打开 IDE 的 `Peripherals -> Core Peripherals -> Fault Reports`（或直接查看以下物理内存地址）：
* **`SCB->HFSR` (`0xE000ED2C`)**：若 `Bit 30 (FORCED) == 1`，说明是由总线错误、存储保护错误或用法错误升级而来，继续看 `CFSR`：
* **`SCB->CFSR` (`0xE000ED28`)**（拆分为高 16 位 `UFSR`、中 8 位 `BFSR`、低 8 位 `MMFSR`）：
  * **`PRECISERR` (`CFSR Bit 9`) + `BFARVALID` (`Bit 15`)**：精确数据总线访问异常！此时直接读取 **`SCB->BFAR` (`0xE000ED38`)**，里面存放的就是**导致崩溃的精确非法指针内存地址**（如 `0x20020000` 越过 SRAM 上界，或 `0x08xxxxxx` 试图往 Flash 写数据）！
  * **`DACCVIOL` (`CFSR Bit 1`) + `MMARVALID` (`Bit 7`)**：空指针或 MPU 越权访问，读取 **`SCB->MMFAR` (`0xE000ED34`)** 查看非法地址。
  * **`NOCP` (`CFSR Bit 19`)**：代码编译开启了硬件浮点指令（`VLDR / VMUL.F32`），但启动代码 `SystemInit()` 忘了置位 `SCB->CPACR |= (0xFUL << 20)` 开启协处理器权限！
  * **`INVSTATE` (`Bit 17`) / `INVPC` (`Bit 18`)**：多半是**未初始化的回调函数指针为 `NULL (0x00000000)`**，或栈缓冲区越界覆写了返回地址 `LR`！
  * **`UNALIGNED` (`Bit 24`)**：非对齐多字节访问（注：电赛高频主控 TI **`MSPM0G3507` 属于 `Cortex-M0+` 架构**，无 `CFSR` 寄存器，且**硬件完全不支持任何非对齐 `uint16_t / uint32_t / float` 读写**——只要对奇数地址指针解引用 `*(float *)&rx_buf[1]`，必进 `HardFault`！务必使用 `memcpy` 或逐字节移位解包）。

#### (2) 异常自动压栈帧 (`MSP` / `PSP`) 提取崩溃指令地址 `PC` 与调用者 `LR`
硬件进入 `HardFault` 瞬间，内核自动将故障前瞬间的 8 个核心寄存器按固定顺序压入当前栈（裸机与中断内压入主栈 **`MSP`**；FreeRTOS / RT-Thread 任务线程内压入进程栈 **`PSP`**，由进入异常时的 `LR (EXC_RETURN)` 第 2 位区分）：

| 栈偏移 | `sp[0]` | `sp[1]` | `sp[2]` | `sp[3]` | `sp[4]` | **`sp[5]`** | **`sp[6]`** | `sp[7]` |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| 寄存器 | `R0` | `R1` | `R2` | `R3` | `R12` | **`LR` (调用返回地址)** | **`PC` (肇事指令精确地址!)** | `xPSR` |

在工程中放入以下轻量级 `HardFault` 捕获桩，即使赛场跑车不挂仿真器，也能通过全局变量快照或串口急停打印出精确 `PC` 与 `LR`：

```c
typedef struct {
    uint32_t r0, r1, r2, r3, r12;
    uint32_t lr;   /* 发生故障前刚调用的返回地址 */
    uint32_t pc;   /* 引发精确异常的那条机器指令地址 */
    uint32_t xpsr;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t bfar;
} fault_snapshot_t;

volatile fault_snapshot_t g_fault_snap;

void hardfault_capture_c(const uint32_t *stack_frame) {
    /* 第一时间无条件关断所有电机 PWM 与 H 桥使能引脚! */
    /* board_emergency_disable_all_actuators(); */

    g_fault_snap.r0   = stack_frame[0];
    g_fault_snap.r1   = stack_frame[1];
    g_fault_snap.r2   = stack_frame[2];
    g_fault_snap.r3   = stack_frame[3];
    g_fault_snap.r12  = stack_frame[4];
    g_fault_snap.lr   = stack_frame[5];
    g_fault_snap.pc   = stack_frame[6];
    g_fault_snap.xpsr = stack_frame[7];

#if defined(__CORTEX_M) && (__CORTEX_M >= 3U)
    g_fault_snap.cfsr = (*((volatile uint32_t *)0xE000ED28UL));
    g_fault_snap.hfsr = (*((volatile uint32_t *)0xE000ED2CUL));
    g_fault_snap.bfar = (*((volatile uint32_t *)0xE000ED38UL));
#endif

    while (1) {
        /* 挂起等待仿真器 Attach 读取 g_fault_snap，或翻转故障报警红灯 */
    }
}

/* GCC / ARMClang (AC6) 通用裸函数入口: 根据 EXC_RETURN bit2 自动区分 MSP 与 PSP */
#if defined(__GNUC__) || defined(__clang__)
__attribute__((naked)) void HardFault_Handler(void) {
    __asm volatile(
        "movs r0, #4            \n"
        "mov  r1, lr            \n"
        "tst  r0, r1            \n"
        "beq  1f                \n"
        "mrs  r0, psp           \n"
        "b    2f                \n"
        "1:                     \n"
        "mrs  r0, msp           \n"
        "2:                     \n"
        "b    hardfault_capture_c\n"
    );
}
#endif
```

#### (3) 三大主流开发环境由 `PC / LR` 定位源码行号实操步骤
1. **Keil MDK (`STM32 / GD32`)**：
   * 挂仿真器停下后，在 `Command` 窗口输入 `PC` 查看 `g_fault_snap.pc`（若 `pc == 0x00000000` 说明调用了空函数指针，改看 `g_fault_snap.lr` 找是谁调的！）；
   * 在 `Disassembly` 反汇编窗口右键 $\to$ **`Show Disassembly at Address...`** $\to$ 输入 `g_fault_snap.pc`（或 `lr - 1`），反汇编窗口将直接高亮跳转到对应的 **C 源代码文件名与具体行号**！
2. **TI CCS (`MSPM0G3507` / `TM4C`)**：
   * 在调试视图打开 `View -> Registers` 读取 `MSP`（或 `Expressions` 窗口查看 `g_fault_snap.pc` 与 `g_fault_snap.lr`）；
   * 打开 `View -> Disassembly`，在地址栏粘贴 `pc` 十六进制地址回车，即可直接看到触发非对齐访问或数组越界的 C 语句；同时可在命令行执行 `arm-none-eabi-addr2line -pfiaC -e Debug/project.out 0x00001A4C`。
3. **ESP-IDF (`ESP32-S3` / `ESP32-C3`)**：
   * 串口崩溃打印出 `Backtrace: 0x42008a1c:0x3fc9b120 0x42008945:0x3fc9b150 ...` 时，使用官方监视器 `idf.py monitor` 会**全自动将整条调用栈解码为 `文件路径:行号 (函数名)`**；
   * 若用第三方串口助手抓到了日志，手动执行单条命令秒解：
     `xtensa-esp32s3-elf-addr2line -pfiaC -e build/your_app.elf 0x42008a1c 0x42008945`
