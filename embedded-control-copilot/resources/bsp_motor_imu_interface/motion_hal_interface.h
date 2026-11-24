/**
 * @file motion_hal_interface.h
 * @brief Cross-MCU Hardware Abstraction Layer (HAL) interfaces for contest motion control.
 *        跨平台运动控制硬件抽象接口（纯 C99 结构体函数指针解耦，零原厂头文件依赖）。
 *
 * Supported targets via thin BSP adapters:
 *   - STM32 (HAL / LL)
 *   - TI MSPM0 (DriverLib DL_*)
 *   - ESP32 (ESP-IDF MCPWM / PCNT / esp_timer)
 *   - Renesas RA (FSP R_GPT / R_SCI)
 *
 * Design Rule: Algorithm and App layers include ONLY this header, never MCU headers.
 * Changing pins or migrating chips on contest day only requires editing the BSP glue file.
 */

#ifndef MOTION_HAL_INTERFACE_H
#define MOTION_HAL_INTERFACE_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MOTION_TWO_PI_F
#define MOTION_TWO_PI_F (6.283185307f)
#endif

/**
 * @brief Platform timebase and critical-section interface.
 *        时间基准与极短临界区接口（裸机用 PRIMASK，FreeRTOS 用 portENTER_CRITICAL）。
 */
typedef struct {
    void *ctx;
    uint32_t (*get_tick_ms)(void *ctx);
    uint32_t (*get_tick_us)(void *ctx);           /**< Optional: NULL if microsecond timer unused */
    uint32_t (*enter_critical)(void *ctx);        /**< Returns saved interrupt state / PRIMASK */
    void (*exit_critical)(void *ctx, uint32_t saved_state);
} hal_timebase_t;

/**
 * @brief Unified actuator/motor abstraction (-1.0f .. +1.0f normalized duty or torque).
 *        通用电机接口（适用于 TB6612/DRV8833 PWM、MCPWM、CAN 电流归一化、步进速度归一化）。
 */
typedef struct {
    void *ctx;
    bool (*init)(void *ctx);
    void (*set_duty)(void *ctx, float duty_norm); /**< Clamped [-1.0f, +1.0f] after polarity & deadzone */
    void (*brake)(void *ctx);                     /**< Short-brake / active stop */
    void (*coast)(void *ctx);                     /**< High-Z freewheel / disable bridge */
    bool invert_dir;                              /**< Flip motor polarity in 1 bit without rewiring */
    float deadzone_norm;                          /**< Static friction deadzone [0.0f, 0.3f], e.g. 0.06f */
    float max_duty_norm;                          /**< Safety output ceiling (0.0f, 1.0f], e.g. 0.90f */
} hal_motor_t;

/**
 * @brief Apply normalized motor command with NaN guard, deadzone compensation, clamp & polarity.
 * @param m         Pointer to motor descriptor.
 * @param cmd_norm  Desired normalized command in [-1.0f, +1.0f].
 * @return Actual normalized duty sent to BSP driver (before invert_dir).
 */
static inline float hal_motor_apply_cmd(const hal_motor_t *m, float cmd_norm) {
    if ((m == NULL) || (m->set_duty == NULL)) {
        return 0.0f;
    }
    if (!isfinite(cmd_norm)) {
        if (m->brake != NULL) {
            m->brake(m->ctx);
        } else {
            m->set_duty(m->ctx, 0.0f);
        }
        return 0.0f;
    }

    float max_out = m->max_duty_norm;
    if ((max_out <= 0.0f) || (max_out > 1.0f)) {
        max_out = 1.0f;
    }

    float dz = m->deadzone_norm;
    if (dz < 0.0f) {
        dz = 0.0f;
    } else if (dz >= max_out) {
        dz = 0.0f;
    }

    float mag = fabsf(cmd_norm);
    float out = 0.0f;
    if (mag > 1.0e-4f) {
        if (mag > 1.0f) {
            mag = 1.0f;
        }
        /* Linear mapping from (0, 1] into [dz, max_out] avoids jump discontinuity. */
        out = dz + mag * (max_out - dz);
        if (cmd_norm < 0.0f) {
            out = -out;
        }
    }

    float hw_cmd = m->invert_dir ? (-out) : out;
    m->set_duty(m->ctx, hw_cmd);
    return out;
}

/**
 * @brief Quadrature / pulse encoder abstraction using wrap-safe 16-bit free-running counter diff.
 *        编码器抽象接口（推荐硬件计数器自由回绕，严禁每次清零 CNT=0 丢脉冲）。
 */
typedef struct {
    void *ctx;
    bool (*init)(void *ctx);
    uint16_t (*read_raw_cnt)(void *ctx); /**< Read current 16-bit hardware/software counter */
    float counts_per_rev;                /**< Total counts per output shaft rev (PPR * 4 * gear_ratio) */
    float lpf_alpha;                     /**< Speed 1st-order LPF alpha in (0.0f, 1.0f], 1.0f = no filter */
    bool invert_sign;                    /**< Flip encoder sign in 1 bit during open-loop polarity test */

    /* Internal runtime state (observable in Keil Watch / VOFA+) */
    uint16_t last_raw_cnt;
    int64_t total_counts;
    int16_t last_delta_cnt;
    float speed_rad_s;
    bool initialized;
} hal_encoder_t;

/**
 * @brief Reset encoder zero reference and runtime states.
 */
static inline void hal_encoder_reset(hal_encoder_t *enc) {
    if (enc == NULL) {
        return;
    }
    enc->last_raw_cnt = (enc->read_raw_cnt != NULL) ? enc->read_raw_cnt(enc->ctx) : 0u;
    enc->total_counts = 0;
    enc->last_delta_cnt = 0;
    enc->speed_rad_s = 0.0f;
    enc->initialized = true;
}

/**
 * @brief Sample encoder delta and compute filtered shaft angular velocity (rad/s).
 * @param enc   Encoder instance.
 * @param dt_s  Sampling period in seconds (must be > 0).
 * @return Filtered shaft angular speed in rad/s.
 */
static inline float hal_encoder_update(hal_encoder_t *enc, float dt_s) {
    if ((enc == NULL) || (enc->read_raw_cnt == NULL) || (dt_s <= 1.0e-6f) ||
        (enc->counts_per_rev <= 1.0e-3f)) {
        return 0.0f;
    }

    uint16_t now_cnt = enc->read_raw_cnt(enc->ctx);
    if (!enc->initialized) {
        enc->last_raw_cnt = now_cnt;
        enc->initialized = true;
        return 0.0f;
    }

    /* Two's-complement wrap-around subtraction handles 0 <-> 65535 seamlessly. */
    int16_t delta = (int16_t)((uint16_t)(now_cnt - enc->last_raw_cnt));
    enc->last_raw_cnt = now_cnt;

    if (enc->invert_sign) {
        delta = (int16_t)(-delta);
    }

    enc->last_delta_cnt = delta;
    enc->total_counts += (int64_t)delta;

    float raw_rad_s = ((float)delta / enc->counts_per_rev) * (MOTION_TWO_PI_F / dt_s);
    float alpha = enc->lpf_alpha;
    if ((alpha <= 0.0f) || (alpha > 1.0f)) {
        alpha = 1.0f;
    }
    enc->speed_rad_s = enc->speed_rad_s + alpha * (raw_rad_s - enc->speed_rad_s);
    return enc->speed_rad_s;
}

/**
 * @brief Get cumulative output shaft angle in radians.
 */
static inline float hal_encoder_get_angle_rad(const hal_encoder_t *enc) {
    if ((enc == NULL) || (enc->counts_per_rev <= 1.0e-3f)) {
        return 0.0f;
    }
    return ((float)enc->total_counts / enc->counts_per_rev) * MOTION_TWO_PI_F;
}

/**
 * @brief Standardized IMU sample snapshot (FLU right-handed coordinate convention recommended).
 */
typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float gx_dps;
    float gy_dps;
    float gz_dps;
    float ax_g;
    float ay_g;
    float az_g;
    uint32_t timestamp_ms;
    bool is_valid;
} hal_imu_data_t;

/**
 * @brief IMU sensor abstraction (MPU6050 / ICM42688 / BMI088 / WIT JY901).
 */
typedef struct {
    void *ctx;
    bool (*init)(void *ctx);
    bool (*read)(void *ctx, hal_imu_data_t *out_data);
    bool (*calibrate_gyro)(void *ctx, uint16_t sample_count);
} hal_imu_t;

/**
 * @brief Servo actuator abstraction (PWM 50-333Hz or serial bus servo).
 */
typedef struct {
    void *ctx;
    void (*set_angle_deg)(void *ctx, float angle_deg);
    float min_deg;
    float max_deg;
    float zero_offset_deg;
    bool invert_dir;
    float last_cmd_deg;
} hal_servo_t;

/**
 * @brief Apply mechanical-limit clamped servo angle with offset & polarity.
 */
static inline float hal_servo_apply_angle(hal_servo_t *s, float target_deg) {
    if ((s == NULL) || (s->set_angle_deg == NULL) || !isfinite(target_deg)) {
        return 0.0f;
    }
    float clamped = target_deg;
    if (clamped < s->min_deg) {
        clamped = s->min_deg;
    } else if (clamped > s->max_deg) {
        clamped = s->max_deg;
    }
    s->last_cmd_deg = clamped;
    float hw_deg = s->zero_offset_deg + (s->invert_dir ? (-clamped) : clamped);
    s->set_angle_deg(s->ctx, hw_deg);
    return clamped;
}

#ifdef __cplusplus
}
#endif

#endif /* MOTION_HAL_INTERFACE_H */
