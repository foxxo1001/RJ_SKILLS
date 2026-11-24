/**
 * @file pid_controller.h
 * @brief Contest-grade single-precision C99 PID & Cascaded PID controller library.
 *        竞赛级全功能 PID 控制器（抗积分饱和、积分分离、微分先行+低通、前馈、死区、斜率限制、角度回绕、串级分频）。
 *
 * Key properties:
 *   - Pure ISO C99, zero dynamic memory, zero MCU headers.
 *   - Strict float + 'f' suffix math (no double promotion on Cortex-M4F / M33 / ESP32-S3).
 *   - Built-in NaN/Inf safety guard (auto-resets integrator and outputs 0.0f).
 *   - All internal P/I/D/FF terms exposed directly in struct for Keil Watch / VOFA+ tuning.
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;

    float out_min;              /**< Output lower clamp (e.g. -1.0f or -1000.0f) */
    float out_max;              /**< Output upper clamp (e.g. +1.0f or +1000.0f) */
    float integral_limit;       /**< Symmetric clamp for integral term |i_out| <= integral_limit */
    float integral_sep_thresh;  /**< Freeze/zero integration when |err| > thresh; <= 0 disables */
    float err_deadband;         /**< Zero error when |err| < err_deadband; <= 0 disables */
    float d_lpf_alpha;          /**< 1st-order LPF on D term in (0.0f, 1.0f]; 1.0f = unfiltered */
    float max_slew_rate;        /**< Max |du/dt| output ramp rate (units/s); <= 0 disables */

    bool deriv_on_measurement;  /**< true = -Kd * d(meas)/dt (recommended for vision/gimbal!) */
    bool enable_anti_windup;    /**< true = conditional integration clamping when saturated */
} pid_cfg_t;

typedef struct {
    pid_cfg_t cfg;

    /* Internal controller states */
    float integrator;
    float prev_err;
    float prev_prev_err;
    float prev_meas;
    float d_filtered;
    float last_out;
    bool initialized;

    /* Live telemetry fields for Keil Watch / J-Scope / VOFA+ */
    float target;
    float meas;
    float err;
    float p_out;
    float i_out;
    float d_out;
    float ff_out;
    float out;
} pid_ctrl_t;

/**
 * @brief Multi-rate cascaded dual-loop PID (e.g. Outer Angle/Position -> Inner Rate/Speed).
 */
typedef struct {
    pid_ctrl_t outer;
    pid_ctrl_t inner;
    uint16_t outer_div;         /**< Run outer loop every N inner calls (>= 1, e.g. 5 or 10) */
    uint16_t tick_cnt;
    float outer_ref_out;        /**< Cached outer output feeding inner target */
    bool outer_is_angle_deg;    /**< true = use +-180 deg wrap-around on outer loop */
} pid_cascade_t;

/**
 * @brief Populate safe default PID configuration (positional, anti-windup & deriv-on-meas enabled).
 */
void pid_cfg_default(pid_cfg_t *cfg, float kp, float ki, float kd, float out_min, float out_max);

/**
 * @brief Initialize PID controller instance and reset all states.
 */
void pid_init(pid_ctrl_t *pid, const pid_cfg_t *cfg);

/**
 * @brief Reset integrator, derivative history, and last output (call on mode switch / E-stop).
 */
void pid_reset(pid_ctrl_t *pid);

/**
 * @brief Update Kp/Ki/Kd gains live (e.g. from VOFA+ serial tuner) without resetting state.
 */
void pid_set_gains(pid_ctrl_t *pid, float kp, float ki, float kd);

/**
 * @brief Wrap angular difference (degrees) cleanly into [-180.0f, +180.0f].
 */
float pid_wrap_deg180(float err_deg);

/**
 * @brief Positional PID with feedforward injection.
 * @param pid     Controller instance.
 * @param target  Setpoint.
 * @param meas    Current feedback measurement.
 * @param dt_s    Control loop period in seconds (must be > 0).
 * @param ff_val  Feedforward term added directly before output clamp.
 * @return Clamped control output.
 */
float pid_compute_ff(pid_ctrl_t *pid, float target, float meas, float dt_s, float ff_val);

/**
 * @brief Standard positional PID (ff_val = 0.0f).
 */
float pid_compute(pid_ctrl_t *pid, float target, float meas, float dt_s);

/**
 * @brief Positional PID for circular angles in degrees (handles +-180 deg discontinuity safely).
 */
float pid_compute_angle_deg(pid_ctrl_t *pid, float target_deg, float meas_deg, float dt_s,
                            float ff_val);

/**
 * @brief Incremental (velocity-form) PID for wheel speed loops: u[k] = u[k-1] + du.
 *        Notice: Ki and Kd here are discrete gains scaled to the fixed loop period.
 */
float pid_compute_inc(pid_ctrl_t *pid, float target, float meas);

/**
 * @brief Initialize cascaded PID controller.
 */
void pid_cascade_init(pid_cascade_t *cas, const pid_cfg_t *outer_cfg, const pid_cfg_t *inner_cfg,
                      uint16_t outer_div, bool outer_is_angle_deg);

/**
 * @brief Reset both outer and inner loops and divider counter.
 */
void pid_cascade_reset(pid_cascade_t *cas);

/**
 * @brief Execute one inner-loop tick of cascaded PID (outer updates every outer_div ticks).
 */
float pid_cascade_compute(pid_cascade_t *cas, float outer_target, float outer_meas,
                          float inner_meas, float inner_dt_s, float inner_ff);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROLLER_H */
