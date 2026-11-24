/**
 * @file pid_controller.c
 * @brief Implementation of contest-grade single-precision C99 PID & Cascaded PID controllers.
 */

#include "pid_controller.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float clampf_local(float v, float lo, float hi) {
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

void pid_cfg_default(pid_cfg_t *cfg, float kp, float ki, float kd, float out_min, float out_max) {
    if (cfg == NULL) {
        return;
    }
    cfg->kp = kp;
    cfg->ki = ki;
    cfg->kd = kd;
    cfg->out_min = out_min;
    cfg->out_max = out_max;
    float abs_max = fmaxf(fabsf(out_min), fabsf(out_max));
    cfg->integral_limit = 0.5f * abs_max;
    cfg->integral_sep_thresh = 0.0f; /* Disabled by default */
    cfg->err_deadband = 0.0f;
    cfg->d_lpf_alpha = 0.35f;        /* Moderate noise filtering on derivative */
    cfg->max_slew_rate = 0.0f;       /* Disabled by default */
    cfg->deriv_on_measurement = true;
    cfg->enable_anti_windup = true;
}

void pid_init(pid_ctrl_t *pid, const pid_cfg_t *cfg) {
    if ((pid == NULL) || (cfg == NULL)) {
        return;
    }
    memset(pid, 0, sizeof(*pid));
    pid->cfg = *cfg;
    if (pid->cfg.out_min > pid->cfg.out_max) {
        float tmp = pid->cfg.out_min;
        pid->cfg.out_min = pid->cfg.out_max;
        pid->cfg.out_max = tmp;
    }
    if (pid->cfg.integral_limit < 0.0f) {
        pid->cfg.integral_limit = fabsf(pid->cfg.integral_limit);
    }
    if ((pid->cfg.d_lpf_alpha <= 0.0f) || (pid->cfg.d_lpf_alpha > 1.0f)) {
        pid->cfg.d_lpf_alpha = 1.0f;
    }
}

void pid_reset(pid_ctrl_t *pid) {
    if (pid == NULL) {
        return;
    }
    pid->integrator = 0.0f;
    pid->prev_err = 0.0f;
    pid->prev_prev_err = 0.0f;
    pid->prev_meas = 0.0f;
    pid->d_filtered = 0.0f;
    pid->last_out = 0.0f;
    pid->initialized = false;
    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;
    pid->ff_out = 0.0f;
    pid->out = 0.0f;
    pid->err = 0.0f;
}

void pid_set_gains(pid_ctrl_t *pid, float kp, float ki, float kd) {
    if (pid == NULL) {
        return;
    }
    if (isfinite(kp)) {
        pid->cfg.kp = kp;
    }
    if (isfinite(ki)) {
        pid->cfg.ki = ki;
    }
    if (isfinite(kd)) {
        pid->cfg.kd = kd;
    }
}

float pid_wrap_deg180(float err_deg) {
    if (!isfinite(err_deg)) {
        return 0.0f;
    }
    float wrapped = fmodf(err_deg + 180.0f, 360.0f);
    if (wrapped < 0.0f) {
        wrapped += 360.0f;
    }
    return wrapped - 180.0f;
}

static float pid_compute_core(pid_ctrl_t *pid, float target, float meas, float err,
                              float d_meas_delta, float dt_s, float ff_val) {
    if ((pid == NULL) || !isfinite(target) || !isfinite(meas) || !isfinite(err) ||
        !isfinite(ff_val) || !isfinite(dt_s) || (dt_s <= 1.0e-6f)) {
        if (pid != NULL) {
            pid_reset(pid);
        }
        return 0.0f;
    }

    pid->target = target;
    pid->meas = meas;
    pid->ff_out = ff_val;

    /* Deadband suppresses micro-chatter on servos/encoders at rest */
    if ((pid->cfg.err_deadband > 0.0f) && (fabsf(err) < pid->cfg.err_deadband)) {
        err = 0.0f;
    }
    pid->err = err;

    /* Proportional term */
    float p_term = pid->cfg.kp * err;

    /* Derivative term (first-tick guard prevents startup kick) */
    float d_raw = 0.0f;
    if (pid->initialized) {
        if (pid->cfg.deriv_on_measurement) {
            d_raw = -pid->cfg.kd * (d_meas_delta / dt_s);
        } else {
            d_raw = pid->cfg.kd * ((err - pid->prev_err) / dt_s);
        }
    }
    float alpha = pid->cfg.d_lpf_alpha;
    pid->d_filtered = pid->d_filtered + alpha * (d_raw - pid->d_filtered);
    float d_term = pid->d_filtered;

    /* Integral term with separation + conditional anti-windup clamping */
    float i_candidate = pid->integrator;
    bool allow_integrate = true;

    if ((pid->cfg.integral_sep_thresh > 0.0f) && (fabsf(err) > pid->cfg.integral_sep_thresh)) {
        allow_integrate = false;
    }

    if (allow_integrate && (pid->cfg.ki != 0.0f)) {
        float pre_out = p_term + pid->integrator + d_term + ff_val;
        bool sat_high = (pre_out >= pid->cfg.out_max) && ((pid->cfg.ki * err) > 0.0f);
        bool sat_low = (pre_out <= pid->cfg.out_min) && ((pid->cfg.ki * err) < 0.0f);

        if (!pid->cfg.enable_anti_windup || (!sat_high && !sat_low)) {
            i_candidate = pid->integrator + pid->cfg.ki * err * dt_s;
            if (pid->cfg.integral_limit > 0.0f) {
                i_candidate = clampf_local(i_candidate, -pid->cfg.integral_limit,
                                           pid->cfg.integral_limit);
            }
        }
    }
    pid->integrator = i_candidate;

    /* Combine & clamp */
    float raw_out = p_term + pid->integrator + d_term + ff_val;
    float out = clampf_local(raw_out, pid->cfg.out_min, pid->cfg.out_max);

    /* Optional output slew-rate limiting (soft start / mechanical jerk protection) */
    if ((pid->cfg.max_slew_rate > 0.0f) && pid->initialized) {
        float max_step = pid->cfg.max_slew_rate * dt_s;
        out = clampf_local(out, pid->last_out - max_step, pid->last_out + max_step);
    }

    pid->prev_err = err;
    pid->prev_meas = meas;
    pid->last_out = out;
    pid->initialized = true;

    pid->p_out = p_term;
    pid->i_out = pid->integrator;
    pid->d_out = d_term;
    pid->out = out;
    return out;
}

float pid_compute_ff(pid_ctrl_t *pid, float target, float meas, float dt_s, float ff_val) {
    if (pid == NULL) {
        return 0.0f;
    }
    float err = target - meas;
    float d_meas = pid->initialized ? (meas - pid->prev_meas) : 0.0f;
    return pid_compute_core(pid, target, meas, err, d_meas, dt_s, ff_val);
}

float pid_compute(pid_ctrl_t *pid, float target, float meas, float dt_s) {
    return pid_compute_ff(pid, target, meas, dt_s, 0.0f);
}

float pid_compute_angle_deg(pid_ctrl_t *pid, float target_deg, float meas_deg, float dt_s,
                            float ff_val) {
    if (pid == NULL) {
        return 0.0f;
    }
    float err = pid_wrap_deg180(target_deg - meas_deg);
    float d_meas = pid->initialized ? pid_wrap_deg180(meas_deg - pid->prev_meas) : 0.0f;
    return pid_compute_core(pid, target_deg, meas_deg, err, d_meas, dt_s, ff_val);
}

float pid_compute_inc(pid_ctrl_t *pid, float target, float meas) {
    if ((pid == NULL) || !isfinite(target) || !isfinite(meas)) {
        if (pid != NULL) {
            pid_reset(pid);
        }
        return 0.0f;
    }

    float err = target - meas;
    if ((pid->cfg.err_deadband > 0.0f) && (fabsf(err) < pid->cfg.err_deadband)) {
        err = 0.0f;
    }

    pid->target = target;
    pid->meas = meas;
    pid->err = err;

    if (!pid->initialized) {
        /* Safe startup without derivative spike */
        float init_du = pid->cfg.kp * err + pid->cfg.ki * err;
        float out = clampf_local(pid->last_out + init_du, pid->cfg.out_min, pid->cfg.out_max);
        pid->prev_prev_err = err;
        pid->prev_err = err;
        pid->prev_meas = meas;
        pid->last_out = out;
        pid->out = out;
        pid->p_out = pid->cfg.kp * err;
        pid->i_out = pid->cfg.ki * err;
        pid->d_out = 0.0f;
        pid->initialized = true;
        return out;
    }

    float dp = pid->cfg.kp * (err - pid->prev_err);
    float di = pid->cfg.ki * err;
    if ((pid->cfg.integral_sep_thresh > 0.0f) && (fabsf(err) > pid->cfg.integral_sep_thresh)) {
        di = 0.0f;
    }
    float dd = pid->cfg.kd * (err - 2.0f * pid->prev_err + pid->prev_prev_err);

    float delta_u = dp + di + dd;
    float out = clampf_local(pid->last_out + delta_u, pid->cfg.out_min, pid->cfg.out_max);

    pid->prev_prev_err = pid->prev_err;
    pid->prev_err = err;
    pid->prev_meas = meas;
    pid->p_out = dp;
    pid->i_out = di;
    pid->d_out = dd;
    pid->last_out = out;
    pid->out = out;
    return out;
}

void pid_cascade_init(pid_cascade_t *cas, const pid_cfg_t *outer_cfg, const pid_cfg_t *inner_cfg,
                      uint16_t outer_div, bool outer_is_angle_deg) {
    if ((cas == NULL) || (outer_cfg == NULL) || (inner_cfg == NULL)) {
        return;
    }
    pid_init(&cas->outer, outer_cfg);
    pid_init(&cas->inner, inner_cfg);
    cas->outer_div = (outer_div == 0u) ? 1u : outer_div;
    cas->tick_cnt = 0u;
    cas->outer_ref_out = 0.0f;
    cas->outer_is_angle_deg = outer_is_angle_deg;
}

void pid_cascade_reset(pid_cascade_t *cas) {
    if (cas == NULL) {
        return;
    }
    pid_reset(&cas->outer);
    pid_reset(&cas->inner);
    cas->tick_cnt = 0u;
    cas->outer_ref_out = 0.0f;
}

float pid_cascade_compute(pid_cascade_t *cas, float outer_target, float outer_meas,
                          float inner_meas, float inner_dt_s, float inner_ff) {
    if ((cas == NULL) || (inner_dt_s <= 1.0e-6f)) {
        return 0.0f;
    }

    if (cas->tick_cnt == 0u) {
        float outer_dt_s = inner_dt_s * (float)cas->outer_div;
        if (cas->outer_is_angle_deg) {
            cas->outer_ref_out =
                pid_compute_angle_deg(&cas->outer, outer_target, outer_meas, outer_dt_s, 0.0f);
        } else {
            cas->outer_ref_out = pid_compute(&cas->outer, outer_target, outer_meas, outer_dt_s);
        }
    }

    cas->tick_cnt++;
    if (cas->tick_cnt >= cas->outer_div) {
        cas->tick_cnt = 0u;
    }

    return pid_compute_ff(&cas->inner, cas->outer_ref_out, inner_meas, inner_dt_s, inner_ff);
}
