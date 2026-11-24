/**
 * @file contest_scheduler_fsm.c
 * @brief Implementation of deterministic cooperative scheduler, contest task FSM, and safety guard.
 */

#include "contest_scheduler_fsm.h"

#include <math.h>
#include <string.h>

/* ===================== 1. Cooperative Time-Slice Scheduler ===================== */

void sched_init(sched_t *s, sched_us_fn_t get_tick_us_opt) {
    if (s == NULL) {
        return;
    }
    memset(s, 0, sizeof(*s));
    s->get_tick_us = get_tick_us_opt;
}

int8_t sched_add_task(sched_t *s, const char *name, sched_task_fn_t fn, void *ctx,
                      uint32_t period_ms, uint32_t now_ms, bool enabled) {
    if ((s == NULL) || (fn == NULL) || (period_ms == 0u) || (s->task_count >= SCHED_MAX_TASKS)) {
        return -1;
    }
    uint8_t idx = s->task_count++;
    sched_task_t *t = &s->tasks[idx];
    t->name = name;
    t->fn = fn;
    t->ctx = ctx;
    t->period_ms = period_ms;
    t->last_run_ms = now_ms;
    t->max_exec_us = 0u;
    t->run_cnt = 0u;
    t->enabled = enabled;
    return (int8_t)idx;
}

void sched_set_enable(sched_t *s, int8_t task_id, bool enabled, uint32_t now_ms) {
    if ((s == NULL) || (task_id < 0) || ((uint8_t)task_id >= s->task_count)) {
        return;
    }
    sched_task_t *t = &s->tasks[(uint8_t)task_id];
    if (enabled && !t->enabled) {
        t->last_run_ms = now_ms;
    }
    t->enabled = enabled;
}

uint8_t sched_run(sched_t *s, uint32_t now_ms) {
    if (s == NULL) {
        return 0u;
    }
    uint8_t executed = 0u;
    for (uint8_t i = 0u; i < s->task_count; i++) {
        sched_task_t *t = &s->tasks[i];
        if (!t->enabled || (t->fn == NULL)) {
            continue;
        }
        uint32_t elapsed_ms = (uint32_t)(now_ms - t->last_run_ms);
        if (elapsed_ms >= t->period_ms) {
            t->last_run_ms = now_ms;
            float dt_s = (float)elapsed_ms * 0.001f;

            uint32_t t0_us = (s->get_tick_us != NULL) ? s->get_tick_us() : 0u;
            t->fn(t->ctx, dt_s);
            if (s->get_tick_us != NULL) {
                uint32_t dur_us = (uint32_t)(s->get_tick_us() - t0_us);
                if (dur_us > t->max_exec_us) {
                    t->max_exec_us = dur_us;
                }
            }
            t->run_cnt++;
            executed++;
        }
    }
    return executed;
}

/* ===================== 2. Contest Table-Driven Sub-Task FSM ===================== */

void contest_fsm_init(contest_fsm_t *fsm, void *user_ctx,
                      void (*on_emergency_disarm)(void *ctx, uint32_t fault_mask),
                      uint32_t now_ms) {
    if (fsm == NULL) {
        return;
    }
    memset(fsm, 0, sizeof(*fsm));
    fsm->user_ctx = user_ctx;
    fsm->on_emergency_disarm = on_emergency_disarm;
    fsm->mode = CONTEST_MODE_IDLE;
    fsm->prev_mode = CONTEST_MODE_IDLE;
    fsm->mode_enter_ms = now_ms;
    fsm->sub_step = 0u;
    fsm->sub_step_enter_ms = now_ms;
    fsm->fault_mask = SAFETY_FAULT_NONE;
}

bool contest_fsm_register_mode(contest_fsm_t *fsm, contest_mode_t mode,
                               const contest_state_hooks_t *hooks) {
    if ((fsm == NULL) || (hooks == NULL) || ((uint32_t)mode >= (uint32_t)CONTEST_MODE_COUNT)) {
        return false;
    }
    fsm->table[mode] = *hooks;
    return true;
}

bool contest_fsm_request_mode(contest_fsm_t *fsm, contest_mode_t next_mode, uint32_t now_ms) {
    if ((fsm == NULL) || ((uint32_t)next_mode >= (uint32_t)CONTEST_MODE_COUNT)) {
        return false;
    }
    if (fsm->mode == CONTEST_MODE_ESTOP) {
        /* Latched E-Stop cannot be bypassed via normal mode change! */
        return false;
    }
    if (next_mode == CONTEST_MODE_ESTOP) {
        contest_fsm_trigger_estop(fsm, SAFETY_FAULT_MANUAL_ESTOP, now_ms);
        return true;
    }
    if (next_mode == fsm->mode) {
        return true;
    }

    if (fsm->table[fsm->mode].on_exit != NULL) {
        fsm->table[fsm->mode].on_exit(fsm, fsm->user_ctx);
    }

    fsm->prev_mode = fsm->mode;
    fsm->mode = next_mode;
    fsm->mode_enter_ms = now_ms;
    fsm->sub_step = 0u;
    fsm->sub_step_enter_ms = now_ms;

    if (fsm->table[fsm->mode].on_enter != NULL) {
        fsm->table[fsm->mode].on_enter(fsm, fsm->user_ctx);
    }
    return true;
}

void contest_fsm_set_substep(contest_fsm_t *fsm, uint16_t next_sub_step, uint32_t now_ms) {
    if (fsm == NULL) {
        return;
    }
    fsm->sub_step = next_sub_step;
    fsm->sub_step_enter_ms = now_ms;
}

void contest_fsm_trigger_estop(contest_fsm_t *fsm, uint32_t fault_mask, uint32_t now_ms) {
    if (fsm == NULL) {
        return;
    }
    fsm->fault_mask |= (fault_mask == 0u) ? (uint32_t)SAFETY_FAULT_MANUAL_ESTOP : fault_mask;

    /* Hardware disarm executes FIRST before any state hook! */
    if (fsm->on_emergency_disarm != NULL) {
        fsm->on_emergency_disarm(fsm->user_ctx, fsm->fault_mask);
    }

    if (fsm->mode != CONTEST_MODE_ESTOP) {
        if (fsm->table[fsm->mode].on_exit != NULL) {
            fsm->table[fsm->mode].on_exit(fsm, fsm->user_ctx);
        }
        fsm->prev_mode = fsm->mode;
        fsm->mode = CONTEST_MODE_ESTOP;
        fsm->mode_enter_ms = now_ms;
        fsm->sub_step = 0u;
        fsm->sub_step_enter_ms = now_ms;
        if (fsm->table[CONTEST_MODE_ESTOP].on_enter != NULL) {
            fsm->table[CONTEST_MODE_ESTOP].on_enter(fsm, fsm->user_ctx);
        }
    }
}

void contest_fsm_clear_estop(contest_fsm_t *fsm, uint32_t now_ms) {
    if ((fsm == NULL) || (fsm->mode != CONTEST_MODE_ESTOP)) {
        return;
    }
    if (fsm->table[CONTEST_MODE_ESTOP].on_exit != NULL) {
        fsm->table[CONTEST_MODE_ESTOP].on_exit(fsm, fsm->user_ctx);
    }
    fsm->fault_mask = SAFETY_FAULT_NONE;
    fsm->prev_mode = CONTEST_MODE_ESTOP;
    fsm->mode = CONTEST_MODE_IDLE;
    fsm->mode_enter_ms = now_ms;
    fsm->sub_step = 0u;
    fsm->sub_step_enter_ms = now_ms;
    if (fsm->table[CONTEST_MODE_IDLE].on_enter != NULL) {
        fsm->table[CONTEST_MODE_IDLE].on_enter(fsm, fsm->user_ctx);
    }
}

void contest_fsm_step(contest_fsm_t *fsm, float dt_s, uint32_t now_ms) {
    if ((fsm == NULL) || ((uint32_t)fsm->mode >= (uint32_t)CONTEST_MODE_COUNT)) {
        return;
    }
    if (fsm->table[fsm->mode].on_step != NULL) {
        uint32_t elapsed_ms = (uint32_t)(now_ms - fsm->mode_enter_ms);
        fsm->table[fsm->mode].on_step(fsm, fsm->user_ctx, dt_s, elapsed_ms);
    }
}

/* ===================== 3. Multi-Condition Safety Watchdog ===================== */

void safety_guard_init(safety_guard_t *sg, float max_tilt_deg, uint32_t comm_timeout_ms,
                       float min_vbat_v, uint32_t vbat_debounce_ms) {
    if (sg == NULL) {
        return;
    }
    sg->max_tilt_deg = max_tilt_deg;
    sg->comm_timeout_ms = comm_timeout_ms;
    sg->min_vbat_v = min_vbat_v;
    sg->vbat_debounce_ms = vbat_debounce_ms;
    sg->low_vbat_start_ms = 0u;
    sg->low_vbat_active = false;
}

uint32_t safety_guard_check(safety_guard_t *sg, const safety_inputs_t *in, contest_fsm_t *fsm,
                            uint32_t now_ms) {
    if ((sg == NULL) || (in == NULL)) {
        return SAFETY_FAULT_NONE;
    }

    uint32_t faults = SAFETY_FAULT_NONE;

    if (in->manual_estop) {
        faults |= (uint32_t)SAFETY_FAULT_MANUAL_ESTOP;
    }

    if (sg->max_tilt_deg > 0.0f) {
        if (!isfinite(in->roll_deg) || !isfinite(in->pitch_deg) ||
            (fabsf(in->roll_deg) > sg->max_tilt_deg) ||
            (fabsf(in->pitch_deg) > sg->max_tilt_deg)) {
            faults |= (uint32_t)SAFETY_FAULT_TILT_LIMIT;
        }
    }

    if ((sg->comm_timeout_ms > 0u) && in->comm_required) {
        if ((uint32_t)(now_ms - in->last_comm_ms) > sg->comm_timeout_ms) {
            faults |= (uint32_t)SAFETY_FAULT_COMM_TIMEOUT;
        }
    }

    if (sg->min_vbat_v > 0.0f) {
        if (isfinite(in->vbat_v) && (in->vbat_v < sg->min_vbat_v)) {
            if (!sg->low_vbat_active) {
                sg->low_vbat_active = true;
                sg->low_vbat_start_ms = now_ms;
            }
            if ((uint32_t)(now_ms - sg->low_vbat_start_ms) >= sg->vbat_debounce_ms) {
                faults |= (uint32_t)SAFETY_FAULT_LOW_BATTERY;
            }
        } else {
            sg->low_vbat_active = false;
        }
    }

    if ((faults != SAFETY_FAULT_NONE) && (fsm != NULL)) {
        contest_fsm_trigger_estop(fsm, faults, now_ms);
    }

    return faults;
}
