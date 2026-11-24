/**
 * @file contest_scheduler_fsm.h
 * @brief Deterministic cooperative time-slice scheduler, NUEDC table-driven task isolation FSM,
 *        and multi-condition actuator safety watchdog.
 *        非阻塞时间片轮询调度器 + 电赛分问隔离表驱动状态机 + 倾角/断连/欠压失控保护。
 */

#ifndef CONTEST_SCHEDULER_FSM_H
#define CONTEST_SCHEDULER_FSM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 1. Cooperative Time-Slice Scheduler ===================== */

#ifndef SCHED_MAX_TASKS
#define SCHED_MAX_TASKS (16u)
#endif

typedef void (*sched_task_fn_t)(void *ctx, float dt_s);
typedef uint32_t (*sched_us_fn_t)(void);

typedef struct {
    const char *name;
    sched_task_fn_t fn;
    void *ctx;
    uint32_t period_ms;
    uint32_t last_run_ms;
    uint32_t max_exec_us;   /**< Profiling watermark (if microsecond clock supplied) */
    uint32_t run_cnt;
    bool enabled;
} sched_task_t;

typedef struct {
    sched_task_t tasks[SCHED_MAX_TASKS];
    uint8_t task_count;
    sched_us_fn_t get_tick_us;
} sched_t;

void sched_init(sched_t *s, sched_us_fn_t get_tick_us_opt);

/**
 * @brief Register periodic task.
 * @return Task index [0..SCHED_MAX_TASKS-1], or -1 if full/invalid.
 */
int8_t sched_add_task(sched_t *s, const char *name, sched_task_fn_t fn, void *ctx,
                      uint32_t period_ms, uint32_t now_ms, bool enabled);

void sched_set_enable(sched_t *s, int8_t task_id, bool enabled, uint32_t now_ms);

/**
 * @brief Poll and dispatch all due tasks (handles uint32_t 49.7-day tick rollover cleanly).
 * @return Number of tasks executed in this sweep.
 */
uint8_t sched_run(sched_t *s, uint32_t now_ms);

/* ===================== 2. Contest Table-Driven Sub-Task FSM ===================== */

typedef enum {
    CONTEST_MODE_IDLE = 0,     /**< Disarmed safe idle / OLED menu mode selection */
    CONTEST_MODE_CALIB,        /**< Pre-run IMU / servo / sensor calibration */
    CONTEST_MODE_BASIC_1,      /**< NUEDC Basic Task 1 (isolated!) */
    CONTEST_MODE_BASIC_2,      /**< NUEDC Basic Task 2 */
    CONTEST_MODE_BASIC_3,      /**< NUEDC Basic Task 3 */
    CONTEST_MODE_BASIC_4,      /**< NUEDC Basic Task 4 */
    CONTEST_MODE_ADV_1,        /**< NUEDC Extended Task 1 */
    CONTEST_MODE_ADV_2,        /**< NUEDC Extended Task 2 */
    CONTEST_MODE_ESTOP,        /**< Latched Emergency Stop (motors cut!) */
    CONTEST_MODE_COUNT
} contest_mode_t;

typedef struct contest_fsm_s contest_fsm_t;

typedef struct {
    void (*on_enter)(contest_fsm_t *fsm, void *ctx);
    void (*on_step)(contest_fsm_t *fsm, void *ctx, float dt_s, uint32_t elapsed_ms);
    void (*on_exit)(contest_fsm_t *fsm, void *ctx);
} contest_state_hooks_t;

struct contest_fsm_s {
    contest_state_hooks_t table[CONTEST_MODE_COUNT];
    void *user_ctx;
    void (*on_emergency_disarm)(void *ctx, uint32_t fault_mask);

    contest_mode_t mode;
    contest_mode_t prev_mode;
    uint32_t mode_enter_ms;
    uint16_t sub_step;         /**< Sequential waypoint / phase index inside active task */
    uint32_t sub_step_enter_ms;
    uint32_t fault_mask;
};

void contest_fsm_init(contest_fsm_t *fsm, void *user_ctx,
                      void (*on_emergency_disarm)(void *ctx, uint32_t fault_mask),
                      uint32_t now_ms);

bool contest_fsm_register_mode(contest_fsm_t *fsm, contest_mode_t mode,
                               const contest_state_hooks_t *hooks);

/**
 * @brief Request clean mode transition (executes old on_exit -> resets sub_step -> new on_enter).
 *        Blocked if currently in CONTEST_MODE_ESTOP until contest_fsm_clear_estop() is called.
 */
bool contest_fsm_request_mode(contest_fsm_t *fsm, contest_mode_t next_mode, uint32_t now_ms);

/**
 * @brief Advance internal waypoint/sub-step index inside the current mode cleanly.
 */
void contest_fsm_set_substep(contest_fsm_t *fsm, uint16_t next_sub_step, uint32_t now_ms);

/**
 * @brief Latch Emergency Stop immediately and invoke hardware disarm callback.
 */
void contest_fsm_trigger_estop(contest_fsm_t *fsm, uint32_t fault_mask, uint32_t now_ms);

/**
 * @brief Clear latched E-Stop and return strictly to CONTEST_MODE_IDLE (safe disarmed).
 */
void contest_fsm_clear_estop(contest_fsm_t *fsm, uint32_t now_ms);

/**
 * @brief Step currently active mode hook.
 */
void contest_fsm_step(contest_fsm_t *fsm, float dt_s, uint32_t now_ms);

/* ===================== 3. Multi-Condition Safety Watchdog ===================== */

typedef enum {
    SAFETY_FAULT_NONE = 0u,
    SAFETY_FAULT_TILT_LIMIT = (1u << 0),    /**< Roll or Pitch exceeded safe threshold */
    SAFETY_FAULT_COMM_TIMEOUT = (1u << 1),  /**< Vision / RC / IMU heartbeat timed out */
    SAFETY_FAULT_LOW_BATTERY = (1u << 2),   /**< Battery voltage below limit for debounce window */
    SAFETY_FAULT_MANUAL_ESTOP = (1u << 3)   /**< External E-stop key / serial STOP command */
} safety_fault_t;

typedef struct {
    float max_tilt_deg;         /**< <= 0 disables tilt protection (e.g. 40.0f for UAV) */
    uint32_t comm_timeout_ms;   /**< 0 disables comm watchdog (e.g. 250u ms for vision/RC) */
    float min_vbat_v;           /**< <= 0 disables battery guard (e.g. 10.5f for 3S LiPo) */
    uint32_t vbat_debounce_ms;  /**< Debounce time to ignore motor startup voltage sag */

    /* Internal debounce state */
    uint32_t low_vbat_start_ms;
    bool low_vbat_active;
} safety_guard_t;

typedef struct {
    float roll_deg;
    float pitch_deg;
    float vbat_v;
    uint32_t last_comm_ms;
    bool comm_required;         /**< Set true only in modes that depend on external link */
    bool manual_estop;
} safety_inputs_t;

void safety_guard_init(safety_guard_t *sg, float max_tilt_deg, uint32_t comm_timeout_ms,
                       float min_vbat_v, uint32_t vbat_debounce_ms);

/**
 * @brief Evaluate safety conditions; triggers fsm E-Stop automatically on any fault.
 * @return Bitmask of active safety_fault_t flags.
 */
uint32_t safety_guard_check(safety_guard_t *sg, const safety_inputs_t *in, contest_fsm_t *fsm,
                            uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* CONTEST_SCHEDULER_FSM_H */
