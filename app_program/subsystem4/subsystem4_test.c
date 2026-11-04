/**
 * @file    subsystem4_test.c
 * @brief   On-target tests for subsystem 4.  Built only with: make TEST=1
 *
 * Runs in the initial task (priority 1) after every subsystem has started.
 * Record unit results in testing/subsystem4/TEST_LOG.md.  Needs the MPU-9250
 * wired to GP2/GP3 and the car standing still on a level bench.
 *
 * NOTE: S4-U06 injects a simulated impact.  The impact is reported as an
 * IMPACT telemetry event only; it does not stop the car.
 */

#include "subsystem4.h"
#include "subsystem4_test.h"
#include "../test_report.h"

#if APP_TEST

#include "../common/bitpack.h"
#include "imu_monitor.h"
#include "imu_sim.h"
#include "drv_us_timer.h"
#include "../navigation/navigation.h"

#define READY_TIMEOUT_MS        (3000)
#define POLL_MS                 (10)
#define TIMING_WINDOW_MS        (1000)
#define CYCLES_PER_WINDOW       (100u)
#define CYCLE_TOLERANCE         (3u)
#define EXEC_BUDGET_US          (2000u)     /* 20 % of the 10 ms period */
#define LEVEL_PITCH_Q           (8)         /* 2 deg in 0.25 deg steps  */

#define STRESS_ITERATIONS       (20000u)
#define STRESS_TIMEOUT_MS       (20000)
#define STRESS_TASKS            (2u)
#define STRESS_PRIORITY         (19)
#define STRESS_HANDLER_MS       (10u)
#define TEST_BIT_TASK_A         (0x10000000u)
#define TEST_BIT_TASK_B         (0x20000000u)
#define TEST_BIT_HANDLER        (0x40000000u)

#define IMPACT_WAIT_MS          (100)
#define IMPACT_LATENCY_US       (50000u)    /* a few periods + poll jitter */
#define RECOVERY_WAIT_MS        (1500)

#define SIM_STEPS               (700u)
#define SIM_CALIB_STEPS         (130u)
#define SIM_HUMP_MM             (50.0f)
#define SIM_TOLERANCE           (0.10f)
#define SIM_DT_S                (0.01f)
#define SIM_STEP_BUDGET_US      (500u)
#define SIM_SLOW_LOG_MAX        (8u)        /* slow steps kept for print */

/* One step over budget, re-timed from the same starting state. */
typedef struct
{
    uint32_t step;
    uint32_t used_us;
    uint32_t replay_us;         /* interrupts on, cache now warm  */
    uint32_t replay_di_us;      /* interrupts off, cache now warm */
} sim_slow_t;

/* Fault-flag stress test state. */
static ID volatile       g_h_stress_sem = 0;
static uint32_t volatile g_stress_errors[STRESS_TASKS];
static uint32_t volatile g_handler_errors = 0u;
static uint32_t volatile g_handler_calls  = 0u;

/* On-target algorithm run; static to keep it off the initial task stack. */
static imu_monitor_t g_test_monitor;
static imu_monitor_t g_test_saved;      /* state before the current step  */
static imu_monitor_t g_test_replay;     /* scratch copy for a re-timing   */
static sim_slow_t    g_slow_log[SIM_SLOW_LOG_MAX];

static void     test_bring_up (void);
static void     test_timing (void);
static void     test_packing (void);
static void     test_fault_flag_stress (void);
static void     test_impact (void);
static void     test_algorithm_on_target (void);
static uint32_t replay_step (drv_imu_sample_t const * p_raw,
                             imu_odom_counts_t const * p_counts,
                             bool b_irq_off);
static uint32_t stress_toggle (uint32_t bit);
static void     stress_task_a (INT start_code, void * p_exinf);
static void     stress_task_b (INT start_code, void * p_exinf);
static void     stress_handler (void * p_exinf);

/**
 * @brief   Run every automatic subsystem 4 test.
 */
void
subsystem4_test (void)
{
    test_bring_up();
    test_timing();
    test_packing();
    test_fault_flag_stress();
    test_impact();
    test_algorithm_on_target();
}

/**
 * @brief   S4-U01 / S4-U03: sensor found, calibrated, sensible output.
 */
static void
test_bring_up (void)
{
    TEST_INIT();
    TMO waited = 0;

    while (!faults_is_set(FAULTS_READY_IMU) && (waited < READY_TIMEOUT_MS))
    {
        (void)tk_dly_tsk(POLL_MS);
        waited += POLL_MS;
    }

    TEST_CHECK("S4-U01", faults_is_set(FAULTS_READY_IMU) &&
                         !faults_is_set(FAULTS_IMU),
               "IMU found on I2C1 and calibrated (READY_IMU set)");

    imu_pack_snapshot_t snap;

    imu_pack_unpack_snapshot(imu_task_get_snapshot(), &snap);
    TEST_CHECK("S4-U03",
               (0u == snap.motion_class) && (0u == snap.terrain_state) &&
               (snap.pitch_q < LEVEL_PITCH_Q) &&
               (snap.pitch_q > -LEVEL_PITCH_Q),
               "still car: STATIONARY, LEVEL, |pitch| < 2 deg");
    TEST_SUMMARY("subsystem4 bring-up");
}

/**
 * @brief   S4-U02: 10 ms period held, no overrun, execution budget.
 */
static void
test_timing (void)
{
    TEST_INIT();
    imu_task_stats_t before;
    imu_task_stats_t after;

    imu_task_test_reset_window();
    imu_task_get_stats(&before);
    (void)tk_dly_tsk(TIMING_WINDOW_MS);
    imu_task_get_stats(&after);

    uint32_t const cycles = after.cycles - before.cycles;

    /* Cast: tm_printf() takes const UB *; the literal is ASCII. */
    tm_printf((UB const *)"[TEST]   cycles/s %d, overruns %d, max exec %d us\n",
              (INT)cycles, (INT)(after.overruns - before.overruns),
              (INT)after.max_exec_us);
    /* Cast: tm_printf() takes const UB *; the literal is ASCII. */
    tm_printf((UB const *)"[TEST]   worst at cycle %d (window %d..%d), "
              "read %d us, %d lines printed\n",
              (INT)after.max_exec_cycle, (INT)before.cycles,
              (INT)after.cycles, (INT)after.max_exec_read_us,
              (INT)after.max_exec_lines);
    /* Cast: tm_printf() takes const UB *; the literal is ASCII. */
    tm_printf((UB const *)"[TEST]   worst in window %d us at cycle %d, "
              "read %d us\n",
              (INT)after.win_max_exec_us, (INT)after.win_max_cycle,
              (INT)after.win_max_read_us);
    TEST_CHECK("S4-U02",
               (cycles + CYCLE_TOLERANCE >= CYCLES_PER_WINDOW) &&
               (cycles <= CYCLES_PER_WINDOW + CYCLE_TOLERANCE) &&
               (after.overruns == before.overruns) &&
               (after.max_exec_us < EXEC_BUDGET_US),
               "100 samples/s, no overrun, worst cycle < 2 ms");
    TEST_SUMMARY("subsystem4 timing");
}

/**
 * @brief   S4-U04 / S4-U09: golden vectors identical to the host test.
 */
static void
test_packing (void)
{
    TEST_INIT();
    imu_pack_snapshot_t snap;

    /* Golden vector 2 of host test H-U08. */
    snap.pitch_q          = -3;
    snap.peak_mm          = 64u;
    snap.b_peak_saturated = false;
    snap.motion_class     = 3u;
    snap.terrain_state    = 1u;
    snap.b_imu_fault      = false;
    snap.b_tilted         = true;
    snap.b_no_encoder     = false;
    snap.yaw_rate_q       = 5;

    TEST_CHECK("S4-U04", 0x054740FDu == imu_pack_snapshot(&snap),
               "snapshot golden vector matches the host build");
    TEST_CHECK("S4-U09", (4u == imu_odom_delta(0x00000002u, 0xFFFFFFFEu)) &&
                         (127u == bitpack_clamp_u32(500u, 127u)),
               "32-bit encoder wrap and clamp helpers on target");
    TEST_SUMMARY("subsystem4 packing");
}

/**
 * @brief   S4-U05 (test T1, gap G1): two tasks on any core and one
 *          handler toggle their own bits in the shared flag concurrently;
 *          no bit may be lost or appear, and real fault bits are untouched.
 */
static void
test_fault_flag_stress (void)
{
    TEST_INIT();
    T_CSEM csem = {0};
    T_CCYC ccyc = {0};
    uint32_t const faults_before = faults_get() & FAULTS_ALL_MASK;

    csem.sematr  = TA_TFIFO;
    csem.isemcnt = 0;
    csem.maxsem  = (INT)STRESS_TASKS;
    g_h_stress_sem = tk_cre_sem(&csem);

    ccyc.cycatr = TA_HLNG | TA_STA;
    /* Cast: the kernel stores handlers as the generic FP type. */
    ccyc.cychdr = (FP)stress_handler;
    ccyc.cyctim = STRESS_HANDLER_MS;
    ccyc.cycphs = STRESS_HANDLER_MS;

    ID const h_cyc = tk_cre_cyc(&ccyc);

    /* Casts: the kernel takes every task entry as the generic FP type. */
    (void)app_start_task((FP)stress_task_a, STRESS_PRIORITY, 0u, "stress_a");
    (void)app_start_task((FP)stress_task_b, STRESS_PRIORITY, 0u, "stress_b");

    ER const er = tk_wai_sem(g_h_stress_sem, (INT)STRESS_TASKS,
                             STRESS_TIMEOUT_MS);

    (void)tk_del_cyc(h_cyc);
    (void)tk_del_sem(g_h_stress_sem);
    (void)faults_clear_status(FAULTS_TEST_MASK);

    /* Cast: tm_printf() takes const UB *; the literal is ASCII. */
    tm_printf((UB const *)"[TEST]   stress errors A %d, B %d, handler %d/%d\n",
              (INT)g_stress_errors[0], (INT)g_stress_errors[1],
              (INT)g_handler_errors, (INT)g_handler_calls);
    TEST_CHECK("S4-U05",
               (E_OK == er) && (0u == g_stress_errors[0]) &&
               (0u == g_stress_errors[1]) && (0u == g_handler_errors) &&
               (g_handler_calls > 0u) &&
               (faults_before == (faults_get() & FAULTS_ALL_MASK)),
               "fault flag: 2 tasks + 1 handler, no lost or phantom bits");
    TEST_SUMMARY("subsystem4 fault flag");
}

/**
 * @brief   S4-U06 / S4-U07 (tests T6/T7): an injected impact is reported as
 *          an IMPACT telemetry event within a few periods, and the IMU keeps
 *          running normally afterwards (impact does not latch any stop).
 */
static void
test_impact (void)
{
    TEST_INIT();
    imu_pack_event_t type     = IMU_PACK_EVT_HUMP;
    uint32_t         payload  = 0u;
    uint16_t         event    = 0u;
    bool             b_impact = false;
    TMO              waited   = 0;

    /* Discard events left over from earlier tests. */
    while (E_OK == imu_task_read_event(&event))
    {
        /* Nothing to do: drain the event buffer. */
    }

    uint32_t const start_us = drv_us_timer_now();

    imu_task_test_inject_shock();

    /* Poll the event stream for the IMPACT event. */
    while ((waited < IMPACT_WAIT_MS) && !b_impact)
    {
        while (E_OK == imu_task_read_event(&event))
        {
            imu_pack_unpack_event(event, &type, &payload);

            if (IMU_PACK_EVT_IMPACT == type)
            {
                b_impact = true;
            }
        }

        if (!b_impact)
        {
            (void)tk_dly_tsk(POLL_MS);
            waited += POLL_MS;
        }
    }

    uint32_t const latency_us = drv_us_timer_elapsed(start_us);

    /* Cast: tm_printf() takes const UB *; the literal is ASCII. */
    tm_printf((UB const *)"[TEST]   IMPACT event after %d us\n",
              (INT)latency_us);
    TEST_CHECK("S4-U06",
               b_impact && (latency_us < IMPACT_LATENCY_US),
               "impact emits an IMPACT telemetry event within 50 ms");

    /* Let the car settle; the monitor leaves the Impact state on its own. */
    (void)tk_dly_tsk(RECOVERY_WAIT_MS);

    imu_pack_snapshot_t snap;

    imu_pack_unpack_snapshot(imu_task_get_snapshot(), &snap);
    TEST_CHECK("S4-U07",
               faults_is_set(FAULTS_READY_IMU) && !faults_is_set(FAULTS_IMU) &&
               (0u == (nav_get_faults() & NAV_FLT_IMU)) &&
               (0u == snap.motion_class),
               "impact is telemetry only: IMU keeps running, nothing latched");
    TEST_SUMMARY("subsystem4 impact");
}

/**
 * @brief   S4-U08: the hump algorithm on the RP2040 soft-float, with the
 *          simulator; also measures the processing time per sample.
 *
 * Every step over budget is re-timed twice from the state saved before it:
 * once with interrupts on and once with them off on this core.  Both fast:
 * the first run met a cold cache or flash contention.  Only the second
 * fast: interrupts.  Both slow: the code path itself is heavy.
 */
static void
test_algorithm_on_target (void)
{
    TEST_INIT();
    imu_sim_t            sim;
    imu_sim_profile_t    prof;
    drv_imu_sample_t raw;
    imu_odom_counts_t    counts;
    imu_monitor_output_t out;
    float32_t            peak       = -1.0f;
    uint32_t             busy_us    = 0u;
    uint32_t             worst_us   = 0u;
    uint32_t             worst_step = 0u;
    uint32_t             slow       = 0u;
    uint32_t             slow_on    = 0u;   /* replay, irq on, still slow  */
    uint32_t             slow_di    = 0u;   /* replay, irq off, still slow */
    uint32_t             worst_di   = 0u;

    imu_monitor_init(&g_test_monitor);
    g_test_monitor.terrain.b_rear_climbs = true;    /* as imu_sim drives */
    imu_sim_init(&sim, 0x0FF0u, 1u);
    imu_sim_flat_profile(&prof);

    for (uint32_t step = 0u; step < SIM_STEPS; step++)
    {
        if (SIM_CALIB_STEPS == step)
        {
            prof.hump_height_mm = SIM_HUMP_MM;
            prof.hump_length_mm = 300.0f;
            prof.hump_start_mm  = 200.0f;
            prof.speed_mm_s     = 200.0f;
        }

        (void)imu_sim_step(&sim, &prof, SIM_DT_S, &raw);
        counts.left_count  = imu_sim_encoder(&sim);
        counts.right_count = counts.left_count;
        counts.b_has_left  = true;
        counts.b_has_right = true;
        g_test_saved       = g_test_monitor;

        uint32_t const start_us = drv_us_timer_now();

        imu_monitor_step(&g_test_monitor, &raw, &counts, SIM_DT_S, &out);

        uint32_t const used_us = drv_us_timer_elapsed(start_us);

        busy_us += used_us;
        if (used_us > worst_us)
        {
            worst_us   = used_us;
            worst_step = step;
        }

        if (used_us >= SIM_STEP_BUDGET_US)
        {
            uint32_t const replay_us    = replay_step(&raw, &counts, false);
            uint32_t const replay_di_us = replay_step(&raw, &counts, true);

            slow_on += (replay_us >= SIM_STEP_BUDGET_US) ? 1u : 0u;
            slow_di += (replay_di_us >= SIM_STEP_BUDGET_US) ? 1u : 0u;
            if (replay_di_us > worst_di)
            {
                worst_di = replay_di_us;
            }

            if (slow < SIM_SLOW_LOG_MAX)
            {
                g_slow_log[slow].step         = step;
                g_slow_log[slow].used_us      = used_us;
                g_slow_log[slow].replay_us    = replay_us;
                g_slow_log[slow].replay_di_us = replay_di_us;
            }

            slow++;
        }

        if (out.b_hump_closed)
        {
            peak = out.hump.peak_mm;
        }
    }

    /* Cast: tm_printf() takes const UB *; the literal is ASCII. */
    tm_printf((UB const *)"[TEST]   sim hump peak x10 = %d, mean %d us, "
              "worst %d us per sample (step %d)\n",
              (INT)imu_math_round(peak * 10.0f), (INT)(busy_us / SIM_STEPS),
              (INT)worst_us, (INT)worst_step);

    /* Printed after the loop so the console does not disturb the timing. */
    for (uint32_t idx = 0u; (idx < slow) && (idx < SIM_SLOW_LOG_MAX); idx++)
    {
        /* Cast: tm_printf() takes const UB *; the literal is ASCII. */
        tm_printf((UB const *)"[TEST]   slow step %d: %d us, replay %d us, "
                  "replay irq off %d us\n",
                  (INT)g_slow_log[idx].step, (INT)g_slow_log[idx].used_us,
                  (INT)g_slow_log[idx].replay_us,
                  (INT)g_slow_log[idx].replay_di_us);
    }

    /* Cast: tm_printf() takes const UB *; the literal is ASCII. */
    tm_printf((UB const *)"[TEST]   steps >= %d us: %d of %d; still slow on "
              "replay: irq on %d, irq off %d; worst irq off %d us\n",
              (INT)SIM_STEP_BUDGET_US, (INT)slow, (INT)SIM_STEPS,
              (INT)slow_on, (INT)slow_di, (INT)worst_di);
    TEST_CHECK("S4-U08",
               (imu_math_abs(peak - SIM_HUMP_MM) <=
                (SIM_TOLERANCE * SIM_HUMP_MM)) &&
               (worst_us < SIM_STEP_BUDGET_US),
               "50 mm simulated hump within 10 % on target; step < 0.5 ms");
    TEST_SUMMARY("subsystem4 algorithm");
}

/**
 * @brief   Re-time one monitor step from the state saved before it, on a
 *          scratch copy so the real run is not changed.
 * @param   p_raw      The sample the step used.
 * @param   p_counts   The encoder counts the step used.
 * @param   b_irq_off  true to mask interrupts on this core while timing.
 * @return  Microseconds the step took.
 */
static uint32_t
replay_step (drv_imu_sample_t const * p_raw,
             imu_odom_counts_t const * p_counts, bool b_irq_off)
{
    imu_monitor_output_t out;
    UW                   intsts = 0u;

    g_test_replay = g_test_saved;

    if (b_irq_off)
    {
        DI(intsts);
    }

    uint32_t const start_us = drv_us_timer_now();

    imu_monitor_step(&g_test_replay, p_raw, p_counts, SIM_DT_S, &out);

    uint32_t const used_us = drv_us_timer_elapsed(start_us);

    if (b_irq_off)
    {
        EI(intsts);
    }

    return (used_us);
}

/**
 * @brief   Set, check, clear and check one test bit many times.
 * @param   bit  The caller's own test bit.
 * @return  Number of read-backs that were wrong.
 */
static uint32_t
stress_toggle (uint32_t bit)
{
    uint32_t errors = 0u;

    for (uint32_t loop = 0u; loop < STRESS_ITERATIONS; loop++)
    {
        (void)faults_signal(bit);

        if (!faults_is_set(bit))
        {
            errors++;
        }

        (void)faults_clear_status(bit);

        if (faults_is_set(bit))
        {
            errors++;
        }
    }

    return (errors);
}

/**
 * @brief   Stress helper task A (exits and deletes itself when done).
 * @param   start_code  Unused.
 * @param   p_exinf     Unused.
 */
static void
stress_task_a (INT start_code, void * p_exinf)
{
    (void)start_code;
    (void)p_exinf;
    g_stress_errors[0] = stress_toggle(TEST_BIT_TASK_A);
    (void)tk_sig_sem(g_h_stress_sem, 1);
    tk_exd_tsk();
}

/**
 * @brief   Stress helper task B (exits and deletes itself when done).
 * @param   start_code  Unused.
 * @param   p_exinf     Unused.
 */
static void
stress_task_b (INT start_code, void * p_exinf)
{
    (void)start_code;
    (void)p_exinf;
    g_stress_errors[1] = stress_toggle(TEST_BIT_TASK_B);
    (void)tk_sig_sem(g_h_stress_sem, 1);
    tk_exd_tsk();
}

/**
 * @brief   Cyclic handler (interrupt context): toggles its own bit with
 *          the same helpers, proving they are handler-safe (gap G1).
 * @param   p_exinf  Unused.
 */
static void
stress_handler (void * p_exinf)
{
    (void)p_exinf;
    g_handler_calls++;

    if (0u == (g_handler_calls % 2u))
    {
        (void)faults_signal(TEST_BIT_HANDLER);

        if (!faults_is_set(TEST_BIT_HANDLER))
        {
            g_handler_errors++;
        }
    }
    else
    {
        (void)faults_clear_status(TEST_BIT_HANDLER);

        if (faults_is_set(TEST_BIT_HANDLER))
        {
            g_handler_errors++;
        }
    }
}

#endif /* APP_TEST */

/*** end of file ***/
