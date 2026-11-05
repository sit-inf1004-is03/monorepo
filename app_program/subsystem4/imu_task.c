/**
 * @file    imu_task.c
 * @brief   IMU / Terrain task: sampling, sensor-fault handling,
 *          publishing and event reporting.
 *
 * Task-level statechart (adds the sensor-fault handling of Week 6 6.3.3 and
 * the "Course completed" exit of Figure 5 around imu_monitor.c):
 *
 *   PROBE --sensor OK--> RUNNING --course done--> COMPLETE
 *     ^                     |  (Calibrating / Monitoring / Impact)
 *     |                read failure or frozen data
 *     +--- every 1 s --- OFFLINE  (IMU fault bit set, impact detection off)
 *
 * "Init failure" (6.3.3 rule 1): the IMU is flagged offline (fault bit 3)
 * and never enters Monitoring until a later probe succeeds.  "Read failure"
 * (rule 2): IMU_TASK_MAX_READ_ERRORS failed reads in a row, or
 * IMU_TASK_FROZEN_SAMPLES identical samples, set fault bit 3, free the bus
 * and go OFFLINE (gap G5).  The fault reaches Navigation as NAV_FLT_IMU
 * (IF-12).  It only degrades: the car keeps driving without impact or hump
 * data, and the bit is cleared when a later probe succeeds.
 */

/* NULL comes from tk/typedef.h; <stddef.h> would clash with the kernel's
 * own size_t in tk/syslib.h.
 */
#include "../app.h"
#include "../common/faults.h"
#include "imu_task.h"
#include "imu_ramp.h"
#include "imu_monitor.h"
#include "imu_pack.h"
#include "drv_i2c1.h"
#include "drv_imu.h"
#include "drv_us_timer.h"
#include "../subsystem2/drv_encoder.h"     /* IF-06: wheel encoder counts (Buddy 2) */
#include "../navigation/navigation.h"       /* IF-12: IMU fault to Navigation */
#include "../subsystem3/line_barcode.h"     /* barcode_beep(): IMU ready */
#if APP_TEST
#include "../common/bitpack.h"
#endif

/* Safety net if the cyclic handler ever stops waking the task. */
#define WAKE_TIMEOUT_MS         (50)

/* Accepted sample interval; outside it the nominal 10 ms is used. */
#define NOMINAL_DT_S            (0.010f)
#define MIN_DT_US               (2000u)
#define MAX_DT_US               (100000u)
#define US_PER_S                (1000000.0f)

/* One message = 4-byte kernel header + 2-byte event rounded up to 4
 * (kernel/tkernel/messagebuf.h, HEADERSZ and ROUNDSZ).
 */
#define EVENT_SLOT_BYTES        (8u)

/* Test-only shock: 3 g on the up axis, same sign as gravity (4096/g). */
#define TEST_SHOCK_COUNTS       (12288)
#define TEST_SHOCK_SAMPLES      (2u)

#define TENTHS_PER_UNIT         (10.0f)
#define TENTHS_DIVISOR          (10)

/* Shared with other tasks and the cyclic handler. */
static ID volatile                       g_h_imu_tid      = 0;
static ID volatile                       g_h_imu_cyc      = 0;
static ID volatile                       g_h_imu_mbf      = 0;
static uint32_t volatile                 g_snapshot_word  = 0u;
static uint32_t volatile                 g_aux_word       = 0u;
static bool volatile                     g_b_course_done  = false;
static bool volatile                     g_b_restart      = false;
static imu_task_stats_t volatile         g_stats;
#if APP_TEST
static uint32_t volatile                 g_test_shock     = 0u;
#endif

/* Private to the IMU task. */
static imu_monitor_t        g_monitor;
static drv_imu_sample_t g_last_sample;
static imu_task_state_t     g_state             = IMU_TASK_STATE_PROBE;
static uint32_t             g_retry_wait        = 0u;
static uint32_t             g_read_fail_count   = 0u;
static uint32_t             g_frozen_count      = 0u;
static uint32_t             g_last_time_us      = 0u;
static bool                 g_b_have_time       = false;
static bool                 g_b_fault_reported  = false;
static bool                 g_b_ready_beeped    = false;
static uint32_t             g_cycle_read_us     = 0u;   /* this cycle only */
static uint32_t             g_cycle_lines       = 0u;   /* this cycle only */
static int32_t              g_enc_head          = 0;    /* left - right    */
static int32_t              g_side_start_head   = 0;    /* at side hump open */
static bool                 g_b_side_open       = false;
#if (1 == IMU_TASK_TRACE_ENABLE)
/* Largest impact inputs since the last trace line, so short jolts that
 * fall between trace lines still show (impact tuning, test S4-M03).
 */
static float32_t            g_trace_peak_dev_g  = 0.0f;
static float32_t            g_trace_peak_fwd_g  = 0.0f;
#endif

static void      imu_period_isr (void * p_exinf);
static void      imu_sampling_task (INT start_code, void * p_exinf);
static void      run_state_machine (void);
static void      try_probe (void);
static void      run_cycle (void);
static void      process_sample (drv_imu_sample_t * p_sample);
static void      handle_outputs (imu_monitor_output_t const * p_out);
static void      sensor_lost (imu_pack_fault_reason_t reason);
static void      report_fault (imu_pack_fault_reason_t reason);
static void      report_recovered (void);
static void      report_hump (imu_terrain_hump_t const * p_hump);
static void      update_side_heading (void);
static void      finish_course (void);
#if (1 == IMU_TASK_HUMP_TRACE)
static void      trace_hump (void);
#endif
static void      update_ramp (void);
static void      publish (void);
static bool      sample_is_frozen (drv_imu_sample_t const * p_sample);
static float32_t measure_dt (void);
static void      read_encoders (imu_odom_counts_t * p_counts);
static void      emit_event (imu_pack_event_t type, uint32_t payload);
static void      log_line (char const * p_format, int32_t arg_a,
                           int32_t arg_b, int32_t arg_c);
static void      log_tenths (char const * p_format, float32_t value,
                             int32_t arg_c);
static void      log_state (char const * p_format,
                            imu_motion_class_t motion,
                            imu_terrain_state_t terrain);
static char const * motion_name (imu_motion_class_t motion);
static char const * terrain_name (imu_terrain_state_t terrain);

/**
 * @brief   Create the IMU's kernel objects (phase 1 of usermain()).
 * @return  E_OK or a negative kernel error code.
 */
ER
imu_task_init (void)
{
    ER     result = E_OK;
    T_CMBF cmbf   = {0};
    T_CCYC ccyc   = {0};

    (void)drv_us_timer_init();
    imu_monitor_init(&g_monitor);

    /* Event buffer read by the Comm task with TMO_POL (never blocks). */
    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = IMU_TASK_EVENT_DEPTH * EVENT_SLOT_BYTES;
    /* Cast: sizeof(uint16_t) is 2, fits in INT. */
    cmbf.maxmsz = (INT)sizeof(uint16_t);

    ID const h_mbf = tk_cre_mbf(&cmbf);

    if (h_mbf < E_OK)
    {
        result = h_mbf;
    }
    else
    {
        g_h_imu_mbf = h_mbf;

        /* 10 ms cyclic handler, created stopped; started with the task. */
        ccyc.cycatr = TA_HLNG;
        /* Cast: the kernel stores handlers as the generic FP type. */
        ccyc.cychdr = (FP)imu_period_isr;
        ccyc.cyctim = IMU_TASK_PERIOD_MS;
        ccyc.cycphs = IMU_TASK_PERIOD_MS;

        ID const h_cyc = tk_cre_cyc(&ccyc);

        if (h_cyc < E_OK)
        {
            result = h_cyc;
        }
        else
        {
            g_h_imu_cyc = h_cyc;
        }
    }

    return (result);
}

/**
 * @brief   Create and start the IMU task, then its cyclic handler
 *          (phase 2 of usermain()).
 * @param   priority  Task priority (S4_PRI_IMU, TEAM.md section 4).
 * @return  E_OK or a negative kernel error code.
 */
ER
imu_task_start (PRI priority)
{
    ER result = E_OK;

    /* Cast: the kernel takes every task entry as the generic FP type. */
    ID const h_tid = app_start_task((FP)imu_sampling_task, priority, 0u,
                                    "imu_task");

    if (h_tid < E_OK)
    {
        result = h_tid;
    }
    else
    {
        g_h_imu_tid = h_tid;
        result      = tk_sta_cyc(g_h_imu_cyc);
    }

    return (result);
}

/**
 * @brief   Latest snapshot word (layout in imu_pack.h).  Lock-free: the
 *          word is written with one 32-bit store.
 * @return  Snapshot word.
 */
uint32_t
imu_task_get_snapshot (void)
{
    return (g_snapshot_word);
}

/**
 * @brief   Latest aux word (layout in imu_pack.h).
 * @return  Aux word.
 */
uint32_t
imu_task_get_aux (void)
{
    return (g_aux_word);
}

/**
 * @brief   Take the oldest IMU event without waiting (for the Comm task).
 * @param   p_event  Receives a 16-bit event (layout in imu_pack.h).
 * @return  E_OK, E_TMOUT if the buffer is empty, E_PAR or E_NOEXS.
 */
ER
imu_task_read_event (uint16_t * p_event)
{
    ER result = E_PAR;

    if (NULL == p_event)
    {
        result = E_PAR;
    }
    else if (g_h_imu_mbf <= 0)
    {
        result = E_NOEXS;
    }
    else
    {
        INT const size = tk_rcv_mbf(g_h_imu_mbf, p_event, TMO_POL);

        /* Cast: sizeof(uint16_t) is 2, fits in INT. */
        result = (size < E_OK) ? size :
                 (((INT)sizeof(uint16_t) == size) ? E_OK : E_OBJ);
    }

    return (result);
}

/**
 * @brief   Tell the IMU the course is completed (Figure 5 final state).
 *          The task reports the run summary and goes idle.
 */
void
imu_task_signal_course_done (void)
{
    g_b_course_done = true;
}

/**
 * @brief   Leave COMPLETE and start again from PROBE for a new run; the
 *          run statistics (highest hump, hump count) restart from zero.
 * @return  E_OK or a kernel error code.
 */
ER
imu_task_restart (void)
{
    ER result = E_NOEXS;

    if ((g_h_imu_tid > 0) && (g_h_imu_cyc > 0))
    {
        g_b_restart = true;
        result      = tk_sta_cyc(g_h_imu_cyc);
        (void)tk_wup_tsk(g_h_imu_tid);
    }

    return (result);
}

/**
 * @brief   Copy the diagnostic counters.
 * @param   p_stats  Receives the counters.
 */
void
imu_task_get_stats (imu_task_stats_t * p_stats)
{
    if (NULL != p_stats)
    {
        p_stats->cycles         = g_stats.cycles;
        p_stats->overruns       = g_stats.overruns;
        p_stats->read_errors    = g_stats.read_errors;
        p_stats->recoveries     = g_stats.recoveries;
        p_stats->events_dropped = g_stats.events_dropped;
        p_stats->max_exec_us    = g_stats.max_exec_us;
        p_stats->last_exec_us   = g_stats.last_exec_us;
        p_stats->max_exec_cycle   = g_stats.max_exec_cycle;
        p_stats->max_exec_read_us = g_stats.max_exec_read_us;
        p_stats->max_exec_lines   = g_stats.max_exec_lines;
        p_stats->win_max_exec_us  = g_stats.win_max_exec_us;
        p_stats->win_max_cycle    = g_stats.win_max_cycle;
        p_stats->win_max_read_us  = g_stats.win_max_read_us;
        p_stats->state          = g_stats.state;
    }
}

#if APP_TEST
/**
 * @brief   Test hook: add a +3 g shock to the next two samples, so the
 *          whole impact -> ESTOP -> recovery path can be timed on target.
 */
void
imu_task_test_inject_shock (void)
{
    g_test_shock = TEST_SHOCK_SAMPLES;
}

/**
 * @brief   Test hook: start a new window for the win_max_* statistics, so
 *          a test can time steady running without the start-up cycles.
 *          A cycle finishing during the reset may leave its own value;
 *          that is one real cycle from the window edge, so it is harmless.
 */
void
imu_task_test_reset_window (void)
{
    g_stats.win_max_exec_us = 0u;
    g_stats.win_max_cycle   = 0u;
    g_stats.win_max_read_us = 0u;
}
#endif

/**
 * @brief   10 ms cyclic handler: wake the IMU task.  Runs in the kernel's
 *          timer-interrupt context (hence the _isr suffix); it only calls
 *          tk_wup_tsk(), which handlers may call.
 * @param   p_exinf  Unused.
 */
static void
imu_period_isr (void * p_exinf)
{
    (void)p_exinf;

    if (g_h_imu_tid > 0)
    {
        (void)tk_wup_tsk(g_h_imu_tid);
    }
}

/**
 * @brief   IMU task body.  Queued wake-ups found after waking mean whole
 *          periods were missed; they are counted as overruns, not replayed.
 * @param   start_code  Unused.
 * @param   p_exinf     Unused.
 */
static void
imu_sampling_task (INT start_code, void * p_exinf)
{
    (void)start_code;
    (void)p_exinf;

    log_line("imu: task started, period %d ms\n",
             (int32_t)IMU_TASK_PERIOD_MS, 0, 0);

    for (;;)
    {
        TMO const timeout = (IMU_TASK_STATE_COMPLETE == g_state) ?
                            TMO_FEVR : WAKE_TIMEOUT_MS;

        (void)tk_slp_tsk(timeout);

        INT const queued = tk_can_wup(TSK_SELF);

        if (queued > 0)
        {
            /* Cast: queued > 0, so it is representable as uint32_t. */
            g_stats.overruns += (uint32_t)queued;
        }

        bool const     b_sampling = (IMU_TASK_STATE_RUNNING == g_state);
        uint32_t const start_us   = drv_us_timer_now();

        g_cycle_read_us = 0u;
        g_cycle_lines   = 0u;
        run_state_machine();

        uint32_t const exec_us = drv_us_timer_elapsed(start_us);

        g_stats.last_exec_us = exec_us;

        /* Only sampling cycles count towards the budget: a probe (sensor
         * start-up delays, tens of ms) runs before sampling starts and
         * discards the wake-ups it delays (try_probe()).
         */
        if (b_sampling && (exec_us > g_stats.max_exec_us))
        {
            g_stats.max_exec_us      = exec_us;
            g_stats.max_exec_cycle   = g_stats.cycles;
            g_stats.max_exec_read_us = g_cycle_read_us;
            g_stats.max_exec_lines   = g_cycle_lines;
        }

        if (b_sampling && (exec_us > g_stats.win_max_exec_us))
        {
            g_stats.win_max_exec_us = exec_us;
            g_stats.win_max_cycle   = g_stats.cycles;
            g_stats.win_max_read_us = g_cycle_read_us;
        }
    }
}

/**
 * @brief   Top-level task statechart, one step per period.
 */
static void
run_state_machine (void)
{
    if (g_b_restart)
    {
        /* A new run: forget the previous run's peak and hump count. */
        imu_monitor_init(&g_monitor);
        g_b_restart     = false;
        g_b_course_done = false;
        g_retry_wait    = 0u;
        g_state         = IMU_TASK_STATE_PROBE;
    }

    if (g_b_course_done && (IMU_TASK_STATE_COMPLETE != g_state))
    {
        finish_course();
    }

    switch (g_state)
    {
        case IMU_TASK_STATE_PROBE:
        case IMU_TASK_STATE_OFFLINE:
            /* Both states retry the sensor; PROBE starts with no wait. */
            if (0u == g_retry_wait)
            {
                try_probe();
            }
            else
            {
                g_retry_wait--;
            }
        break;

        case IMU_TASK_STATE_RUNNING:
            run_cycle();
        break;

        case IMU_TASK_STATE_COMPLETE:
            /* Idle until imu_task_restart(). */
        break;

        default:
            g_state = IMU_TASK_STATE_PROBE;
        break;
    }

    /* Cast: the enum constants are 0..3, non-negative. */
    g_stats.state = (uint32_t)g_state;
}

/**
 * @brief   Initialise I2C1 and the sensor; enter RUNNING on success.
 */
static void
try_probe (void)
{
    drv_imu_status_t status = DRV_IMU_ERR_BUS;

    if (DRV_I2C1_OK == drv_i2c1_init())
    {
        status = drv_imu_init();
    }

    if (DRV_IMU_OK == status)
    {
        /* Casts: the enum constants and 8-bit values are non-negative. */
        log_line((DRV_IMU_KIND_LSM303D == drv_imu_kind()) ?
                 "imu: LSM303D (no gyro) found at 0x%x, WHO_AM_I 0x%x\n" :
                 "imu: MPU found at 0x%x, WHO_AM_I 0x%x\n",
                 (int32_t)drv_imu_address(),
                 (int32_t)drv_imu_who_am_i(), 0);

        if (g_b_fault_reported)
        {
#if (1 == IMU_TASK_SELF_CLEAR_FAULTS)
            (void)faults_clear(FAULTS_IMU);
            (void)nav_clear_fault(NAV_FLT_IMU);
#endif
            g_b_fault_reported = false;
            emit_event(IMU_PACK_EVT_IMU_OK, 0u);
        }

        /* Back after a dropout: the car may be on a slope, so keep the
         * level.  Calibrate only if there is no level yet (boot, new run).
         */
        if (imu_monitor_resume(&g_monitor))
        {
            (void)faults_signal(FAULTS_READY_IMU);
            log_line("===== imu: back, level kept =====\n", 0, 0, 0);
        }
        else
        {
            imu_monitor_recalibrate(&g_monitor);
        }

        g_read_fail_count = 0u;
        g_frozen_count    = 0u;
        g_b_have_time     = false;
        g_state           = IMU_TASK_STATE_RUNNING;
    }
    else
    {
        imu_pack_fault_reason_t reason = IMU_PACK_FAULT_NO_DEVICE;

        if (DRV_IMU_ERR_WHO_AM_I == status)
        {
            reason = IMU_PACK_FAULT_WHO_AM_I;
        }
        else if (DRV_IMU_ERR_CONFIG == status)
        {
            reason = IMU_PACK_FAULT_CONFIG;
        }
        else
        {
            reason = IMU_PACK_FAULT_NO_DEVICE;
        }

        report_fault(reason);
        g_state      = IMU_TASK_STATE_OFFLINE;
        g_retry_wait = IMU_TASK_RETRY_CYCLES;
    }

    /* The sensor start-up delays (about 120 ms) queue wake-ups that are
     * not real overruns; discard them.
     */
    (void)tk_can_wup(TSK_SELF);
}

/**
 * @brief   RUNNING: read one sample and process it, or count the failure.
 */
static void
run_cycle (void)
{
    drv_imu_sample_t       sample;
    uint32_t const         read_start_us = drv_us_timer_now();
    drv_imu_status_t const status        = drv_imu_read(&sample);

    g_cycle_read_us = drv_us_timer_elapsed(read_start_us);

    if (DRV_IMU_OK != status)
    {
        g_stats.read_errors++;
        g_read_fail_count++;

        if (g_read_fail_count >= IMU_TASK_MAX_READ_ERRORS)
        {
            sensor_lost(IMU_PACK_FAULT_READ);
        }
    }
    else if (sample_is_frozen(&sample))
    {
        sensor_lost(IMU_PACK_FAULT_FROZEN);
    }
    else
    {
        g_read_fail_count = 0u;
        process_sample(&sample);
    }
}

/**
 * @brief   Run the statechart step on a good sample and publish.
 * @param   p_sample  Raw sample (may be modified by the test hook).
 */
static void
process_sample (drv_imu_sample_t * p_sample)
{
    imu_odom_counts_t    counts;
    imu_monitor_output_t out;

#if APP_TEST
    if (g_test_shock > 0u)
    {
        /* Add the shock in the direction gravity already reads on this
         * axis, so it always adds 3 g to |a|: an upside-down board reads
         * about -1 g here, and +3 g would only make |a| about 2 g.
         */
        int32_t const up      = (int32_t)p_sample->accel[IMU_MOUNT_UP_AXIS];
        int32_t const shocked = up + ((up < 0) ? -TEST_SHOCK_COUNTS :
                                                 TEST_SHOCK_COUNTS);

        /* Cast: clamped to the int16_t range first. */
        p_sample->accel[IMU_MOUNT_UP_AXIS] =
            (int16_t)bitpack_clamp_s32(shocked, INT16_MIN, INT16_MAX);
        g_test_shock--;
    }
#endif

    read_encoders(&counts);
    /* Cast: the wrapped difference of two cumulative counts, as line_follow
     * computes it.
     */
    g_enc_head = (int32_t)(counts.left_count - counts.right_count);
    imu_monitor_step(&g_monitor, p_sample, &counts, measure_dt(), &out);
    handle_outputs(&out);
    update_side_heading();
    update_ramp();
#if (1 == IMU_TASK_HUMP_TRACE)
    trace_hump();
#endif
    publish();
    g_stats.cycles++;

#if (1 == IMU_TASK_TRACE_ENABLE)
    imu_filter_attitude_t const * const p_att =
        imu_monitor_get_attitude(&g_monitor);
    float32_t const dev_g = imu_math_abs(p_att->accel_mag_g - 1.0f);
    float32_t const fwd_g = imu_math_abs(p_att->lin_fwd_g);

    g_trace_peak_dev_g = (dev_g > g_trace_peak_dev_g) ? dev_g :
                                                        g_trace_peak_dev_g;
    g_trace_peak_fwd_g = (fwd_g > g_trace_peak_fwd_g) ? fwd_g :
                                                        g_trace_peak_fwd_g;

    if (0u == (g_stats.cycles % IMU_TASK_TRACE_CYCLES))
    {
        /* Header line so each trace block stands apart on the console. */
        log_line("\n========== imu trace, cycle %d ==========\n",
                 (int32_t)g_stats.cycles, 0, 0);

        /* Raw sensor axes: 4096 LSB/g -> mg; 65.5 LSB/dps -> whole dps;
         * compass in raw counts (for the hard-iron offsets).
         */
        log_line("imu: accel mg   x %d  y %d  z %d\n",
                 ((int32_t)p_sample->accel[0] * 1000) / 4096,
                 ((int32_t)p_sample->accel[1] * 1000) / 4096,
                 ((int32_t)p_sample->accel[2] * 1000) / 4096);
        if (p_sample->b_has_gyro)
        {
            log_line("imu: gyro dps   x %d  y %d  z %d\n",
                     ((int32_t)p_sample->gyro[0] * 10) / 655,
                     ((int32_t)p_sample->gyro[1] * 10) / 655,
                     ((int32_t)p_sample->gyro[2] * 10) / 655);
        }
        if (p_sample->b_has_mag)
        {
            log_line("imu: mag raw    x %d  y %d  z %d\n",
                     (int32_t)p_sample->mag[0], (int32_t)p_sample->mag[1],
                     (int32_t)p_sample->mag[2]);
        }
        log_line("imu: pitch %d deg, roll %d deg\n",
                 imu_math_round(p_att->pitch_deg),
                 imu_math_round(p_att->roll_deg), 0);
        log_line("imu: yaw rate %d dps, heading %d deg\n",
                 imu_math_round(p_att->yaw_rate_dps),
                 imu_math_round(p_att->heading_deg), 0);
        log_state("imu: state %s, %s\n", out.motion_class,
                  out.terrain_state);
        /* Impact limits: dev > IMU_IMPACT_DEV_G or fwd > IMU_IMPACT_FWD_G
         * (doubled on a hump).
         */
        log_line("imu: peak mg    dev %d  fwd %d\n",
                 imu_math_round(g_trace_peak_dev_g * 1000.0f),
                 imu_math_round(g_trace_peak_fwd_g * 1000.0f), 0);
        g_trace_peak_dev_g = 0.0f;
        g_trace_peak_fwd_g = 0.0f;
    }
#endif
}

/**
 * @brief   Act on what the statechart step reported.  ESTOP is raised
 *          FIRST, before any logging or event work (Tribunal 1, IR-7).
 * @param   p_out  Step output.
 */
static void
handle_outputs (imu_monitor_output_t const * p_out)
{
    if (p_out->b_impact)
    {
        emit_event(IMU_PACK_EVT_IMPACT,
                   imu_pack_impact_payload(p_out->impact_peak_g));
        log_tenths("===== imu: IMPACT %d.%d g =====\n",
                   p_out->impact_peak_g, 0);
    }

    if (p_out->b_recovered)
    {
        report_recovered();
    }

    if (p_out->b_calibrated)
    {
        (void)faults_signal(FAULTS_READY_IMU);

        if (!g_b_ready_beeped)
        {
            g_b_ready_beeped = true;
            barcode_beep(2U, BC_BEEP_MS);   /* 2 beeps: IMU ready. */
        }

        log_line("===== imu: calibrated (forced=%d) =====\n",
                 p_out->b_calib_forced ? 1 : 0, 0, 0);
    }

    if (p_out->b_hump_closed)
    {
        report_hump(&p_out->hump);
    }

    if (p_out->b_motion_changed)
    {
        /* Cast: the enum constants are 0..3, non-negative. */
        emit_event(IMU_PACK_EVT_MOTION, (uint32_t)p_out->motion_class);
    }

    if (p_out->b_terrain_changed)
    {
        /* Cast: the enum constants are 0..3, non-negative. */
        emit_event(IMU_PACK_EVT_TERRAIN, (uint32_t)p_out->terrain_state);
    }

    if (p_out->b_motion_changed || p_out->b_terrain_changed)
    {
        log_state("===== imu: now %s, %s =====\n", p_out->motion_class,
                  p_out->terrain_state);
    }

    if (p_out->b_filter_fault)
    {
        g_stats.read_errors++;
    }
}

/**
 * @brief   The sensor stopped giving usable data: flag it, free the bus and
 *          go OFFLINE to re-probe.
 * @param   reason  Why the sensor was declared lost.
 */
static void
sensor_lost (imu_pack_fault_reason_t reason)
{
    imu_pack_fault_reason_t final_reason = reason;
    imu_terrain_hump_t      hump;

    g_stats.recoveries++;

    /* Keep the hump measured so far; re-probing would drop it. */
    if (imu_monitor_close_hump(&g_monitor, &hump))
    {
        report_hump(&hump);
        log_line("imu: hump cut short, sensor lost\n", 0, 0, 0);
    }

    if (DRV_I2C1_ERR_BUS_STUCK == drv_i2c1_recover_bus())
    {
        final_reason = IMU_PACK_FAULT_BUS_STUCK;
    }

    report_fault(final_reason);
    g_read_fail_count = 0u;
    g_frozen_count    = 0u;
    g_state           = IMU_TASK_STATE_OFFLINE;
    g_retry_wait      = IMU_TASK_RETRY_CYCLES;
}

/**
 * @brief   Set the IMU fault bit, drop READY_IMU and report once per
 *          outage (a retry that fails again is not re-reported).
 * @param   reason  Fault reason for the event.
 */
static void
report_fault (imu_pack_fault_reason_t reason)
{
    if (!g_b_fault_reported)
    {
        imu_pack_snapshot_t snap;

        (void)faults_set(FAULTS_IMU);
        (void)nav_raise_fault(NAV_FLT_IMU);
        (void)faults_clear_status(FAULTS_READY_IMU);
        imu_ramp_stop();
        g_b_fault_reported = true;

        /* Cast: the enum constants are 0..7, non-negative. */
        emit_event(IMU_PACK_EVT_IMU_FAULT, (uint32_t)reason);
        log_line("===== imu: FAULT reason %d, impact detection unavailable "
                 "=====\n",
                 (int32_t)reason, 0, 0);

        /* Only this task writes the snapshot word, so updating one flag
         * through a local copy cannot lose another writer's field.
         */
        imu_pack_unpack_snapshot(g_snapshot_word, &snap);
        snap.b_imu_fault = true;
        g_snapshot_word  = imu_pack_snapshot(&snap);
    }
}

/**
 * @brief   Publish one closed hump.
 * @param   p_hump  The result.
 */
static void
report_hump (imu_terrain_hump_t const * p_hump)
{
    emit_event(IMU_PACK_EVT_HUMP,
               imu_pack_hump_payload(p_hump->peak_mm_packed,
                                     p_hump->b_saturated,
                                     p_hump->max_pitch_deg));
    if (IMU_TERRAIN_SIDE_NONE == p_hump->side)
    {
        log_tenths("===== imu: HUMP peak %d.%d mm, packed %d mm =====\n",
                   p_hump->peak_mm, (int32_t)p_hump->peak_mm_packed);
    }
    else
    {
        log_tenths((IMU_TERRAIN_SIDE_RIGHT == p_hump->side) ?
                   "===== imu: SIDE HUMP right wheel, peak %d.%d mm, "
                   "packed %d mm =====\n" :
                   "===== imu: SIDE HUMP left wheel, peak %d.%d mm, "
                   "packed %d mm =====\n",
                   p_hump->peak_mm, (int32_t)p_hump->peak_mm_packed);
        log_line("imu: side hump roll %d deg, heading moved %d pulses "
                 "(~7 = 1 deg)\n", (int32_t)p_hump->max_pitch_deg,
                 g_enc_head - g_side_start_head, 0);
    }
#if (1 == IMU_TASK_HUMP_TRACE)
    log_line("imu h: max pitch %d deg\n",
             (int32_t)p_hump->max_pitch_deg, 0, 0);
#endif
}

/**
 * @brief   Note the encoder heading when a side hump opens, so report_hump()
 *          can say how far the car turned across it.  The compass cannot
 *          tell: it swings by tens of degrees around the ramp.
 */
static void
update_side_heading (void)
{
    bool const b_open = imu_terrain_side_is_open(&g_monitor.terrain);

    if (b_open && !g_b_side_open)
    {
        g_side_start_head = g_enc_head;
    }

    g_b_side_open = b_open;
}

/**
 * @brief   Impact over: log that the car settled.  Recovery is not a
 *          published event; the IMPACT event is the only impact telemetry.
 */
static void
report_recovered (void)
{
    log_line("===== imu: recovered from impact =====\n", 0, 0, 0);
}

/**
 * @brief   "Course completed": report the run summary and go idle.
 */
static void
finish_course (void)
{
    bool           b_saturated = false;
    uint32_t const peak_mm     = imu_pack_quantize_peak(
        imu_monitor_get_run_peak_mm(&g_monitor), &b_saturated);
    uint32_t const humps       = imu_monitor_get_hump_count(&g_monitor);

    emit_event(IMU_PACK_EVT_COURSE_DONE,
               imu_pack_course_payload(peak_mm, b_saturated, humps));
    log_tenths("===== imu: course done, highest hump %d.%d mm, humps %d "
               "=====\n",
               imu_monitor_get_run_peak_mm(&g_monitor), (int32_t)humps);

    imu_ramp_stop();
    g_state = IMU_TASK_STATE_COMPLETE;
    (void)tk_stp_cyc(g_h_imu_cyc);
}

#if (1 == IMU_TASK_HUMP_TRACE)
/**
 * @brief   Diagnostic: while a hump is open, print every 50 ms what the
 *          height estimate is built from.  All values are whole numbers:
 *          pitch in 0.1 deg, forward acceleration in 0.01 g, front height
 *          in 0.1 mm, travel in mm.
 */
static void
trace_hump (void)
{
    static uint32_t s_div = 0u;

    if (IMU_TERRAIN_LEVEL == imu_terrain_get_state(&g_monitor.terrain))
    {
        s_div = 0u;
    }
    else if (0u == (s_div++ % 5u))
    {
        imu_filter_attitude_t const * const p_att =
            imu_monitor_get_attitude(&g_monitor);

        /* Casts: small values, well inside the int range. */
        tm_printf((UB *)"imu h: pitch %d fwd %d front %d travel %d\n",
                  (int)(p_att->pitch_deg * 10.0f),
                  (int)(p_att->lin_fwd_g * 100.0f),
                  (int)(g_monitor.terrain.front_mm * 10.0f),
                  (int)g_monitor.terrain.travel_mm);
    }
    else
    {
        /* Between prints. */
    }
}
#endif

/**
 * @brief   Feed Line Following's ramp detection (IF-S4-7).  Skipped during
 *          calibration, when pitch and roll have no level reference yet,
 *          and if this sample just lost the sensor.
 */
static void
update_ramp (void)
{
    if ((IMU_TASK_STATE_RUNNING == g_state) &&
        (IMU_MONITOR_CALIBRATING != imu_monitor_get_mode(&g_monitor)))
    {
        imu_filter_attitude_t const * const p_att =
            imu_monitor_get_attitude(&g_monitor);

        if (imu_ramp_update(p_att->pitch_deg, p_att->roll_deg))
        {
            if (imu_on_ramp())
            {
                log_line("===== imu: ramp, tilt %d deg =====\n",
                         (int32_t)(imu_get_tilt_x10() / 10u), 0, 0);
            }
            else
            {
                log_line("===== imu: flat again =====\n", 0, 0, 0);
            }
        }
    }
}

/**
 * @brief   Build and publish the snapshot and aux words (one store each).
 */
static void
publish (void)
{
    imu_pack_snapshot_t snap;
    imu_pack_aux_t      aux;
    uint32_t const      flags = faults_get();

    imu_monitor_fill_publish(&g_monitor, &snap, &aux);
    snap.b_imu_fault = (0u != (flags & FAULTS_IMU));

    g_snapshot_word = imu_pack_snapshot(&snap);
    g_aux_word      = imu_pack_aux(&aux);
}

/**
 * @brief   Detect a sensor that keeps returning the very same sample.
 * @param   p_sample  New sample.
 * @return  true once IMU_TASK_FROZEN_SAMPLES identical samples were seen.
 */
static bool
sample_is_frozen (drv_imu_sample_t const * p_sample)
{
    bool b_same = true;

    for (uint32_t axis = 0u; axis < DRV_IMU_AXES; axis++)
    {
        if ((p_sample->accel[axis] != g_last_sample.accel[axis]) ||
            (p_sample->gyro[axis] != g_last_sample.gyro[axis]))
        {
            b_same = false;
        }
    }

    g_last_sample  = *p_sample;
    g_frozen_count = b_same ? (g_frozen_count + 1u) : 0u;

    return (g_frozen_count >= IMU_TASK_FROZEN_SAMPLES);
}

/**
 * @brief   Measured time since the previous sample.
 * @return  dt in seconds; the nominal 10 ms if the measurement is out of
 *          range (first sample, or the 1 MHz timer is not running).
 */
static float32_t
measure_dt (void)
{
    uint32_t const now = drv_us_timer_now();
    float32_t      dt  = NOMINAL_DT_S;

    if (g_b_have_time)
    {
        uint32_t const elapsed = now - g_last_time_us;

        if ((elapsed >= MIN_DT_US) && (elapsed <= MAX_DT_US))
        {
            /* Cast: elapsed <= 100000, exact in float. */
            dt = (float32_t)elapsed / US_PER_S;
        }
    }

    g_last_time_us = now;
    g_b_have_time  = true;

    return (dt);
}

/**
 * @brief   Read Buddy 2's wheel encoder counts (IF-06, drv_encoder.h).
 *
 * Each getter returns a 32-bit cumulative pulse count that is safe to read
 * from any task on either core.  The counts are always available in the
 * integrated firmware (Buddy 2 owns the ISR), so both words are present;
 * imu_odom's "no encoder" fallback is only exercised by the host tests.
 *
 * @param   p_counts  Receives the counts and which words exist.
 */
static void
read_encoders (imu_odom_counts_t * p_counts)
{
    p_counts->b_has_left  = true;
    p_counts->b_has_right = true;
    p_counts->left_count  = drv_encoder_get_count(DRV_ENCODER_LEFT);
    p_counts->right_count = drv_encoder_get_count(DRV_ENCODER_RIGHT);
}

/**
 * @brief   Queue one event for the Comm task without waiting.  If the
 *          buffer is full the event is dropped and counted; the snapshot
 *          word still carries the latest state and the run peak.
 * @param   type     Event type.
 * @param   payload  Event payload.
 */
static void
emit_event (imu_pack_event_t type, uint32_t payload)
{
    uint16_t event = imu_pack_event(type, payload);

    if (g_h_imu_mbf > 0)
    {
        /* Cast: sizeof(uint16_t) is 2, fits in INT. */
        if (E_OK != tk_snd_mbf(g_h_imu_mbf, &event, (INT)sizeof(event),
                               TMO_POL))
        {
            g_stats.events_dropped++;
        }
    }
}

/**
 * @brief   Print one console line with up to three integer arguments.
 * @param   p_format  printf-style format using %d / %x only.
 * @param   arg_a     First argument.
 * @param   arg_b     Second argument.
 * @param   arg_c     Third argument.
 */
static void
log_line (char const * p_format, int32_t arg_a, int32_t arg_b,
          int32_t arg_c)
{
    g_cycle_lines++;

    /* Cast: tm_printf() takes const UB *; the formats are plain ASCII.
     * Casts to INT: int32_t and INT are both 32 bits on the RP2040.
     */
    (void)tm_printf((UB const *)p_format, (INT)arg_a, (INT)arg_b,
                    (INT)arg_c);
}

/**
 * @brief   Print a value with one decimal ("%d.%d") and one more integer.
 * @param   p_format  Format with "%d.%d" first, then one optional "%d".
 * @param   value     Value to print with one decimal.
 * @param   arg_c     Extra integer argument.
 */
static void
log_tenths (char const * p_format, float32_t value, int32_t arg_c)
{
    int32_t const tenths   = imu_math_round(value * TENTHS_PER_UNIT);
    int32_t const whole    = tenths / TENTHS_DIVISOR;
    int32_t       fraction = tenths - (whole * TENTHS_DIVISOR);

    if (fraction < 0)
    {
        fraction = -fraction;
    }

    log_line(p_format, whole, fraction, arg_c);
}

/**
 * @brief   Print the motion class and terrain state as words.
 * @param   p_format  Format with two "%s": motion first, then terrain.
 * @param   motion    Motion class.
 * @param   terrain   Terrain state.
 */
static void
log_state (char const * p_format, imu_motion_class_t motion,
           imu_terrain_state_t terrain)
{
    g_cycle_lines++;

    /* Casts: tm_printf() takes const UB *; all strings are plain ASCII. */
    (void)tm_printf((UB const *)p_format, (UB const *)motion_name(motion),
                    (UB const *)terrain_name(terrain));
}

/**
 * @brief   Console name of a motion class.
 * @param   motion  Motion class.
 * @return  Upper-case name; "UNKNOWN" if out of range.
 */
static char const *
motion_name (imu_motion_class_t motion)
{
    char const * p_name = "UNKNOWN";

    switch (motion)
    {
        case IMU_MOTION_STATIONARY:
            p_name = "STATIONARY";
            break;
        case IMU_MOTION_ACCELERATING:
            p_name = "ACCELERATING";
            break;
        case IMU_MOTION_CRUISING:
            p_name = "CRUISING";
            break;
        case IMU_MOTION_TURNING:
            p_name = "TURNING";
            break;
        default:
            break;
    }

    return (p_name);
}

/**
 * @brief   Console name of a terrain state.
 * @param   terrain  Terrain state.
 * @return  Upper-case name; "UNKNOWN" if out of range.
 */
static char const *
terrain_name (imu_terrain_state_t terrain)
{
    char const * p_name = "UNKNOWN";

    switch (terrain)
    {
        case IMU_TERRAIN_LEVEL:
            p_name = "LEVEL";
            break;
        case IMU_TERRAIN_CLIMBING:
            p_name = "CLIMBING";
            break;
        case IMU_TERRAIN_DESCENDING:
            p_name = "DESCENDING";
            break;
        case IMU_TERRAIN_HUMP_CLOSED:
            p_name = "HUMP_CLOSED";
            break;
        default:
            break;
    }

    return (p_name);
}

/*** end of file ***/
