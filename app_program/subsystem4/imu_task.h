/**
 * @file    imu_task.h
 * @brief   IMU / Terrain task: the only owner of I2C1 and the MPU-9250.
 *
 * Task table (Week 6 Table 5, Tribunal 1): periodic 10 ms, Medium-High
 * priority, sole owner of I2C1 (GP2/GP3).  Paced by a 10 ms cyclic handler
 * that wakes the task, so the period does not drift with execution time.
 *
 * Outputs to the other subsystems (TEAM.md section 5):
 *   IF-S4-1  snapshot word   imu_task_get_snapshot()   any task, any time
 *   IF-S4-2  aux word        imu_task_get_aux()        any task, any time
 *   IF-S4-3  event stream    imu_task_read_event()     Buddy 1 (Comm task)
 *   IF-12    IMU fault       nav_raise_fault(NAV_FLT_IMU),
 *                            nav_clear_fault(NAV_FLT_IMU)  Navigation
 *   IF-S4-4  IMU fault, READY_IMU bits in common/faults.h  S4 only so far
 *   IF-S4-7  ramp            imu_on_ramp(), imu_get_tilt_x10() (imu.h)
 *                                                      Line Following
 * Inputs:
 *   IF-06    encoder counts  drv_encoder_get_count()        from Buddy 2
 *   IF-S4-6  course done     imu_task_signal_course_done()  from Navigation
 *
 * The snapshot and aux words are each written with ONE 32-bit store, so a
 * reader on either core always sees a consistent word without any lock.
 */

#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#define IMU_TASK_PERIOD_MS          (10u)
#define IMU_TASK_RETRY_CYCLES       (100u)  /* re-probe every 1 s offline */
#define IMU_TASK_MAX_READ_ERRORS    (3u)    /* 30 ms of failed reads      */
#define IMU_TASK_FROZEN_SAMPLES     (50u)   /* 500 ms of identical data   */
#define IMU_TASK_EVENT_DEPTH        (32u)   /* events held for Comm       */

/*
 * The IMU owns its IMU-fault bit and clears it itself on recovery
 * (re-initialisation after an I2C fault), in common/faults.h and in
 * Navigation (nav_clear_fault(), IF-12).  Set to 0 to leave it latched
 * until a remote CLEAR instead.
 */
#define IMU_TASK_SELF_CLEAR_FAULTS  (1)

/*
 * Diagnostic trace: raw accel / gyro / compass, attitude, terrain and
 * motion every IMU_TASK_TRACE_CYCLES samples (tests S4-M01..M02, compass
 * hard-iron offsets).  Off by default to keep the console free for other
 * subsystems.
 */
#define IMU_TASK_TRACE_ENABLE       (0)
#define IMU_TASK_TRACE_CYCLES       (50u)

/** Top-level state of the task (3 bits). */
typedef enum
{
    IMU_TASK_STATE_PROBE    = 0,    /**< Looking for the sensor.            */
    IMU_TASK_STATE_RUNNING  = 1,    /**< Calibrating, Monitoring or Impact. */
    IMU_TASK_STATE_OFFLINE  = 2,    /**< Sensor fault; retrying.            */
    IMU_TASK_STATE_COMPLETE = 3     /**< Course completed; idle.            */
} imu_task_state_t;

/** Diagnostic counters (each field is a single 32-bit word). */
typedef struct
{
    uint32_t cycles;            /**< Samples processed.                    */
    uint32_t overruns;          /**< Periods missed (queued wake-ups).     */
    uint32_t read_errors;       /**< Failed sensor reads.                  */
    uint32_t recoveries;        /**< I2C bus recoveries performed.         */
    uint32_t events_dropped;    /**< Events lost because Comm was slow.    */
    uint32_t max_exec_us;       /**< Longest sampling cycle (no probes).   */
    uint32_t last_exec_us;      /**< Most recent cycle.                    */
    uint32_t max_exec_cycle;    /**< cycles count after the longest cycle. */
    uint32_t max_exec_read_us;  /**< Sensor read time in the longest cycle.*/
    uint32_t max_exec_lines;    /**< Console lines it printed.             */
    uint32_t win_max_exec_us;   /**< Longest cycle since the window reset. */
    uint32_t win_max_cycle;     /**< cycles count after that cycle.        */
    uint32_t win_max_read_us;   /**< Sensor read time in that cycle.       */
    uint32_t state;             /**< imu_task_state_t value.               */
} imu_task_stats_t;

ER       imu_task_init (void);
ER       imu_task_start (PRI priority);
uint32_t imu_task_get_snapshot (void);
uint32_t imu_task_get_aux (void);
ER       imu_task_read_event (uint16_t * p_event);
void     imu_task_signal_course_done (void);
ER       imu_task_restart (void);
void     imu_task_get_stats (imu_task_stats_t * p_stats);

#if APP_TEST
void     imu_task_test_inject_shock (void);
void     imu_task_test_reset_window (void);
#endif

#endif /* IMU_TASK_H */

/*** end of file ***/
