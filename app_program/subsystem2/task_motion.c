/** @file task_motion.c
 *
 * @brief Subsystem 2 Motion Control task (see motion.h).
 *
 * Every 10 ms the task measures both wheel speeds, then either takes the
 * next command (when idle) or runs one step of the current move.  Between
 * steps it waits on its event flag with a timeout of the rest of the
 * period, so StopTurn ends a pivot at once instead of up to 10 ms later.
 *
 * Each wheel's speed is held by a PID controller on top of a feedforward
 * duty.  A straight move also keeps the two wheels' pulse counts equal, so
 * the car holds its heading, except while a turn level is steering it.
 *
 * NOTE: The task drives the motors only while a move runs.  When idle it
 * leaves them alone, so test S2-M02 can drive them in TEST=1 builds.
 *
 * NOTE: Every float cast in this file converts a pulse count, a duration
 * or a speed that is far below 2^24, so the conversion is exact; and every
 * float-to-integer cast converts a value already clamped to its range.
 */

#include <stdint.h>
#include <stdbool.h>

#include "motion.h"
#include "drv_encoder.h"
#include "drv_motor.h"
#include "pid.h"
#include "../gpio_irq.h"

#define MOTION_CORE         (TP_PRC2)   // Core 1: Wi-Fi's service task runs
                                        // at priority 4 on core 0.
#define USEC_PER_MSEC       (1000u)
#define USEC_PER_SEC        (1000000.0f)
#define MM_S_PER_UM_USEC    (1000u)     // 1 um/usec = 1 m/s = 1000 mm/s.
#define PERIOD_USEC         (MOTION_PERIOD_MSEC * USEC_PER_MSEC)
#define SPEED_WINDOW        (4u)        // Speed over the last 4 periods.
#define SPEED_SAMPLES       (SPEED_WINDOW + 1u)

// Tuning.  Starting values from test S2-M02 (400 duty gave about 345 mm/s
// and 1000 about 820 mm/s, on a stand); tune them with test S2-M03.
//
#define FF_DUTY_PER_MM_S    (1.1f)      // Feedforward: duty per mm/s.
#define PID_KP              (1.0f)      // Duty per mm/s of error.
#define PID_KI              (5.0f)      // Duty per mm of accumulated error.
#define PID_KD              (0.0f)
#define PID_LIMIT           (400.0f)    // PID authority on top of the FF.
#define ACCEL_MM_S2         (800.0f)    // Ramp rate, up and down.
#define SLOW_ZONE_MM        (100u)      // Slow down over the last 100 mm.
#define MIN_SPEED_MM_S      (80u)       // Speed at the end of the slow zone.
#define BALANCE_PER_PULSE   (2.0f)      // mm/s per pulse of count difference.
#define BALANCE_MAX_MM_S    (60.0f)
#define TURN_STEP_MM_S      (20.0f)     // Speed difference per turn level.
#define PIVOT_SPEED_MM_S    (150.0f)    // Calibrate with test M1.

#define STALL_USEC          (500000u)   // No pulse for this long: stall.
#define PIVOT_TIMEOUT_USEC  (5000000u)  // PROVISIONAL: see motion_pivot_step.

#define EVT_STOP_TURN       (1u << 0)   // Motion's own flag until G1/G2.
#define EVT_BRAKE           (1u << 1)
#define EVT_ALL             (EVT_STOP_TURN | EVT_BRAKE)

#define STATE_IDLE          (0u)
#define STATE_STRAIGHT      (1u)
#define STATE_PIVOT         (2u)

#define CMD_BUF_BYTES       (64u)       // Room for one command or reply
#define REPLY_BUF_BYTES     (64u)       // plus the kernel's header.

typedef struct
{
    uint32_t     state;             // STATE_
    motion_cmd_t cmd;
    uint32_t     start_count[DRV_ENCODER_WHEELS];
    uint32_t     target_pulses;     // Straight: average of both wheels.
    int32_t      balance_base;      // Count difference to hold.
    uint32_t     start_usec;
    float        ramp_mm_s;         // Speed allowed by the ramp.
} motion_move_t;

typedef struct
{
    uint32_t count[DRV_ENCODER_WHEELS][SPEED_SAMPLES];
    uint32_t usec[SPEED_SAMPLES];
    uint32_t newest;                // Index of the newest sample.
    uint32_t filled;                // Samples stored, up to SPEED_SAMPLES.
    float    mm_s[DRV_ENCODER_WHEELS];
} motion_speed_t;

static ID                g_flg_id       = 0;
static ID                g_cmd_mbf_id   = 0;
static ID                g_reply_mbf_id = 0;

// Written by the Line Following task only; read by task_motion.
//
static int32_t volatile  g_turn_level   = 0;

// Written by Navigation only; read by task_motion.  0 means no cap.
//
static uint32_t volatile g_speed_cap    = 0u;

// Written by task_motion only.
//
static uint32_t volatile g_faults        = 0u;
static uint32_t volatile g_max_loop_usec = 0u;
static uint32_t volatile g_activity      = MOTION_ACTIVITY_IDLE;

// Used by task_motion only.  Static storage starts zeroed: idle, no speed
// samples (motion_init() sets up the PID controllers).
//
static motion_move_t     g_move;
static motion_speed_t    g_speed;
static pid_ctrl_t        g_pid[DRV_ENCODER_WHEELS];

static void     motion_task(INT start_code, void * p_ext_info);
static void     motion_on_event(UINT pattern);
static void     motion_cycle(uint32_t now_usec, float dt_sec);
static void     motion_speed_sample(uint32_t now_usec);
static void     motion_take_command(uint32_t now_usec);
static bool     motion_cmd_valid(motion_cmd_t const * p_cmd);
static void     motion_straight_step(uint32_t now_usec, float dt_sec);
static void     motion_pivot_step(uint32_t now_usec, float dt_sec);
static bool     motion_stalled(uint32_t now_usec);
static void     motion_drive(float left_mm_s, float right_mm_s,
                             bool b_left_back, bool b_right_back,
                             float dt_sec);
static int32_t  motion_wheel_duty(uint32_t wheel, float setpoint_mm_s,
                                  float dt_sec);
static void     motion_finish(uint32_t status, uint32_t reason);
static uint32_t motion_moved(uint32_t wheel);
static float    motion_ramp(float dt_sec, float limit_mm_s);
static float    motion_clamp(float value, float low, float high);


/*!
 * @brief Create Motion's event flag and message buffers.  Call from
 *        subsystem2_init(), after drv_motor_init().
 *
 * @return E_OK, or the error from creating a kernel object.
 */
ER
motion_init (void)
{
    T_CFLG cflg   = { 0 };
    T_CMBF cmbf   = { 0 };
    ER     result = E_OK;

    // Only task_motion waits on the flag; any task may set StopTurn.
    //
    cflg.flgatr  = (TA_TFIFO | TA_WSGL);
    cflg.iflgptn = 0u;
    g_flg_id     = tk_cre_flg(&cflg);
    result       = ((g_flg_id < E_OK) ? g_flg_id : E_OK);

    if (E_OK == result)
    {
        cmbf.mbfatr  = TA_TFIFO;
        cmbf.bufsz   = CMD_BUF_BYTES;
        cmbf.maxmsz  = (INT) sizeof(motion_cmd_t);  // Cast: 16 bytes.
        g_cmd_mbf_id = tk_cre_mbf(&cmbf);
        result       = ((g_cmd_mbf_id < E_OK) ? g_cmd_mbf_id : E_OK);
    }

    if (E_OK == result)
    {
        cmbf.bufsz     = REPLY_BUF_BYTES;
        cmbf.maxmsz    = (INT) sizeof(motion_reply_t);  // Cast: 8 bytes.
        g_reply_mbf_id = tk_cre_mbf(&cmbf);
        result         = ((g_reply_mbf_id < E_OK) ? g_reply_mbf_id : E_OK);
    }

    for (uint32_t wheel = 0u; wheel < DRV_ENCODER_WHEELS; wheel++)
    {
        pid_init(&g_pid[wheel], PID_KP, PID_KI, PID_KD, -PID_LIMIT,
                 PID_LIMIT);
    }

    return (result);
}

/*!
 * @brief Start task_motion.  Call from subsystem2_start().
 *
 * @return E_OK, or the error from creating or starting the task.
 */
ER
motion_start (void)
{
    // Cast: the kernel stores every task entry as FP.  TA_HLNG tasks are
    // called with (INT, void *), which is motion_task()'s signature.
    //
    ID const task_id = app_start_task((FP) motion_task, MOTION_PRIORITY,
                                      MOTION_CORE, "task_motion");

    return ((task_id < E_OK) ? task_id : E_OK);
}

/*!
 * @brief Send a command to task_motion (Navigation).
 *
 * Send the next command only after the reply to the previous one.
 *
 * @param[in] p_cmd         The command.
 * @param[in] timeout_msec  How long to wait for room, or TMO_POL.
 *
 * @return E_OK, or the error from tk_snd_mbf().
 */
ER
motion_send (motion_cmd_t const * p_cmd, TMO timeout_msec)
{
    // Cast: sizeof is 16 bytes, which fits an INT.
    //
    return (tk_snd_mbf(g_cmd_mbf_id, p_cmd, (INT) sizeof(motion_cmd_t),
                       timeout_msec));
}

/*!
 * @brief Wait for the reply to the last command (Navigation).
 *
 * @param[out] p_reply       Where to put the reply.
 * @param[in]  timeout_msec  How long to wait, TMO_POL or TMO_FEVR.
 *
 * @return E_OK, or the error from tk_rcv_mbf() (E_TMOUT if none yet).
 */
ER
motion_wait_reply (motion_reply_t * p_reply, TMO timeout_msec)
{
    INT const size = tk_rcv_mbf(g_reply_mbf_id, p_reply, timeout_msec);

    return ((size < E_OK) ? size : E_OK);
}

/*!
 * @brief Set the turn level (Line Following, every 20 ms).
 *
 * @param[in] level  -7 (hard left) to +7 (hard right); 0 is straight.
 *                   Values outside the range are clamped.
 */
void
motion_set_turn_level (int32_t level)
{
    int32_t clamped = level;

    if (clamped > MOTION_TURN_LEVEL_MAX)
    {
        clamped = MOTION_TURN_LEVEL_MAX;
    }
    else if (clamped < -MOTION_TURN_LEVEL_MAX)
    {
        clamped = -MOTION_TURN_LEVEL_MAX;
    }
    else
    {
        // In range.
    }

    g_turn_level = clamped;     // One aligned store.
}

/*!
 * @brief End the current pivot (Line Following after the line is lost and
 *        found again, or Obstacle Scan during a bypass).
 *
 * Ignored when no pivot is running.
 *
 * @return E_OK, or the error from tk_set_flg().
 */
ER
motion_stop_turn (void)
{
    return (tk_set_flg(g_flg_id, EVT_STOP_TURN));
}

/*!
 * @brief Brake now and end the running move (Navigation: an obstacle, or
 *        before a junction turn).
 *
 * The move replies MOTION_BRAKED.  A command sent but not yet started is
 * also ended, with the same reply, so every command still gets exactly one
 * reply.  Ignored when no move is running or waiting.
 *
 * @return E_OK, or the error from tk_set_flg().
 */
ER
motion_brake (void)
{
    return (tk_set_flg(g_flg_id, EVT_BRAKE));
}

/*!
 * @brief Cap the speed of straight moves (Navigation, e.g. slowing down
 *        approaching a junction).
 *
 * Applies at once to the running straight and to every later one, until
 * changed.  The speed ramps down or up to the new value.  Pivots ignore it.
 *
 * @param[in] speed_mm_s  Highest speed, 1 to MOTION_SPEED_MAX_MM_S, or 0
 *                        for no cap.  Higher values are clamped.
 */
void
motion_set_speed_cap (uint32_t speed_mm_s)
{
    g_speed_cap = ((speed_mm_s > MOTION_SPEED_MAX_MM_S) ?
                   MOTION_SPEED_MAX_MM_S : speed_mm_s);
}

/*!
 * @brief Read what task_motion is doing (MOTION_ACTIVITY_).
 */
uint32_t
motion_get_activity (void)
{
    return (g_activity);
}

/*!
 * @brief Read Motion's fault bits (MOTION_FAULT_).  They are set when a
 *        move ends on a fault, and cleared when the next command starts.
 */
uint32_t
motion_get_faults (void)
{
    return (g_faults);
}

/*!
 * @brief Read the longest time one 10 ms step has taken since boot, for
 *        the B1 baseline and test T2.
 */
uint32_t
motion_get_max_loop_usec (void)
{
    return (g_max_loop_usec);
}

/*!
 * @brief Task body: one cycle every MOTION_PERIOD_MSEC, forever.
 *
 * @param[in] start_code  Unused.
 * @param[in] p_ext_info  Unused.
 */
static void
motion_task (INT start_code, void * p_ext_info)
{
    uint32_t last_usec = gpio_irq_get_time_usec();
    uint32_t deadline  = last_usec;

    (void) start_code;
    (void) p_ext_info;

    for (;;)
    {
        UINT pattern = 0u;

        // Cast: the difference of two timer values is wrap-safe, and the
        // deadline is never more than one period away, far below 2^31.
        //
        int32_t const remaining_usec =
            (int32_t) (deadline - gpio_irq_get_time_usec());
        TMO wait_msec = TMO_POL;

        if (remaining_usec > 0)
        {
            wait_msec = (TMO) ((remaining_usec + (int32_t) USEC_PER_MSEC - 1)
                               / (int32_t) USEC_PER_MSEC);
        }

        if (E_OK == tk_wai_flg(g_flg_id, EVT_ALL, (TWF_ORW | TWF_BITCLR),
                               &pattern, wait_msec))
        {
            motion_on_event(pattern);
        }

        uint32_t const now_usec = gpio_irq_get_time_usec();

        // Cast: as above.
        //
        if ((int32_t) (now_usec - deadline) >= 0)
        {
            // Cast: one period is about 10000 usec.
            //
            float const dt_sec = (float) (now_usec - last_usec) / USEC_PER_SEC;

            motion_cycle(now_usec, dt_sec);
            last_usec = now_usec;
            deadline += PERIOD_USEC;

            // Fell more than a period behind: start counting again.
            //
            if ((int32_t) (now_usec - deadline) >= 0)
            {
                deadline = now_usec + PERIOD_USEC;
            }
        }
    }
}

/*!
 * @brief Act on a brake or StopTurn at once, without waiting for the next
 *        10 ms step.  A brake wins if both arrive together.
 *
 * @param[in] pattern  The event bits that woke the task.
 */
static void
motion_on_event (UINT pattern)
{
    if (0u != (pattern & EVT_BRAKE))
    {
        if (STATE_IDLE != g_move.state)
        {
            motion_finish(MOTION_BRAKED, MOTION_REASON_NONE);
        }
        else
        {
            motion_cmd_t cmd = { 0u };

            // A command sent just before the brake ends unstarted.
            //
            if (tk_rcv_mbf(g_cmd_mbf_id, &cmd, TMO_POL) > 0)
            {
                motion_finish(MOTION_BRAKED, MOTION_REASON_NONE);
            }
        }
    }
    else if ((0u != (pattern & EVT_STOP_TURN)) &&
             (STATE_PIVOT == g_move.state))
    {
        motion_finish(MOTION_DONE, MOTION_REASON_NONE);
    }
    else
    {
        // StopTurn with no pivot running: ignored.
    }
}

/*!
 * @brief One 10 ms step: measure, then take a command or run the move.
 *
 * @param[in] now_usec  Time of this step.
 * @param[in] dt_sec    Time since the last step.
 */
static void
motion_cycle (uint32_t now_usec, float dt_sec)
{
    motion_speed_sample(now_usec);

    if (STATE_IDLE == g_move.state)
    {
        motion_take_command(now_usec);
    }
    else if (STATE_STRAIGHT == g_move.state)
    {
        motion_straight_step(now_usec, dt_sec);
    }
    else
    {
        motion_pivot_step(now_usec, dt_sec);
    }

    uint32_t const loop_usec = gpio_irq_get_time_usec() - now_usec;

    if (loop_usec > g_max_loop_usec)
    {
        g_max_loop_usec = loop_usec;
    }
}

/*!
 * @brief Store this step's counts and work out both wheel speeds over the
 *        last SPEED_WINDOW periods.
 *
 * One encoder channel gives no direction, so the speeds are magnitudes.
 *
 * @param[in] now_usec  Time of this step.
 */
static void
motion_speed_sample (uint32_t now_usec)
{
    uint32_t const newest = (g_speed.newest + 1u) % SPEED_SAMPLES;

    g_speed.newest      = newest;
    g_speed.usec[newest] = now_usec;

    if (g_speed.filled < SPEED_SAMPLES)
    {
        g_speed.filled++;
    }

    // The oldest stored sample: SPEED_WINDOW back, or fewer after boot.
    //
    uint32_t const back   = g_speed.filled - 1u;
    uint32_t const oldest = (newest + SPEED_SAMPLES - back) % SPEED_SAMPLES;
    uint32_t const span   = now_usec - g_speed.usec[oldest];

    for (uint32_t wheel = 0u; wheel < DRV_ENCODER_WHEELS; wheel++)
    {
        g_speed.count[wheel][newest] = drv_encoder_get_count(wheel);
        g_speed.mm_s[wheel]          = 0.0f;

        if (span > 0u)
        {
            uint32_t const pulses = g_speed.count[wheel][newest] -
                                    g_speed.count[wheel][oldest];

            // um per usec is m/s.  At most ~110 pulses (36 mm) per window,
            // so the product stays below 2^32.
            //
            g_speed.mm_s[wheel] =
                (float) ((drv_encoder_pulses_to_um(pulses) *
                          MM_S_PER_UM_USEC) / span);
        }
    }
}

/*!
 * @brief Take the next command, if one is waiting, and start it.
 *
 * @param[in] now_usec  Time of this step.
 */
static void
motion_take_command (uint32_t now_usec)
{
    motion_cmd_t cmd  = { 0u };
    INT const    size = tk_rcv_mbf(g_cmd_mbf_id, &cmd, TMO_POL);

    if (size > 0)
    {
        if (!motion_cmd_valid(&cmd))
        {
            motion_finish(MOTION_FAULT, MOTION_REASON_BAD_CMD);
        }
        else
        {
            g_faults          = 0u;
            g_move.cmd        = cmd;
            g_move.start_usec = now_usec;
            g_move.ramp_mm_s  = 0.0f;
            g_move.balance_base = 0;

            for (uint32_t wheel = 0u; wheel < DRV_ENCODER_WHEELS; wheel++)
            {
                g_move.start_count[wheel] = drv_encoder_get_count(wheel);
                pid_reset(&g_pid[wheel]);
            }

            if (MOTION_CMD_STRAIGHT == cmd.type)
            {
                g_move.target_pulses =
                    drv_encoder_mm_to_pulses(cmd.distance_mm);
                g_move.state = STATE_STRAIGHT;
                g_activity   = MOTION_ACTIVITY_STRAIGHT;
            }
            else
            {
                // Drop a StopTurn left over from before this pivot.
                //
                (void) tk_clr_flg(g_flg_id, ~EVT_STOP_TURN);
                g_move.state = STATE_PIVOT;
                g_activity   = MOTION_ACTIVITY_PIVOT;
            }
        }
    }
}

/*!
 * @brief Check a command's type and ranges.
 */
static bool
motion_cmd_valid (motion_cmd_t const * p_cmd)
{
    bool b_valid = false;

    if (MOTION_CMD_STRAIGHT == p_cmd->type)
    {
        b_valid = ((p_cmd->direction <= MOTION_DIR_BACKWARDS) &&
                   (p_cmd->speed_mm_s >= 1u) &&
                   (p_cmd->speed_mm_s <= MOTION_SPEED_MAX_MM_S) &&
                   (p_cmd->distance_mm >= 1u) &&
                   (p_cmd->distance_mm <= MOTION_DISTANCE_MAX_MM));
    }
    else if (MOTION_CMD_PIVOT == p_cmd->type)
    {
        b_valid = (p_cmd->direction <= MOTION_DIR_RIGHT);
    }
    else
    {
        // Unknown type.
    }

    return (b_valid);
}

/*!
 * @brief One step of a straight move.
 *
 * The speed ramps at ACCEL_MM_S2 to the command's speed, or to
 * Navigation's speed cap if that is lower, and falls linearly over the
 * last SLOW_ZONE_MM to MIN_SPEED_MM_S; then the move brakes on the target.  A
 * turn level steers by TURN_STEP_MM_S per level; with level 0 the wheels'
 * count difference is held where the last steering left it.
 *
 * @param[in] now_usec  Time of this step.
 * @param[in] dt_sec    Time since the last step.
 */
static void
motion_straight_step (uint32_t now_usec, float dt_sec)
{
    uint32_t const left  = motion_moved(DRV_ENCODER_LEFT);
    uint32_t const right = motion_moved(DRV_ENCODER_RIGHT);
    uint32_t const done  = (left + right) / 2u;

    if (done >= g_move.target_pulses)
    {
        motion_finish(MOTION_DONE, MOTION_REASON_NONE);
    }
    else if (motion_stalled(now_usec))
    {
        g_faults = MOTION_FAULT_STALL;
        motion_finish(MOTION_FAULT, MOTION_REASON_STALL);
    }
    else
    {
        uint32_t const left_mm = drv_encoder_pulses_to_mm(
                                     g_move.target_pulses - done);
        uint32_t const cap     = g_speed_cap;
        uint32_t const wanted  = (((0u != cap) &&
                                   (cap < g_move.cmd.speed_mm_s)) ?
                                  cap : g_move.cmd.speed_mm_s);
        float const    cruise  = (float) wanted;
        float          speed   = motion_ramp(dt_sec, cruise);

        if ((left_mm < SLOW_ZONE_MM) && (cruise > (float) MIN_SPEED_MM_S))
        {
            float const limit = (float) MIN_SPEED_MM_S +
                                (((cruise - (float) MIN_SPEED_MM_S) *
                                  (float) left_mm) / (float) SLOW_ZONE_MM);

            if (speed > limit)
            {
                speed = limit;
            }
        }

        // Cast: both counts are below 2^31 within one move, so the
        // difference fits an int32_t.  Positive steer speeds up the left
        // wheel and turns the car right.
        //
        int32_t const difference = (int32_t) left - (int32_t) right;
        int32_t const level      = g_turn_level;
        float         steer      = 0.0f;

        if (0 != level)
        {
            g_move.balance_base = difference;
            steer = (float) level * TURN_STEP_MM_S;
        }
        else
        {
            steer = motion_clamp((-BALANCE_PER_PULSE *
                                  (float) (difference - g_move.balance_base)),
                                 -BALANCE_MAX_MM_S, BALANCE_MAX_MM_S);
        }

        bool const b_back = (MOTION_DIR_BACKWARDS == g_move.cmd.direction);

        motion_drive((speed + steer), (speed - steer), b_back, b_back,
                     dt_sec);
    }
}

/*!
 * @brief One step of a pivot: the wheels turn in opposite directions at
 *        PIVOT_SPEED_MM_S, with their counts kept equal so the car turns
 *        about its centre.  StopTurn ends it (see motion_task()).
 *
 * PROVISIONAL: the design has no pivot time limit; the StopTurn sender
 * bounds its own search and raises its own fault bit.  Until those fault
 * bits reach Motion (G1, G2), a pivot ends with MOTION_REASON_TIMEOUT
 * after PIVOT_TIMEOUT_USEC, so the car cannot spin forever.
 *
 * @param[in] now_usec  Time of this step.
 * @param[in] dt_sec    Time since the last step.
 */
static void
motion_pivot_step (uint32_t now_usec, float dt_sec)
{
    if ((now_usec - g_move.start_usec) >= PIVOT_TIMEOUT_USEC)
    {
        motion_finish(MOTION_FAULT, MOTION_REASON_TIMEOUT);
    }
    else if (motion_stalled(now_usec))
    {
        g_faults = MOTION_FAULT_STALL;
        motion_finish(MOTION_FAULT, MOTION_REASON_STALL);
    }
    else
    {
        uint32_t const left   = motion_moved(DRV_ENCODER_LEFT);
        uint32_t const right  = motion_moved(DRV_ENCODER_RIGHT);

        // Cast: as in motion_straight_step().
        //
        int32_t const difference = (int32_t) left - (int32_t) right;
        float const   speed      = motion_ramp(dt_sec, PIVOT_SPEED_MM_S);
        float const   steer      =
            motion_clamp((-BALANCE_PER_PULSE * (float) difference),
                         -BALANCE_MAX_MM_S, BALANCE_MAX_MM_S);
        bool const    b_left     = (MOTION_DIR_LEFT == g_move.cmd.direction);

        // A left pivot turns the left wheel backwards.
        //
        motion_drive((speed + steer), (speed - steer), b_left, !b_left,
                     dt_sec);
    }
}

/*!
 * @brief True if a wheel has had no pulse for STALL_USEC while driven.
 *
 * The first STALL_USEC of a move are skipped: a wheel starting from rest
 * has had no pulse for a long time.
 */
static bool
motion_stalled (uint32_t now_usec)
{
    bool b_stalled = false;

    if ((now_usec - g_move.start_usec) >= STALL_USEC)
    {
        for (uint32_t wheel = 0u; wheel < DRV_ENCODER_WHEELS; wheel++)
        {
            if (drv_encoder_get_idle_usec(wheel) >= STALL_USEC)
            {
                b_stalled = true;
            }
        }
    }

    return (b_stalled);
}

/*!
 * @brief Drive both wheels at speed magnitudes, each in its direction.
 *
 * @param[in] left_mm_s     Left wheel speed; below 0 counts as 0.
 * @param[in] right_mm_s    Right wheel speed; below 0 counts as 0.
 * @param[in] b_left_back   True to turn the left wheel backwards.
 * @param[in] b_right_back  True to turn the right wheel backwards.
 * @param[in] dt_sec        Time since the last step.
 */
static void
motion_drive (float left_mm_s, float right_mm_s, bool b_left_back,
              bool b_right_back, float dt_sec)
{
    int32_t const left  = motion_wheel_duty(DRV_ENCODER_LEFT, left_mm_s,
                                            dt_sec);
    int32_t const right = motion_wheel_duty(DRV_ENCODER_RIGHT, right_mm_s,
                                            dt_sec);

    (void) drv_motor_set(DRV_MOTOR_LEFT, (b_left_back ? -left : left));
    (void) drv_motor_set(DRV_MOTOR_RIGHT, (b_right_back ? -right : right));
}

/*!
 * @brief Duty for one wheel: feedforward plus the wheel's PID.
 *
 * @param[in] wheel          DRV_ENCODER_LEFT or DRV_ENCODER_RIGHT.
 * @param[in] setpoint_mm_s  Wanted speed magnitude.
 * @param[in] dt_sec         Time since the last step.
 *
 * @return Duty magnitude, 0 to DRV_MOTOR_DUTY_MAX.
 */
static int32_t
motion_wheel_duty (uint32_t wheel, float setpoint_mm_s, float dt_sec)
{
    float const setpoint = motion_clamp(setpoint_mm_s, 0.0f,
                                        (float) MOTION_SPEED_MAX_MM_S +
                                        BALANCE_MAX_MM_S);
    float const duty     = (FF_DUTY_PER_MM_S * setpoint) +
                           pid_update(&g_pid[wheel], setpoint,
                                      g_speed.mm_s[wheel], dt_sec);

    return ((int32_t) motion_clamp(duty, 0.0f, (float) DRV_MOTOR_DUTY_MAX));
}

/*!
 * @brief End the move: brake both wheels, reply, and go idle.
 *
 * @param[in] status  MOTION_DONE or MOTION_FAULT.
 * @param[in] reason  MOTION_REASON_.
 */
static void
motion_finish (uint32_t status, uint32_t reason)
{
    motion_reply_t const reply = { status, reason };

    (void) drv_motor_brake(DRV_MOTOR_LEFT);
    (void) drv_motor_brake(DRV_MOTOR_RIGHT);
    g_move.state = STATE_IDLE;
    g_activity   = MOTION_ACTIVITY_IDLE;

    // TMO_POL: Motion never waits for Navigation.  Navigation reads each
    // reply before sending the next command, so there is always room.
    //
    if (E_OK != tk_snd_mbf(g_reply_mbf_id, &reply,
                           (INT) sizeof(motion_reply_t), TMO_POL))
    {
        tm_printf((UB *) "task_motion: reply lost (status %lu reason %lu)\n",
                  status, reason);
    }
}

/*!
 * @brief Pulses a wheel has turned since the move started.
 */
static uint32_t
motion_moved (uint32_t wheel)
{
    return (drv_encoder_get_count(wheel) - g_move.start_count[wheel]);
}

/*!
 * @brief Move the ramp one step towards a speed and return the allowed
 *        speed.  It rises or falls at ACCEL_MM_S2.
 *
 * @param[in] dt_sec      Time since the last step.
 * @param[in] limit_mm_s  Speed the ramp heads for.
 */
static float
motion_ramp (float dt_sec, float limit_mm_s)
{
    float const step = ACCEL_MM_S2 * dt_sec;

    if (g_move.ramp_mm_s < limit_mm_s)
    {
        g_move.ramp_mm_s = motion_clamp((g_move.ramp_mm_s + step), 0.0f,
                                        limit_mm_s);
    }
    else
    {
        g_move.ramp_mm_s = motion_clamp((g_move.ramp_mm_s - step),
                                        limit_mm_s, g_move.ramp_mm_s);
    }

    return (g_move.ramp_mm_s);
}

/*!
 * @brief Limit a value to a range.
 */
static float
motion_clamp (float value, float low, float high)
{
    float result = value;

    if (result > high)
    {
        result = high;
    }
    else if (result < low)
    {
        result = low;
    }
    else
    {
        // Already in range.
    }

    return (result);
}

/*** end of file ***/

