/** @file s2_test_motion.c
 *
 * @brief Manual test S2-M03: send task_motion four commands, as Navigation
 *        will, with the car on a stand, and print how each one went.
 *
 *   1. straight forwards, 300 mm/s, 1000 mm: expect DONE, both wheels
 *      within a few pulses of 1000 mm, and the speed near 300 mm/s;
 *   2. pivot left, StopTurn sent after 1.5 s: expect DONE at about 1.5 s;
 *   3. straight backwards, 300 mm/s, 500 mm: expect DONE;
 *   4. pivot right, no StopTurn: expect FAULT (timeout) after 5 s.
 * While a command runs, a line every 100 ms shows both wheel speeds from
 * the encoders, so the PID tuning can be read off the log.  Record the
 * result as S2-M03 in testing/subsystem2/TEST_LOG.md.
 *
 * NOTE: Each tm_printf() format is cast to (UB *).  tm_printf() takes UB *
 * and the formats are ASCII string literals, so only the signedness of the
 * character type changes.
 */

#include <stdint.h>
#include <stdbool.h>

#include "s2_test_motion.h"

#if APP_TEST

#include "drv_encoder.h"
#include "motion.h"
#include "../gpio_irq.h"

#define M03_TASK_PRIORITY   (15)        // Below task_motion (14).
#define M03_CORE            (TP_PRC1)   // Core 0, as the other S2 tests.
#define M03_LEAD_IN_MSEC    (10000u)
#define M03_PAUSE_MSEC      (2000u)     // Between commands.
#define M03_TRACE_MSEC      (100u)
#define M03_REPLY_MSEC      (20000u)    // Give up on a reply after 20 s.
#define M03_STOP_TURN_MSEC  (1500u)
#define M03_NO_STOP_TURN    (0u)
#define M03_STEPS           (4u)
#define USEC_PER_MSEC       (1000u)

typedef struct
{
    char const * p_name;
    motion_cmd_t cmd;
    uint32_t     stop_turn_msec;    // M03_NO_STOP_TURN: never sent.
    uint32_t     expect_status;
} m03_step_t;

static m03_step_t const g_steps[M03_STEPS] =
{
    { "straight forwards 300 mm/s 1000 mm",
      { MOTION_CMD_STRAIGHT, MOTION_DIR_FORWARDS, 300u, 1000u },
      M03_NO_STOP_TURN, MOTION_DONE },
    { "pivot left, StopTurn at 1.5 s",
      { MOTION_CMD_PIVOT, MOTION_DIR_LEFT, 0u, 0u },
      M03_STOP_TURN_MSEC, MOTION_DONE },
    { "straight backwards 300 mm/s 500 mm",
      { MOTION_CMD_STRAIGHT, MOTION_DIR_BACKWARDS, 300u, 500u },
      M03_NO_STOP_TURN, MOTION_DONE },
    { "pivot right, no StopTurn (expect timeout)",
      { MOTION_CMD_PIVOT, MOTION_DIR_RIGHT, 0u, 0u },
      M03_NO_STOP_TURN, MOTION_FAULT },
};

static void s2_test_motion_task(INT start_code, void * p_ext_info);
static void m03_run_step(uint32_t step);


/*!
 * @brief Start the S2-M03 task.  Called at the end of subsystem2_test().
 *
 * @return E_OK, or the error from creating or starting the task.
 */
ER
s2_test_motion_start (void)
{
    // Cast: the kernel stores every task entry as FP.  TA_HLNG tasks are
    // called with (INT, void *), which is s2_test_motion_task()'s
    // signature.
    //
    ID const task_id = app_start_task((FP) s2_test_motion_task,
                                      M03_TASK_PRIORITY, M03_CORE,
                                      "s2_test_motion_task");

    return ((task_id < E_OK) ? task_id : E_OK);
}

/*!
 * @brief Task body: wait for the lead-in, run the four commands, exit.
 *
 * @param[in] start_code  Unused.
 * @param[in] p_ext_info  Unused.
 */
static void
s2_test_motion_task (INT start_code, void * p_ext_info)
{
    (void) start_code;
    (void) p_ext_info;

    tm_printf((UB *) "[M03] motion test starts in %u s.  Car on a stand, "
              "wheels off the ground.\n", (M03_LEAD_IN_MSEC / 1000u));
    (void) tk_dly_tsk(M03_LEAD_IN_MSEC);

    for (uint32_t step = 0u; step < M03_STEPS; step++)
    {
        m03_run_step(step);
        (void) tk_dly_tsk(M03_PAUSE_MSEC);
    }

    tm_printf((UB *) "[M03] done.  Longest task_motion step: %lu usec "
              "(period 10000).\n", motion_get_max_loop_usec());
    tk_exd_tsk();
}

/*!
 * @brief One command: send it, trace the wheel speeds until the reply,
 *        then print the result.
 *
 * @param[in] step  Step index, 0 to M03_STEPS - 1.
 */
static void
m03_run_step (uint32_t step)
{
    m03_step_t const * const p_step      = &g_steps[step];
    uint32_t const           start_left  =
        drv_encoder_get_count(DRV_ENCODER_LEFT);
    uint32_t const           start_right =
        drv_encoder_get_count(DRV_ENCODER_RIGHT);
    uint32_t const           start_usec  = gpio_irq_get_time_usec();
    motion_reply_t           reply       = { 0u, 0u };
    bool                     b_replied   = false;
    bool                     b_stop_sent = false;
    ER                       result      = E_OK;

    tm_printf((UB *) "[M03] step %lu: %s\n", (step + 1u), p_step->p_name);
    result = motion_send(&p_step->cmd, TMO_POL);

    if (E_OK != result)
    {
        tm_printf((UB *) "[M03]   motion_send error %d\n", result);
    }

    for (uint32_t waited = 0u;
         (E_OK == result) && (!b_replied) && (waited < M03_REPLY_MSEC);
         waited += M03_TRACE_MSEC)
    {
        b_replied = (E_OK == motion_wait_reply(&reply, M03_TRACE_MSEC));

        if ((!b_replied) && (!b_stop_sent) &&
            (M03_NO_STOP_TURN != p_step->stop_turn_msec) &&
            (waited >= p_step->stop_turn_msec))
        {
            (void) motion_stop_turn();
            b_stop_sent = true;
            tm_printf((UB *) "[M03]   StopTurn sent\n");
        }

        if (!b_replied)
        {
            tm_printf((UB *) "[M03]   t=%lu ms  L=%lu R=%lu mm/s\n",
                      (waited + M03_TRACE_MSEC),
                      drv_encoder_get_speed(DRV_ENCODER_LEFT),
                      drv_encoder_get_speed(DRV_ENCODER_RIGHT));
        }
    }

    uint32_t const left  = drv_encoder_get_count(DRV_ENCODER_LEFT) -
                           start_left;
    uint32_t const right = drv_encoder_get_count(DRV_ENCODER_RIGHT) -
                           start_right;

    tm_printf((UB *) "[M03] result step %lu: %s, status %lu reason %lu "
              "(expect status %lu); %lu ms; L=%lu R=%lu pulses = "
              "%lu / %lu mm\n", (step + 1u),
              (b_replied ? "replied" : "NO REPLY"), reply.status,
              reply.reason, p_step->expect_status,
              ((gpio_irq_get_time_usec() - start_usec) / USEC_PER_MSEC),
              left, right, drv_encoder_pulses_to_mm(left),
              drv_encoder_pulses_to_mm(right));
}

#endif /* APP_TEST */

/*** end of file ***/

