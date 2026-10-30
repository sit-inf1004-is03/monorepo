/** @file s2_test_b2.c
 *
 * @brief Manual test B2: measure the encoder pulses per wheel revolution
 *        while a person turns each wheel one revolution by hand, forwards
 *        and then backwards.
 *
 * The test runs in its own low-priority task, so it does not hold up the
 * other subsystems' tests.  It prompts four timed steps on the console,
 * prints the counts whenever they change, and ends each step with the
 * pulses counted and rejected on both wheels.  Expected: about the same
 * count on all four steps (that count is the pulses per revolution), 0 on
 * the wheel not being turned, and 0 rejected.  One channel cannot tell
 * direction, so both directions count up.  Record the result as S2-M01 in
 * testing/subsystem2/TEST_LOG.md.
 *
 * NOTE: Each tm_printf() format is cast to (UB *).  tm_printf() takes UB *
 * and the formats are ASCII string literals, so only the signedness of the
 * character type changes.
 */

#include <stdint.h>

#include "s2_test_b2.h"

#if APP_TEST

#include "drv_encoder.h"

#define B2_TASK_PRIORITY    (15)        // Subsystem 2 band is 14 to 15.
#define B2_CORE             (TP_PRC1)   // Core 0.  The encoder readers are
                                        // safe from either core.
#define B2_LEAD_IN_MSEC     (10000u)    // Lets the other tests finish and
                                        // gives time to grab the wheel.
#define B2_STEP_MSEC        (15000u)
#define B2_POLL_MSEC        (20u)
#define B2_POLLS_PER_STEP   (B2_STEP_MSEC / B2_POLL_MSEC)
#define B2_STEPS            (4u)
#define MSEC_PER_SECOND     (1000u)

typedef struct
{
    char const * p_wheel;
    char const * p_direction;
} b2_step_t;

static b2_step_t const g_steps[B2_STEPS] =
{
    { "LEFT (GP5)",  "forwards"  },
    { "LEFT (GP5)",  "backwards" },
    { "RIGHT (GP7)", "forwards"  },
    { "RIGHT (GP7)", "backwards" },
};

static void s2_test_b2_task(INT start_code, void * p_ext_info);
static void b2_run_step(uint32_t step);


/*!
 * @brief Start the B2 task.  Called at the end of subsystem2_test(), after
 *        the automatic checks that need the wheels to be still.
 *
 * @return E_OK, or the error from creating or starting the task.
 */
ER
s2_test_b2_start (void)
{
    // Cast: the kernel stores every task entry as FP.  TA_HLNG tasks are
    // called with (INT, void *), which is s2_test_b2_task()'s signature.
    //
    ID const task_id = app_start_task((FP) s2_test_b2_task, B2_TASK_PRIORITY,
                                      B2_CORE, "s2_test_b2_task");

    return ((task_id < E_OK) ? task_id : E_OK);
}

/*!
 * @brief Task body: wait for the lead-in, run the four steps, then exit.
 *
 * @param[in] start_code  Unused.
 * @param[in] p_ext_info  Unused.
 */
static void
s2_test_b2_task (INT start_code, void * p_ext_info)
{
    (void) start_code;
    (void) p_ext_info;

    tm_printf((UB *) "[B2] encoder pulse count: %u steps of %u s, "
              "starting in %u s.  Wheels off the ground, motors off.\n",
              B2_STEPS, (B2_STEP_MSEC / MSEC_PER_SECOND),
              (B2_LEAD_IN_MSEC / MSEC_PER_SECOND));
    (void) tk_dly_tsk(B2_LEAD_IN_MSEC);

    for (uint32_t step = 0u; step < B2_STEPS; step++)
    {
        b2_run_step(step);
    }

    tm_printf((UB *) "[B2] done.  Copy the four result lines into "
              "TEST_LOG.md as S2-M01.\n");
    tk_exd_tsk();
}

/*!
 * @brief One step: prompt, print each pulse, then print the step's totals.
 *
 * Counts are measured from a snapshot taken at the start of the step; the
 * driver's counts are never reset.
 *
 * @param[in] step  Step index, 0 to B2_STEPS - 1.
 */
static void
b2_run_step (uint32_t step)
{
    uint32_t const start_left    = drv_encoder_get_count(DRV_ENCODER_LEFT);
    uint32_t const start_right   = drv_encoder_get_count(DRV_ENCODER_RIGHT);
    uint32_t const chatter_left  = drv_encoder_get_chatter(DRV_ENCODER_LEFT);
    uint32_t const chatter_right = drv_encoder_get_chatter(DRV_ENCODER_RIGHT);
    uint32_t       moved_left    = 0u;
    uint32_t       moved_right   = 0u;

    tm_printf((UB *) "[B2] step %lu: turn the %s wheel ONE revolution %s, "
              "slowly, from tape mark to tape mark (%u s)\n", (step + 1u),
              g_steps[step].p_wheel, g_steps[step].p_direction,
              (B2_STEP_MSEC / MSEC_PER_SECOND));

    for (uint32_t poll = 0u; poll < B2_POLLS_PER_STEP; poll++)
    {
        uint32_t const now_left  =
            drv_encoder_get_count(DRV_ENCODER_LEFT) - start_left;
        uint32_t const now_right =
            drv_encoder_get_count(DRV_ENCODER_RIGHT) - start_right;

        if ((now_left != moved_left) || (now_right != moved_right))
        {
            tm_printf((UB *) "[B2]   L=%lu R=%lu\n", now_left, now_right);
            moved_left  = now_left;
            moved_right = now_right;
        }

        (void) tk_dly_tsk(B2_POLL_MSEC);
    }

    tm_printf((UB *) "[B2] result step %lu, %s %s: L=%lu R=%lu; "
              "rejected L=%lu R=%lu (expect the same count on every step, "
              "0 on the other wheel, 0 rejected)\n",
              (step + 1u), g_steps[step].p_wheel, g_steps[step].p_direction,
              moved_left, moved_right,
              (drv_encoder_get_chatter(DRV_ENCODER_LEFT) - chatter_left),
              (drv_encoder_get_chatter(DRV_ENCODER_RIGHT) - chatter_right));
}

#endif /* APP_TEST */

/*** end of file ***/

