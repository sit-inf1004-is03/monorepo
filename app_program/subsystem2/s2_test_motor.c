/** @file s2_test_motor.c
 *
 * @brief Manual test S2-M02: run each motor forwards, backwards and at full
 *        speed, with the car on a stand, and print what the encoders saw.
 *
 * The test runs in its own low-priority task, which owns PWM slices 4 and
 * 5 in TEST=1 builds.  For each step it prints the pulses counted on both
 * wheels and the pulse rate over the second half of the step, once the
 * motor is up to speed.  The person watching notes which wheel turned and
 * whether "forwards" turned it the way the car drives forwards; one encoder
 * channel cannot tell direction.  Record the result as S2-M02 in
 * testing/subsystem2/TEST_LOG.md.
 *
 * NOTE: Each tm_printf() format is cast to (UB *).  tm_printf() takes UB *
 * and the formats are ASCII string literals, so only the signedness of the
 * character type changes.
 */

#include <stdint.h>

#include "s2_test_motor.h"

#if APP_TEST

#include "drv_encoder.h"
#include "drv_motor.h"

#define M02_TASK_PRIORITY   (15)        // Subsystem 2 band is 14 to 15.
#define M02_CORE            (TP_PRC1)   // Core 0, as s2_test_b2_task.
#define M02_LEAD_IN_MSEC    (10000u)    // Lets the other tests finish.
#define M02_PAUSE_MSEC      (3000u)     // Before each motor starts.
#define M02_STEPS           (6u)
#define MSEC_PER_SECOND     (1000u)

typedef struct
{
    char const * p_name;
    int32_t      duty;          // Parts per 1000; 0 brakes.
    uint32_t     msec;
} m02_step_t;

typedef struct
{
    char const * p_name;
    uint32_t     motor;
    uint32_t     wheel;         // Encoder expected to move.
} m02_motor_t;

static m02_step_t const g_steps[M02_STEPS] =
{
    { "forwards 40 %",    400,  2000u },
    { "brake",              0,  1000u },
    { "backwards 40 %",  -400,  2000u },
    { "brake",              0,  1000u },
    { "forwards 100 %",  1000,  2000u },
    { "brake",              0,  1000u },
};

static m02_motor_t const g_motor_list[DRV_MOTORS] =
{
    { "LEFT motor (M1: GP8/GP9)",   DRV_MOTOR_LEFT,  DRV_ENCODER_LEFT  },
    { "RIGHT motor (M2: GP10/GP11)", DRV_MOTOR_RIGHT, DRV_ENCODER_RIGHT },
};

static void s2_test_motor_task(INT start_code, void * p_ext_info);
static void m02_run_motor(m02_motor_t const * p_motor);
static void m02_run_step(m02_motor_t const * p_motor,
                         m02_step_t const * p_step);


/*!
 * @brief Start the S2-M02 task.  Called at the end of subsystem2_test(),
 *        after the automatic checks that need the wheels to be still.
 *
 * @return E_OK, or the error from creating or starting the task.
 */
ER
s2_test_motor_start (void)
{
    // Cast: the kernel stores every task entry as FP.  TA_HLNG tasks are
    // called with (INT, void *), which is s2_test_motor_task()'s signature.
    //
    ID const task_id = app_start_task((FP) s2_test_motor_task,
                                      M02_TASK_PRIORITY, M02_CORE,
                                      "s2_test_motor_task");

    return ((task_id < E_OK) ? task_id : E_OK);
}

/*!
 * @brief Task body: wait for the lead-in, run both motors, brake, exit.
 *
 * @param[in] start_code  Unused.
 * @param[in] p_ext_info  Unused.
 */
static void
s2_test_motor_task (INT start_code, void * p_ext_info)
{
    (void) start_code;
    (void) p_ext_info;

    tm_printf((UB *) "[M02] motor test starts in %u s.  Car on a stand, "
              "wheels off the ground, battery on Vin.\n",
              (M02_LEAD_IN_MSEC / MSEC_PER_SECOND));
    (void) tk_dly_tsk(M02_LEAD_IN_MSEC);

    for (uint32_t index = 0u; index < DRV_MOTORS; index++)
    {
        m02_run_motor(&g_motor_list[index]);
    }

    // Both motors are braked by the last step; brake again in case a step
    // failed part-way.
    //
    (void) drv_motor_brake(DRV_MOTOR_LEFT);
    (void) drv_motor_brake(DRV_MOTOR_RIGHT);

    tm_printf((UB *) "[M02] done.  For each motor note which wheel turned "
              "and whether 'forwards' turned it the car's forward way.\n");
    tk_exd_tsk();
}

/*!
 * @brief Run every step on one motor.
 *
 * @param[in] p_motor  The motor and the encoder expected to move.
 */
static void
m02_run_motor (m02_motor_t const * p_motor)
{
    tm_printf((UB *) "[M02] %s in %u s: watch which wheel turns, and "
              "which way\n", p_motor->p_name,
              (M02_PAUSE_MSEC / MSEC_PER_SECOND));
    (void) tk_dly_tsk(M02_PAUSE_MSEC);

    for (uint32_t step = 0u; step < M02_STEPS; step++)
    {
        m02_run_step(p_motor, &g_steps[step]);
    }
}

/*!
 * @brief One step: set the duty, wait, then print the pulses counted.
 *
 * @param[in] p_motor  The motor and the encoder expected to move.
 * @param[in] p_step   The duty and how long to hold it.
 */
static void
m02_run_step (m02_motor_t const * p_motor, m02_step_t const * p_step)
{
    uint32_t const half_msec     = p_step->msec / 2u;
    uint32_t const start_left    = drv_encoder_get_count(DRV_ENCODER_LEFT);
    uint32_t const start_right   = drv_encoder_get_count(DRV_ENCODER_RIGHT);
    uint32_t const chatter_left  = drv_encoder_get_chatter(DRV_ENCODER_LEFT);
    uint32_t const chatter_right = drv_encoder_get_chatter(DRV_ENCODER_RIGHT);
    ER const       result        = drv_motor_set(p_motor->motor,
                                                 p_step->duty);

    if (E_OK != result)
    {
        tm_printf((UB *) "[M02] drv_motor_set error %d\n", result);
    }

    (void) tk_dly_tsk(half_msec);

    uint32_t const mid_left  = drv_encoder_get_count(DRV_ENCODER_LEFT);
    uint32_t const mid_right = drv_encoder_get_count(DRV_ENCODER_RIGHT);

    (void) tk_dly_tsk(half_msec);

    uint32_t const end_left  = drv_encoder_get_count(DRV_ENCODER_LEFT);
    uint32_t const end_right = drv_encoder_get_count(DRV_ENCODER_RIGHT);
    uint32_t const moved     = ((DRV_ENCODER_LEFT == p_motor->wheel) ?
                                (end_left - start_left) :
                                (end_right - start_right));
    uint32_t const other     = ((DRV_ENCODER_LEFT == p_motor->wheel) ?
                                (end_right - start_right) :
                                (end_left - start_left));

    tm_printf((UB *) "[M02]   %s: L=%lu R=%lu; second half %lu / %lu "
              "pulses/s; rejected L=%lu R=%lu\n", p_step->p_name,
              (end_left - start_left), (end_right - start_right),
              (((end_left - mid_left) * MSEC_PER_SECOND) / half_msec),
              (((end_right - mid_right) * MSEC_PER_SECOND) / half_msec),
              (drv_encoder_get_chatter(DRV_ENCODER_LEFT) - chatter_left),
              (drv_encoder_get_chatter(DRV_ENCODER_RIGHT) - chatter_right));

    // A driven step should move only this motor's own encoder.
    //
    if ((0 != p_step->duty) && ((0u == moved) || (other > moved)))
    {
        tm_printf((UB *) "[M02]   WARNING: the %s encoder did not lead.  "
                  "Motor and encoder sides may be swapped, or the motor "
                  "did not start.\n",
                  ((DRV_ENCODER_LEFT == p_motor->wheel) ? "LEFT" : "RIGHT"));
    }
}

#endif /* APP_TEST */

/*** end of file ***/

