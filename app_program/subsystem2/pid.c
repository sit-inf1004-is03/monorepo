/** @file pid.c
 *
 * @brief Subsystem 2 PID controller (see pid.h).
 *
 * An instance belongs to the one task that calls pid_update() on it; the
 * functions keep no shared state.
 */

#include <stdbool.h>

#include "pid.h"

static float pid_clamp(float value, float low, float high);


/*!
 * @brief Set an instance's gains and output limits, and clear its memory.
 *
 * @param[out] p_pid    The instance.
 * @param[in]  kp       Proportional gain.
 * @param[in]  ki       Integral gain, per second.
 * @param[in]  kd       Derivative gain, in seconds.
 * @param[in]  out_min  Lowest output.
 * @param[in]  out_max  Highest output; must be above out_min.
 */
void
pid_init (pid_ctrl_t * p_pid, float kp, float ki, float kd, float out_min,
          float out_max)
{
    p_pid->kp      = kp;
    p_pid->ki      = ki;
    p_pid->kd      = kd;
    p_pid->out_min = out_min;
    p_pid->out_max = out_max;
    pid_reset(p_pid);
}

/*!
 * @brief Clear the controller's memory and keep its gains ("Reset Metric
 *        Values" in the design).  Call at the start of every move.
 *
 * @param[in,out] p_pid  The instance.
 */
void
pid_reset (pid_ctrl_t * p_pid)
{
    p_pid->integral      = 0.0f;
    p_pid->prev_measured = 0.0f;
    p_pid->b_primed      = false;
}

/*!
 * @brief Run one control step.
 *
 * @param[in,out] p_pid     The instance.
 * @param[in]     setpoint  Wanted value.
 * @param[in]     measured  Measured value.
 * @param[in]     dt_sec    Time since the last step, in seconds.  If it is
 *                          not above 0, only the P and stored I terms act.
 *
 * @return The output, between out_min and out_max.
 */
float
pid_update (pid_ctrl_t * p_pid, float setpoint, float measured, float dt_sec)
{
    float const error      = setpoint - measured;
    float const p_term     = p_pid->kp * error;
    float       d_term     = 0.0f;
    float       integral   = p_pid->integral;

    if (dt_sec > 0.0f)
    {
        float const i_step    = p_pid->ki * error * dt_sec;
        float const unclamped = p_term + integral + i_step;

        // Anti-windup: integrate unless that pushes further into a limit.
        //
        if (!(((unclamped > p_pid->out_max) && (error > 0.0f)) ||
              ((unclamped < p_pid->out_min) && (error < 0.0f))))
        {
            integral = pid_clamp((integral + i_step), p_pid->out_min,
                                 p_pid->out_max);
        }

        if (p_pid->b_primed)
        {
            d_term = -p_pid->kd * ((measured - p_pid->prev_measured) /
                                   dt_sec);
        }

        p_pid->prev_measured = measured;
        p_pid->b_primed      = true;
    }

    p_pid->integral = integral;

    return (pid_clamp((p_term + integral + d_term), p_pid->out_min,
                      p_pid->out_max));
}

/*!
 * @brief Limit a value to a range.
 */
static float
pid_clamp (float value, float low, float high)
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

