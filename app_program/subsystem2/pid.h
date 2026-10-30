/** @file pid.h
 *
 * @brief Subsystem 2 PID controller: one instance per controlled value
 *        (task_motion runs one per wheel speed).
 *
 * The internals stay full precision in float (the team's data rules keep
 * PID memory out of the minimum-bit formats).  The derivative acts on the
 * measurement, not the error, so a setpoint step does not kick the output,
 * and the integral stops growing while the output is saturated in the
 * same direction (anti-windup).
 *
 * NOTE: The RP2040 has no FPU, so float maths runs in libgcc's software
 * routines.  A few dozen operations per 10 ms period cost microseconds.
 */

#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct
{
    float kp;               // Output units per unit of error.
    float ki;               // Output units per unit of error per second.
    float kd;               // Output units per unit of rate of change.
    float out_min;
    float out_max;
    float integral;         // Integral term, in output units.
    float prev_measured;    // For the derivative.
    bool  b_primed;         // False until the first update after a reset.
} pid_ctrl_t;

void  pid_init(pid_ctrl_t * p_pid, float kp, float ki, float kd,
               float out_min, float out_max);
void  pid_reset(pid_ctrl_t * p_pid);
float pid_update(pid_ctrl_t * p_pid, float setpoint, float measured,
                 float dt_sec);

#endif /* PID_H */

/*** end of file ***/

