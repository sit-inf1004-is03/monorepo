/**
 * @file    net_backoff.c
 * @brief   Retry timing for the network bridge: exponential backoff with
 *          jitter.  Coded to BARR-C:2018.
 *
 * The wait doubles after each failure in a row, up to max_ms.  Each wait
 * is randomised to between half and all of the current step ("equal
 * jitter"), so cars that lost the same access point or broker do not all
 * retry at the same moment.
 */

#include "net_backoff.h"

/**
 * @brief   Set the limits and start from the shortest wait.
 * @param   p_backoff  The layer's timing.
 * @param   min_ms     First wait.
 * @param   max_ms     Longest wait.
 */
void
net_backoff_init (net_backoff_t * p_backoff, uint32_t min_ms,
                  uint32_t max_ms)
{
    p_backoff->min_ms  = min_ms;
    p_backoff->max_ms  = max_ms;
    p_backoff->step_ms = min_ms;
}

/**
 * @brief   Start again from the shortest wait, after a success.
 * @param   p_backoff  The layer's timing.
 */
void
net_backoff_reset (net_backoff_t * p_backoff)
{
    p_backoff->step_ms = p_backoff->min_ms;
}

/**
 * @brief   The wait before the next attempt, after one more failure.
 * @param   p_backoff  The layer's timing; its step doubles, up to max_ms.
 * @param   random     Any random number.  Passed in so that the tests can
 *                     fix it.
 * @return  A wait between half and all of the current step, in ms.
 */
uint32_t
net_backoff_next (net_backoff_t * p_backoff, uint32_t random)
{
    uint32_t const step = p_backoff->step_ms;
    uint32_t const half = step / 2u;

    if (step >= (p_backoff->max_ms / 2u))
    {
        p_backoff->step_ms = p_backoff->max_ms;
    }
    else
    {
        p_backoff->step_ms = step * 2u;
    }

    return (half + (random % ((step - half) + 1u)));
}

/*** end of file ***/
