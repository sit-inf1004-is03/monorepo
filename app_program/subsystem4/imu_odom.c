/**
 * @file    imu_odom.c
 * @brief   Distance travelled per sample from the wheel-encoder words.
 */

#include <stddef.h>

#include "imu_odom.h"

static uint32_t wheel_pulses (imu_odom_t * p_odom, uint32_t now_count,
                              uint32_t * p_last);

/**
 * @brief   Reset the odometry state.
 * @param   p_odom  Odometry state.
 */
void
imu_odom_init (imu_odom_t * p_odom)
{
    if (NULL != p_odom)
    {
        p_odom->last_left        = 0u;
        p_odom->last_right       = 0u;
        p_odom->idle_samples     = IMU_ODOM_IDLE_SAMPLES;
        p_odom->moving_samples   = 0u;
        p_odom->glitch_count     = 0u;
        p_odom->b_primed         = false;
        p_odom->b_moving         = false;
        p_odom->b_using_fallback = false;
    }
}

/**
 * @brief   Wrap-safe difference of two 32-bit encoder counts (IF-06).
 *
 * Example: last = 0xFFFFFFFE, now = 0x00000002 gives 4.  The count only
 * wraps at 2^32 (~1380 km), so the per-sample difference is always exact.
 *
 * @param   now_count   Current encoder word.
 * @param   last_count  Encoder word at the previous call.
 * @return  Pulses counted in between.
 */
uint32_t
imu_odom_delta (uint32_t now_count, uint32_t last_count)
{
    return (((now_count & IMU_ODOM_COUNT_MASK) -
             (last_count & IMU_ODOM_COUNT_MASK)) & IMU_ODOM_COUNT_MASK);
}

/**
 * @brief   Distance travelled since the previous sample.
 *
 * Uses the mean of the two wheels when both words are registered, one
 * wheel when only one is, and the fallback speed when none is.
 *
 * @param   p_odom              Odometry state.
 * @param   p_counts            Encoder words read this sample.
 * @param   dt_s                Sample interval in seconds.
 * @param   b_vibration_moving  IMU's own "moving" guess (fallback only).
 * @return  Distance in mm, >= 0.
 */
float32_t
imu_odom_update (imu_odom_t * p_odom, imu_odom_counts_t const * p_counts,
                 float32_t dt_s, bool b_vibration_moving)
{
    float32_t distance_mm = 0.0f;

    if ((NULL == p_odom) || (NULL == p_counts))
    {
        distance_mm = 0.0f;
    }
    else if (!p_counts->b_has_left && !p_counts->b_has_right)
    {
        p_odom->b_using_fallback = true;
        p_odom->b_moving         = b_vibration_moving;

        if (b_vibration_moving && (dt_s > 0.0f))
        {
            distance_mm = IMU_ODOM_FALLBACK_MM_PER_S * dt_s;
        }
    }
    else
    {
        uint32_t pulses = 0u;
        uint32_t wheels = 0u;

        p_odom->b_using_fallback = false;

        if (!p_odom->b_primed)
        {
            /* First sample: remember the counts, report no movement. */
            p_odom->last_left  = p_counts->left_count;
            p_odom->last_right = p_counts->right_count;
            p_odom->b_primed   = true;
        }

        if (p_counts->b_has_left)
        {
            pulses += wheel_pulses(p_odom, p_counts->left_count,
                                   &p_odom->last_left);
            wheels++;
        }

        if (p_counts->b_has_right)
        {
            pulses += wheel_pulses(p_odom, p_counts->right_count,
                                   &p_odom->last_right);
            wheels++;
        }

        if (0u == pulses)
        {
            if (p_odom->idle_samples < IMU_ODOM_IDLE_SAMPLES)
            {
                p_odom->idle_samples++;
            }
        }
        else
        {
            p_odom->idle_samples = 0u;
        }

        p_odom->b_moving = (p_odom->idle_samples < IMU_ODOM_IDLE_SAMPLES);

        /* Cast: pulses <= 2 * IMU_ODOM_MAX_PULSES and wheels is 1 or 2,
         * both exact in float.
         */
        distance_mm = ((float32_t)pulses * IMU_ODOM_MM_PER_PULSE) /
                      (float32_t)wheels;
    }

    if (NULL != p_odom)
    {
        if (!p_odom->b_moving)
        {
            p_odom->moving_samples = 0u;
        }
        else if (p_odom->moving_samples < IMU_ODOM_SETTLE_SAMPLES)
        {
            p_odom->moving_samples++;
        }
        else
        {
            /* Settled; the counter saturates. */
        }
    }

    return (distance_mm);
}

/**
 * @brief   Whether the car has been moving long enough for the start-up
 *          acceleration to be over (used by the accelerometer-only mode).
 * @param   p_odom  Odometry state.
 * @return  true after IMU_ODOM_SETTLE_SAMPLES samples of movement.
 */
bool
imu_odom_is_settled (imu_odom_t const * p_odom)
{
    return ((NULL != p_odom) && p_odom->b_moving &&
            (p_odom->moving_samples >= IMU_ODOM_SETTLE_SAMPLES));
}

/**
 * @brief   Whether the encoders (or the fallback guess) show movement.
 * @param   p_odom  Odometry state.
 * @return  true if a pulse was seen within the last 200 ms.
 */
bool
imu_odom_is_moving (imu_odom_t const * p_odom)
{
    return ((NULL != p_odom) && p_odom->b_moving);
}

/**
 * @brief   Whether the last update used the fallback speed.
 * @param   p_odom  Odometry state.
 * @return  true if no encoder word is registered.
 */
bool
imu_odom_is_fallback (imu_odom_t const * p_odom)
{
    return ((NULL == p_odom) || p_odom->b_using_fallback);
}

/**
 * @brief   Number of implausible jumps that were discarded.
 * @param   p_odom  Odometry state.
 * @return  Glitch count.
 */
uint32_t
imu_odom_glitches (imu_odom_t const * p_odom)
{
    return ((NULL != p_odom) ? p_odom->glitch_count : 0u);
}

/**
 * @brief   Pulses of one wheel since its last reading; jumps larger than
 *          IMU_ODOM_MAX_PULSES (e.g. a counter reset) are discarded.
 * @param   p_odom     Odometry state (glitch counter).
 * @param   now_count  Current encoder word.
 * @param   p_last     Previous encoder word; updated.
 * @return  Valid pulse count for this sample.
 */
static uint32_t
wheel_pulses (imu_odom_t * p_odom, uint32_t now_count, uint32_t * p_last)
{
    uint32_t pulses = imu_odom_delta(now_count, *p_last);

    *p_last = now_count;

    if (pulses > IMU_ODOM_MAX_PULSES)
    {
        p_odom->glitch_count++;
        pulses = 0u;
    }

    return (pulses);
}

/*** end of file ***/
