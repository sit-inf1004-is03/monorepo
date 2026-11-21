/*!
 * @file    imu_flip.c
 * @brief   Upside-down detection from the raw gravity direction (see
 *          imu_flip.h).  Pure logic: no kernel calls, host tested.
 */

#include "imu_flip.h"

/*!
 * @brief  Start with the car upright and both counts at zero.
 *
 * @param[out] p_flip  State to clear.
 */
void
imu_flip_init (imu_flip_t * p_flip)
{
    p_flip->on_count      = 0u;
    p_flip->off_count     = 0u;
    p_flip->b_upside_down = false;
}

/*!
 * @brief  Take one sample.
 *
 * @param[in,out] p_flip  State.
 * @param[in]     up_g    Gravity along the car's up axis, in g: about +1
 *                        upright, about -1 upside down.
 * @param[in]     mag_g   |a| of the same sample, in g.
 *
 * @return true if imu_flip_is_upside_down() just changed.
 */
bool
imu_flip_update (imu_flip_t * p_flip, float32_t up_g, float32_t mag_g)
{
    bool const b_was = p_flip->b_upside_down;

    if ((!imu_math_is_finite(up_g)) || (!imu_math_is_finite(mag_g)) ||
        (mag_g < IMU_FLIP_MIN_G) || (mag_g > IMU_FLIP_MAX_G))
    {
        // A knock or a fall: no direction to trust.
        //
        p_flip->on_count  = 0u;
        p_flip->off_count = 0u;
    }
    else
    {
        // mag_g >= IMU_FLIP_MIN_G > 0 here, so the division is safe.
        //
        float32_t const cos_up = up_g / mag_g;

        if (cos_up < IMU_FLIP_ON_COS)
        {
            p_flip->off_count = 0u;
            if (p_flip->on_count < IMU_FLIP_ON_SAMPLES)
            {
                p_flip->on_count++;
            }
            if (p_flip->on_count >= IMU_FLIP_ON_SAMPLES)
            {
                p_flip->b_upside_down = true;
            }
        }
        else if (cos_up > IMU_FLIP_OFF_COS)
        {
            p_flip->on_count = 0u;
            if (p_flip->off_count < IMU_FLIP_OFF_SAMPLES)
            {
                p_flip->off_count++;
            }
            if (p_flip->off_count >= IMU_FLIP_OFF_SAMPLES)
            {
                p_flip->b_upside_down = false;
            }
        }
        else
        {
            // Between the two angles: keep the state, start both counts
            // again.
            //
            p_flip->on_count  = 0u;
            p_flip->off_count = 0u;
        }
    }

    return (b_was != p_flip->b_upside_down);
}

/*!
 * @brief  True while the car is upside down.
 *
 * @param[in] p_flip  State.
 */
bool
imu_flip_is_upside_down (imu_flip_t const * p_flip)
{
    return (p_flip->b_upside_down);
}

/*** end of file ***/
