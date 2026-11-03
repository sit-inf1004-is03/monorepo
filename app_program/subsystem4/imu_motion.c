/**
 * @file    imu_motion.c
 * @brief   Motion-classification region of the IMU statechart.
 *
 * The statechart draws Stationary -> Accelerating -> Cruising -> Turning ->
 * Stationary; in practice the car can go from any class to any other (e.g.
 * a pivot turn starts from rest), so any transition is allowed once the new
 * class has persisted.  Priority when several apply: TURNING, then
 * STATIONARY, then ACCELERATING, then CRUISING.
 */

#include <stddef.h>

#include "imu_motion.h"
#include "../common/bitpack.h"

#define ONE_G               (1.0f)

/* Layout of imu_motion_t.state_bits. */
#define CLASS_SHIFT         (0u)
#define CLASS_WIDTH         (2u)
#define CANDIDATE_SHIFT     (2u)
#define CANDIDATE_WIDTH     (2u)
#define COUNT_SHIFT         (4u)
#define COUNT_WIDTH         (3u)
#define COUNT_MAX           (7u)

static imu_motion_class_t classify (imu_motion_t const * p_motion,
                                    imu_motion_inputs_t const * p_in);
static uint32_t           get_field (imu_motion_t const * p_motion,
                                     uint32_t shift, uint32_t width);
static void               set_field (imu_motion_t * p_motion, uint32_t value,
                                     uint32_t shift, uint32_t width);

/**
 * @brief   Reset to STATIONARY with zero variance.
 * @param   p_motion  Classifier state.
 */
void
imu_motion_init (imu_motion_t * p_motion)
{
    if (NULL != p_motion)
    {
        p_motion->mean_g     = ONE_G;
        p_motion->var_g2     = 0.0f;
        p_motion->state_bits = 0u;
    }
}

/**
 * @brief   Classify one sample and apply the persistence rule.
 * @param   p_motion  Classifier state.
 * @param   p_in      Inputs for this sample.
 * @return  true if the accepted class changed on this sample.
 */
bool
imu_motion_update (imu_motion_t * p_motion, imu_motion_inputs_t const * p_in)
{
    bool b_changed = false;

    if ((NULL != p_motion) && (NULL != p_in))
    {
        /* Exponential running mean and variance of |a|. */
        float32_t const error = p_in->accel_mag_g - p_motion->mean_g;

        p_motion->mean_g += IMU_MOTION_VAR_GAIN * error;
        p_motion->var_g2 += IMU_MOTION_VAR_GAIN *
                            ((error * error) - p_motion->var_g2);

        /* Cast: enum values are 0..3, non-negative. */
        uint32_t const candidate = (uint32_t)classify(p_motion, p_in);
        uint32_t const current   = get_field(p_motion, CLASS_SHIFT,
                                             CLASS_WIDTH);
        uint32_t       count     = get_field(p_motion, COUNT_SHIFT,
                                             COUNT_WIDTH);

        if (candidate == current)
        {
            count = 0u;
        }
        else if (candidate != get_field(p_motion, CANDIDATE_SHIFT,
                                        CANDIDATE_WIDTH))
        {
            /* A different candidate restarts the persistence count. */
            count = 1u;
        }
        else
        {
            count = bitpack_clamp_u32(count + 1u, COUNT_MAX);
        }

        if (count >= IMU_MOTION_PERSIST_SAMPLES)
        {
            set_field(p_motion, candidate, CLASS_SHIFT, CLASS_WIDTH);
            count     = 0u;
            b_changed = true;
        }

        set_field(p_motion, candidate, CANDIDATE_SHIFT, CANDIDATE_WIDTH);
        set_field(p_motion, count, COUNT_SHIFT, COUNT_WIDTH);
    }

    return (b_changed);
}

/**
 * @brief   Currently accepted motion class.
 * @param   p_motion  Classifier state.
 * @return  The class.
 */
imu_motion_class_t
imu_motion_get_class (imu_motion_t const * p_motion)
{
    imu_motion_class_t result = IMU_MOTION_STATIONARY;

    if (NULL != p_motion)
    {
        /* Casts: the enum constants are 0..3, non-negative. */
        switch (get_field(p_motion, CLASS_SHIFT, CLASS_WIDTH))
        {
            case (uint32_t)IMU_MOTION_ACCELERATING:
                result = IMU_MOTION_ACCELERATING;
            break;

            case (uint32_t)IMU_MOTION_CRUISING:
                result = IMU_MOTION_CRUISING;
            break;

            case (uint32_t)IMU_MOTION_TURNING:
                result = IMU_MOTION_TURNING;
            break;

            default:
                result = IMU_MOTION_STATIONARY;
            break;
        }
    }

    return (result);
}

/**
 * @brief   Whether vibration shows the motors are running (used when no
 *          encoder is registered).
 * @param   p_motion  Classifier state.
 * @return  true if the acceleration variance is above the still level.
 */
bool
imu_motion_is_vibrating (imu_motion_t const * p_motion)
{
    return ((NULL != p_motion) &&
            (p_motion->var_g2 > IMU_MOTION_STILL_VAR_G2));
}

/**
 * @brief   Instantaneous class for one sample, before persistence.
 * @param   p_motion  Classifier state (variance).
 * @param   p_in      Inputs for this sample.
 * @return  Candidate class.
 */
static imu_motion_class_t
classify (imu_motion_t const * p_motion, imu_motion_inputs_t const * p_in)
{
    imu_motion_class_t result = IMU_MOTION_CRUISING;
    bool const b_quiet = (p_motion->var_g2 <= IMU_MOTION_STILL_VAR_G2);

    if (imu_math_abs(p_in->yaw_rate_dps) > IMU_MOTION_TURN_RATE_DPS)
    {
        result = IMU_MOTION_TURNING;
    }
    else if (!p_in->b_moving && b_quiet)
    {
        result = IMU_MOTION_STATIONARY;
    }
    else if (imu_math_abs(p_in->lin_fwd_g) > IMU_MOTION_ACCEL_G)
    {
        result = IMU_MOTION_ACCELERATING;
    }
    else
    {
        result = IMU_MOTION_CRUISING;
    }

    return (result);
}

/**
 * @brief   Read one field of the packed state byte.
 * @param   p_motion  Classifier state.
 * @param   shift     Field position.
 * @param   width     Field width.
 * @return  Field value.
 */
static uint32_t
get_field (imu_motion_t const * p_motion, uint32_t shift, uint32_t width)
{
    return (bitpack_extract(p_motion->state_bits, shift, width));
}

/**
 * @brief   Write one field of the packed state byte.
 *
 * Safe as a read-modify-write: the byte is private to the IMU task.
 *
 * @param   p_motion  Classifier state.
 * @param   value     Field value, already within the field width.
 * @param   shift     Field position.
 * @param   width     Field width.
 */
static void
set_field (imu_motion_t * p_motion, uint32_t value, uint32_t shift,
           uint32_t width)
{
    /* Cast: the packed fields occupy bits 0-6, so the word fits in 8 bits. */
    p_motion->state_bits = (uint8_t)bitpack_insert(p_motion->state_bits,
                                                   value, shift, width);
}

/*** end of file ***/
