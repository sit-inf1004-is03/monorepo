/**
 * @file    imu_filter.c
 * @brief   Sensor calibration and complementary-filter attitude estimate.
 *
 * Reference: complementary filter as in W. T. Higgins, "A Comparison of
 * Complementary and Kalman Filtering", IEEE Trans. AES-11(3), 1975.  A
 * Kalman filter was rejected in the Tribunal 1 review (Buddy 4, weak idea 2).
 */

#include <stddef.h>

#include "imu_filter.h"

#define ONE_G                   (1.0f)
#define MIN_ONE_G               (0.5f)
#define MAX_DT_S                (0.1f)
#define DEG_HALF_TURN           (180.0f)
#define DEG_QUARTER_TURN        (90.0f)

/* Mounting table built from the IMU_MOUNT_* settings in imu_filter.h. */
static uint32_t const  g_mount_axis[IMU_FILTER_AXES] =
{
    IMU_MOUNT_FWD_AXIS, IMU_MOUNT_LEFT_AXIS, IMU_MOUNT_UP_AXIS
};
static float32_t const g_mount_sign[IMU_FILTER_AXES] =
{
    IMU_MOUNT_FWD_SIGN, IMU_MOUNT_LEFT_SIGN, IMU_MOUNT_UP_SIGN
};
static float32_t const g_mag_offset[DRV_IMU_AXES] =
{
    IMU_MAG_OFFSET_X, IMU_MAG_OFFSET_Y, IMU_MAG_OFFSET_Z
};

static void      reset_sums (imu_filter_t * p_filter);
static bool      sample_is_still (imu_filter_t const * p_filter,
                                  imu_filter_input_t const * p_input);
static float32_t accel_magnitude (imu_filter_input_t const * p_input);
static float32_t accel_pitch_deg (imu_filter_input_t const * p_input);
static float32_t accel_roll_deg (imu_filter_input_t const * p_input);
static void      finish_calibration (imu_filter_t * p_filter);
static float32_t accel_correction (float32_t raw_error, bool b_mag_ok,
                                   bool b_still, float32_t dt_s,
                                   float32_t * p_error, uint8_t * p_disagree);
static void      update_with_gyro (imu_filter_t * p_filter,
                                   imu_filter_input_t const * p_input,
                                   float32_t pitch_acc, float32_t roll_acc,
                                   bool b_mag_ok, float32_t dt_s,
                                   bool b_still);
static void      update_accel_only (imu_filter_t * p_filter,
                                    float32_t accel_fwd_g,
                                    float32_t pitch_acc, float32_t roll_acc,
                                    bool b_mag_ok, float32_t dt_s);
static void      update_heading (imu_filter_t * p_filter,
                                 imu_filter_input_t const * p_input,
                                 float32_t dt_s);
static float32_t low_pass_gain (float32_t tau_s, float32_t dt_s);
static float32_t cos_deg (float32_t angle_deg);
static float32_t wrap_180 (float32_t angle_deg);

/**
 * @brief   Reset the filter to "not calibrated".
 * @param   p_filter  Filter state.
 */
void
imu_filter_init (imu_filter_t * p_filter)
{
    if (NULL != p_filter)
    {
        for (uint32_t axis = 0u; axis < IMU_FILTER_AXES; axis++)
        {
            p_filter->gyro_bias_dps[axis] = 0.0f;
            p_filter->mag_filt[axis]      = 0.0f;
        }

        p_filter->one_g            = ONE_G;
        p_filter->pitch_slow_deg   = 0.0f;
        p_filter->yaw_prev_deg     = 0.0f;
        p_filter->yaw_ref_deg      = 0.0f;
        p_filter->b_yaw_primed     = false;
        p_filter->pitch_offset_deg = 0.0f;
        p_filter->roll_offset_deg  = 0.0f;
        p_filter->calib_attempts   = 0u;
        p_filter->error_pitch      = 0.0f;
        p_filter->error_roll       = 0.0f;
        p_filter->disagree_pitch   = 0u;
        p_filter->disagree_roll    = 0u;
        p_filter->b_calibrated     = false;

        p_filter->attitude.pitch_deg       = 0.0f;
        p_filter->attitude.roll_deg        = 0.0f;
        p_filter->attitude.yaw_rate_dps    = 0.0f;
        p_filter->attitude.heading_deg     = 0.0f;
        p_filter->attitude.accel_mag_g     = ONE_G;
        p_filter->attitude.lin_fwd_g       = 0.0f;
        p_filter->attitude.b_accel_trusted = true;

        reset_sums(p_filter);
    }
}

/**
 * @brief   Convert a raw sensor sample to body axes and physical units.
 * @param   p_raw    Raw 16-bit counts in sensor axes.
 * @param   p_input  Receives g and deg/s in body axes.
 */
void
imu_filter_scale_raw (drv_imu_sample_t const * p_raw,
                      imu_filter_input_t * p_input)
{
    if ((NULL != p_raw) && (NULL != p_input))
    {
        for (uint32_t axis = 0u; axis < IMU_FILTER_AXES; axis++)
        {
            uint32_t const src = g_mount_axis[axis];

            /* Cast: int16_t to float32_t is exact. */
            p_input->accel_g[axis] = (g_mount_sign[axis] *
                                      (float32_t)p_raw->accel[src]) /
                                     DRV_IMU_ACCEL_LSB_PER_G;
            p_input->gyro_dps[axis] = (g_mount_sign[axis] *
                                       (float32_t)p_raw->gyro[src]) /
                                      DRV_IMU_GYRO_LSB_PER_DPS;
            p_input->mag[axis]      = g_mount_sign[axis] *
                                      ((float32_t)p_raw->mag[src] -
                                       g_mag_offset[src]);
        }

        p_input->b_has_gyro = p_raw->b_has_gyro;
        p_input->b_has_mag  = p_raw->b_has_mag;
    }
}

/**
 * @brief   Feed one sample to the start-up calibration.
 *
 * Collects IMU_FILTER_CALIB_SAMPLES still samples, then sets the gyro bias
 * and the level reference.  If the car moves, collection restarts, at most
 * IMU_FILTER_CALIB_MAX_ATTEMPTS times; after that the samples are used
 * anyway so the IMU never blocks the run (result IMU_FILTER_CALIB_FORCED).
 *
 * @param   p_filter  Filter state.
 * @param   p_input   Sample in body axes.
 * @return  Calibration progress.
 */
imu_filter_calib_t
imu_filter_calibrate (imu_filter_t * p_filter,
                      imu_filter_input_t const * p_input)
{
    imu_filter_calib_t result = IMU_FILTER_CALIB_BUSY;

    if ((NULL == p_filter) || (NULL == p_input))
    {
        result = IMU_FILTER_CALIB_BUSY;
    }
    else if (!sample_is_still(p_filter, p_input) &&
             (p_filter->calib_attempts < IMU_FILTER_CALIB_MAX_ATTEMPTS))
    {
        p_filter->calib_attempts++;
        reset_sums(p_filter);
        result = IMU_FILTER_CALIB_RESTART;
    }
    else
    {
        for (uint32_t axis = 0u; axis < IMU_FILTER_AXES; axis++)
        {
            if (0u == p_filter->calib_count)
            {
                p_filter->calib_ref_g[axis] = p_input->accel_g[axis];
            }

            p_filter->gyro_sum[axis]  += p_input->gyro_dps[axis];
            p_filter->accel_sum[axis] += p_input->accel_g[axis];
        }

        p_filter->calib_count++;

        if (p_filter->calib_count >= IMU_FILTER_CALIB_SAMPLES)
        {
            finish_calibration(p_filter);
            result = (p_filter->calib_attempts >=
                      IMU_FILTER_CALIB_MAX_ATTEMPTS) ?
                     IMU_FILTER_CALIB_FORCED : IMU_FILTER_CALIB_DONE;
        }
    }

    return (result);
}

/**
 * @brief   Advance the attitude estimate by one sample.
 * @param   p_filter  Calibrated filter state.
 * @param   p_input   Sample in body axes.
 * @param   dt_s      Time since the previous sample, (0, 0.1] s.
 * @param   b_still   true while the car is classified STATIONARY.
 * @return  true on success; false if not calibrated, @p dt_s is invalid,
 *          or the result was not finite (the attitude is then re-seeded
 *          from the accelerometer).
 */
bool
imu_filter_update (imu_filter_t * p_filter,
                   imu_filter_input_t const * p_input, float32_t dt_s,
                   bool b_still)
{
    bool b_ok = false;

    if ((NULL != p_filter) && (NULL != p_input) && p_filter->b_calibrated &&
        (dt_s > 0.0f) && (dt_s <= MAX_DT_S))
    {
        imu_filter_attitude_t * const p_att = &p_filter->attitude;
        imu_filter_input_t            input = *p_input;

        /* Scale the accelerometer so that |a| at rest is exactly 1 g. */
        for (uint32_t axis = 0u; axis < IMU_FILTER_AXES; axis++)
        {
            input.accel_g[axis] = input.accel_g[axis] / p_filter->one_g;
        }

        /* Wrapped: a board mounted upside down has a level roll near
         * +/-180 deg, where atan2 jumps by 360 deg (host test H-A10).
         */
        float32_t const pitch_acc = wrap_180(accel_pitch_deg(&input) -
                                             p_filter->pitch_offset_deg);
        float32_t const roll_acc  = wrap_180(accel_roll_deg(&input) -
                                             p_filter->roll_offset_deg);

        p_att->accel_mag_g = accel_magnitude(&input);

        bool const b_mag_ok = (imu_math_abs(p_att->accel_mag_g - ONE_G) <
                               IMU_FILTER_ACCEL_TRUST_G);

        if (input.b_has_gyro)
        {
            update_with_gyro(p_filter, &input, pitch_acc, roll_acc, b_mag_ok,
                             dt_s, b_still);
        }
        else
        {
            update_accel_only(p_filter, input.accel_g[IMU_FILTER_FWD],
                              pitch_acc, roll_acc, b_mag_ok, dt_s);
            update_heading(p_filter, &input, dt_s);
        }

        b_ok = imu_math_is_finite(p_att->pitch_deg) &&
               imu_math_is_finite(p_att->roll_deg) &&
               imu_math_is_finite(p_att->lin_fwd_g) &&
               imu_math_is_finite(p_att->accel_mag_g) &&
               imu_math_is_finite(p_att->yaw_rate_dps);

        if (!b_ok)
        {
            p_att->pitch_deg = imu_math_is_finite(pitch_acc) ? pitch_acc : 0.0f;
            p_att->roll_deg  = imu_math_is_finite(roll_acc) ? roll_acc : 0.0f;
            p_att->lin_fwd_g = 0.0f;
            p_att->yaw_rate_dps    = 0.0f;
            p_filter->pitch_slow_deg = p_att->pitch_deg;
            p_filter->b_yaw_primed   = false;
        }
    }

    return (b_ok);
}

/**
 * @brief   Slowly re-estimate the gyro bias while the car is stationary.
 *
 * Only axes whose rate is already near zero are adapted, so a slow pivot
 * that was misclassified as stationary cannot drag the bias away.
 *
 * @param   p_filter  Calibrated filter state.
 * @param   p_input   Current sample.
 */
void
imu_filter_track_bias (imu_filter_t * p_filter,
                       imu_filter_input_t const * p_input)
{
    if ((NULL != p_filter) && (NULL != p_input) && p_filter->b_calibrated)
    {
        for (uint32_t axis = 0u; axis < IMU_FILTER_AXES; axis++)
        {
            float32_t const error = p_input->gyro_dps[axis] -
                                    p_filter->gyro_bias_dps[axis];

            if (imu_math_abs(error) < IMU_FILTER_BIAS_STILL_DPS)
            {
                p_filter->gyro_bias_dps[axis] += IMU_FILTER_BIAS_GAIN * error;
            }
        }
    }
}

/**
 * @brief   Carry on after a sensor dropout without measuring level again.
 *
 * The car may be on a slope when the sensor comes back, so the level
 * reference (offsets, 1 g scale, gyro bias) is kept.  Calibrating there
 * would make the slope read as level and level ground read as a slope.
 * Only the state that is stale after a gap is cleared.
 *
 * @param   p_filter  Filter state.
 * @return  true if the filter can carry on; false if it never calibrated or
 *          its reference is not finite, so it must be calibrated.
 */
bool
imu_filter_resume (imu_filter_t * p_filter)
{
    bool b_ok = false;

    if ((NULL != p_filter) && p_filter->b_calibrated &&
        imu_math_is_finite(p_filter->pitch_offset_deg) &&
        imu_math_is_finite(p_filter->roll_offset_deg) &&
        imu_math_is_finite(p_filter->one_g))
    {
        imu_filter_attitude_t * const p_att = &p_filter->attitude;

        if (!imu_math_is_finite(p_att->pitch_deg))
        {
            p_att->pitch_deg = 0.0f;
        }

        if (!imu_math_is_finite(p_att->roll_deg))
        {
            p_att->roll_deg = 0.0f;
        }

        p_att->lin_fwd_g         = 0.0f;
        p_att->yaw_rate_dps      = 0.0f;
        p_filter->pitch_slow_deg = p_att->pitch_deg;
        p_filter->error_pitch    = 0.0f;
        p_filter->error_roll     = 0.0f;
        p_filter->disagree_pitch = 0u;
        p_filter->disagree_roll  = 0u;
        p_filter->b_yaw_primed   = false;
        b_ok = true;
    }

    return (b_ok);
}

/**
 * @brief   Report whether calibration has finished.
 * @param   p_filter  Filter state.
 * @return  true once calibrated.
 */
bool
imu_filter_is_calibrated (imu_filter_t const * p_filter)
{
    return ((NULL != p_filter) && p_filter->b_calibrated);
}

/**
 * @brief   Read-only access to the latest attitude.
 * @param   p_filter  Filter state.
 * @return  Pointer to the attitude inside @p p_filter, or NULL.
 */
imu_filter_attitude_t const *
imu_filter_get_attitude (imu_filter_t const * p_filter)
{
    imu_filter_attitude_t const * p_att = NULL;

    if (NULL != p_filter)
    {
        p_att = &p_filter->attitude;
    }

    return (p_att);
}

/**
 * @brief   Clear the calibration accumulators.
 * @param   p_filter  Filter state.
 */
static void
reset_sums (imu_filter_t * p_filter)
{
    for (uint32_t axis = 0u; axis < IMU_FILTER_AXES; axis++)
    {
        p_filter->gyro_sum[axis]    = 0.0f;
        p_filter->accel_sum[axis]   = 0.0f;
        p_filter->calib_ref_g[axis] = 0.0f;
    }

    p_filter->calib_count = 0u;
}

/**
 * @brief   Decide whether a calibration sample was taken while still.
 *
 * Judged by change, not by |a| = 1 g: a cheap sensor can read well off 1 g
 * at rest (0.87 g on the GY-511 tested), which made every attempt fail and
 * the calibration always "forced".  Comparing whole vectors also catches a
 * push along the floor, which barely changes |a|.
 *
 * @param   p_filter  Filter state (the collection's first sample).
 * @param   p_input   Sample in body axes.
 * @return  true if |a| is plausible, the accelerometer is within
 *          IMU_FILTER_CALIB_STEADY_G of the first sample on every axis, and
 *          every gyro axis is small.
 */
static bool
sample_is_still (imu_filter_t const * p_filter,
                 imu_filter_input_t const * p_input)
{
    bool b_still = (imu_math_abs(accel_magnitude(p_input) - ONE_G) <
                    IMU_FILTER_CALIB_ACCEL_TOL_G);

    for (uint32_t axis = 0u; axis < IMU_FILTER_AXES; axis++)
    {
        if (imu_math_abs(p_input->gyro_dps[axis]) >=
            IMU_FILTER_CALIB_GYRO_MAX_DPS)
        {
            b_still = false;
        }

        /* The first sample of a collection is the reference itself. */
        if ((p_filter->calib_count > 0u) &&
            (imu_math_abs(p_input->accel_g[axis] -
                          p_filter->calib_ref_g[axis]) >=
             IMU_FILTER_CALIB_STEADY_G))
        {
            b_still = false;
        }
    }

    return (b_still);
}

/**
 * @brief   Length of the acceleration vector.
 * @param   p_input  Sample in body axes.
 * @return  |a| in g.
 */
static float32_t
accel_magnitude (imu_filter_input_t const * p_input)
{
    float32_t const ax = p_input->accel_g[IMU_FILTER_FWD];
    float32_t const ay = p_input->accel_g[IMU_FILTER_LEFT];
    float32_t const az = p_input->accel_g[IMU_FILTER_UP];

    return (imu_math_sqrt((ax * ax) + (ay * ay) + (az * az)));
}

/**
 * @brief   Pitch implied by the gravity direction alone.
 * @param   p_input  Sample in body axes.
 * @return  Pitch in degrees, nose up positive.
 */
static float32_t
accel_pitch_deg (imu_filter_input_t const * p_input)
{
    float32_t const ay = p_input->accel_g[IMU_FILTER_LEFT];
    float32_t const az = p_input->accel_g[IMU_FILTER_UP];

    return (imu_math_atan2_deg(p_input->accel_g[IMU_FILTER_FWD],
                               imu_math_sqrt((ay * ay) + (az * az))));
}

/**
 * @brief   Roll implied by the gravity direction alone.
 * @param   p_input  Sample in body axes.
 * @return  Roll in degrees, left side up positive.
 */
static float32_t
accel_roll_deg (imu_filter_input_t const * p_input)
{
    return (imu_math_atan2_deg(p_input->accel_g[IMU_FILTER_LEFT],
                               p_input->accel_g[IMU_FILTER_UP]));
}

/**
 * @brief   Turn the accumulated sums into bias and level reference.
 * @param   p_filter  Filter state with calib_count samples summed.
 */
static void
finish_calibration (imu_filter_t * p_filter)
{
    imu_filter_input_t mean;

    /* Cast: calib_count <= IMU_FILTER_CALIB_SAMPLES, exact in float. */
    float32_t const count = (float32_t)p_filter->calib_count;

    for (uint32_t axis = 0u; axis < IMU_FILTER_AXES; axis++)
    {
        p_filter->gyro_bias_dps[axis] = p_filter->gyro_sum[axis] / count;
        mean.accel_g[axis]            = p_filter->accel_sum[axis] / count;
        mean.gyro_dps[axis]           = 0.0f;
    }

    float32_t const mean_g = accel_magnitude(&mean);

    p_filter->one_g            = (mean_g > MIN_ONE_G) ? mean_g : ONE_G;
    p_filter->pitch_offset_deg = accel_pitch_deg(&mean);
    p_filter->roll_offset_deg  = accel_roll_deg(&mean);
    p_filter->pitch_slow_deg       = 0.0f;
    p_filter->b_yaw_primed         = false;
    p_filter->attitude.pitch_deg   = 0.0f;
    p_filter->attitude.roll_deg    = 0.0f;
    p_filter->attitude.heading_deg = 0.0f;
    p_filter->b_calibrated = true;
    reset_sums(p_filter);
}

/**
 * @brief   Accelerometer correction for one axis of the complementary
 *          filter: gain times the smoothed disagreement, rate-limited.
 * @param   raw_error   Accelerometer angle minus gyro-propagated angle.
 * @param   b_mag_ok    |a| close to 1 g.
 * @param   b_still     Car classified STATIONARY.
 * @param   dt_s        Sample interval.
 * @param   p_error     Smoothed disagreement; updated.
 * @param   p_disagree  Saturating count of disagreeing samples; updated.
 * @return  Degrees to add to the gyro-propagated angle.
 */
static float32_t
accel_correction (float32_t raw_error, bool b_mag_ok, bool b_still,
                  float32_t dt_s, float32_t * p_error, uint8_t * p_disagree)
{
    float32_t gain  = 0.0f;
    float32_t limit = IMU_FILTER_MAX_CORR_DPS * dt_s;

    /* Jolts (|a| far from 1 g) are kept out of the average. */
    if (b_mag_ok)
    {
        *p_error += IMU_FILTER_ERROR_GAIN * (raw_error - *p_error);
    }

    float32_t const error   = *p_error;
    bool const      b_agree = (imu_math_abs(error) < IMU_FILTER_AGREE_DEG);

    if (b_agree)
    {
        *p_disagree = 0u;
    }
    else if (b_mag_ok && (*p_disagree < UINT8_MAX))
    {
        (*p_disagree)++;
    }
    else
    {
        /* Jolt, or the counter is already saturated. */
    }

    if (!b_mag_ok)
    {
        gain = 0.0f;
    }
    else if (b_still)
    {
        gain = 1.0f - IMU_FILTER_ALPHA_STILL;
    }
    else if (b_agree)
    {
        gain = 1.0f - IMU_FILTER_ALPHA;
    }
    else if (*p_disagree >= IMU_FILTER_RESYNC_SAMPLES)
    {
        gain  = 1.0f - IMU_FILTER_ALPHA_STILL;
        limit = IMU_FILTER_RESYNC_DPS * dt_s;
    }
    else
    {
        /* Linear acceleration: trust the gyro alone. */
        gain = 0.0f;
    }

    return (imu_math_clamp(gain * error, -limit, limit));
}

/**
 * @brief   Complementary-filter step (sensor with a gyroscope).
 * @param   p_filter   Filter state.
 * @param   p_input    Sample, accelerometer already scaled to 1 g.
 * @param   pitch_acc  Accelerometer pitch relative to the level reference.
 * @param   roll_acc   Accelerometer roll relative to the level reference.
 * @param   b_mag_ok   |a| close to 1 g.
 * @param   dt_s       Sample interval.
 * @param   b_still    Car classified STATIONARY.
 */
static void
update_with_gyro (imu_filter_t * p_filter, imu_filter_input_t const * p_input,
                  float32_t pitch_acc, float32_t roll_acc, bool b_mag_ok,
                  float32_t dt_s, bool b_still)
{
    imu_filter_attitude_t * const p_att = &p_filter->attitude;
    float32_t const rate_fwd  = p_input->gyro_dps[IMU_FILTER_FWD] -
                                p_filter->gyro_bias_dps[IMU_FILTER_FWD];
    float32_t const rate_left = p_input->gyro_dps[IMU_FILTER_LEFT] -
                                p_filter->gyro_bias_dps[IMU_FILTER_LEFT];
    float32_t const rate_up   = p_input->gyro_dps[IMU_FILTER_UP] -
                                p_filter->gyro_bias_dps[IMU_FILTER_UP];

    /* Gyro propagation; nose-up is a negative rotation about +Y. */
    float32_t const pitch_gyro = p_att->pitch_deg - (rate_left * dt_s);
    float32_t const roll_gyro  = p_att->roll_deg + (rate_fwd * dt_s);

    float32_t const corr_pitch = accel_correction(
        pitch_acc - pitch_gyro, b_mag_ok, b_still, dt_s,
        &p_filter->error_pitch, &p_filter->disagree_pitch);
    float32_t const corr_roll  = accel_correction(
        roll_acc - roll_gyro, b_mag_ok, b_still, dt_s,
        &p_filter->error_roll, &p_filter->disagree_roll);

    p_att->b_accel_trusted = (imu_math_abs(corr_pitch) > 0.0f);
    p_att->pitch_deg       = pitch_gyro + corr_pitch;
    p_att->roll_deg        = roll_gyro + corr_roll;
    p_att->yaw_rate_dps = rate_up;
    p_att->heading_deg  = imu_math_wrap_360(p_att->heading_deg +
                                            (rate_up * dt_s));

    /* ax = sin(true sensor pitch) + forward acceleration, in g. */
    p_att->lin_fwd_g = p_input->accel_g[IMU_FILTER_FWD] -
                       imu_math_sin_deg(p_att->pitch_deg +
                                        p_filter->pitch_offset_deg);
}

/**
 * @brief   Accelerometer-only tilt step (no gyroscope): low-passed
 *          accelerometer angles, held during jolts.
 *
 * lin_fwd_g is measured against the slow pitch reference, so it shows
 * accelerations shorter than about IMU_FILTER_SLOW_TAU_S.
 *
 * @param   p_filter     Filter state.
 * @param   accel_fwd_g  Forward accelerometer reading, scaled to 1 g.
 * @param   pitch_acc    Accelerometer pitch relative to the level reference.
 * @param   roll_acc     Accelerometer roll relative to the level reference.
 * @param   b_mag_ok     |a| close to 1 g.
 * @param   dt_s         Sample interval.
 */
static void
update_accel_only (imu_filter_t * p_filter, float32_t accel_fwd_g,
                   float32_t pitch_acc, float32_t roll_acc, bool b_mag_ok,
                   float32_t dt_s)
{
    imu_filter_attitude_t * const p_att = &p_filter->attitude;
    float32_t const gain      = low_pass_gain(IMU_FILTER_TILT_TAU_S, dt_s);
    float32_t const slow_gain = low_pass_gain(IMU_FILTER_SLOW_TAU_S, dt_s);

    /* Jolts (|a| far from 1 g) are not gravity: hold the last angles. */
    if (b_mag_ok)
    {
        p_att->pitch_deg         += gain * (pitch_acc - p_att->pitch_deg);
        p_att->roll_deg          += gain * (roll_acc - p_att->roll_deg);
        p_filter->pitch_slow_deg += slow_gain *
                                    (pitch_acc - p_filter->pitch_slow_deg);
    }

    p_att->b_accel_trusted = b_mag_ok;

    /* The slow reference stands in for the true pitch, so a short
     * acceleration remains visible.
     */
    p_att->lin_fwd_g = accel_fwd_g -
                       imu_math_sin_deg(p_filter->pitch_slow_deg +
                                        p_filter->pitch_offset_deg);
}

/**
 * @brief   Yaw rate and heading from the tilt-compensated compass (used
 *          when there is no gyroscope).
 *
 * Body axes: X forward, Y left, Z up; pitch nose up, roll left side up.
 * Rotating the field into the level frame:
 *   xh = mx cos(p) - (my sin(r) + mz cos(r)) sin(p)
 *   yh = my cos(r) - mz sin(r)
 * North lies atan2(yh, xh) counter-clockwise of the nose, so turning left
 * (positive yaw) makes that angle fall: yaw = -atan2(yh, xh).
 *
 * @param   p_filter  Filter state, attitude already updated.
 * @param   p_input   Sample in body axes.
 * @param   dt_s      Sample interval.
 */
static void
update_heading (imu_filter_t * p_filter, imu_filter_input_t const * p_input,
                float32_t dt_s)
{
    imu_filter_attitude_t * const p_att = &p_filter->attitude;

    if (!p_input->b_has_mag)
    {
        p_att->yaw_rate_dps = 0.0f;
    }
    else
    {
        for (uint32_t axis = 0u; axis < IMU_FILTER_AXES; axis++)
        {
            p_filter->mag_filt[axis] = p_filter->b_yaw_primed ?
                (p_filter->mag_filt[axis] + (IMU_FILTER_MAG_GAIN *
                 (p_input->mag[axis] - p_filter->mag_filt[axis]))) :
                p_input->mag[axis];
        }

        float32_t const pitch = p_att->pitch_deg + p_filter->pitch_offset_deg;
        float32_t const roll  = p_att->roll_deg + p_filter->roll_offset_deg;
        float32_t const mx    = p_filter->mag_filt[IMU_FILTER_FWD];
        float32_t const my    = p_filter->mag_filt[IMU_FILTER_LEFT];
        float32_t const mz    = p_filter->mag_filt[IMU_FILTER_UP];
        float32_t const xh    = (mx * cos_deg(pitch)) -
                                (((my * imu_math_sin_deg(roll)) +
                                  (mz * cos_deg(roll))) *
                                 imu_math_sin_deg(pitch));
        float32_t const yh    = (my * cos_deg(roll)) -
                                (mz * imu_math_sin_deg(roll));

        if (imu_math_sqrt((xh * xh) + (yh * yh)) < IMU_FILTER_MAG_MIN_COUNTS)
        {
            /* No usable horizontal field (compass not working). */
            p_att->yaw_rate_dps = 0.0f;
        }
        else
        {
            float32_t const yaw = -imu_math_atan2_deg(yh, xh);

            if (!p_filter->b_yaw_primed)
            {
                p_filter->yaw_ref_deg  = yaw;
                p_filter->b_yaw_primed = true;
                p_att->yaw_rate_dps    = 0.0f;
            }
            else
            {
                float32_t const rate = wrap_180(yaw - p_filter->yaw_prev_deg) /
                                       dt_s;

                p_att->yaw_rate_dps += IMU_FILTER_YAW_RATE_GAIN *
                                       (rate - p_att->yaw_rate_dps);
            }

            p_filter->yaw_prev_deg = yaw;
            p_att->heading_deg     = imu_math_wrap_360(yaw -
                                                       p_filter->yaw_ref_deg);
        }
    }
}

/**
 * @brief   Per-sample gain of a first-order low-pass filter.
 * @param   tau_s  Time constant.
 * @param   dt_s   Sample interval, > 0.
 * @return  dt / (tau + dt), in (0, 1].
 */
static float32_t
low_pass_gain (float32_t tau_s, float32_t dt_s)
{
    return (dt_s / (tau_s + dt_s));
}

/**
 * @brief   Cosine of an angle in degrees.
 * @param   angle_deg  Angle.
 * @return  cos(angle).
 */
static float32_t
cos_deg (float32_t angle_deg)
{
    return (imu_math_sin_deg(angle_deg + DEG_QUARTER_TURN));
}

/**
 * @brief   Wrap an angle into [-180, 180).
 * @param   angle_deg  Angle.
 * @return  Wrapped angle.
 */
static float32_t
wrap_180 (float32_t angle_deg)
{
    float32_t const wrapped = imu_math_wrap_360(angle_deg + DEG_HALF_TURN);

    return (wrapped - DEG_HALF_TURN);
}

/*** end of file ***/
