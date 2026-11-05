/*!
 * @file    imu_ramp.c
 * @brief   Ramp detection from imu_task's pitch and roll (see imu_ramp.h).
 *
 * Ramp on at IMU_RAMP_ON_DEG for IMU_ON_SAMPLES samples, off at
 * IMU_RAMP_OFF_DEG for IMU_OFF_SAMPLES; between the two the state is kept.
 * Pitch and roll already come through imu_filter's low-pass (50 ms, about
 * the same as the 0.3 smoothing at 20 ms this module used to apply).
 *
 * Each word below has one writer (imu_task) and is a single store of 32
 * bits or less, so readers on either core need no lock.
 */

#include "imu_ramp.h"

// imu_task runs every 10 ms, where this module used to run every 20 ms, so
// the sample counts are doubled to keep the same 60 ms and 100 ms.
//
#define IMU_ON_SAMPLES      (6u)
#define IMU_OFF_SAMPLES     (10u)

static bool     volatile g_b_ok     = false;
static bool     volatile g_b_ramp   = false;
static uint32_t volatile g_tilt_x10 = 0u;

static uint32_t g_on_cnt  = 0u;
static uint32_t g_off_cnt = 0u;

/*!
 * @brief  Take one good sample (imu_task only).
 *
 * @param[in] pitch_deg  Pitch from the calibrated level.
 * @param[in] roll_deg   Roll from the calibrated level.
 *
 * @return true if imu_on_ramp() just changed.
 */
bool
imu_ramp_update (float32_t pitch_deg, float32_t roll_deg)
{
    float32_t const tilt_deg = imu_math_sqrt((pitch_deg * pitch_deg) +
                                             (roll_deg * roll_deg));

    // Cast: tilt_deg is never negative, and at most about 255 deg.
    //
    uint32_t const tilt       = (uint32_t)((tilt_deg * 10.0f) + 0.5f);
    bool const     b_was_ramp = g_b_ramp;

    g_tilt_x10 = tilt;
    g_b_ok     = true;

    if (tilt >= (IMU_RAMP_ON_DEG * 10u))
    {
        g_off_cnt = 0u;
        g_on_cnt++;
        if ((!g_b_ramp) && (g_on_cnt >= IMU_ON_SAMPLES))
        {
            g_b_ramp = true;
        }
    }
    else if (tilt <= (IMU_RAMP_OFF_DEG * 10u))
    {
        g_on_cnt = 0u;
        g_off_cnt++;
        if (g_b_ramp && (g_off_cnt >= IMU_OFF_SAMPLES))
        {
            g_b_ramp = false;
        }
    }
    else
    {
        // Between the thresholds: keep the state.
    }

    return (b_was_ramp != g_b_ramp);
}

/*!
 * @brief  No more data (sensor lost, or the course is done): never "on a
 *         ramp" until imu_ramp_update() is called again (imu_task only).
 */
void
imu_ramp_stop (void)
{
    g_b_ok     = false;
    g_b_ramp   = false;
    g_tilt_x10 = 0u;
    g_on_cnt   = 0u;
    g_off_cnt  = 0u;
}

/*!
 * @brief  True while imu_task has good calibrated data.
 */
bool
imu_is_ok (void)
{
    return (g_b_ok);
}

/*!
 * @brief  True while the car is tilted (on a ramp), with hysteresis.
 */
bool
imu_on_ramp (void)
{
    return (g_b_ramp);
}

/*!
 * @brief  Tilt from the calibrated attitude, in 0.1 deg.
 */
uint32_t
imu_get_tilt_x10 (void)
{
    return (g_tilt_x10);
}

/*** end of file ***/
