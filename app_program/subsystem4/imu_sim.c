/**
 * @file    imu_sim.c
 * @brief   Test-only simulator of IMU samples and encoder counts.
 */

#include <stddef.h>

#include "imu_sim.h"

#if defined(IMU_HOST_TEST) || (defined(APP_TEST) && (1 == APP_TEST))

#include "imu_filter.h"
#include "imu_odom.h"
#include "../common/bitpack.h"

#define MM_PER_S2_PER_G         (9806.65f)
#define GYRO_BIAS_FWD_DPS       (0.6f)
#define GYRO_BIAS_LEFT_DPS      (-0.9f)
#define GYRO_BIAS_UP_DPS        (1.3f)
#define GYRO_NOISE_DPS          (0.3f)
#define VIBRATION_G             (0.03f)
#define MAG_HORIZONTAL_COUNTS   (2400.0f)   /* ~0.38 gauss at 0.16 mG/LSB */
#define MAG_UP_COUNTS           (650.0f)    /* ~15 deg inclination       */
#define MAG_NOISE_COUNTS        (15.0f)
#define SLOPE_STEP_MM           (0.5f)
#define FRONT_ITERATIONS        (3u)
#define DEG_FULL_TURN           (360.0f)
#define DEG_QUARTER_TURN        (90.0f)
#define LCG_MULTIPLIER          (1664525u)
#define LCG_INCREMENT           (1013904223u)
#define LCG_SHIFT               (8u)
#define LCG_SCALE               (5.9604645e-8f)     /* 2^-24 */
#define INT16_LIMIT             (32767)

static float32_t random_unit (imu_sim_t * p_sim);
static float32_t cos_deg (float32_t angle_deg);
static float32_t body_pitch_deg (imu_sim_profile_t const * p_profile,
                                 float32_t rear_mm);
static int16_t   to_counts (float32_t value, float32_t counts_per_unit);

/**
 * @brief   Reset the simulator.
 * @param   p_sim          Simulator state.
 * @param   encoder_start  Initial 32-bit count (e.g. 0xFFFFFFF0 to test wrap).
 * @param   seed           Noise seed.
 */
void
imu_sim_init (imu_sim_t * p_sim, uint32_t encoder_start, uint32_t seed)
{
    if (NULL != p_sim)
    {
        p_sim->rear_pos_mm         = 0.0f;
        p_sim->yaw_deg             = 0.0f;
        p_sim->prev_pitch_deg      = 0.0f;
        p_sim->encoder_fraction_mm = 0.0f;
        p_sim->encoder_count       = encoder_start;
        p_sim->random_state        = seed;
        p_sim->b_started           = false;
    }
}

/**
 * @brief   Fill a profile for a stationary car on flat ground.
 * @param   p_profile  Profile to fill.
 */
void
imu_sim_flat_profile (imu_sim_profile_t * p_profile)
{
    if (NULL != p_profile)
    {
        p_profile->hump_height_mm = 0.0f;
        p_profile->hump_length_mm = 1.0f;
        p_profile->hump_start_mm  = 0.0f;
        p_profile->wheelbase_mm   = 75.0f;
        p_profile->speed_mm_s     = 0.0f;
        p_profile->accel_g        = 0.0f;
        p_profile->yaw_rate_dps   = 0.0f;
        p_profile->noise_g        = 0.01f;
        p_profile->b_no_gyro      = false;
        p_profile->b_rear_flat    = false;
    }
}

/**
 * @brief   Ground height: h = H/2 * (1 - cos(2 pi u)), u in [0, 1] over
 *          the hump, 0 elsewhere.
 * @param   p_profile    Track profile.
 * @param   position_mm  Ground position.
 * @return  Height in mm.
 */
float32_t
imu_sim_ground_height (imu_sim_profile_t const * p_profile,
                       float32_t position_mm)
{
    float32_t height = 0.0f;
    float32_t const offset = position_mm - p_profile->hump_start_mm;

    if ((p_profile->hump_height_mm > 0.0f) && (offset > 0.0f) &&
        (offset < p_profile->hump_length_mm))
    {
        float32_t const phase = DEG_FULL_TURN *
                                (offset / p_profile->hump_length_mm);

        height = 0.5f * p_profile->hump_height_mm * (1.0f - cos_deg(phase));
    }

    return (height);
}

/**
 * @brief   Advance by @p dt_s and produce one raw sample.
 * @param   p_sim      Simulator state.
 * @param   p_profile  Track and driving profile.
 * @param   dt_s       Step length in seconds.
 * @param   p_raw      Receives the raw sample (sensor = body axes).
 * @return  The true body pitch in degrees (for test reporting).
 */
float32_t
imu_sim_step (imu_sim_t * p_sim, imu_sim_profile_t const * p_profile,
              float32_t dt_s, drv_imu_sample_t * p_raw)
{
    float32_t const path_mm  = p_profile->speed_mm_s * dt_s;
    float32_t const slope    = (imu_sim_ground_height(p_profile,
                                    p_sim->rear_pos_mm + SLOPE_STEP_MM) -
                                imu_sim_ground_height(p_profile,
                                    p_sim->rear_pos_mm)) / SLOPE_STEP_MM;
    float32_t const cos_rear = 1.0f / imu_math_sqrt(1.0f + (slope * slope));

    /* The encoder counts path length; the ground position advances by the
     * horizontal part of it.
     */
    p_sim->rear_pos_mm         += path_mm * cos_rear;
    p_sim->encoder_fraction_mm += path_mm;

    while (p_sim->encoder_fraction_mm >= IMU_ODOM_MM_PER_PULSE)
    {
        p_sim->encoder_fraction_mm -= IMU_ODOM_MM_PER_PULSE;
        p_sim->encoder_count++;
    }

    float32_t const pitch = body_pitch_deg(p_profile, p_sim->rear_pos_mm);
    float32_t       rate  = 0.0f;

    if (p_sim->b_started && (dt_s > 0.0f))
    {
        rate = (pitch - p_sim->prev_pitch_deg) / dt_s;
    }

    p_sim->prev_pitch_deg = pitch;
    p_sim->b_started      = true;

    float32_t const vibration = (p_profile->speed_mm_s > 0.0f) ?
                                VIBRATION_G : 0.0f;
    float32_t const noise     = p_profile->noise_g + vibration;
    float32_t const centripetal = (p_profile->speed_mm_s *
                                   (rate * IMU_MATH_RAD_PER_DEG)) /
                                  MM_PER_S2_PER_G;
    float32_t const ax = imu_math_sin_deg(pitch) + p_profile->accel_g +
                         (noise * random_unit(p_sim));
    float32_t const ay = noise * random_unit(p_sim);
    float32_t const az = cos_deg(pitch) + centripetal +
                         (noise * random_unit(p_sim));

    p_raw->accel[0] = to_counts(ax, DRV_IMU_ACCEL_LSB_PER_G);
    p_raw->accel[1] = to_counts(ay, DRV_IMU_ACCEL_LSB_PER_G);
    p_raw->accel[2] = to_counts(az, DRV_IMU_ACCEL_LSB_PER_G);

    p_sim->yaw_deg = imu_math_wrap_360(p_sim->yaw_deg +
                                       (p_profile->yaw_rate_dps * dt_s));

    if (!p_profile->b_no_gyro)
    {
        /* Nose-up pitching is a negative rotation about +Y (left). */
        p_raw->gyro[0] = to_counts(GYRO_BIAS_FWD_DPS +
                                   (GYRO_NOISE_DPS * random_unit(p_sim)),
                                   DRV_IMU_GYRO_LSB_PER_DPS);
        p_raw->gyro[1] = to_counts(GYRO_BIAS_LEFT_DPS - rate +
                                   (GYRO_NOISE_DPS * random_unit(p_sim)),
                                   DRV_IMU_GYRO_LSB_PER_DPS);
        p_raw->gyro[2] = to_counts(GYRO_BIAS_UP_DPS +
                                   p_profile->yaw_rate_dps +
                                   (GYRO_NOISE_DPS * random_unit(p_sim)),
                                   DRV_IMU_GYRO_LSB_PER_DPS);

        for (uint32_t axis = 0u; axis < DRV_IMU_AXES; axis++)
        {
            p_raw->mag[axis] = 0;
        }
    }
    else
    {
        /* Level-frame field: north is yaw degrees clockwise of the nose
         * after a left turn of yaw degrees; then tilt it by the pitch.
         */
        float32_t const north_fwd  = MAG_HORIZONTAL_COUNTS *
                                     cos_deg(p_sim->yaw_deg);
        float32_t const north_left = -MAG_HORIZONTAL_COUNTS *
                                     imu_math_sin_deg(p_sim->yaw_deg);
        float32_t const mx = (north_fwd * cos_deg(pitch)) +
                             (MAG_UP_COUNTS * imu_math_sin_deg(pitch));
        float32_t const mz = (MAG_UP_COUNTS * cos_deg(pitch)) -
                             (north_fwd * imu_math_sin_deg(pitch));

        /* The car's own field (hard iron) adds the measured offsets, which
         * the filter must then remove again.
         */
        p_raw->mag[0] = to_counts(mx + IMU_MAG_OFFSET_X +
                                  (MAG_NOISE_COUNTS * random_unit(p_sim)),
                                  1.0f);
        p_raw->mag[1] = to_counts(north_left + IMU_MAG_OFFSET_Y +
                                  (MAG_NOISE_COUNTS * random_unit(p_sim)),
                                  1.0f);
        p_raw->mag[2] = to_counts(mz + IMU_MAG_OFFSET_Z +
                                  (MAG_NOISE_COUNTS * random_unit(p_sim)),
                                  1.0f);

        for (uint32_t axis = 0u; axis < DRV_IMU_AXES; axis++)
        {
            p_raw->gyro[axis] = 0;
        }
    }

    p_raw->temperature = 0;
    p_raw->b_has_gyro  = !p_profile->b_no_gyro;
    p_raw->b_has_mag   = p_profile->b_no_gyro;

    return (pitch);
}

/**
 * @brief   Simulated encoder word (as Motion Control's ISR would hold it).
 * @param   p_sim  Simulator state.
 * @return  Encoder word.
 */
uint32_t
imu_sim_encoder (imu_sim_t const * p_sim)
{
    return (p_sim->encoder_count);
}

/**
 * @brief   Uniform pseudo-random number in [-1, 1) (linear congruential
 *          generator, Numerical Recipes constants).
 * @param   p_sim  Simulator state.
 * @return  Random value.
 */
static float32_t
random_unit (imu_sim_t * p_sim)
{
    p_sim->random_state = (p_sim->random_state * LCG_MULTIPLIER) +
                          LCG_INCREMENT;

    /* Cast: a 24-bit integer is exact in float. */
    return (((float32_t)(p_sim->random_state >> LCG_SHIFT) * LCG_SCALE *
             2.0f) - 1.0f);
}

/**
 * @brief   Cosine via the sine helper.
 * @param   angle_deg  Angle.
 * @return  cos(angle).
 */
static float32_t
cos_deg (float32_t angle_deg)
{
    return (imu_math_sin_deg(angle_deg + DEG_QUARTER_TURN));
}

/**
 * @brief   Body pitch for a given rear-contact position: the front contact
 *          is one wheelbase away along the chord.
 * @param   p_profile  Track profile.
 * @param   rear_mm    Ground position of the rear contact.
 * @return  Pitch in degrees.
 */
static float32_t
body_pitch_deg (imu_sim_profile_t const * p_profile, float32_t rear_mm)
{
    float32_t const rear_h = p_profile->b_rear_flat ? 0.0f :
        imu_sim_ground_height(p_profile, rear_mm);
    float32_t       pitch  = 0.0f;

    for (uint32_t pass = 0u; pass < FRONT_ITERATIONS; pass++)
    {
        float32_t const front_mm = rear_mm +
                                   (p_profile->wheelbase_mm * cos_deg(pitch));
        float32_t const rise     = imu_sim_ground_height(p_profile, front_mm) -
                                   rear_h;
        float32_t const run      = imu_math_sqrt(
            (p_profile->wheelbase_mm * p_profile->wheelbase_mm) -
            (rise * rise));

        pitch = imu_math_atan2_deg(rise, run);
    }

    return (pitch);
}

/**
 * @brief   Convert a physical value to clamped 16-bit sensor counts.
 * @param   value            Value in g or deg/s.
 * @param   counts_per_unit  Sensor scale.
 * @return  Counts.
 */
static int16_t
to_counts (float32_t value, float32_t counts_per_unit)
{
    /* Cast: clamped to the int16_t range first. */
    return ((int16_t)bitpack_clamp_s32(imu_math_round(value *
                                                      counts_per_unit),
                                       -INT16_LIMIT, INT16_LIMIT));
}

#endif /* test builds */

/*** end of file ***/
