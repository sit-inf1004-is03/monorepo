/**
 * @file    imu_sim.h
 * @brief   Test-only simulator: raw MPU-9250 samples and encoder counts for
 *          a two-contact car driving over a cosine-shaped hump.
 *
 * Built only for tests (make TEST=1, or host_test with IMU_HOST_TEST), so it
 * adds nothing to the normal firmware image.
 *
 * Model: rear contact moves along the ground at the given speed (the wheel
 * encoder counts this path length); the front contact is one wheelbase
 * ahead; body pitch is the slope of the line between the two contacts.  The
 * accelerometer sees gravity, the commanded forward acceleration and the
 * centripetal term v * d(pitch)/dt; the gyro sees d(pitch)/dt and the yaw
 * rate.  A fixed gyro bias and pseudo-random noise are added.
 *
 * With b_no_gyro set the sample looks like the LSM303D (GY-511): gyro zero,
 * and a compass that sees a mostly horizontal Earth field (as in
 * Singapore, near the magnetic equator) turned by the car's heading.
 */

#ifndef IMU_SIM_H
#define IMU_SIM_H

#include <stdint.h>
#include <stdbool.h>

#include "imu_math.h"
#include "drv_imu.h"

#if defined(IMU_HOST_TEST) || (defined(APP_TEST) && (1 == APP_TEST))

/** Track and driving profile for one simulation step. */
typedef struct
{
    float32_t hump_height_mm;   /**< 0 for flat ground.                   */
    float32_t hump_length_mm;   /**< Ground length of the hump.           */
    float32_t hump_start_mm;    /**< Ground position where it begins.     */
    float32_t wheelbase_mm;     /**< Distance between the two contacts.   */
    float32_t speed_mm_s;       /**< Rear-contact speed along the ground. */
    float32_t accel_g;          /**< Extra forward acceleration, in g.    */
    float32_t yaw_rate_dps;     /**< Turn rate (left positive).           */
    float32_t noise_g;          /**< Accelerometer noise amplitude.       */
    bool      b_no_gyro;        /**< LSM303D: no gyro, compass instead.   */
    bool      b_rear_flat;      /**< Rear wheels pass beside the hump.    */
} imu_sim_profile_t;

/** Simulator state. */
typedef struct
{
    float32_t rear_pos_mm;
    float32_t yaw_deg;
    float32_t prev_pitch_deg;
    float32_t encoder_fraction_mm;
    uint32_t  encoder_count;
    uint32_t  random_state;
    bool      b_started;
} imu_sim_t;

void      imu_sim_init (imu_sim_t * p_sim, uint32_t encoder_start,
                        uint32_t seed);
void      imu_sim_flat_profile (imu_sim_profile_t * p_profile);
float32_t imu_sim_ground_height (imu_sim_profile_t const * p_profile,
                                 float32_t position_mm);
float32_t imu_sim_step (imu_sim_t * p_sim,
                        imu_sim_profile_t const * p_profile, float32_t dt_s,
                        drv_imu_sample_t * p_raw);
uint32_t  imu_sim_encoder (imu_sim_t const * p_sim);

#endif /* test builds */

#endif /* IMU_SIM_H */

/*** end of file ***/
