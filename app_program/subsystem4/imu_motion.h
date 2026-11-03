/**
 * @file    imu_motion.h
 * @brief   Motion-classification region of the IMU statechart
 *          (Week 6 Figure 5, left region; Section 6.3.1).
 *
 *   STATIONARY    not moving and low acceleration variance
 *   ACCELERATING  forward acceleration (or braking) sustained above a limit
 *   CRUISING      moving at a steady speed with low variance
 *   TURNING       heading rate above w (IMU_MOTION_TURN_RATE_DPS)
 *
 * A new class is accepted only after it persists for
 * IMU_MOTION_PERSIST_SAMPLES consecutive samples (Week 6 6.3.2 rule 1).  The
 * counter is 3 bits, so the persistence can be at most 7 samples (70 ms),
 * the limit recorded in Tribunal 1, 2.2.4.
 *
 * "Moving" comes from the wheel encoders when Motion Control has registered
 * them, otherwise from vibration (acceleration variance).
 *
 * Classes are stored and published in 2 bits (Tribunal 1, 2.6).
 */

#ifndef IMU_MOTION_H
#define IMU_MOTION_H

#include <stdint.h>
#include <stdbool.h>

#include "imu_math.h"

#define IMU_MOTION_TURN_RATE_DPS    (40.0f)     /* w in Week 6 6.3.1 */
#define IMU_MOTION_ACCEL_G          (0.10f)
#define IMU_MOTION_STILL_VAR_G2     (0.0004f)   /* (0.02 g)^2 */
#define IMU_MOTION_VAR_GAIN         (0.0625f)   /* ~16-sample average */
#define IMU_MOTION_PERSIST_SAMPLES  (5u)        /* <= 7 (3-bit counter) */

/** Motion class, 2 bits. */
typedef enum
{
    IMU_MOTION_STATIONARY   = 0,
    IMU_MOTION_ACCELERATING = 1,
    IMU_MOTION_CRUISING     = 2,
    IMU_MOTION_TURNING      = 3
} imu_motion_class_t;

/** Inputs for one classification step. */
typedef struct
{
    float32_t accel_mag_g;
    float32_t lin_fwd_g;
    float32_t yaw_rate_dps;
    bool      b_moving;         /**< From encoders, or vibration fallback. */
} imu_motion_inputs_t;

/** Classifier state.  Treat as opaque outside imu_motion.c. */
typedef struct
{
    float32_t mean_g;
    float32_t var_g2;
    uint8_t   state_bits;       /**< [1:0] class [3:2] candidate [6:4] count */
} imu_motion_t;

void               imu_motion_init (imu_motion_t * p_motion);
bool               imu_motion_update (imu_motion_t * p_motion,
                                      imu_motion_inputs_t const * p_in);
imu_motion_class_t imu_motion_get_class (imu_motion_t const * p_motion);
bool               imu_motion_is_vibrating (imu_motion_t const * p_motion);

#endif /* IMU_MOTION_H */

/*** end of file ***/
