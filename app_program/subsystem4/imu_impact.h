/**
 * @file    imu_impact.h
 * @brief   Sudden-impact (collision) detection and recovery: the "Impact"
 *          state of Week 6 Figure 5.
 *
 * Detection ("Shock detected is valid from any substate", Week 6 6.3.2):
 *   |  |a| - 1 g  | > IMU_IMPACT_DEV_G       (a jolt in any direction), or
 *   | forward a  | > IMU_IMPACT_FWD_G       (a sudden stop against something)
 * A shock is detected on a single sample; it is not delayed by persistence,
 * so the impact is reported promptly as telemetry.
 *
 * Hump false alarms (Tribunal 1, Buddy 4: "impact detection suppressed while
 * descending a hump, with a timed recovery"): while DESCENDING, and for
 * IMU_IMPACT_HOLDOFF_SAMPLES after a hump closes, both limits are multiplied
 * by IMU_IMPACT_HUMP_SCALE.  A hump landing is ignored, but a real collision
 * on a hump is still caught.
 *
 * Recovery ("Recovered fires once the acceleration settles back near 1 g
 * for a set time", Week 6 6.3.2 rule 3): | |a| - 1 g | < IMU_IMPACT_SETTLE_G
 * for IMU_IMPACT_RECOVER_SAMPLES consecutive samples.
 *
 * ASSUMPTION: thresholds are first estimates for a small robot car with the
 * sensor at +/-8 g; tune them with test S4-M03 (tap test) on the real car.
 */

#ifndef IMU_IMPACT_H
#define IMU_IMPACT_H

#include <stdint.h>
#include <stdbool.h>

#include "imu_math.h"

#define IMU_IMPACT_DEV_G            (1.5f)
#define IMU_IMPACT_FWD_G            (1.0f)
#define IMU_IMPACT_HUMP_SCALE       (2.0f)
#define IMU_IMPACT_SETTLE_G         (0.10f)
#define IMU_IMPACT_RECOVER_SAMPLES  (50u)   /* 500 ms; 6-bit counter */
#define IMU_IMPACT_HOLDOFF_SAMPLES  (30u)   /* 300 ms; 5-bit counter */

/** Impact detector state.  Treat as opaque outside imu_impact.c. */
typedef struct
{
    float32_t peak_g;       /**< Largest deviation seen during the impact. */
    uint8_t   state_bits;   /**< [0] active [6:1] settle count             */
    uint8_t   holdoff;      /**< Samples left in the post-hump hold-off.   */
} imu_impact_t;

void      imu_impact_init (imu_impact_t * p_impact);
void      imu_impact_start_holdoff (imu_impact_t * p_impact);
bool      imu_impact_detect (imu_impact_t * p_impact, float32_t accel_mag_g,
                             float32_t lin_fwd_g, bool b_on_hump);
bool      imu_impact_check_recovered (imu_impact_t * p_impact,
                                      float32_t accel_mag_g);
bool      imu_impact_is_active (imu_impact_t const * p_impact);
float32_t imu_impact_get_peak_g (imu_impact_t const * p_impact);

#endif /* IMU_IMPACT_H */

/*** end of file ***/
