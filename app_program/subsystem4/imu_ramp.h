/*!
 * @file    imu_ramp.h
 * @brief   Ramp detection for Line Following, part of Subsystem 4.
 *
 * Written for the ramp in s5-local-test, now fed by imu_task, the only
 * owner of I2C1: imu_task calls imu_ramp_update() on every good sample and
 * Line Following reads the result (IF-S4-7).  The thresholds and timing are
 * unchanged; the separate sensor task and chip driver were replaced by
 * imu_task's data (they are in git history at 925d3ee).
 *
 * The tilt is the angle away from the attitude at calibration (car standing
 * flat at boot), so it does not depend on how the IMU is mounted; up and
 * down ramps both read positive.
 */

#ifndef IMU_RAMP_H
#define IMU_RAMP_H

#include <stdbool.h>
#include <stdint.h>

#include "imu_math.h"

#define IMU_RAMP_ON_DEG     (6u)    // Tilt at or above this: on a ramp.
#define IMU_RAMP_OFF_DEG    (4u)    // Tilt at or below this: flat again.

// Written by imu_task only.
//
bool     imu_ramp_update(float32_t pitch_deg, float32_t roll_deg);
void     imu_ramp_stop(void);

// Read by anyone, any time.
//
bool     imu_is_ok(void);           // imu_task has good calibrated data.
bool     imu_on_ramp(void);         // Tilted (with hysteresis).
uint32_t imu_get_tilt_x10(void);    // Tilt in 0.1 deg.

#endif // IMU_RAMP_H
