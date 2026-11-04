/*!
 * @file    imu.h
 * @brief   LOCAL TEST (s5-local-test, Subsystem 4 is Andrew's): MPU9250
 *          accelerometer tilt for ramp detection.  I2C1 "iicb", GP2 SDA /
 *          GP3 SCL (Grove 2).
 *
 * The tilt is the angle between gravity now and gravity at boot (car
 * standing flat), so it does not depend on how the IMU is mounted; up and
 * down ramps both read positive.
 */

#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"

#define IMU_RAMP_ON_DEG     (6u)    // Tilt at or above this: on a ramp.
#define IMU_RAMP_OFF_DEG    (4u)    // Tilt at or below this: flat again.

void     imu_task(INT stacd, void * p_exinf);
bool     imu_is_ok(void);           // Talking to the IMU.
bool     imu_on_ramp(void);         // Tilted (with hysteresis).
uint32_t imu_get_tilt_x10(void);    // Tilt in 0.1 deg.

#endif // IMU_H
