/*!
 * @file    imu_flip.h
 * @brief   Upside-down detection, part of Subsystem 4.
 *
 * Requirement: when the car is lifted and turned upside down, the motors
 * stop.  imu_task feeds every good sample in; while this module reports
 * the car upside down, imu_task raises NAV_FLT_ESTOP, a stop fault, so
 * Navigation brakes and holds until a remote CLEAR (IF-S4-8).
 *
 * Only the upside-down part is detected.  An accelerometer cannot tell a
 * car held still in the air from one on the floor (both read 1 g), but
 * the car cannot be turned over without being lifted.
 *
 * The test uses the raw gravity direction and the fixed mounting below,
 * NOT the level found at calibration: a car switched on upside down would
 * otherwise take upside down as level and never be caught.
 *
 * Upside down = more than IMU_FLIP_ON_DEG away from upright for
 * IMU_FLIP_ON_SAMPLES samples in a row.  Upright again = less than
 * IMU_FLIP_OFF_DEG for IMU_FLIP_OFF_SAMPLES in a row.  Between the two the
 * state is kept.  A sample with |a| outside IMU_FLIP_MIN_G..IMU_FLIP_MAX_G
 * (a knock, or falling) says nothing about the direction of gravity and
 * restarts both counts.
 */

#ifndef IMU_FLIP_H
#define IMU_FLIP_H

#include <stdbool.h>
#include <stdint.h>

#include "imu_math.h"

// The board is mounted face down: on an upright car the sensor's Z axis
// reads about -0.87 g (logs of 28 Oct 2026 onwards).  Change this if the
// board is ever mounted face up.  The sensor axis is IMU_MOUNT_UP_AXIS.
//
#define IMU_FLIP_UPRIGHT_SIGN   (-1.0f)

// Angles from upright, as cosines: cos 120 deg = -0.5, cos 110 deg = -0.342.
//
#define IMU_FLIP_ON_DEG         (120u)
#define IMU_FLIP_ON_COS         (-0.5f)
#define IMU_FLIP_OFF_DEG        (110u)
#define IMU_FLIP_OFF_COS        (-0.342f)
#define IMU_FLIP_ON_SAMPLES     (20u)       // 200 ms at 10 ms.
#define IMU_FLIP_OFF_SAMPLES    (20u)
#define IMU_FLIP_MIN_G          (0.5f)
#define IMU_FLIP_MAX_G          (1.5f)

typedef struct
{
    uint32_t on_count;
    uint32_t off_count;
    bool     b_upside_down;
} imu_flip_t;

void imu_flip_init(imu_flip_t * p_flip);
bool imu_flip_update(imu_flip_t * p_flip, float32_t up_g, float32_t mag_g);
bool imu_flip_is_upside_down(imu_flip_t const * p_flip);

#endif // IMU_FLIP_H
