/**
 * @file    imu_math.h
 * @brief   Small single-precision maths helpers for the IMU subsystem.
 *
 * The firmware is linked without libm (build_make/makefile has no -lm), so
 * the few functions the attitude filter needs are implemented here with
 * polynomial approximations.  Accuracy (checked by host_test):
 *   imu_math_sqrt()       relative error < 1e-6
 *   imu_math_atan2_deg()  absolute error < 0.001 deg
 *   imu_math_sin_deg()    absolute error < 1e-5
 *
 * The RP2040 has no FPU, so every float operation is a soft-float library
 * call; the filter uses roughly a dozen per 10 ms sample.
 */

#ifndef IMU_MATH_H
#define IMU_MATH_H

#include <stdint.h>
#include <stdbool.h>

/** C99 has no float32_t; Barr-C Rule 5.4.b.i asks for this name. */
typedef float float32_t;

#define IMU_MATH_PI             (3.14159265f)
#define IMU_MATH_DEG_PER_RAD    (57.2957795f)
#define IMU_MATH_RAD_PER_DEG    (0.0174532925f)

float32_t imu_math_abs (float32_t value);
float32_t imu_math_clamp (float32_t value, float32_t min_value,
                          float32_t max_value);
float32_t imu_math_sqrt (float32_t value);
float32_t imu_math_atan2_deg (float32_t y_value, float32_t x_value);
float32_t imu_math_sin_deg (float32_t angle_deg);
float32_t imu_math_wrap_360 (float32_t angle_deg);
int32_t   imu_math_round (float32_t value);
bool      imu_math_is_finite (float32_t value);

#endif /* IMU_MATH_H */

/*** end of file ***/
