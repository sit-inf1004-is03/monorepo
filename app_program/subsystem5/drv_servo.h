/**
 * @file    drv_servo.h
 * @brief   Scan servo driver (rotates the ultrasonic sensor), GP12 PWM
 *          slice 6 (Subsystem 5, Obstacle Scan).
 *
 * Minimum-bit data representation: an angle is stored as 0..180 degrees
 * in a uint8_t (8 bits). Callers normally keep only a 3- or 4-bit angle
 * INDEX and convert it to degrees with a lookup table when moving.
 *
 * Coding standard: BARR-C:2018.
 */

#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include <stdint.h>

#include "../app.h"

#define SERVO_PIN             (12u)    /* PWM slice 6, channel A */
#define SERVO_MAX_DEG         (180u)
#define SERVO_CENTRE_DEG      (90u)

/* Time for the horn to reach a new angle and stop vibrating before a
 * reading is taken. SG90 is about 0.1 s per 60 degrees. */
#define SERVO_SETTLE_MS       (200)

/**
 * @brief  Set GP12 up as a 50 Hz PWM output and move to the centre.
 *         Call once before servo_set_angle_deg().
 */
void servo_init(void);

/**
 * @brief  Move the servo to an absolute angle.
 * @param  angle_deg  0..180; larger values are clamped to 180.
 *
 * NOTE: Returns immediately. Wait SERVO_SETTLE_MS before measuring.
 */
void servo_set_angle_deg(uint8_t angle_deg);

#endif /* DRV_SERVO_H */
