/** @file drv_motor.h
 *
 * @brief Subsystem 2 motor driver: the Robo Pico's two motor channels,
 *        driven by PWM at 20 kHz.
 *
 * Each motor takes a signed duty in parts per thousand: positive drives
 * the wheel forwards, negative backwards.  During the off part of each PWM
 * period both driver inputs are low, which brakes the motor, so the speed
 * follows the duty closely.  drv_motor_brake() holds both inputs low and
 * drv_motor_coast() holds both high (Robo Pico datasheet, table 3).
 *
 * WARNING: PWM slices 4 and 5 belong to one owning task (TEAM.md section
 * 3).  These functions are not safe to call from two tasks.  Ramping and
 * gradual braking are the caller's job; the driver changes the duty at
 * the end of the current PWM period.
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include <stdint.h>

#include "../app.h"

#define DRV_MOTOR_PIN_M1A       (8u)        // TEAM.md section 2.
#define DRV_MOTOR_PIN_M1B       (9u)
#define DRV_MOTOR_PIN_M2A       (10u)
#define DRV_MOTOR_PIN_M2B       (11u)

#define DRV_MOTOR_LEFT          (0u)        // Motor index.
#define DRV_MOTOR_RIGHT         (1u)
#define DRV_MOTORS              (2u)

#define DRV_MOTOR_DUTY_MAX      (1000)      // Full speed, parts per 1000.

ER drv_motor_init(void);
ER drv_motor_set(uint32_t motor, int32_t duty);
ER drv_motor_brake(uint32_t motor);
ER drv_motor_coast(uint32_t motor);

#endif /* DRV_MOTOR_H */

/*** end of file ***/

