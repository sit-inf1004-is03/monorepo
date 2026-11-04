/** @file drv_ultrasonic.h
 *
 * @brief HC-SR04 ultrasonic driver (Subsystem 5, Obstacle Scan).
 *
 * TRIG is a GPIO pulse; ECHO edges are timed by the team's shared GPIO
 * interrupt dispatcher (gpio_irq.h), never by an interrupt of our own.
 *
 * Minimum-bit data representation (Tribunal 1 change requirement): a
 * reading is ONE 16-bit word, written by one task with a single store.
 *
 *   bits 15..11  unused
 *   bit  10      silent: the sensor did not respond at all     (1 bit)
 *   bit  9       no echo: nothing measured (out of range too)  (1 bit)
 *   bits 8..0    distance in cm, 0..400                        (9 bits)
 *
 * "No echo" alone is normal (nothing within range, e.g. an open track).
 * "Silent" means the ECHO line never went high, or only glitched: the
 * sensor is unpowered or unplugged.  Only silence is a fault.
 *
 * NOTE: Timing uses gpio_irq_get_time_usec(), which runs at 1 MHz only
 *       after drv_encoder_init() (Subsystem 2) starts the tick generator.
 *       usermain() runs subsystem2_init() before subsystem5_init().
 *
 * Coding standard: BARR-C:2018.
 */

#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"

// Pins: Robo Pico Grove port 4 (TEAM.md section 2).
//
#define ULTRASONIC_TRIG_PIN       (16u)
#define ULTRASONIC_ECHO_PIN       (17u)

// Packed reading layout.
//
#define ULTRASONIC_MAX_CM         (400u)
#define ULTRASONIC_DIST_MASK      (0x01FFu)
#define ULTRASONIC_NO_ECHO_BIT    (0x0200u)
#define ULTRASONIC_SILENT_BIT     (0x0400u)

typedef uint16_t ultrasonic_reading_t;

/*!
 * @brief  Set up TRIG/ECHO and register the ECHO handler with gpio_irq.
 *         Call only from subsystem5_init().
 * @return E_OK, or the error from gpio_irq_register().
 */
ER ultrasonic_init(void);

/*!
 * @brief  Fire one trigger pulse and wait (yielding) for the echo.
 * @return Packed reading. Check ultrasonic_is_no_echo() first.
 *
 * NOTE: Call from the Obstacle Scan task only (the TRIG pin's owner).
 */
ultrasonic_reading_t ultrasonic_read(void);

/*!
 * @brief  Most recent packed reading; safe from any task (16-bit load).
 */
ultrasonic_reading_t ultrasonic_last_reading(void);

/*!
 * @brief  Echo width to distance: width x 343 / 20000, rounded, clamped
 *         to ULTRASONIC_MAX_CM so it always fits 9 bits.  Public for the
 *         subsystem 5 unit tests.
 * @param  width_us  Echo pulse width in microseconds.
 * @return Distance in cm.
 */
uint16_t ultrasonic_width_to_cm(uint32_t width_us);

/*!
 * @brief  Distance field of a packed reading, in cm.
 */
static inline uint16_t
ultrasonic_dist_cm(ultrasonic_reading_t reading)
{
    return ((uint16_t)(reading & ULTRASONIC_DIST_MASK));
}

/*!
 * @brief  Was nothing measured (no echo, out of range or silent)?
 */
static inline bool
ultrasonic_is_no_echo(ultrasonic_reading_t reading)
{
    return (0u != (reading & ULTRASONIC_NO_ECHO_BIT));
}

/*!
 * @brief  Did the sensor fail to respond at all (fault condition)?
 */
static inline bool
ultrasonic_is_silent(ultrasonic_reading_t reading)
{
    return (0u != (reading & ULTRASONIC_SILENT_BIT));
}

#endif /* DRV_ULTRASONIC_H */

/*** end of file ***/
