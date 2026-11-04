/**
 * @file    drv_servo.c
 * @brief   Scan servo driver on GP12 using the kernel's libbsp PWM calls.
 *
 * PWM timing: the slice clock divider is set so that one counter tick is
 * exactly 1 us. Then the period (wrap) is 20000 ticks = 20 ms = 50 Hz,
 * and the compare value IS the pulse width in microseconds.
 *
 * WARNING: SERVO_SYS_CLK_HZ must match the kernel's system clock. If the
 *          servo jitters or every angle is off by the same ratio, check
 *          the clock set in kernel/sysdepend/pico_rp2040.
 *
 * Coding standard: BARR-C:2018.
 */

#include <stdbool.h>
#include <stdint.h>

#include "drv_servo.h"

#define SERVO_SYS_CLK_HZ      (125000000u)
#define SERVO_TICK_HZ         (1000000u)          /* 1 tick = 1 us */
#define SERVO_CLK_DIV         (SERVO_SYS_CLK_HZ / SERVO_TICK_HZ)  /* 125 */
#define SERVO_PERIOD_US       (20000u)            /* 50 Hz */

/* Pulse range for 0..180 degrees. Typical SG90 values; calibrate on your
 * servo: if it buzzes at an end, move that end inwards by 50-100 us. */
#define SERVO_MIN_PULSE_US    (600u)
#define SERVO_MAX_PULSE_US    (2400u)

/* Trim: added to every requested angle (start-up centre included) so that
 * 90 deg really points straight ahead.  The horn sits on its spline one
 * tooth (~17 deg on an SG90) off, so the true servo centre looks a bit to
 * one side.  On this car low angles look RIGHT: if the sensor still
 * points right at start-up, make this larger; if it now points left,
 * smaller.  0 once the horn is re-seated straight. */
#define SERVO_TRIM_DEG        (10)

/* RP2040 PWM block: libbsp has no divider call, so DIV is written here.
 * Per-slice register stride 0x14, DIV at offset 0x04; integer part in
 * bits 11..4 (fraction bits 3..0 left at 0). */
#define SERVO_PWM_BASE        (0x40050000u)
#define SERVO_PWM_STRIDE      (0x14u)
#define SERVO_PWM_DIV_OFFSET  (0x04u)
#define SERVO_PWM_DIV_INT_SHIFT (4u)
#define SERVO_PWM_SLICE       ((SERVO_PIN >> 1) & 0x07u)
#define SERVO_PWM_DIV_REG     (SERVO_PWM_BASE + SERVO_PWM_DIV_OFFSET + \
                               (SERVO_PWM_SLICE * SERVO_PWM_STRIDE))

/* RESETS block: RESET_DONE is 0x08 after the RESET register. */
#define SERVO_RESET_DONE_REG  (RESETS_RESET + 0x08u)

#if ((SERVO_CLK_DIV < 1u) || (SERVO_CLK_DIV > 255u))
#error "drv_servo: clock divider out of range (must be 1..255)"
#endif

static uint32_t angle_to_pulse_us(uint8_t angle_deg);

void
servo_init(void)
{
    /* NOTE: The port leaves the PWM block in reset (USE_PTMR is 0).
     *       drv_motor_init() (Subsystem 2) normally releases it first;
     *       release it here too so the servo also works in builds
     *       without Subsystem 2.  Clearing a cleared bit is harmless. */
    clr_w(RESETS_RESET, RESETS_RESET_PWM);

    while (0u == (in_w(SERVO_RESET_DONE_REG) & RESETS_RESET_PWM))
    {
        /* Intentionally empty: wait for the PWM block to leave reset. */
    }

    (void)pwm_set_pin(SERVO_PIN);
    out_w(SERVO_PWM_DIV_REG, (SERVO_CLK_DIV << SERVO_PWM_DIV_INT_SHIFT));
    (void)pwm_set_wrap(SERVO_PIN, (SERVO_PERIOD_US - 1u));
    (void)pwm_set_cc(SERVO_PIN, angle_to_pulse_us(SERVO_CENTRE_DEG));
    (void)pwm_set_enabled(SERVO_PIN, TRUE);
}

void
servo_set_angle_deg(uint8_t angle_deg)
{
    (void)pwm_set_cc(SERVO_PIN, angle_to_pulse_us(angle_deg));
}

/**
 * @brief  Map 0..180 degrees (plus SERVO_TRIM_DEG) linearly onto the
 *         calibrated pulse range.
 * @param  angle_deg  Requested angle; the trimmed angle is clamped to
 *                    0..SERVO_MAX_DEG.
 * @return Pulse width in microseconds (= PWM compare value).
 */
static uint32_t
angle_to_pulse_us(uint8_t angle_deg)
{
    int32_t deg = (int32_t)angle_deg + SERVO_TRIM_DEG;

    if (deg < 0)
    {
        deg = 0;                                    /* clamp, never wrap */
    }
    else if (deg > (int32_t)SERVO_MAX_DEG)
    {
        deg = (int32_t)SERVO_MAX_DEG;
    }
    else
    {
        /* In range. */
    }

    return (SERVO_MIN_PULSE_US +
            (((uint32_t)deg * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) /
             SERVO_MAX_DEG));
}
