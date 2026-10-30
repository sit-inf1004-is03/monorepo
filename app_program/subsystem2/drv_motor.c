/** @file drv_motor.c
 *
 * @brief Subsystem 2 motor driver (see drv_motor.h).
 *
 * Each motor channel uses one PWM slice: channel A drives driver input A
 * (M1A or M2A) and channel B drives input B.  To drive forwards, A carries
 * the PWM and B stays low; backwards swaps them.  Both channels' compare
 * values share one register, so a single write changes both at once.
 *
 * NOTE: The port does not link the Pico SDK runtime.  libbsp sets the pin
 * function, the wrap and the enable; the reset release, the divider and
 * the compare register are written directly, as listed in the RP2040
 * datasheet.
 */

#include <stdint.h>
#include <stdbool.h>

#include "drv_motor.h"

// PWM timing.  Cast: sysdef.h defines TMCLK_MHz as an int constant (125).
//
#define PWM_CLOCK_HZ        ((uint32_t) TMCLK_MHz * 1000000u)  // clk_sys
#define PWM_FREQ_HZ         (20000u)    // Robo Pico maximum; above hearing.
#define PWM_PERIOD_TICKS    (PWM_CLOCK_HZ / PWM_FREQ_HZ)       // 6250
#define PWM_TOP             (PWM_PERIOD_TICKS - 1u)
#define PWM_LEVEL_LOW       (0u)                // Compare 0: always low.
#define PWM_LEVEL_HIGH      (PWM_PERIOD_TICKS)  // Compare > TOP: always high.
#define DUTY_MAX            (1000u)             // DRV_MOTOR_DUTY_MAX.

// PWM slice registers.  Cast: sysdef.h defines these as int constants; all
// are positive and below 2^31, so they fit a uint32_t.
//
#define PWM_SLICE_BASE      ((uint32_t) PWM_BASE)
#define PWM_SLICE_STRIDE    (0x14u)
#define PWM_CSR_OFFSET      ((uint32_t) PWM_CHx_CSR)
#define PWM_DIV_OFFSET      ((uint32_t) PWM_CHx_DIV)
#define PWM_CTR_OFFSET      ((uint32_t) PWM_CHx_CTR)
#define PWM_CC_OFFSET       ((uint32_t) PWM_CHx_CC)
#define PWM_CSR_OFF         (0x0u)      // Disabled, free-running, no invert.
#define PWM_DIV_ONE         (0x10u)     // Divider 1.0 (8.4 fixed point).
#define PWM_CC_B_SHIFT      (16u)       // Channel B compare, bits 31:16.

#define RESETS_PWM_BIT      ((uint32_t) RESETS_RESET_PWM)
#define RESETS_RESET_REG    ((uint32_t) RESETS_RESET)
#define RESETS_DONE_REG     ((uint32_t) RESETS_RESET_DONE)

typedef struct
{
    uint32_t pin_a;         // Driver input A: GP8 or GP10 (PWM channel A).
    uint32_t pin_b;         // Driver input B: GP9 or GP11 (PWM channel B).
    uint32_t slice;         // PWM slice of both pins.
    bool     b_reversed;    // True if input A high turns the wheel
                            // backwards.  Set from test S2-M02.
} motor_wiring_t;

// Which connector drives which wheel, and its direction, from test S2-M02
// (testing/subsystem2/TEST_LOG.md).  The right motor faces the other way,
// so input A high turns its wheel backwards.
//
static motor_wiring_t const g_motors[DRV_MOTORS] =
{
    { DRV_MOTOR_PIN_M1A, DRV_MOTOR_PIN_M1B, 4u, false },    // LEFT  = M1
    { DRV_MOTOR_PIN_M2A, DRV_MOTOR_PIN_M2B, 5u, true  },    // RIGHT = M2
};

static void     motor_pwm_unreset(void);
static ER       motor_slice_init(motor_wiring_t const * p_motor);
static void     motor_set_levels(motor_wiring_t const * p_motor,
                                 uint32_t level_a, uint32_t level_b);
static uint32_t motor_slice_reg(uint32_t slice, uint32_t offset);


/*!
 * @brief Set up both motor channels at 20 kHz, with both motors braked.
 *
 * Call once, from subsystem2_init().
 *
 * @return E_OK, or the first error from libbsp's PWM functions.
 */
ER
drv_motor_init (void)
{
    ER result = E_OK;

    motor_pwm_unreset();

    for (uint32_t motor = 0u; (motor < DRV_MOTORS) && (E_OK == result);
         motor++)
    {
        result = motor_slice_init(&g_motors[motor]);
    }

    return (result);
}

/*!
 * @brief Drive one motor at a signed duty.
 *
 * The new duty takes effect at the end of the current 50 usec PWM period.
 * A duty of 0 brakes the motor.
 *
 * @param[in] motor  DRV_MOTOR_LEFT or DRV_MOTOR_RIGHT.
 * @param[in] duty   -DRV_MOTOR_DUTY_MAX (full backwards) to
 *                   DRV_MOTOR_DUTY_MAX (full forwards).
 *
 * @return E_OK, or E_PAR for a bad motor or duty.
 */
ER
drv_motor_set (uint32_t motor, int32_t duty)
{
    ER result = E_OK;

    if ((motor >= DRV_MOTORS) || (duty > DRV_MOTOR_DUTY_MAX) ||
        (duty < -DRV_MOTOR_DUTY_MAX))
    {
        result = E_PAR;
    }
    else
    {
        motor_wiring_t const * const p_motor = &g_motors[motor];
        int32_t const drive     = (p_motor->b_reversed ? -duty : duty);

        // Cast: the magnitude is 0 to 1000, so it fits a uint32_t, and
        // 1000 * 6250 fits too.
        //
        uint32_t const magnitude = (uint32_t) ((drive < 0) ? -drive : drive);
        uint32_t const level     = (magnitude * PWM_PERIOD_TICKS) / DUTY_MAX;

        if (drive >= 0)
        {
            motor_set_levels(p_motor, level, PWM_LEVEL_LOW);
        }
        else
        {
            motor_set_levels(p_motor, PWM_LEVEL_LOW, level);
        }
    }

    return (result);
}

/*!
 * @brief Brake one motor: both driver inputs low, which shorts the motor.
 *
 * @param[in] motor  DRV_MOTOR_LEFT or DRV_MOTOR_RIGHT.
 *
 * @return E_OK, or E_PAR for a bad motor.
 */
ER
drv_motor_brake (uint32_t motor)
{
    ER result = E_PAR;

    if (motor < DRV_MOTORS)
    {
        motor_set_levels(&g_motors[motor], PWM_LEVEL_LOW, PWM_LEVEL_LOW);
        result = E_OK;
    }

    return (result);
}

/*!
 * @brief Let one motor coast: both driver inputs high, outputs open.
 *
 * @param[in] motor  DRV_MOTOR_LEFT or DRV_MOTOR_RIGHT.
 *
 * @return E_OK, or E_PAR for a bad motor.
 */
ER
drv_motor_coast (uint32_t motor)
{
    ER result = E_PAR;

    if (motor < DRV_MOTORS)
    {
        motor_set_levels(&g_motors[motor], PWM_LEVEL_HIGH, PWM_LEVEL_HIGH);
        result = E_OK;
    }

    return (result);
}

/*!
 * @brief Take the PWM block out of reset, if it is still held there.
 *
 * The port releases PWM only when USE_PTMR is set in config.h, and it is
 * 0.  NOTE: Never put PWM back into reset: that would also stop the other
 * slices (the servo on slice 6).
 */
static void
motor_pwm_unreset (void)
{
    if (0u != (in_w(RESETS_RESET_REG) & RESETS_PWM_BIT))
    {
        clr_w(RESETS_RESET_REG, RESETS_PWM_BIT);

        while (0u == (in_w(RESETS_DONE_REG) & RESETS_PWM_BIT))
        {
            // Wait: the release takes a few clock cycles.
        }
    }
}

/*!
 * @brief Set up one slice at 20 kHz with both outputs low, then hand both
 *        pins to it.
 *
 * The pins are switched to PWM last, so they go straight from the pad's
 * pull-down (which also brakes) to a low output, with no pulse between.
 *
 * @param[in] p_motor  The motor's wiring.
 *
 * @return E_OK, or the first error from libbsp's PWM functions.
 */
static ER
motor_slice_init (motor_wiring_t const * p_motor)
{
    ER result = E_OK;

    out_w(motor_slice_reg(p_motor->slice, PWM_CSR_OFFSET), PWM_CSR_OFF);
    out_w(motor_slice_reg(p_motor->slice, PWM_DIV_OFFSET), PWM_DIV_ONE);
    out_w(motor_slice_reg(p_motor->slice, PWM_CTR_OFFSET), 0u);
    motor_set_levels(p_motor, PWM_LEVEL_LOW, PWM_LEVEL_LOW);

    result = pwm_set_wrap(p_motor->pin_a, PWM_TOP);

    if (E_OK == result)
    {
        result = pwm_set_enabled(p_motor->pin_a, TRUE);
    }

    if (E_OK == result)
    {
        result = pwm_set_pin(p_motor->pin_a);
    }

    if (E_OK == result)
    {
        result = pwm_set_pin(p_motor->pin_b);
    }

    return (result);
}

/*!
 * @brief Write both channels' compare values in one store.
 *
 * NOTE: libbsp's pwm_set_cc() changes one channel at a time.  One store
 * keeps A and B in step, and the slice latches it at the end of the
 * period, so the inputs never pass through a state in between.
 *
 * @param[in] p_motor  The motor's wiring.
 * @param[in] level_a  Input A compare value, PWM_LEVEL_LOW to _HIGH.
 * @param[in] level_b  Input B compare value, PWM_LEVEL_LOW to _HIGH.
 */
static void
motor_set_levels (motor_wiring_t const * p_motor, uint32_t level_a,
                  uint32_t level_b)
{
    out_w(motor_slice_reg(p_motor->slice, PWM_CC_OFFSET),
          ((level_b << PWM_CC_B_SHIFT) | level_a));
}

/*!
 * @brief Address of one register of a PWM slice.
 */
static uint32_t
motor_slice_reg (uint32_t slice, uint32_t offset)
{
    return (PWM_SLICE_BASE + (slice * PWM_SLICE_STRIDE) + offset);
}

/*** end of file ***/

