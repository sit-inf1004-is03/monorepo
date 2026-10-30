/** @file drv_encoder.c
 *
 * @brief Subsystem 2 wheel encoder driver (see drv_encoder.h).
 *
 * The interrupt is the only writer of g_wheels[].  A reader on the other
 * core could see t_prev_usec and t_last_usec half updated, so the interrupt
 * makes update_seq odd while it writes and even again when it is done.  A
 * reader retries until it sees the same even value before and after its
 * copy.
 *
 * NOTE: The Cortex-M0+ completes each store before it starts the next, so
 * the other core sees the stores in program order without a barrier.
 */

#include <stdint.h>
#include <stdbool.h>

#include "drv_encoder.h"
#include "../gpio_irq.h"

#define WHEEL_CIRCUM_UM     (210000u)   // 21 cm.  A measured 1 m drive
                                        // will check it.
#define MICRONS_PER_PULSE   ((WHEEL_CIRCUM_UM +                              \
                              (DRV_ENCODER_PULSES_PER_REV / 2u)) /           \
                             DRV_ENCODER_PULSES_PER_REV)    // 330, rounded.
#define MICRONS_PER_MM      (1000u)
#define MM_PER_METRE        (1000u)
#define MIN_LOW_USEC        (50u)       // See encoder_edge_isr().
#define MIN_PULSES_SPEED    (2u)        // A speed needs one pulse interval.
#define SEQ_WRITING_BIT     (1u)

// Pad control (RP2040 PADS_BANK0, as GPIO(), GPIO_PDE etc. in sysdef.h).
// Cast: sysdef.h defines the base as an int constant; it is positive and
// below 2^31, so it fits a uint32_t.
//
#define PADS_BASE           ((uint32_t) PADS_BANK0_BASE)
#define PADS_GPIO_OFFSET    (0x04u)
#define PADS_GPIO_STRIDE    (0x04u)
#define PAD_SCHMITT         (0x02u)
#define PAD_PULL_DOWN       (0x04u)
#define PAD_PULL_UP         (0x08u)

// Watchdog tick generator (RP2040 datasheet, WATCHDOG TICK register).  The
// TIMER behind every encoder timestamp counts one tick per watchdog tick.
// Cast: sysdef.h defines XOSC_MHz as an int constant; it is 12, which fits
// the 9-bit CYCLES field.
//
#define WATCHDOG_TICK_REG   (0x4005802Cu)   // WATCHDOG_BASE + 0x2C.
#define WATCHDOG_TICK_ON    (0x200u)        // ENABLE, bit 9.
#define WATCHDOG_TICK_DIV   ((uint32_t) XOSC_MHz)   // 12 MHz clk_ref / 12.

typedef struct
{
    uint32_t volatile update_seq;   // Odd while the interrupt is writing.
    uint32_t volatile count;        // Pulses since boot.
    uint32_t volatile t_prev_usec;  // TIMERAWL of the pulse before t_last.
    uint32_t volatile t_last_usec;  // TIMERAWL of the latest pulse.
    uint32_t volatile chatter;      // Rises rejected after a short low.
    uint32_t volatile t_low_usec;   // TIMERAWL when the pin last went low.
    bool     volatile b_was_high;   // Pin level at the last interrupt.
                                    // t_low_usec and b_was_high are used
                                    // only inside the interrupt.
} encoder_wheel_t;

typedef struct
{
    uint32_t count;
    uint32_t t_prev_usec;
    uint32_t t_last_usec;
} encoder_snapshot_t;

static encoder_wheel_t g_wheels[DRV_ENCODER_WHEELS] = { { 0u } };

static void     encoder_timer_start(void);
static ER       encoder_pin_init(uint32_t pin);
static uint32_t encoder_wheel_of(uint32_t pin);
static void     encoder_read(uint32_t wheel, encoder_snapshot_t * p_snapshot);
static uint32_t encoder_pad_reg(uint32_t pin);
static void     encoder_edge_isr(uint32_t gpio, uint32_t events,
                                 uint32_t now_usec);


/*!
 * @brief Start the 1 MHz timer, then set up both encoder pins and register
 *        them with the shared GPIO interrupt dispatcher.
 *
 * @return E_OK, or the first error from gpio_set_pin() or
 *         gpio_irq_register().
 */
ER
drv_encoder_init (void)
{
    ER result = E_OK;

    encoder_timer_start();
    result = encoder_pin_init(DRV_ENCODER_PIN_LEFT);

    if (E_OK == result)
    {
        result = encoder_pin_init(DRV_ENCODER_PIN_RIGHT);
    }

    return (result);
}

/*!
 * @brief Read a wheel's pulse count.
 *
 * The count is never reset and wraps after 2^32 pulses.  A reader that
 * wants the distance of one move keeps its own snapshot from the start of
 * the move and subtracts it as uint32_t, which is wrap-safe.
 *
 * @param[in] wheel  DRV_ENCODER_LEFT or DRV_ENCODER_RIGHT.
 *
 * @return Pulses since boot, or 0 for an invalid wheel.
 */
uint32_t
drv_encoder_get_count (uint32_t wheel)
{
    uint32_t count = 0u;

    // One aligned 32-bit word: a plain read is atomic, so no retry needed.
    //
    if (wheel < DRV_ENCODER_WHEELS)
    {
        count = g_wheels[wheel].count;
    }

    return (count);
}

/*!
 * @brief Read a wheel's speed, from the time between its last two pulses.
 *
 * While no new pulse arrives the speed can be at most one pulse per idle
 * time, so the result falls towards 0 as the wheel slows.  It reads 0
 * after DRV_ENCODER_TIMEOUT_USEC with no pulse.
 *
 * @param[in] wheel  DRV_ENCODER_LEFT or DRV_ENCODER_RIGHT.
 *
 * @return Speed in millimetres per second, or 0 for an invalid wheel.
 */
uint32_t
drv_encoder_get_speed (uint32_t wheel)
{
    uint32_t speed = 0u;

    if (wheel < DRV_ENCODER_WHEELS)
    {
        encoder_snapshot_t snapshot = { 0u, 0u, 0u };

        // Read the clock only after the snapshot, so idle_usec can never
        // go negative.
        //
        encoder_read(wheel, &snapshot);

        uint32_t const idle_usec =
            gpio_irq_get_time_usec() - snapshot.t_last_usec;
        uint32_t interval_usec   = snapshot.t_last_usec - snapshot.t_prev_usec;

        if (interval_usec < idle_usec)
        {
            interval_usec = idle_usec;
        }

        // Microns per microsecond is metres per second.  The product is
        // 330,000, which fits a uint32_t, and interval_usec is at least
        // MIN_LOW_USEC whenever two pulses have been counted.
        //
        if ((snapshot.count >= MIN_PULSES_SPEED) &&
            (idle_usec < DRV_ENCODER_TIMEOUT_USEC))
        {
            speed = (MICRONS_PER_PULSE * MM_PER_METRE) / interval_usec;
        }
    }

    return (speed);
}

/*!
 * @brief Read the time since a wheel's last pulse.
 *
 * For the stall check, Motion compares this with its stall limit only
 * while it is driving the motor.
 *
 * @param[in] wheel  DRV_ENCODER_LEFT or DRV_ENCODER_RIGHT.
 *
 * @return Microseconds since the last pulse (since boot if none yet), or 0
 *         for an invalid wheel.
 */
uint32_t
drv_encoder_get_idle_usec (uint32_t wheel)
{
    uint32_t idle_usec = 0u;

    if (wheel < DRV_ENCODER_WHEELS)
    {
        // Two statements, so t_last_usec is read before the clock.
        //
        uint32_t const t_last_usec = g_wheels[wheel].t_last_usec;

        idle_usec = gpio_irq_get_time_usec() - t_last_usec;
    }

    return (idle_usec);
}

/*!
 * @brief Read how many rising edges a wheel's chatter filter rejected.
 *
 * Diagnostic for test B2 and the encoder fault check: many rejections with
 * few pulses mean the sensor chatters badly or the filter is too strict.
 *
 * @param[in] wheel  DRV_ENCODER_LEFT or DRV_ENCODER_RIGHT.
 *
 * @return Rejected rises since boot, or 0 for an invalid wheel.
 */
uint32_t
drv_encoder_get_chatter (uint32_t wheel)
{
    uint32_t chatter = 0u;

    if (wheel < DRV_ENCODER_WHEELS)
    {
        chatter = g_wheels[wheel].chatter;
    }

    return (chatter);
}

/*!
 * @brief Convert a number of pulses to millimetres travelled.
 *
 * WARNING: Exact up to 12.9 million pulses (4.29 km); beyond that the
 * product overflows.  Pass the difference of two counts, not a raw count.
 *
 * @param[in] pulses  Pulses counted.
 *
 * @return Distance in millimetres (0.330 mm per pulse, rounded down).
 */
uint32_t
drv_encoder_pulses_to_mm (uint32_t pulses)
{
    return ((pulses * MICRONS_PER_PULSE) / MICRONS_PER_MM);
}

/*!
 * @brief Convert a number of pulses to micrometres travelled.
 *
 * For speeds over a short window, where whole millimetres are too coarse.
 *
 * WARNING: Exact up to 12.9 million pulses, as drv_encoder_pulses_to_mm().
 *
 * @param[in] pulses  Pulses counted.
 *
 * @return Distance in micrometres (330 per pulse).
 */
uint32_t
drv_encoder_pulses_to_um (uint32_t pulses)
{
    return (pulses * MICRONS_PER_PULSE);
}

/*!
 * @brief Convert a distance to the nearest number of pulses.
 *
 * WARNING: Exact up to 4,294 m; beyond that the product overflows.
 *
 * @param[in] mm  Distance in millimetres.
 *
 * @return Pulses for that distance, rounded to the nearest pulse.
 */
uint32_t
drv_encoder_mm_to_pulses (uint32_t mm)
{
    return (((mm * MICRONS_PER_MM) + (MICRONS_PER_PULSE / 2u)) /
            MICRONS_PER_PULSE);
}

/*!
 * @brief Make the RP2040 TIMER count at 1 MHz.
 *
 * The TIMER counts watchdog ticks.  The Pico SDK sets the tick to 1 MHz in
 * clocks_init(), but the kernel port does not link the SDK and never sets
 * it, so after a power-on the TIMER ran at about 23.7 kHz (test S2-U06);
 * only after a BOOTSEL flash had the boot ROM left it at 1 MHz.
 *
 * NOTE: The tick is shared by the whole chip (Wi-Fi's time_us_64() reads
 * the same TIMER).  Writing the same 1 MHz setting again is harmless, and
 * the watchdog reset stays off: only the tick generator is enabled.
 */
static void
encoder_timer_start (void)
{
    out_w(WATCHDOG_TICK_REG, (WATCHDOG_TICK_ON | WATCHDOG_TICK_DIV));
}

/*!
 * @brief Make one encoder pin an input with a pull-up and a Schmitt
 *        trigger, and register its handler for both edges.
 *
 * @param[in] pin  GPIO pin number.
 *
 * @return E_OK, or the error from gpio_set_pin() or gpio_irq_register().
 */
static ER
encoder_pin_init (uint32_t pin)
{
    ER result = gpio_set_pin(pin, GPIO_MODE_IN);

    if (E_OK == result)
    {
        // The pads come out of reset with the pull-down on.
        //
        clr_w(encoder_pad_reg(pin), PAD_PULL_DOWN);
        set_w(encoder_pad_reg(pin), (PAD_PULL_UP | PAD_SCHMITT));

        // Seed the level tracking before the interrupt is enabled.  A pin
        // that starts low counts as low since boot.
        //
        g_wheels[encoder_wheel_of(pin)].b_was_high = (0u != gpio_get_val(pin));
        result = gpio_irq_register(pin,
                                   (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE),
                                   encoder_edge_isr);
    }

    return (result);
}

/*!
 * @brief Wheel index of an encoder pin.
 *
 * @param[in] pin  DRV_ENCODER_PIN_LEFT or DRV_ENCODER_PIN_RIGHT.
 *
 * @return DRV_ENCODER_LEFT or DRV_ENCODER_RIGHT.
 */
static uint32_t
encoder_wheel_of (uint32_t pin)
{
    return ((DRV_ENCODER_PIN_LEFT == pin) ? DRV_ENCODER_LEFT :
                                            DRV_ENCODER_RIGHT);
}

/*!
 * @brief Copy one wheel's state without a torn read (see the file comment).
 *
 * @param[in]  wheel       Wheel index, already checked.
 * @param[out] p_snapshot  Where to put the copy.
 */
static void
encoder_read (uint32_t wheel, encoder_snapshot_t * p_snapshot)
{
    encoder_wheel_t const * const p_wheel = &g_wheels[wheel];
    uint32_t                      seq     = 0u;
    bool                          b_torn  = true;

    while (b_torn)
    {
        seq                     = p_wheel->update_seq;
        p_snapshot->count       = p_wheel->count;
        p_snapshot->t_prev_usec = p_wheel->t_prev_usec;
        p_snapshot->t_last_usec = p_wheel->t_last_usec;

        b_torn = ((0u != (seq & SEQ_WRITING_BIT)) ||
                  (seq != p_wheel->update_seq));
    }
}

/*!
 * @brief Address of a pin's pad control register.
 */
static uint32_t
encoder_pad_reg (uint32_t pin)
{
    return (PADS_BASE + PADS_GPIO_OFFSET + (pin * PADS_GPIO_STRIDE));
}

/*!
 * @brief Edge on an encoder pin: count a pulse when the pin goes high after
 *        a steady low, and stamp its time.
 *
 * Called by gpio_irq_isr() in interrupt context.  When a slot edge is in
 * the beam, the sensor's comparator chatters high and low, often faster
 * than one interrupt per edge.  So the handler ignores which edge bits
 * fired and reads the pin instead:
 *   - high to low: a low period starts now;
 *   - low to high: a pulse, but only if the low lasted MIN_LOW_USEC;
 *   - same level as last time: edges came in a burst; the level, and the
 *     start of a low already under way, are unchanged.
 *
 * NOTE: The encoder is on the motor shaft: test B2 counted 637 pulses per
 * wheel revolution.  Test S2-M02 measured 2500 pulses per second at full
 * duty on USB power, so about 2700 on a 5 V supply: a pulse every 370
 * usec, with the pin low for about 185 usec.  MIN_LOW_USEC (50 usec)
 * leaves a margin of more than three.  A 2 ms limit rejected pulses even
 * when the wheel was turned by hand.
 *
 * @param[in] gpio      DRV_ENCODER_PIN_LEFT or DRV_ENCODER_PIN_RIGHT.
 * @param[in] events    Edge bits; unused, see above.
 * @param[in] now_usec  Time of the interrupt.
 */
static void
encoder_edge_isr (uint32_t gpio, uint32_t events, uint32_t now_usec)
{
    encoder_wheel_t * const p_wheel = &g_wheels[encoder_wheel_of(gpio)];
    bool const              b_high  = (0u != gpio_get_val(gpio));
    bool const              b_rose  = (b_high && (!p_wheel->b_was_high));

    (void) events;

    if ((!b_high) && (p_wheel->b_was_high))
    {
        p_wheel->t_low_usec = now_usec;     // A low period starts here.
    }
    else if (b_rose)
    {
        if ((now_usec - p_wheel->t_low_usec) >= MIN_LOW_USEC)
        {
            p_wheel->update_seq++;
            p_wheel->t_prev_usec = p_wheel->t_last_usec;
            p_wheel->t_last_usec = now_usec;
            p_wheel->count++;
            p_wheel->update_seq++;
        }
        else
        {
            p_wheel->chatter++;             // A rise after a short low.
        }
    }
    else
    {
        // No change of level: a burst of edges within one interrupt.
    }

    p_wheel->b_was_high = b_high;
}

/*** end of file ***/

