/**
 * @file    drv_us_timer.c
 * @brief   RP2040 1 MHz TIMER access (RP2040 datasheet, section 4.6).
 *
 * The TIMER only counts while the watchdog tick generator divides clk_ref
 * (12 MHz XOSC) down to 1 MHz (datasheet 4.7.2).  pico-sdk starts that tick
 * in its runtime; this port does not link the pico-sdk runtime, so
 * drv_us_timer_init() checks that the counter moves and, only if it does not,
 * starts the tick with the same value pico-sdk uses (12 cycles per tick).
 */

#include <tk/tkernel.h>

#include "drv_us_timer.h"

#define WATCHDOG_TICK_REG       (0x4005802Cu)
#define WATCHDOG_TICK_ENABLE    (0x00000200u)
#define TICK_CYCLES_PER_US      (12u)           /* clk_ref = 12 MHz XOSC */

/* Busy loop used only to see whether the counter is moving at all. */
#define PROBE_SPIN_COUNT        (5000u)

/* Upper bound on the iterations of drv_us_timer_delay(), so a stopped
 * timer can never hang the caller.  About 8 cycles each at 125 MHz.
 */
#define DELAY_SPIN_PER_US       (16u)

static bool counter_is_moving (void);

/**
 * @brief   Make sure the 1 MHz counter runs.
 * @return  true if the counter is running after the call.
 */
bool
drv_us_timer_init (void)
{
    bool b_running = counter_is_moving();

    if (!b_running)
    {
        out_w(WATCHDOG_TICK_REG, WATCHDOG_TICK_ENABLE | TICK_CYCLES_PER_US);
        b_running = counter_is_moving();
    }

    return (b_running);
}

/**
 * @brief   Current time in microseconds (wraps every 71.6 minutes).
 * @return  Low 32 bits of the free-running counter.
 */
uint32_t
drv_us_timer_now (void)
{
    return (in_w(TIMER_TIMERAWL));
}

/**
 * @brief   Microseconds since @p start_us, correct across one wrap.
 * @param   start_us  Earlier value from drv_us_timer_now().
 * @return  Elapsed time in microseconds.
 */
uint32_t
drv_us_timer_elapsed (uint32_t start_us)
{
    return (drv_us_timer_now() - start_us);
}

/**
 * @brief   Busy-wait for a short time (used for I2C bus recovery only).
 *
 * WARNING: busy-waits; keep @p delay_us small (a few microseconds).
 *
 * @param   delay_us  Delay in microseconds.
 */
void
drv_us_timer_delay (uint32_t delay_us)
{
    uint32_t const          start  = drv_us_timer_now();
    uint32_t const          limit  = (delay_us + 1u) * DELAY_SPIN_PER_US;
    uint32_t volatile       spins  = 0u;

    while ((drv_us_timer_elapsed(start) < delay_us) && (spins < limit))
    {
        spins++;
    }
}

/**
 * @brief   Check that the counter advances during a short busy loop.
 * @return  true if it advanced.
 */
static bool
counter_is_moving (void)
{
    uint32_t const    first = drv_us_timer_now();
    uint32_t volatile spins = 0u;

    while (spins < PROBE_SPIN_COUNT)
    {
        spins++;
    }

    return (drv_us_timer_now() != first);
}

/*** end of file ***/
