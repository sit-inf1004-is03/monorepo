/** @file gpio_irq.c
 *
 * @brief Shared GPIO interrupt dispatcher (see gpio_irq.h).
 *
 * NOTE: The port does not link the Pico SDK runtime, so the IO_BANK0
 * interrupt registers (INTR, PROCx_INTE, PROCx_INTS) are programmed
 * directly, as listed in the RP2040 datasheet.  Each core has its own
 * enable and status copy; this module uses the copy of the core whose
 * NVIC takes the interrupt.
 */

#include <stdint.h>
#include <stdbool.h>

// NOTE: No <stddef.h>: tk/syslib.h defines its own size_t, which clashes.
// NULL comes from tk/typedef.h through gpio_irq.h.
//
#include "gpio_irq.h"

#define IO_IRQ_BANK0_INTNO      (13u)   // IO_IRQ_BANK0 in the RP2040 IRQ table.
#define GPIO_IRQ_PRIORITY       (2)     // NVIC level 0 (high) to 3 (low); the
                                        // kernel's physical timer also uses 2.
#define GPIO_IRQ_PIN_COUNT      (30u)   // GP0 to GP29 (GPIO_NUM in sysdef.h).
#define GPIO_IRQ_BANKS          (4u)    // Event registers per core.
#define GPIO_PINS_PER_BANK      (8u)
#define GPIO_BITS_PER_PIN       (4u)
#define GPIO_EVENT_MASK         (0xFu)
#define GPIO_EDGE_EVENTS        (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE)
#define GPIO_EDGE_BITS_ALL      (0xCCCCCCCCu)   // Edge bits of all 8 pins.

// IO_BANK0 register layout.  Cast: sysdef.h defines these addresses as
// int constants; all are positive and below 2^31, so they fit a uint32_t.
//
#define GPIO_BANK0_BASE         ((uint32_t) IO_BANK0_BASE)
#define GPIO_TIMER_RAW_LOW      ((uint32_t) TIMER_TIMERAWL)
#define GPIO_INTR_OFFSET        (0x0F0u)
#define GPIO_INTE_OFFSET        (0x100u)
#define GPIO_INTS_OFFSET        (0x120u)
#define GPIO_CORE_STRIDE        (0x030u)    // PROC1 copy follows PROC0 copy.
#define GPIO_BANK_STRIDE        (4u)

// Read by the ISR, written only by gpio_irq_register() before the pin's
// interrupt is enabled.
//
static gpio_irq_handler_t volatile g_handlers[GPIO_IRQ_PIN_COUNT] = { NULL };
static uint32_t volatile           g_irq_core     = 0u;
static bool                        gb_irq_defined = false;

static ER       gpio_irq_define(uint32_t core);
static void     gpio_irq_enable_pin(uint32_t gpio, uint32_t events,
                                    gpio_irq_handler_t p_handler);
static uint32_t gpio_intr_reg(uint32_t bank);
static uint32_t gpio_inte_reg(uint32_t core, uint32_t bank);
static uint32_t gpio_ints_reg(uint32_t core, uint32_t bank);
static void     gpio_irq_isr(UINT intno);


/*!
 * @brief Register the interrupt handler for one pin.
 *
 * Call only from a subsystemN_init() function: usermain() runs them all on
 * one core, before any task starts.  The first call defines IO_IRQ_BANK0 on
 * the calling core.
 *
 * @param[in] gpio       Pin number, 0 to 29.
 * @param[in] events     One or more GPIO_IRQ_ event bits.
 * @param[in] p_handler  Handler to call when one of the events fires.
 *
 * @return E_OK on success; E_PAR for a bad pin, events or handler; E_OBJ if
 *         the pin already has a handler; E_CTX if called on a different core
 *         from the first registration; or the error from tk_def_int().
 */
ER
gpio_irq_register (uint32_t gpio, uint32_t events,
                   gpio_irq_handler_t p_handler)
{
    uint32_t const core   = in_w(SIO_CPUID_ADDR);
    ER             result = E_OK;

    if ((gpio >= GPIO_IRQ_PIN_COUNT) || (NULL == p_handler) ||
        (0u == events) || (0u != (events & ~GPIO_EVENT_MASK)))
    {
        result = E_PAR;
    }
    else if (NULL != g_handlers[gpio])
    {
        result = E_OBJ;
    }
    else if (gb_irq_defined && (core != g_irq_core))
    {
        result = E_CTX;     // The other core's NVIC never sees this IRQ.
    }
    else
    {
        if (!gb_irq_defined)
        {
            result = gpio_irq_define(core);
        }

        if (E_OK == result)
        {
            gpio_irq_enable_pin(gpio, events, p_handler);
        }
    }

    return (result);
}

/*!
 * @brief Read the timer that every handler's now_usec comes from.
 *
 * NOTE: This is the RP2040 TIMER (TIMERAWL).  It counts at 1 MHz only when
 * the watchdog tick generator runs at 1 MHz.  The kernel port never starts
 * it; drv_encoder_init() does, and test S2-U06 checks the rate.
 *
 * @return Timer count in microseconds; wraps every 71.6 minutes.
 */
uint32_t
gpio_irq_get_time_usec (void)
{
    return (in_w(GPIO_TIMER_RAW_LOW));
}

/*!
 * @brief Define IO_IRQ_BANK0 and enable it in the calling core's NVIC.
 *
 * @param[in] core  The calling core, 0 or 1.
 *
 * @return E_OK, or the error from tk_def_int().
 */
static ER
gpio_irq_define (uint32_t core)
{
    T_DINT dint   = { 0 };
    ER     result = E_OK;

    // Cast: the kernel stores every handler as FP.  TA_HLNG handlers are
    // called with one UINT argument, which is gpio_irq_isr()'s signature.
    //
    dint.intatr = TA_HLNG;
    dint.inthdr = (FP) gpio_irq_isr;
    result      = tk_def_int(IO_IRQ_BANK0_INTNO, &dint);

    if (E_OK == result)
    {
        g_irq_core     = core;
        gb_irq_defined = true;
        EnableInt(IO_IRQ_BANK0_INTNO, GPIO_IRQ_PRIORITY);
    }

    return (result);
}

/*!
 * @brief Store a pin's handler, then enable its events on the IRQ core.
 *
 * @param[in] gpio       Pin number, already checked.
 * @param[in] events     GPIO_IRQ_ event bits, already checked.
 * @param[in] p_handler  Handler to call.
 */
static void
gpio_irq_enable_pin (uint32_t gpio, uint32_t events,
                     gpio_irq_handler_t p_handler)
{
    uint32_t const bank  = gpio / GPIO_PINS_PER_BANK;
    uint32_t const shift = (gpio % GPIO_PINS_PER_BANK) * GPIO_BITS_PER_PIN;

    // The handler is stored before the events are enabled, so the ISR can
    // never see an enabled pin without one.  Stale edges are dropped first.
    //
    g_handlers[gpio] = p_handler;
    out_w(gpio_intr_reg(bank), ((events & GPIO_EDGE_EVENTS) << shift));
    set_w(gpio_inte_reg(g_irq_core, bank), (events << shift));
}

/*!
 * @brief Address of a bank's raw interrupt register (write 1 to clear an
 *        edge).
 */
static uint32_t
gpio_intr_reg (uint32_t bank)
{
    return (GPIO_BANK0_BASE + GPIO_INTR_OFFSET + (bank * GPIO_BANK_STRIDE));
}

/*!
 * @brief Address of a bank's interrupt enable register for one core.
 */
static uint32_t
gpio_inte_reg (uint32_t core, uint32_t bank)
{
    return (GPIO_BANK0_BASE + GPIO_INTE_OFFSET + (core * GPIO_CORE_STRIDE) +
            (bank * GPIO_BANK_STRIDE));
}

/*!
 * @brief Address of a bank's interrupt status register for one core.
 */
static uint32_t
gpio_ints_reg (uint32_t core, uint32_t bank)
{
    return (GPIO_BANK0_BASE + GPIO_INTS_OFFSET + (core * GPIO_CORE_STRIDE) +
            (bank * GPIO_BANK_STRIDE));
}

/*!
 * @brief IO_IRQ_BANK0 handler: calls the handler of every pin that has an
 *        event pending.
 *
 * NOTE: On the Cortex-M0+ an ISR is an ordinary C function (the hardware
 * stacks the registers), and the kernel's TA_HLNG wrapper calls it, so no
 * interrupt keyword or #pragma is needed.
 *
 * @param[in] intno  Interrupt number; always IO_IRQ_BANK0, so unused.
 */
static void
gpio_irq_isr (UINT intno)
{
    uint32_t const now_usec = in_w(GPIO_TIMER_RAW_LOW);

    (void) intno;

    for (uint32_t bank = 0u; bank < GPIO_IRQ_BANKS; bank++)
    {
        uint32_t const pending = in_w(gpio_ints_reg(g_irq_core, bank));

        // Clear the edges BEFORE calling the handlers, so an edge that
        // arrives meanwhile pends the interrupt again instead of being lost.
        // Level events clear only when the pin changes.
        //
        out_w(gpio_intr_reg(bank), (pending & GPIO_EDGE_BITS_ALL));

        for (uint32_t pin = 0u; pin < GPIO_PINS_PER_BANK; pin++)
        {
            uint32_t const events =
                (pending >> (pin * GPIO_BITS_PER_PIN)) & GPIO_EVENT_MASK;
            uint32_t const gpio   = (bank * GPIO_PINS_PER_BANK) + pin;

            if ((0u != events) && (gpio < GPIO_IRQ_PIN_COUNT) &&
                (NULL != g_handlers[gpio]))
            {
                g_handlers[gpio](gpio, events, now_usec);
            }
        }
    }

    // NOTE: No ClearInt() here.  It could drop an edge that pended the NVIC
    // again while the handlers ran.
}

/*** end of file ***/

