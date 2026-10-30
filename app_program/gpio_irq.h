/** @file gpio_irq.h
 *
 * @brief Shared GPIO interrupt dispatcher.  Changed only by team agreement.
 *
 * The RP2040 has one interrupt (IO_IRQ_BANK0) for every GPIO pin.  Each
 * subsystem that needs a pin interrupt (wheel encoders, ultrasonic echo)
 * registers a handler for its own pin here instead of defining the
 * interrupt itself.  Claim the pin in TEAM.md first.
 */

#ifndef GPIO_IRQ_H
#define GPIO_IRQ_H

#include <stdint.h>

#include "app.h"

// Pin events, in the RP2040's own bit order (one nibble per pin).
//
#define GPIO_IRQ_LEVEL_LOW      (0x1u)
#define GPIO_IRQ_LEVEL_HIGH     (0x2u)
#define GPIO_IRQ_EDGE_FALL      (0x4u)
#define GPIO_IRQ_EDGE_RISE      (0x8u)

// Pin handler, called in interrupt context on the core that registered it.
//   gpio      the pin that fired
//   events    the GPIO_IRQ_ bits that fired; edges are already cleared
//   now_usec  1 MHz timer (TIMERAWL) read at interrupt entry; it wraps
//             every 71.6 minutes, so subtract timestamps as uint32_t
//
// WARNING: A handler must be short and must never block: no tk_dly_tsk(),
// no tk_wai_ calls and no tm_printf().  Non-blocking kernel calls such as
// tk_set_flg() are allowed.
//
typedef void (* gpio_irq_handler_t)(uint32_t gpio, uint32_t events,
                                    uint32_t now_usec);

ER       gpio_irq_register(uint32_t gpio, uint32_t events,
                           gpio_irq_handler_t p_handler);
uint32_t gpio_irq_get_time_usec(void);

#endif /* GPIO_IRQ_H */

/*** end of file ***/

