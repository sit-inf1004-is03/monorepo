/** @file drv_encoder.h
 *
 * @brief Subsystem 2 wheel encoder driver (one channel per wheel, on the
 *        motor shaft before the gearbox).
 *
 * The interrupt only counts pulses and keeps the time of the last two.  A
 * pulse is a rising edge after the pin has been low for at least 50 usec,
 * so sensor chatter at a slot edge is not counted.  Speed, stall and distance
 * are worked out by the readers below, which are safe to call from any
 * task on either core.  Only subsystem2_init() calls drv_encoder_init().
 *
 * NOTE: One channel gives no direction.  The count goes up whichever way
 * the wheel turns; Motion knows the direction it commanded.
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdint.h>

#include "../app.h"

#define DRV_ENCODER_PIN_LEFT        (5u)        // TEAM.md section 2.
#define DRV_ENCODER_PIN_RIGHT       (7u)

#define DRV_ENCODER_LEFT            (0u)        // Wheel index for readers.
#define DRV_ENCODER_RIGHT           (1u)
#define DRV_ENCODER_WHEELS          (2u)

#define DRV_ENCODER_PULSES_PER_REV  (637u)      // Per wheel revolution:
                                                // mean of two B2 runs
                                                // (S2-M01), +/- 1 %.

// Width of the pulse counts.  OPEN (G4, Glenn + Andrew): 32 for now.  A
// reader that wants fewer bits masks the difference of two snapshots.
//
#define DRV_ENCODER_COUNT_BITS      (32u)
#define DRV_ENCODER_TIMEOUT_USEC    (500000u)   // No pulse this long: the
                                                // speed reads 0.

ER       drv_encoder_init(void);
uint32_t drv_encoder_get_count(uint32_t wheel);
uint32_t drv_encoder_get_speed(uint32_t wheel);
uint32_t drv_encoder_get_idle_usec(uint32_t wheel);
uint32_t drv_encoder_get_chatter(uint32_t wheel);
uint32_t drv_encoder_pulses_to_mm(uint32_t pulses);
uint32_t drv_encoder_pulses_to_um(uint32_t pulses);
uint32_t drv_encoder_mm_to_pulses(uint32_t mm);

#endif /* DRV_ENCODER_H */

/*** end of file ***/

