/**
 * @file    drv_us_timer.h
 * @brief   Read-only access to the RP2040 1 MHz TIMER, used for I2C
 *          timeouts and for measuring the IMU sample interval.
 *
 * The TIMER block is free-running and is only READ here (TIMERAWL), so it is
 * safe to share with the kernel, which reads the same register for its own
 * diagnostics (kernel/sysdepend/cpu/rp2040/smp_atomic.c).
 */

#ifndef DRV_US_TIMER_H
#define DRV_US_TIMER_H

#include <stdint.h>
#include <stdbool.h>

bool     drv_us_timer_init (void);
uint32_t drv_us_timer_now (void);
uint32_t drv_us_timer_elapsed (uint32_t start_us);
void     drv_us_timer_delay (uint32_t delay_us);

#endif /* DRV_US_TIMER_H */

/*** end of file ***/
