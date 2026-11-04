/** @file subsystem5.c
 *
 * @brief Subsystem 5 -- Obstacle Scan: creates its objects and starts its
 *        task (see subsystem5.h).
 *
 * Coding standard: BARR-C:2018.
 */

#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"
#include "obstacle_scan.h"

/*!
 * @brief  Set up the sensor and servo.  The ECHO handler is registered
 *         with gpio_irq here, as gpio_irq.h requires (from _init only).
 * @return E_OK, or the error from ultrasonic_init().
 */
EXPORT ER
subsystem5_init(void)
{
    ER const result = ultrasonic_init();

    servo_init();

    return (result);
}

/*!
 * @brief  Start obstacle_scan_task (priority 20, any core).
 * @return E_OK, or the error from app_start_task().
 */
EXPORT ER
subsystem5_start(void)
{
    ID const tid = app_start_task((FP)obstacle_scan_task, S5_PRI_MAIN, 0u,
                                  "obstacle_scan_task");

    return ((tid < E_OK) ? (ER)tid : E_OK);
}

/*** end of file ***/
