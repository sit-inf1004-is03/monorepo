/**
 * @file    subsystem4.c
 * @brief   Subsystem 4 -- creates its kernel objects and starts its task.
 *
 * usermain() (app_main.c) calls subsystem4_init() in phase 1 and
 * subsystem4_start() in phase 2.
 */

#include "subsystem4.h"

/**
 * @brief   Phase 1: create the shared fault flag (owned by Buddy 4, gap
 *          G1) and the IMU's own kernel objects.  Starts no task.
 *
 * NOTE: subsystems 1-3 run their _init() before this one.  They must not
 * set fault or READY bits inside _init(); only their tasks do, and every
 * task starts after all _init() calls.  Moving faults_init() to the top of
 * usermain() would remove this ordering rule (team decision, app_main.c).
 *
 * @return  E_OK or a negative kernel error code.
 */
ER
subsystem4_init (void)
{
    ER result = faults_init();

    if (E_OK == result)
    {
        result = imu_task_init();
    }

    return (result);
}

/**
 * @brief   Phase 2: start the IMU task and its 10 ms cyclic handler.
 * @return  E_OK or a negative kernel error code.
 */
ER
subsystem4_start (void)
{
    return (imu_task_start(S4_PRI_IMU));
}

/*** end of file ***/
