/*
 *	subsystem4.c
 *	Subsystem 4 -- <name>.  EMPTY TEMPLATE.
 *
 *	subsystem4_init():  create this subsystem's kernel objects
 *	                    (semaphores, mailboxes, ...).  Do not start
 *	                    tasks here.
 *	subsystem4_start(): create and start this subsystem's tasks, e.g.
 *	    ID tid = app_start_task(my_task, S4_PRI_MAIN, 0, "my_task");
 *	    return (tid < E_OK) ? tid : E_OK;
 *
 *	Drivers for this subsystem's hardware go in this folder as
 *	drv_<name>.c / drv_<name>.h, called only from the task that owns
 *	the peripheral (TEAM.md section 3).
 *
 *	Record every pin, task, object and interface in TEAM.md.
 */

#include "subsystem4.h"
#include "imu.h"	/* LOCAL TEST: MPU9250 ramp tilt (s5-local-test) */

EXPORT ER subsystem4_init(void)
{
	return E_OK;
}

EXPORT ER subsystem4_start(void)
{
	/* LOCAL TEST: IMU tilt task on I2C1 (GP2/GP3). */
	ID tid = app_start_task((FP)imu_task, S4_PRI_MAIN, 0, "imu_task");
	return (tid < E_OK) ? tid : E_OK;
}
