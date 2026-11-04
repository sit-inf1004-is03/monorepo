/*
 *	subsystem2.c
 *	Subsystem 2 -- Motion control: creates its objects and starts its task.
 */

#include "subsystem2.h"
#include "drv_motor.h"

EXPORT ER subsystem2_init(void)
{
	ER	result = E_OK;

	// Wheel encoders on GP5 and GP7, via the shared GPIO interrupt.
	//
	result = drv_encoder_init();
	if (E_OK != result)
	{
		return (result);
	}

	// Motors on GP8-GP11 (PWM slices 4 and 5), both braked.
	//
	result = drv_motor_init();
	if (E_OK != result)
	{
		return (result);
	}

	// task_motion's event flag and command/reply message buffers.
	//
	return (motion_init());
}

EXPORT ER subsystem2_start(void)
{
	// task_motion: 10 ms, core 1 (motion.h).
	//
	return (motion_start());
}
