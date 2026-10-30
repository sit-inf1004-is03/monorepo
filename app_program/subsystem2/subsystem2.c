/*
 *	subsystem2.c
 *	Subsystem 2 -- Producer: creates its objects and starts its task.
 *
 *	APIs shown: tk_cre_sem, tk_cre_mpf
 */

#include "subsystem2.h"
#include "drv_motor.h"

EXPORT ID	sem_slots;	/* IF-02, see subsystem2.h */
EXPORT ID	mpf_samples;

EXPORT ER subsystem2_init(void)
{
	T_CSEM	csem = {0};
	T_CMPF	cmpf = {0};
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
	result = motion_init();
	if (E_OK != result)
	{
		return (result);
	}

	/* Semaphore: a counter of free "slots".  The producer takes one
	   (tk_wai_sem) before each sample; the consumer gives one back
	   (tk_sig_sem).  It never exceeds the number of memory blocks, so
	   the producer can never run out of memory. */
	csem.sematr  = TA_TFIFO;	/* waiting tasks queue in FIFO order */
	csem.isemcnt = S2_POOL_COUNT;	/* initial count */
	csem.maxsem  = S2_POOL_COUNT;	/* maximum count */
	sem_slots = tk_cre_sem(&csem);
	if (sem_slots < E_OK) {
		return sem_slots;
	}

	/* Fixed-size memory pool: S2_POOL_COUNT blocks, each exactly one
	   SAMPLE_MSG.  Faster and more predictable than malloc(). */
	cmpf.mpfatr = TA_TFIFO | TA_RNG3;
	cmpf.mpfcnt = S2_POOL_COUNT;		/* number of blocks */
	cmpf.blfsz  = sizeof(SAMPLE_MSG);	/* size of one block */
	mpf_samples = tk_cre_mpf(&cmpf);
	return (mpf_samples < E_OK) ? mpf_samples : E_OK;
}

EXPORT ER subsystem2_start(void)
{
	/* Pinned to core 1 (TP_PRC2): samples cross to subsystem 3 on core 0 */
	ID tid = app_start_task(producer_task, S2_PRI_PRODUCER, TP_PRC2,
				"producer_task");
	if (tid < E_OK) {
		return tid;
	}

	// task_motion: 10 ms, core 1 (motion.h).
	//
	return (motion_start());
}
