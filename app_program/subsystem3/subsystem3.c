/*
 *	subsystem3.c
 *	Subsystem 3 -- Consumer: creates its objects and starts its task.
 *
 *	APIs shown: tk_cre_mbx, tk_cre_mtx
 */

// #include "subsystem3.h"

// EXPORT ID		mbx_samples;	/* IF-01 */
// EXPORT ID		mtx_stats;	/* IF-04 */
// EXPORT APP_STATS	app_stats;

// EXPORT ER subsystem3_init(void)
// {
// 	T_CMBX	cmbx = {0};
// 	T_CMTX	cmtx = {0};

// 	/* Mailbox: passes a POINTER to a message (zero copy).  The message
// 	   memory must stay valid until the receiver is done with it, which
// 	   is why subsystem 2 takes it from its memory pool. */
// 	cmbx.mbxatr = TA_TFIFO | TA_MFIFO;	/* FIFO waiters, FIFO messages */
// 	mbx_samples = tk_cre_mbx(&cmbx);
// 	if (mbx_samples < E_OK) {
// 		return mbx_samples;
// 	}

// 	/* Mutex: only one task at a time may touch app_stats.  TA_INHERIT
// 	   (priority inheritance) avoids priority inversion: a low-priority
// 	   holder is temporarily raised to the waiter's priority. */
// 	cmtx.mtxatr = TA_INHERIT;
// 	mtx_stats = tk_cre_mtx(&cmtx);
// 	return (mtx_stats < E_OK) ? mtx_stats : E_OK;
// }

// EXPORT ER subsystem3_start(void)
// {
	/* Pinned to core 0 (TP_PRC1): opposite core to the producer */


/*
 *	subsystem3.c
 *	Subsystem 3 -- Line Following + Barcode (Wing): objects + tasks.
 *
 *	Line-first: subsystem3_init() sets up the line path and
 *	subsystem3_start() starts the 20 ms line task.  The barcode files
 *	compile but their task is dormant.  To enable barcode too:
 *	  - call barcode_init() in subsystem3_init() (shares the one "adca"
 *	    open -- open it once, pass the descriptor, don't open twice), and
 *	  - start barcode_task() in subsystem3_start().
 */
#include "subsystem3.h"
#include "line_barcode.h"

EXPORT ER subsystem3_init(void)
{
	return line_init();		/* opens "adca", sets up line state */
}

EXPORT ER subsystem3_start(void)
{
	ID tid = app_start_task(line_task, S3_PRI_LINE, TP_PRC1, "line_task");

	return (tid < E_OK) ? tid : E_OK;
}

/*** end of file ***/

	ID tid = app_start_task(consumer_task, S3_PRI_CONSUMER, TP_PRC1,
				"consumer_task");
	return (tid < E_OK) ? tid : E_OK;
}
