/*
 *	subsystem3.c
 *	Subsystem 3 -- Line Following + Barcode (Wing): objects + tasks.
 *
 *	subsystem3_init() creates the consumer's mailbox and mutex (IF-01,
 *	IF-04, still used by subsystems 2 and 4) and sets up the line path.
 *	subsystem3_start() starts the consumer and the 20 ms line task.
 *	The barcode files compile but their task is dormant.  To enable it:
 *	  - call barcode_init() in subsystem3_init() (shares the one "adca"
 *	    open -- open it once, pass the descriptor, don't open twice), and
 *	  - start barcode_task() in subsystem3_start().
 */
#include "subsystem3.h"
#include "line_barcode.h"

EXPORT ID		mbx_samples;	/* IF-01 */
EXPORT ID		mtx_stats;	/* IF-04 */
EXPORT APP_STATS	app_stats;

EXPORT ER subsystem3_init(void)
{
	T_CMBX	cmbx = {0};
	T_CMTX	cmtx = {0};

	cmbx.mbxatr = TA_TFIFO | TA_MFIFO;
	mbx_samples = tk_cre_mbx(&cmbx);
	if (mbx_samples < E_OK) {
		return mbx_samples;
	}

	cmtx.mtxatr = TA_INHERIT;
	mtx_stats = tk_cre_mtx(&cmtx);
	if (mtx_stats < E_OK) {
		return mtx_stats;
	}

	return line_init();		/* opens "adca", sets up line state */
}

EXPORT ER subsystem3_start(void)
{
	ID tid = app_start_task(consumer_task, S3_PRI_CONSUMER, TP_PRC1,
				"consumer_task");
	if (tid < E_OK) {
		return tid;
	}

	tid = app_start_task(line_task, S3_PRI_LINE, TP_PRC1, "line_task");
	return (tid < E_OK) ? tid : E_OK;
}

/*** end of file ***/
