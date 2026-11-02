/*
 *	subsystem3.h
 *	Subsystem 3 -- Line Following + Barcode (Wing), plus the consumer
 *	task (IF-01 / IF-04) that subsystems 2 and 4 still use.
 *
 *	Public hooks only.  The line/barcode interface (pins, decode,
 *	prototypes) lives in line_barcode.h, included by the .c files.
 */
#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"

/* Priority band 16-17 (TEAM.md section 4). */
#define S3_PRI_CONSUMER		17
#define S3_PRI_BARCODE		16
#define S3_PRI_LINE		16
#define S3_BATCH_SIZE		5	/* report to subsystem 4 every N samples */

/* IF-01: send SAMPLE_MSG (subsystem2.h) here with tk_snd_mbx */
IMPORT ID	mbx_samples;

/* IF-04: statistics.  Read them ONLY while holding mtx_stats:
 *	tk_loc_mtx(mtx_stats, TMO_FEVR);  copy = app_stats;  tk_unl_mtx(mtx_stats);
 */
typedef struct {
	UW	received;	/* samples received so far */
	INT	min;
	INT	max;
	W	sum;
} APP_STATS;

IMPORT ID		mtx_stats;
IMPORT APP_STATS	app_stats;

/* Private to subsystem 3 */
IMPORT void	consumer_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM3_H */
