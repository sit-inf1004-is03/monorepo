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
