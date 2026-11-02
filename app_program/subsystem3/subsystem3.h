/*
 *	subsystem3.h
 *	Subsystem 3 -- Line Following + Barcode (Wing).  Student ID: <id>.
 *
 *	Public hooks only.  The subsystem's own interface (pins, decode,
 *	prototypes) lives in line_barcode.h, included by the .c files.
 *	usermain() calls subsystem3_init() then subsystem3_start().
 */
#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"

/* Priority band 16-17 (TEAM.md section 4). */
#define S3_PRI_BARCODE		16
#define S3_PRI_LINE		16

#endif /* SUBSYSTEM3_H */
