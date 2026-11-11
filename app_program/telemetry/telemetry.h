/*
 *	telemetry.h
 *	Telemetry -- Wi-Fi / MQTT (Rifa).  Samples the car's state from the
 *	other subsystems, packs it into 64-bit words (protocol.h TLM_*,
 *	TLM_W1_*) and publishes them on robocar/<id>/telemetry.  Humps and
 *	barcodes go out as they happen on robocar/<id>/event (EVT_*).
 *	Settings (CFG_*) arrive on robocar/<id>/config.
 *
 *	Reads only, never commands: every source is a read-only getter or
 *	stream (TEAM.md IF-06, IF-09, IF-14, IF-16, IF-18, IF-S4-1, IF-S4-3,
 *	IF-S4-7).
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "../app.h"
#include "../comms/comms.h"

#define TELEMETRY_PRI		23	/* band 22-23 (TEAM.md section 4) */

/* COMMS_MSG messages in: CFG_* settings from robocar/<id>/config */
IMPORT ID		mbf_telemetry;

/* Current settings, changed only by the telemetry task */
IMPORT volatile UW	telemetry_period_ms;
IMPORT volatile BOOL	telemetry_enabled;

/* One sample of the whole car, packed (protocol.h): word 0, with seq in
   bits 0-7, and word 1 */
IMPORT UD	telemetry_sample(UW seq);
IMPORT UD	telemetry_sample_w1(void);

/* telemetry_events.c: events (EVT_*) and the last barcode read */
IMPORT void	telemetry_events_init(void);
IMPORT void	telemetry_events_poll(void);	/* telemetry task, often */
IMPORT UW	telemetry_last_barcode(void);	/* letter | (count << 8) */

/* Pure helpers, exported for the self-tests */
IMPORT UB	telemetry_barcode_in_log(const char *line);  /* letter, or 0 */
IMPORT INT	telemetry_event_text(UB *buf, UH imu_event); /* 0 = not sent */

IMPORT ER	telemetry_init(void);
IMPORT ER	telemetry_start(void);
#if APP_TEST
IMPORT void	telemetry_test(void);

/* TRUE: leave the IMU's events alone, so subsystem4_test() can read them
   (S4-U06 waits for an IMPACT event).  Set in usermain() around it. */
IMPORT void	telemetry_hold_imu_events(BOOL hold);
#endif

#endif /* TELEMETRY_H */
