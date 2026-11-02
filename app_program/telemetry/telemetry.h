/*
 *	telemetry.h
 *	Telemetry -- Wi-Fi / MQTT (Rifa).  Samples the car's state from the
 *	other subsystems, packs it into one 64-bit word (protocol.h TLM_*)
 *	and publishes it on robocar/<id>/telemetry.  Settings (CFG_*) arrive
 *	on robocar/<id>/config.
 *
 *	Reads only, never commands: every source is a read-only getter
 *	(TEAM.md IF-06, IF-09, IF-14, IF-16).
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

/* One sample of the whole car, packed (protocol.h); seq goes in bits 0-7 */
IMPORT UD	telemetry_sample(UW seq);

IMPORT ER	telemetry_init(void);
IMPORT ER	telemetry_start(void);
#if APP_TEST
IMPORT void	telemetry_test(void);
#endif

#endif /* TELEMETRY_H */
