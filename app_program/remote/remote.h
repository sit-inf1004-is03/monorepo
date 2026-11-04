/*
 *	remote.h
 *	Remote -- Wi-Fi / MQTT (Rifa).  Carries out commands from the
 *	dashboard (protocol.h CMD_*) by passing START / STOP / CLEAR to
 *	Navigation (IF-13), and reports a lost connection as the team's
 *	Wi-Fi-loss fault bit (IF-12: degraded only, the car does not stop).
 *
 *	It never commands Motion: Navigation is the only module that does.
 */

#ifndef REMOTE_H
#define REMOTE_H

#include "../app.h"
#include "../comms/comms.h"

#define REMOTE_PRI		22	/* band 22-23 (TEAM.md section 4) */

/* COMMS_MSG messages in: robocar/<id>/cmd and robocar/all/cmd */
IMPORT ID	mbf_remote;

IMPORT ER	remote_init(void);
IMPORT ER	remote_start(void);
#if APP_TEST
IMPORT void	remote_test(void);
#endif

#endif /* REMOTE_H */
