/**
 * @file    remote.h
 * @brief   Remote: Wi-Fi / MQTT (Rifa).  Carries out commands from the
 *          dashboard (protocol.h CMD_*) by passing START, STOP and CLEAR to
 *          Navigation (IF-13), and reports a lost connection as the team's
 *          Wi-Fi-loss fault bit (IF-12: degraded only, the car does not
 *          stop).  Coded to BARR-C:2018.
 *
 * It never commands Motion: Navigation is the only module that does.
 */

#ifndef REMOTE_H
#define REMOTE_H

#include "../../app.h"

#define REMOTE_PRI              (22)    /* Band 22-23 (TEAM.md section 4). */
#define REMOTE_MBF_MESSAGES     (8u)    /* Commands its buffer holds. */

ER remote_init (void);
ER remote_start (void);
ID remote_get_mbf (void);

#endif /* REMOTE_H */

/*** end of file ***/
