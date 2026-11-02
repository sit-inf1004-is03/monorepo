/*
 *	remote.c
 *	Remote -- dashboard commands to Navigation, and the Wi-Fi-loss fault.
 *
 *	Command flow:  dashboard -> broker -> network bridge -> mbf_remote
 *	               -> here -> nav_post_remote() (IF-13) -> Navigation
 *
 *	Each message arrives as its topic and payload; the payload is parsed
 *	here (comms_parse_cmd) into a command number and its arguments.
 *	nav_post_remote() never waits, so a command reaches Navigation within
 *	its next step; Navigation itself decides whether it applies (START
 *	only from IDLE or STOPPED, CLEAR only from FAULT).
 *
 *	Between commands the task watches the connection.  When an online car
 *	goes offline it raises NAV_FLT_WIFI_LOSS (IF-12) once.  That bit is
 *	outside Navigation's stop mask: the car carries on, degraded.  It stays
 *	set, like every fault bit, until Navigation clears them all on a
 *	remote CLEAR in the FAULT state.
 *
 *	APIs shown: tk_rcv_mbf (with timeout), comms_subscribe, comms_log,
 *	            nav_post_remote, nav_raise_fault
 */

#include "remote.h"
#include "../navigation/navigation.h"

/* protocol.h keeps Navigation's numbers for the three remote commands */
_Static_assert(CMD_START == NAV_REMOTE_START &&
	       CMD_STOP == NAV_REMOTE_STOP &&
	       CMD_CLEAR == NAV_REMOTE_CLEAR,
	       "protocol.h CMD_* differ from navigation.h NAV_REMOTE_*");

#define LINK_CHECK_MS	500	/* how often the connection is looked at */

EXPORT ID	mbf_remote;

LOCAL const char *command_name(UH id)
{
	switch (id) {
	case CMD_START:	return "START";
	case CMD_STOP:	return "STOP";
	case CMD_CLEAR:	return "CLEAR";
	default:	return "?";
	}
}

LOCAL void execute(const COMMS_CMD *cmd, BOOL broadcast)
{
	UB		line[NET_PAYLOAD_MAX];
	const char	*from = broadcast ? " (broadcast)" : "";
	ER		er;

	switch (cmd->id) {
	case CMD_PING:
		tm_sprintf(line, (const UB*)"remote: pong %d%s", cmd->arg[0],
			   from);
		break;

	case CMD_START:
	case CMD_STOP:
	case CMD_CLEAR:
		er = nav_post_remote(cmd->id);
		if (er == E_OK) {
			tm_sprintf(line, (const UB*)"remote: %s to navigation%s",
				   command_name(cmd->id), from);
		} else {
			tm_sprintf(line, (const UB*)"remote: %s refused by"
				   " navigation (%d)%s", command_name(cmd->id),
				   er, from);
		}
		break;

	default:
		tm_sprintf(line, (const UB*)"remote: unknown command %d",
			   cmd->id);
		break;
	}
	comms_log((const char*)line);
}

/* Raise the Wi-Fi-loss fault once each time an online car goes offline */
LOCAL void check_link(BOOL *was_online)
{
	NET_INFO	net;

	comms_get_info(&net);
	if (*was_online && !net.online) {
		CHECK(nav_raise_fault(NAV_FLT_WIFI_LOSS), "nav_raise_fault");
		comms_log("remote: connection lost, Wi-Fi-loss fault raised");
	}
	*was_online = net.online;
}

LOCAL void remote_task(INT stacd, void *exinf)
{
	COMMS_MSG	msg;
	COMMS_CMD	cmd;
	UB		line[NET_PAYLOAD_MAX];
	BOOL		was_online = FALSE;
	ER		er;

	(void)stacd; (void)exinf;

	while (1) {
		er = tk_rcv_mbf(mbf_remote, &msg, LINK_CHECK_MS);
		check_link(&was_online);
		if (er == E_TMOUT) {
			continue;
		}
		if (er < E_OK) {
			CHECK(er, "tk_rcv_mbf");
			continue;
		}

		if (comms_parse_cmd(&msg, &cmd) != E_OK) {
			tm_sprintf(line, (const UB*)"remote: ignored a message"
				   " on %s: not \"<id> [arg ...]\"", msg.topic);
			comms_log((const char*)line);
			continue;
		}
		execute(&cmd, comms_topic_is(&msg,
					     COMMS_BROADCAST_TOPIC(TOPIC_CMD)));
	}
}

EXPORT ER remote_init(void)
{
	T_CMBF	cmbf = {0};
	ER	er;

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = COMMS_MSG_MBF_SIZE(8);
	cmbf.maxmsz = sizeof(COMMS_MSG);
	mbf_remote = tk_cre_mbf(&cmbf);
	if (mbf_remote < E_OK) {
		return mbf_remote;
	}

	/* Two topics, one buffer: commands for this car, and for all cars */
	er = comms_subscribe(COMMS_DEVICE_TOPIC(TOPIC_CMD), mbf_remote);
	if (er < E_OK) {
		return er;
	}
	return comms_subscribe(COMMS_BROADCAST_TOPIC(TOPIC_CMD), mbf_remote);
}

EXPORT ER remote_start(void)
{
	ID tid = app_start_task(remote_task, REMOTE_PRI, 0, "remote_task");
	return (tid < E_OK) ? tid : E_OK;
}
