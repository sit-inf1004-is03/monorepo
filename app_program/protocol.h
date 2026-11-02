/*
 *	protocol.h
 *	What the car and the dashboard say to each other over MQTT.
 *	Owner: Wi-Fi / MQTT (Rifa).  TEAM.md interfaces IF-15 .. IF-17.
 *
 *	THE DASHBOARD HAS A COPY OF THIS FILE:
 *	    mqtt-web-interface/src/shared/protocol.ts
 *	Change both together, or the two sides will disagree.
 *
 *	Topics (<id> is MQTT_CLIENT_ID from config/wifi_secrets.h)
 *	----------------------------------------------------------
 *	  car -> dashboard
 *	    robocar/<id>/status     "online" / "offline"  (retained; "offline"
 *	                            is the MQTT last will, sent by the broker
 *	                            when the car drops off without saying so)
 *	    robocar/<id>/telemetry  one unsigned 64-bit integer in decimal,
 *	                            e.g. "6601995533221430314", packed as TLM_*
 *	    robocar/<id>/log        one line of plain text
 *
 *	  dashboard -> car
 *	    robocar/<id>/cmd        a command:  "<CMD_*> [arg ...]"
 *	    robocar/all/cmd         the same, for every car at once
 *	    robocar/<id>/config     a setting:  "<CFG_*> <value>"
 *
 *	A command or setting is decimal integers separated by spaces, the
 *	first being the identifier, e.g. "1" = CMD_START.  At most
 *	COMMS_MAX_ARGS arguments; missing arguments read as 0.
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

/* ---------------------------------------------------------------------
 *  Topic leaves: the last part of robocar/<id>/<leaf>
 * --------------------------------------------------------------------- */
#define TOPIC_STATUS		"status"
#define TOPIC_TELEMETRY		"telemetry"
#define TOPIC_LOG		"log"
#define TOPIC_CMD		"cmd"
#define TOPIC_CONFIG		"config"

#define STATUS_ONLINE		"online"
#define STATUS_OFFLINE		"offline"

/* ---------------------------------------------------------------------
 *  Commands, on robocar/<id>/cmd and robocar/all/cmd (task: remote).
 *  START, STOP and CLEAR are Navigation's remote commands (IF-13) and
 *  keep its numbers (NAV_REMOTE_*, checked in remote.c).
 * --------------------------------------------------------------------- */
#define CMD_PING	0	/* [token]  reply "pong <token>" on log */
#define CMD_START	1	/* IDLE or STOPPED: follow the line */
#define CMD_STOP	2	/* brake and wait */
#define CMD_CLEAR	3	/* FAULT: clear the faults, back to IDLE */

/* ---------------------------------------------------------------------
 *  Settings, on robocar/<id>/config (task: telemetry)
 * --------------------------------------------------------------------- */
#define CFG_TELEMETRY_PERIOD	1	/* ms between samples, 100..60000 */
#define CFG_TELEMETRY_ENABLE	2	/* 0 = stop sending, 1 = send */

/* ---------------------------------------------------------------------
 *  Telemetry: one 64-bit word, fields packed from bit 0 upwards.
 *
 *  Each field is written as "offset, width" so it can be passed straight
 *  to tlm_put()/tlm_get():   word = tlm_put(word, TLM_NAV_STATE, state);
 *  Every field is unsigned.  Sources are the team's interfaces in TEAM.md.
 * --------------------------------------------------------------------- */
/*	name			offset, bits	   meaning (source) */
#define TLM_SEQ			 0, 8	/* sample counter, wraps; gaps = loss */
#define TLM_NAV_STATE		 8, 3	/* NAV_STATE_*  (nav_get_state, IF-14) */
#define TLM_NAV_REASON		11, 3	/* NAV_REASON_* (nav_get_reason, IF-14) */
#define TLM_FAULTS		14, 8	/* bit n = fault bit n of the team's map,
					   NAV_FLT_*    (nav_get_faults, IF-14) */
#define TLM_MOTION		22, 2	/* MOTION_ACTIVITY_* (motion_get_activity,
					   IF-09) */
#define TLM_LINE		24, 2	/* LINE_ON/LOST/SEARCHING (line_get_state,
					   S3), 3 = not reported */
#define TLM_SPEED_LEFT		26, 10	/* mm/s, 1023 = 1023 or more
					   (drv_encoder_get_speed, IF-06) */
#define TLM_SPEED_RIGHT		36, 10	/* mm/s, as above */
#define TLM_OBSTACLE		46, 9	/* cm to the nearest obstacle ahead,
					   511 = no echo or no sensor
					   (ultrasonic_last_reading, S5) */
#define TLM_RSSI		55, 7	/* Wi-Fi signal as -dBm, 0 = unknown */
#define TLM_VERSION		62, 2	/* layout version: TLM_LAYOUT */
/*					   64 bits used of 64 */

#define TLM_LAYOUT		1	/* bump when the layout above changes */
#define TLM_LINE_UNKNOWN	3
#define TLM_SPEED_MAX		1023
#define TLM_OBSTACLE_NONE	511

/* Store value in a field (excess high bits are cut off) */
static inline unsigned long long tlm_put(unsigned long long word,
					 unsigned int offset, unsigned int bits,
					 unsigned long value)
{
	unsigned long long mask = ((1ULL << bits) - 1) << offset;

	return (word & ~mask) | (((unsigned long long)value << offset) & mask);
}

/* Read a field back (used by the self-tests) */
static inline unsigned long tlm_get(unsigned long long word,
				    unsigned int offset, unsigned int bits)
{
	return (unsigned long)((word >> offset) & ((1ULL << bits) - 1));
}

#endif /* PROTOCOL_H */
