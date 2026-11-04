/*
 *	comms.h
 *	Comms -- what subsystems use to send and receive MQTT messages.
 *
 *	No subsystem task calls lwIP or MQTT. The network bridge
 *	(app_program/net/) runs the MQTT client on the kernel's Wi-Fi service
 *	task, and tasks exchange messages with it through message buffers:
 *
 *	    task --comms_publish()--> TX message buffer --> bridge --> broker
 *	    broker --> bridge --> RX message buffer --tk_rcv_mbf()--> task
 *
 *	This module owns the TX message buffer, collects the subscriptions,
 *	and hands both to the bridge in comms_start(). It also parses the
 *	command format of protocol.h.
 *
 *	Receiving: a task creates its own message buffer (maxmsz =
 *	sizeof(COMMS_MSG)) and, in its _init(), subscribes it to topics:
 *
 *	    comms_subscribe(COMMS_DEVICE_TOPIC(TOPIC_CMD), my_mbf);
 *	    ...
 *	    COMMS_MSG msg;  COMMS_CMD cmd;
 *	    tk_rcv_mbf(my_mbf, &msg, TMO_FEVR);
 *	    if (comms_parse_cmd(&msg, &cmd) == E_OK) ...
 *
 *	A message for a task whose buffer is full is dropped: the bridge never
 *	waits for a task. Size the buffer with COMMS_MSG_MBF_SIZE(n).
 *
 *	Sending, from any task, never blocks:
 *
 *	    comms_log("battery low");
 *	    comms_telemetry(word);
 *	    comms_publish(COMMS_DEVICE_TOPIC("my/topic"), data, len, FALSE);
 *
 *	Log lines and plain publishes wait in the TX buffer while offline and
 *	are sent after reconnecting; telemetry is dropped while offline, as
 *	only the latest value matters. A full TX buffer drops the new message.
 */

#ifndef COMMS_H
#define COMMS_H

#include "../app.h"
#include "../protocol.h"
#include "../net/net_bridge.h"

#if TM_NET_LWIP
#include "wifi_secrets.h"	/* MQTT_CLIENT_ID */
#else
#define MQTT_CLIENT_ID	"offline"	/* built without the network */
#endif

/* Full topic names, as string literals */
#define COMMS_DEVICE_TOPIC(leaf)	"robocar/" MQTT_CLIENT_ID "/" leaf
#define COMMS_BROADCAST_TOPIC(leaf)	"robocar/all/" leaf

/* ---------------------------------------------------------------------
 *  Receiving
 * --------------------------------------------------------------------- */
typedef NET_MSG		COMMS_MSG;	/* topic + payload, as received */

#define COMMS_MAX_SUBS		6	/* topics that can be subscribed */

/* Buffer size for a message buffer holding at least n messages */
#define COMMS_MSG_MBF_SIZE(n)	((n) * (sizeof(COMMS_MSG) + 8))

/* Deliver messages on topic to mbf. topic must never change (a literal,
   e.g. from COMMS_DEVICE_TOPIC). Only before comms_start(). */
IMPORT ER	comms_subscribe(const char *topic, ID mbf);

/* Is msg from this topic? */
IMPORT BOOL	comms_topic_is(const COMMS_MSG *msg, const char *topic);

/* A command or setting: decimal integers, the identifier first */
#define COMMS_MAX_ARGS		4

typedef struct {
	UH	id;			/* CMD_xxx / CFG_xxx */
	INT	argc;			/* how many arguments followed it */
	W	arg[COMMS_MAX_ARGS];	/* unused ones are 0 */
} COMMS_CMD;

/* "2 60 -20 1500" -> id 2, argc 3, arg {60, -20, 1500, 0}; else E_PAR */
IMPORT ER	comms_parse_cmd(const COMMS_MSG *msg, COMMS_CMD *cmd);
IMPORT ER	comms_parse_text(const UB *text, INT len, COMMS_CMD *cmd);

/* ---------------------------------------------------------------------
 *  Sending.  E_OK = queued; E_TMOUT = dropped (queue full, or offline for
 *  telemetry); E_PAR = topic or payload too long (net_bridge.h limits).
 * --------------------------------------------------------------------- */
IMPORT ER	comms_publish(const char *topic, const void *data, INT len,
			      BOOL retain);

/* One line on robocar/<id>/log; also printed. Cut at the payload limit. */
IMPORT ER	comms_log(const char *text);

/* A packed telemetry word (protocol.h) on robocar/<id>/telemetry */
IMPORT ER	comms_telemetry(UD word);

/* 64-bit value -> decimal text + '\0' in buf (>= 21 bytes); returns length */
IMPORT INT	comms_format_u64(UB *buf, UD value);

/* ---------------------------------------------------------------------
 *  Connection state (NET_INFO: online, rssi, ip, counters)
 * --------------------------------------------------------------------- */
IMPORT void	comms_get_info(NET_INFO *info);

/* ---------------------------------------------------------------------
 *  Module entry points (called from usermain)
 * --------------------------------------------------------------------- */
IMPORT ER	comms_init(void);
IMPORT ER	comms_start(void);
#if APP_TEST
IMPORT void	comms_test(void);
#endif

#endif /* COMMS_H */
