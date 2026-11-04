/*
 *	net_bridge.h
 *	The network bridge: MQTT on the kernel's Wi-Fi service task, message
 *	buffers towards the application.
 *
 *	NOT A SUBSYSTEM. No subsystem task calls anything in app_program/net/
 *	except through comms/ (comms.h). Everything that touches lwIP, MQTT or
 *	the radio runs on the kernel's Wi-Fi service task (cyw43_utk_task);
 *	the rest of the application exchanges messages with it like this:
 *
 *	    task --NET_MSG--> TX message buffer --> bridge --> broker
 *	    broker --> bridge --NET_MSG--> RX message buffer --> task
 *
 *	Why it lives here and not in the kernel's mqtt_utk.c: see
 *	KERNEL_ISSUES.md. In short, the kernel is not modified, and its
 *	MQTT=1 build starts a heartbeat client that would clash with this one,
 *	so this is built with "make NET=lwip".
 *
 *	Limits and what happens at them:
 *	  - a topic longer than NET_TOPIC_MAX or a payload longer than
 *	    NET_PAYLOAD_MAX is dropped and counted (rx_dropped / tx_dropped);
 *	  - a full RX message buffer drops the incoming message (counted):
 *	    the bridge never waits for a task;
 *	  - while offline, TX messages stay in the TX buffer and are sent
 *	    after reconnecting; when it is full, the sender's tk_snd_mbf(...,
 *	    TMO_POL) fails and the sender drops the message.
 *
 *	This header uses only <tk/typedef.h> types, so it can be included both
 *	next to tk/tkernel.h and next to the Pico SDK's cyw43.h (which cannot
 *	share a file with tk/tkernel.h: both define size_t).
 */

#ifndef NET_BRIDGE_H
#define NET_BRIDGE_H

#include <tk/typedef.h>

#define NET_TOPIC_MAX		63	/* bytes, without the '\0' */
#define NET_PAYLOAD_MAX		128	/* bytes */

/* One message, in either direction. In a message buffer only the used
 * part of payload[] is stored: send NET_MSG_SIZE(msg.len) bytes, and
 * create each buffer with maxmsz = sizeof(NET_MSG). */
typedef struct {
	UB	qos;				/* TX: 0 or 1 */
	UB	retain;				/* TX: 1 = broker keeps it */
	UH	len;				/* payload bytes */
	char	topic[NET_TOPIC_MAX + 1];	/* '\0'-terminated */
	UB	payload[NET_PAYLOAD_MAX];	/* not terminated */
} NET_MSG;

#define NET_MSG_SIZE(len)	\
	((INT)(sizeof(NET_MSG) - NET_PAYLOAD_MAX) + (INT)(len))

/* A subscription: messages on matching topics ('+' and '#' wildcards
 * allowed) are copied to rx_mbf. The first matching entry wins. */
typedef struct {
	const char	*topic;
	ID		rx_mbf;
	UB		qos;		/* 0 or 1 */
} NET_SUB;

typedef struct {
	ID		tx_mbf;		/* tasks -> broker */
	const NET_SUB	*subs;		/* broker -> tasks */
	INT		sub_count;
	/* Optional, NULL = none. "online" is published here (retained) on
	 * every connect; the broker publishes "offline" as the last will
	 * when the car disappears without saying so. */
	const char	*status_topic;
	/* Optional, NULL = console only. The bridge's own messages (link
	 * lost, retrying in ...) are queued here too, so they reach the
	 * broker after a reconnect. */
	const char	*log_topic;
} NET_BRIDGE;

/* Start the bridge, from a task, once, after creating every message
 * buffer named in *bridge (net_start.c). The bridge keeps the pointer:
 * *bridge, its table and its strings must never change or go away. */
IMPORT ER	net_start(const NET_BRIDGE *bridge);

/* ---------------------------------------------------------------------
 *  Connection state, readable from any task
 * --------------------------------------------------------------------- */
typedef enum {
	NET_WIFI_JOIN,		/* asking the radio to join */
	NET_WIFI_WAIT,		/* joining, or waiting for a DHCP lease */
	NET_WIFI_BACKOFF,	/* Wi-Fi failed; waiting to try again */
	NET_WIFI_SCAN,		/* listing the networks in range */
	NET_MQTT_CONNECT,	/* opening a session with the broker */
	NET_MQTT_WAIT,		/* waiting for the broker to accept it */
	NET_MQTT_SUBSCRIBE,	/* (re)subscribing to every topic */
	NET_MQTT_BACKOFF,	/* broker failed; waiting to try again */
	NET_ONLINE		/* connected and subscribed */
} NET_STATE;

typedef struct {
	NET_STATE	state;
	BOOL		online;		/* state == NET_ONLINE */
	W		rssi;		/* dBm, 0 = unknown */
	UW		ip;		/* IPv4, first octet in bits 0-7 */
	UW		wifi_joins;	/* join attempts since boot */
	UW		mqtt_connects;	/* broker connection attempts */
	UW		rx_dropped;	/* incoming messages not delivered */
	UW		tx_dropped;	/* outgoing messages not sent */
} NET_INFO;

IMPORT void		net_get_info(NET_INFO *info);
IMPORT const char	*net_state_name(NET_STATE state);

/* ---------------------------------------------------------------------
 *  Exposed for the self-tests (comms_test.c)
 * --------------------------------------------------------------------- */
typedef struct {
	UW	min_ms;		/* first wait */
	UW	max_ms;		/* longest wait */
	UW	step_ms;	/* the wait the next failure gets */
} NET_BACKOFF;

#define NET_BACKOFF_INIT(min_ms, max_ms)	{ (min_ms), (max_ms), (min_ms) }

IMPORT void	net_backoff_reset(NET_BACKOFF *b);
IMPORT UW	net_backoff_next(NET_BACKOFF *b, UW random);
IMPORT BOOL	net_topic_matches(const char *filter, const char *topic);

/* ---------------------------------------------------------------------
 *  Internal: between net_start.c and net_bridge.c
 * --------------------------------------------------------------------- */
IMPORT void	net_bridge_arm(const NET_BRIDGE *bridge);

#endif /* NET_BRIDGE_H */
