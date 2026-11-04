/*
 *	net_bridge.c
 *	The network bridge: everything here runs on the kernel's Wi-Fi service
 *	task (cyw43_utk_task), the only task allowed to call lwIP, MQTT or the
 *	radio. How it gets there without changing the kernel: net_start.c.
 *
 *	bridge_tick() runs every TICK_MS as an lwIP timer that re-arms itself;
 *	the kernel's loop runs it from sys_check_timeouts(), as it does the
 *	lwIP callbacks further down. Every message-buffer call here uses
 *	TMO_POL, so a slow or stuck task can never hold up the radio or the
 *	network stack.
 *
 *	bridge_tick() is a state machine over the three layers that can fail:
 *
 *	    Wi-Fi join ---> DHCP lease ---> MQTT session ---> ONLINE
 *
 *	  - Before every join attempt (and once after the kernel's own join at
 *	    boot) the radio scans and the console lists every network it can
 *	    see, marking the one in WIFI_SSID. Set NET_SCAN_BEFORE_JOIN
 *	    (below) to 0 to skip it.
 *	  - A failed layer is retried after a wait that doubles with every
 *	    failure in a row, with jitter (net_util.c): Wi-Fi 1 s .. 60 s,
 *	    MQTT 1 s .. 30 s. Success resets that layer's wait.
 *	  - Losing a lower layer closes the layers above it first.
 *	  - No DHCP lease within WIFI_LINK_TIMEOUT_MS re-joins the Wi-Fi, and
 *	    MQTT_FAILS_BEFORE_REJOIN broker failures in a row re-join it too,
 *	    in case the link itself is stale.
 *	  - lwIP's MQTT keepalive closes a session whose broker went quiet.
 *	  - Subscriptions are made again after every connect (lwIP always
 *	    asks for a clean session, so the broker forgets them).
 *
 *	Pico SDK side of the size_t split: cyw43.h is included, tk/tkernel.h
 *	is not. The two kernel calls used are declared by hand below, as the
 *	kernel's own cyw43_utk.c does for tk_dly_tsk().
 */

#if TM_NET_LWIP

#include <string.h>
#include "cyw43.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/sys.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "net_bridge.h"
#include <tk/errno.h>
#include <tm/tmonitor.h>

/* Exactly as declared in tk/syscall.h */
IMPORT ER	tk_snd_mbf(ID mbfid, CONST void *msg, INT msgsz, TMO tmout);
IMPORT INT	tk_rcv_mbf(ID mbfid, void *msg, TMO tmout);

#define TICK_MS			10	/* how often bridge_tick() runs */
#define WIFI_LINK_TIMEOUT_MS	30000	/* join + DHCP must finish within */
#define MQTT_CONNECT_TIMEOUT_MS	10000	/* broker must accept within */
#define MQTT_KEEPALIVE_S	30
#define MQTT_FAILS_BEFORE_REJOIN 5
#define RSSI_PERIOD_MS		5000
#define TX_PER_TICK		8	/* queued messages sent per tick, at most */

/* List the visible networks before joining; 0 = don't */
#ifndef NET_SCAN_BEFORE_JOIN
#define NET_SCAN_BEFORE_JOIN	1
#endif
#define SCAN_TIMEOUT_MS		10000	/* a scan normally takes 2-4 s */
#define SCAN_MAX		16	/* access points listed per scan */

#define STATUS_ONLINE		"online"
#define STATUS_OFFLINE		"offline"

/* Same default and makefile override as the kernel's join at boot
   (lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c) */
#ifndef WIFI_AUTH_MODE
#define WIFI_AUTH_MODE		CYW43_AUTH_WPA2_MIXED_PSK
#endif

#define MEMORY_BARRIER()	__asm__ volatile ("dmb" ::: "memory")

LOCAL volatile NET_INFO		info;		/* read by any task */

/* Everything below is used on the Wi-Fi task only */
LOCAL const NET_BRIDGE		*bridge;
LOCAL NET_BACKOFF		wifi_backoff = NET_BACKOFF_INIT(1000, 60000);
LOCAL NET_BACKOFF		mqtt_backoff = NET_BACKOFF_INIT(1000, 30000);
LOCAL mqtt_client_t		*client;
LOCAL ip_addr_t			broker_addr;
LOCAL NET_STATE			state;
LOCAL u32_t			deadline;	/* timeout / retry time */
LOCAL INT			mqtt_fails;	/* broker failures in a row */
LOCAL u32_t			next_rssi;
LOCAL INT			sub_next;	/* next subscription to make */
LOCAL BOOL			status_sent;

/* Set by the lwIP callbacks, acted on by bridge_tick() */
LOCAL enum { CONN_NONE, CONN_ACCEPTED, CONN_LOST } conn_event;
LOCAL INT			conn_status;	/* mqtt_connection_status_t */
LOCAL BOOL			subscribe_failed;

/* The networks seen by the scan in progress, one entry per access point */
typedef struct {
	UB	bssid[6];
	char	ssid[33];	/* printable, '\0'-terminated */
	W	rssi;		/* dBm, strongest heard */
	UH	channel;
	UB	auth;		/* scan result bits: 4 RSN, 2 WPA, 1 encrypted */
	BOOL	listed;
} SEEN_AP;

LOCAL SEEN_AP			seen[SCAN_MAX];
LOCAL INT			seen_count;
LOCAL BOOL			seen_more;	/* more APs than SCAN_MAX */
LOCAL NET_STATE			after_scan;	/* state to go to when done */
LOCAL u32_t			scan_started;
LOCAL BOOL			scanned_once;

/* The message being received; copied out before the last callback ends */
LOCAL NET_MSG			rx_msg;
LOCAL INT			rx_sub = -1;	/* index in bridge->subs */
LOCAL BOOL			rx_too_long;

/* The message being sent, kept while lwIP's output buffer is full */
LOCAL NET_MSG			tx_msg;
LOCAL BOOL			tx_pending;

LOCAL BOOL time_reached(u32_t t)
{
	return (W)(sys_now() - t) >= 0;
}

LOCAL void enter(NET_STATE s)
{
	state = s;
	info.state = s;
	info.online = (s == NET_ONLINE);
}

/* ---------------------------------------------------------------------
 *  Logging: console, and queued on the TX buffer like a task's message,
 *  so the dashboard also sees what happened while the car was offline
 * --------------------------------------------------------------------- */
LOCAL void bridge_log(const char *line)
{
	static NET_MSG	msg;
	size_t		len = strlen(line);

	tm_printf((UB*)"%s\n", (UB*)line);
	if (bridge->log_topic == NULL) {
		return;
	}

	if (len > NET_PAYLOAD_MAX) {
		len = NET_PAYLOAD_MAX;			/* cut long lines */
	}
	msg.qos = 0;
	msg.retain = 0;
	msg.len = (UH)len;
	strncpy(msg.topic, bridge->log_topic, NET_TOPIC_MAX);
	msg.topic[NET_TOPIC_MAX] = '\0';
	memcpy(msg.payload, line, len);
	if (tk_snd_mbf(bridge->tx_mbf, &msg, NET_MSG_SIZE(len), TMO_POL) < E_OK) {
		info.tx_dropped++;
	}
}

/* Keep it short: tm_sprintf() does not check the buffer size */
#define LOGF(...)							\
	do {								\
		UB log_line_[NET_PAYLOAD_MAX + 32];			\
		tm_sprintf(log_line_, (const UB*)__VA_ARGS__);		\
		bridge_log((const char*)log_line_);			\
	} while (0)

/* ---------------------------------------------------------------------
 *  Wi-Fi + DHCP layer
 * --------------------------------------------------------------------- */
LOCAL const char *link_text(int link)
{
	switch (link) {
	case CYW43_LINK_DOWN:		return "link down";
	case CYW43_LINK_JOIN:		return "joining";
	case CYW43_LINK_NOIP:		return "no DHCP lease";
	case CYW43_LINK_UP:		return "up";
	case CYW43_LINK_FAIL:		return "association failed";
	case CYW43_LINK_NONET:		return "SSID not found (2.4 GHz only)";
	case CYW43_LINK_BADAUTH:	return "wrong password or security mode";
	default:			return "unknown link state";
	}
}

/*
 * Drop any association or failed attempt, so the link reads DOWN.
 *
 * Clearing wifi_join_state is what the driver itself does when the access
 * point disassociates (cyw43_ctrl.c, CYW43_EV_DISASSOC). It is needed
 * because the kernel's Wi-Fi task re-joins on a fixed 5 s timer whenever
 * it sees a FAILED status; moving the status to DOWN cancels that timer,
 * leaving the timing of the next attempt to the backoff alone.
 */
LOCAL void wifi_reset(void)
{
	(void)cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
	cyw43_state.wifi_join_state = 0;
}

/* ---------------------------------------------------------------------
 *  Scan: list the networks in range, so a failing join can be told apart
 *  from a network the radio cannot see at all (wrong name, out of range,
 *  5 GHz only)
 * --------------------------------------------------------------------- */
LOCAL const char *auth_text(UB auth)
{
	if (auth & 4) {
		return (auth & 2) ? "WPA/WPA2" : "WPA2/WPA3";
	}
	if (auth & 2) {
		return "WPA";
	}
	return (auth & 1) ? "WEP" : "open";
}

/* Called from cyw43_poll() on this task, once per beacon heard; the same
   access point usually answers several times */
LOCAL int on_scan_result(void *env, const cyw43_ev_scan_result_t *r)
{
	SEEN_AP	*ap;
	INT	i;

	(void)env;
	if (r == NULL) {
		return 0;
	}

	for (i = 0; i < seen_count; i++) {
		if (memcmp(seen[i].bssid, r->bssid, sizeof(seen[i].bssid)) == 0) {
			if (r->rssi > seen[i].rssi) {
				seen[i].rssi = r->rssi;
			}
			return 0;
		}
	}
	if (seen_count == SCAN_MAX) {
		seen_more = TRUE;
		return 0;
	}

	ap = &seen[seen_count++];
	memcpy(ap->bssid, r->bssid, sizeof(ap->bssid));
	for (i = 0; i < r->ssid_len && i < 32; i++) {
		UB c = r->ssid[i];
		ap->ssid[i] = (c >= 0x20 && c < 0x7f) ? (char)c : '?';
	}
	ap->ssid[i] = '\0';
	ap->rssi    = r->rssi;
	ap->channel = r->channel;
	ap->auth    = r->auth_mode;
	ap->listed  = FALSE;
	return 0;
}

/* Scan, then go to state next. With NET_SCAN_BEFORE_JOIN 0, go there at once. */
LOCAL void scan_then(NET_STATE next)
{
	cyw43_wifi_scan_options_t	opts = {0};
	int				err;

	after_scan = next;
	scanned_once = TRUE;
	if (!NET_SCAN_BEFORE_JOIN) {
		enter(next);
		return;
	}

	seen_count = 0;
	seen_more = FALSE;
	err = cyw43_wifi_scan(&cyw43_state, &opts, NULL, on_scan_result);
	if (err != 0) {
		tm_printf((UB*)"net: wifi scan did not start (%d)\n", err);
		enter(next);
		return;
	}
	tm_printf((UB*)"net: scanning for Wi-Fi networks ...\n");
	scan_started = sys_now();
	deadline = scan_started + SCAN_TIMEOUT_MS;
	enter(NET_WIFI_SCAN);
}

/* Print what the scan found, strongest first, on the console. Only a
   one-line summary goes to the log topic: a full list on every retry
   would fill the TX buffer while offline. */
LOCAL void scan_report(void)
{
	SEEN_AP	*want = NULL;
	SEEN_AP	*best;
	INT	i, n;

	tm_printf((UB*)"net: %d Wi-Fi access point(s) visible after %u ms:\n",
		  seen_count, (UW)(sys_now() - scan_started));

	for (n = 0; n < seen_count; n++) {
		best = NULL;
		for (i = 0; i < seen_count; i++) {
			if (!seen[i].listed && (best == NULL || seen[i].rssi > best->rssi)) {
				best = &seen[i];
			}
		}
		best->listed = TRUE;

		tm_printf((UB*)"net:   %4d dBm  ch %2d  %02x:%02x:%02x:%02x:%02x:%02x"
			  "  %-9s  \"%s\"%s%s\n",
			  best->rssi, best->channel,
			  best->bssid[0], best->bssid[1], best->bssid[2],
			  best->bssid[3], best->bssid[4], best->bssid[5],
			  auth_text(best->auth), best->ssid,
			  (best->ssid[0] == '\0') ? " (hidden)" : "",
			  (strcmp(best->ssid, WIFI_SSID) == 0) ? "  <-- WIFI_SSID" : "");

		if (want == NULL && strcmp(best->ssid, WIFI_SSID) == 0) {
			want = best;		/* strongest one with our name */
		}
	}
	if (seen_more) {
		tm_printf((UB*)"net:   (and more, not listed)\n");
	}

	if (want != NULL) {
		LOGF("net: wifi scan: %d APs; \"%s\" seen at %d dBm, ch %d, %s",
		     seen_count, WIFI_SSID, want->rssi, want->channel,
		     auth_text(want->auth));
	} else {
		LOGF("net: wifi scan: %d APs; \"%s\" not among them (out of"
		     " range, 5 GHz only, or another name?)", seen_count,
		     WIFI_SSID);
	}
}

LOCAL void wifi_retry_later(const char *why)
{
	UW	wait;

	wifi_reset();
	wait = net_backoff_next(&wifi_backoff, LWIP_RAND());
	LOGF("net: wifi %s; join again in %u ms", why, wait);
	deadline = sys_now() + wait;
	enter(NET_WIFI_BACKOFF);
}

LOCAL void wifi_join(void)
{
	int	err;

	info.wifi_joins++;
	tm_printf((UB*)"net: joining \"%s\" (attempt %u)\n", WIFI_SSID,
		  info.wifi_joins);

	/* The BSSID argument is NULL: the chip picks any AP with this SSID */
	err = cyw43_wifi_join(&cyw43_state,
			      strlen(WIFI_SSID), (const uint8_t*)WIFI_SSID,
			      strlen(WIFI_PASSWORD), (const uint8_t*)WIFI_PASSWORD,
			      WIFI_AUTH_MODE, NULL, CYW43_CHANNEL_NONE);
	if (err != 0) {
		wifi_retry_later("join request refused by the driver");
		return;
	}
	deadline = sys_now() + WIFI_LINK_TIMEOUT_MS;
	enter(NET_WIFI_WAIT);
}

LOCAL void wifi_wait(int link)
{
	UW	ip;

	switch (link) {
	case CYW43_LINK_UP:
		ip = ip4_addr_get_u32(netif_ip4_addr(&cyw43_state.netif[CYW43_ITF_STA]));
		info.ip = ip;
		net_backoff_reset(&wifi_backoff);
		LOGF("net: wifi up, ip %d.%d.%d.%d", ip & 0xff,
		     (ip >> 8) & 0xff, (ip >> 16) & 0xff, ip >> 24);
		if (!scanned_once) {
			/* The kernel joined at boot without a scan: list
			   the networks once now, for the record */
			scan_then(NET_MQTT_CONNECT);
		} else {
			enter(NET_MQTT_CONNECT);
		}
		break;
	case CYW43_LINK_JOIN:
	case CYW43_LINK_NOIP:
		if (time_reached(deadline)) {
			wifi_retry_later(link == CYW43_LINK_NOIP
					 ? "got no DHCP lease in time"
					 : "join timed out");
		}
		break;
	default:			/* DOWN or a failure */
		wifi_retry_later(link_text(link));
		break;
	}
}

/* ---------------------------------------------------------------------
 *  MQTT layer
 * --------------------------------------------------------------------- */
LOCAL void mqtt_retry_later(const char *why)
{
	UW	wait;

	mqtt_disconnect(client);
	enter(NET_MQTT_BACKOFF);	/* not online: telemetry stops queueing */

	if (++mqtt_fails >= MQTT_FAILS_BEFORE_REJOIN) {
		mqtt_fails = 0;
		net_backoff_reset(&mqtt_backoff);
		LOGF("net: mqtt %s, %d times in a row", why,
		     MQTT_FAILS_BEFORE_REJOIN);
		wifi_retry_later("may be stale");
		return;
	}

	wait = net_backoff_next(&mqtt_backoff, LWIP_RAND());
	LOGF("net: mqtt %s (%d); connect again in %u ms", why, conn_status,
	     wait);
	deadline = sys_now() + wait;
}

LOCAL void on_connection(mqtt_client_t *c, void *arg,
			 mqtt_connection_status_t status)
{
	(void)c; (void)arg;

	conn_status = (INT)status;
	conn_event  = (status == MQTT_CONNECT_ACCEPTED) ? CONN_ACCEPTED
							: CONN_LOST;
}

LOCAL void mqtt_connect(void)
{
	struct mqtt_connect_client_info_t	ci = {0};
	err_t					err;

	ci.client_id  = MQTT_CLIENT_ID;
	ci.keep_alive = MQTT_KEEPALIVE_S;
	if (bridge->status_topic != NULL) {
		ci.will_topic  = bridge->status_topic;	/* sent by the broker */
		ci.will_msg    = STATUS_OFFLINE;	/* if we vanish */
		ci.will_qos    = 1;
		ci.will_retain = 1;
	}

	info.mqtt_connects++;
	tm_printf((UB*)"net: connecting to broker " MQTT_BROKER_IP
		  ":%d (attempt %u)\n", MQTT_BROKER_PORT, info.mqtt_connects);

	conn_event  = CONN_NONE;
	conn_status = 0;
	err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
				  on_connection, NULL, &ci);
	if (err != ERR_OK) {
		conn_status = err;
		mqtt_retry_later("connect failed");
		return;
	}
	deadline = sys_now() + MQTT_CONNECT_TIMEOUT_MS;
	enter(NET_MQTT_WAIT);
}

LOCAL void on_subscribed(void *arg, err_t result)
{
	(void)arg;
	if (result != ERR_OK) {
		subscribe_failed = TRUE;
	}
}

/* Subscribe to every topic, then say "online". If lwIP has no room for
 * another request (ERR_MEM), stop and carry on from the same place on the
 * next tick. */
LOCAL void subscribe_all(void)
{
	err_t	err;

	while (sub_next < bridge->sub_count) {
		const NET_SUB *s = &bridge->subs[sub_next];

		err = mqtt_subscribe(client, s->topic, s->qos, on_subscribed,
				     NULL);
		if (err == ERR_MEM) {
			return;
		}
		if (err != ERR_OK) {
			conn_status = err;
			mqtt_retry_later("subscribe failed");
			return;
		}
		sub_next++;
	}

	if (bridge->status_topic != NULL && !status_sent) {
		err = mqtt_publish(client, bridge->status_topic, STATUS_ONLINE,
				   sizeof(STATUS_ONLINE) - 1, 1, 1, NULL, NULL);
		if (err == ERR_MEM) {
			return;
		}
		status_sent = TRUE;
	}

	mqtt_fails = 0;
	net_backoff_reset(&mqtt_backoff);
	next_rssi = sys_now();
	enter(NET_ONLINE);
	LOGF("net: online as " MQTT_CLIENT_ID ", %d topics subscribed",
	     bridge->sub_count);
}

/* Send what the tasks queued on the TX message buffer */
LOCAL void send_queued(void)
{
	INT	n;
	err_t	err;

	for (n = 0; n < TX_PER_TICK; n++) {
		if (!tx_pending) {
			if (tk_rcv_mbf(bridge->tx_mbf, &tx_msg, TMO_POL) < E_OK) {
				return;			/* queue empty */
			}
			tx_pending = TRUE;
		}

		if (tx_msg.len > NET_PAYLOAD_MAX) {
			err = ERR_VAL;			/* malformed: drop */
		} else {
			tx_msg.topic[NET_TOPIC_MAX] = '\0';
			err = mqtt_publish(client, tx_msg.topic, tx_msg.payload,
					   tx_msg.len, tx_msg.qos ? 1 : 0,
					   tx_msg.retain, NULL, NULL);
		}
		if (err == ERR_MEM) {
			return;		/* lwIP's buffer is full: next tick */
		}
		if (err != ERR_OK) {
			info.tx_dropped++;
		}
		tx_pending = FALSE;
	}
}

/* ---------------------------------------------------------------------
 *  Incoming messages: lwIP calls on_incoming_publish() with the topic,
 *  then on_incoming_data() once or more with the payload. Both pointers
 *  are only valid during the call, so everything is copied into rx_msg,
 *  and rx_msg into the RX message buffer before the last call returns.
 * --------------------------------------------------------------------- */
LOCAL void on_incoming_publish(void *arg, const char *topic, u32_t tot_len)
{
	INT	i;

	(void)arg;

	rx_too_long = (strlen(topic) > NET_TOPIC_MAX ||
		       tot_len > NET_PAYLOAD_MAX);
	strncpy(rx_msg.topic, topic, NET_TOPIC_MAX);
	rx_msg.topic[NET_TOPIC_MAX] = '\0';
	rx_msg.qos = 0;
	rx_msg.retain = 0;
	rx_msg.len = 0;

	rx_sub = -1;
	for (i = 0; i < bridge->sub_count; i++) {
		if (net_topic_matches(bridge->subs[i].topic, topic)) {
			rx_sub = i;
			break;
		}
	}
}

LOCAL void on_incoming_data(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	u16_t	n = len;
	ER	er;

	(void)arg;

	if (rx_msg.len + n > NET_PAYLOAD_MAX) {
		n = (u16_t)(NET_PAYLOAD_MAX - rx_msg.len);
	}
	memcpy(&rx_msg.payload[rx_msg.len], data, n);
	rx_msg.len = (UH)(rx_msg.len + n);

	if ((flags & MQTT_DATA_FLAG_LAST) == 0) {
		return;				/* more to come */
	}
	if (rx_sub < 0) {
		info.rx_dropped++;		/* no subscription matches */
		return;
	}
	if (rx_too_long) {
		info.rx_dropped++;
		LOGF("net: dropped a message on %s: over %d bytes",
		     rx_msg.topic, NET_PAYLOAD_MAX);
		return;
	}

	er = tk_snd_mbf(bridge->subs[rx_sub].rx_mbf, &rx_msg,
			NET_MSG_SIZE(rx_msg.len), TMO_POL);
	if (er < E_OK) {
		info.rx_dropped++;
		LOGF("net: dropped a message on %s: receiver busy (%d)",
		     rx_msg.topic, er);
	}
}

/* ---------------------------------------------------------------------
 *  The tick
 * --------------------------------------------------------------------- */
LOCAL BOOL bridge_setup(void)
{
	client = mqtt_client_new();
	if (client == NULL) {
		tm_printf((UB*)"net: out of lwIP memory for the MQTT client\n");
		return FALSE;
	}
	if (!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
		tm_printf((UB*)"net: MQTT_BROKER_IP \"" MQTT_BROKER_IP "\" is"
			  " not an IPv4 address (config/wifi_secrets.h)\n");
		return FALSE;
	}
	IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);
	mqtt_set_inpub_callback(client, on_incoming_publish, on_incoming_data,
				NULL);

	/* The kernel's Wi-Fi task already asked to join at boot: count that
	   as the first attempt and wait for its outcome */
	info.wifi_joins = 1;
	deadline = sys_now() + WIFI_LINK_TIMEOUT_MS;
	enter(NET_WIFI_WAIT);
	return TRUE;
}

LOCAL void bridge_tick(void *arg)
{
	int	link;
	int32_t	rssi;

	(void)arg;

	if (client == NULL && !bridge_setup()) {
		return;			/* misconfigured: stop for good */
	}
	sys_timeout(TICK_MS, bridge_tick, NULL);

	link = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);

	/* The layers above the link only exist while it is up */
	if (state >= NET_MQTT_CONNECT && link != CYW43_LINK_UP) {
		mqtt_disconnect(client);
		info.rssi = 0;
		LOGF("net: wifi lost (%s)", link_text(link));
		deadline = sys_now() + WIFI_LINK_TIMEOUT_MS;
		enter(NET_WIFI_WAIT);
	}

	switch (state) {
	case NET_WIFI_JOIN:
		wifi_join();
		break;
	case NET_WIFI_WAIT:
		wifi_wait(link);
		break;
	case NET_WIFI_BACKOFF:
		if (link < 0) {
			wifi_reset();	/* a late failure report: keep the
					   kernel's own re-join quiet */
		}
		if (time_reached(deadline)) {
			scan_then(NET_WIFI_JOIN);
		}
		break;
	case NET_WIFI_SCAN:
		if (!cyw43_wifi_scan_active(&cyw43_state) || time_reached(deadline)) {
			scan_report();
			enter(after_scan);
		}
		break;
	case NET_MQTT_CONNECT:
		mqtt_connect();
		break;
	case NET_MQTT_WAIT:
		if (conn_event == CONN_ACCEPTED) {
			conn_event = CONN_NONE;
			subscribe_failed = FALSE;
			sub_next = 0;
			status_sent = FALSE;
			enter(NET_MQTT_SUBSCRIBE);
			subscribe_all();
		} else if (conn_event == CONN_LOST) {
			mqtt_retry_later("broker refused or closed");
		} else if (time_reached(deadline)) {
			mqtt_retry_later("broker did not answer");
		}
		break;
	case NET_MQTT_SUBSCRIBE:
	case NET_ONLINE:
		if (conn_event == CONN_LOST) {
			mqtt_retry_later("connection lost");
		} else if (subscribe_failed) {
			mqtt_retry_later("subscription refused");
		} else if (state == NET_MQTT_SUBSCRIBE) {
			subscribe_all();
		} else {
			send_queued();
			if (time_reached(next_rssi)) {
				next_rssi = sys_now() + RSSI_PERIOD_MS;
				if (cyw43_wifi_get_rssi(&cyw43_state, &rssi) == 0) {
					info.rssi = rssi;
				}
			}
		}
		break;
	case NET_MQTT_BACKOFF:
		if (time_reached(deadline)) {
			enter(NET_MQTT_CONNECT);
		}
		break;
	}
}

/* ---------------------------------------------------------------------
 *  Entry points
 * --------------------------------------------------------------------- */

/* The only lwIP call made from outside the Wi-Fi task; net_start.c says
 * why it is safe. b and every message buffer in it already exist. */
EXPORT void net_bridge_arm(const NET_BRIDGE *b)
{
	bridge = b;
	MEMORY_BARRIER();
	sys_timeout(TICK_MS, bridge_tick, NULL);
}

EXPORT void net_get_info(NET_INFO *out)
{
	MEMORY_BARRIER();
	*out = *(const NET_INFO *)&info;
}

#else /* !TM_NET_LWIP: built without the network */

#include "net_bridge.h"

EXPORT void net_get_info(NET_INFO *out)
{
	*out = (NET_INFO){0};
}

#endif /* TM_NET_LWIP */

EXPORT const char *net_state_name(NET_STATE s)
{
	switch (s) {
	case NET_WIFI_JOIN:	return "wifi-join";
	case NET_WIFI_WAIT:	return "wifi-wait";
	case NET_WIFI_BACKOFF:	return "wifi-backoff";
	case NET_WIFI_SCAN:	return "wifi-scan";
	case NET_MQTT_CONNECT:	return "mqtt-connect";
	case NET_MQTT_WAIT:	return "mqtt-wait";
	case NET_MQTT_SUBSCRIBE:	return "mqtt-subscribe";
	case NET_MQTT_BACKOFF:	return "mqtt-backoff";
	case NET_ONLINE:	return "online";
	}
	return "?";
}
