/*
 *	comms.c
 *	Comms -- message buffers to and from the network bridge.
 *
 *	Runs on the CALLER's task and only touches kernel objects; the network
 *	calls are all in app_program/net/net_bridge.c, on the kernel's Wi-Fi
 *	service task.
 *
 *	Start-up order matters: the bridge must not use a message buffer
 *	before it exists. So the buffers are created in the _init() phase,
 *	subscriptions are collected as the other modules' _init()s run, and
 *	only comms_start() -- after every _init() -- hands them to the bridge.
 *	Until then the bridge does not connect to the broker.
 *
 *	APIs shown: tk_cre_mbf, tk_snd_mbf (TMO_POL)
 */

#include "comms.h"

#define TX_MBF_SIZE	2048	/* bytes of queued outgoing messages */

LOCAL ID		mbf_tx;
LOCAL NET_SUB		subs[COMMS_MAX_SUBS];
LOCAL NET_BRIDGE	bridge;
LOCAL BOOL		started;
LOCAL UW		tx_refused;	/* dropped before reaching the bridge */
LOCAL COMMS_LOG_TAP	log_tap;

EXPORT ER comms_init(void)
{
	T_CMBF	cmbf = {0};

	/* Message buffer: copies each message, so a sender's COMMS_MSG can
	   be a local variable.  Only the used part of the payload is stored. */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = TX_MBF_SIZE;
	cmbf.maxmsz = sizeof(COMMS_MSG);
	mbf_tx = tk_cre_mbf(&cmbf);
	return (mbf_tx < E_OK) ? mbf_tx : E_OK;
}

EXPORT ER comms_subscribe(const char *topic, ID mbf)
{
	NET_SUB		*s;

	if (started) {
		return E_OBJ;		/* the bridge already has the table */
	}
	if (bridge.sub_count == COMMS_MAX_SUBS) {
		return E_LIMIT;		/* raise COMMS_MAX_SUBS */
	}
	s = &subs[bridge.sub_count++];
	s->topic  = topic;
	s->rx_mbf = mbf;
	s->qos    = 1;		/* at least once: commands are not lost */
	return E_OK;
}

EXPORT ER comms_start(void)
{
	bridge.tx_mbf       = mbf_tx;
	bridge.subs         = subs;
	bridge.status_topic = COMMS_DEVICE_TOPIC(TOPIC_STATUS);
	bridge.log_topic    = COMMS_DEVICE_TOPIC(TOPIC_LOG);
	started = TRUE;

	return net_start(&bridge);
}

EXPORT ER comms_publish(const char *topic, const void *data, INT len,
			BOOL retain)
{
	COMMS_MSG	msg;
	const UB	*src = data;
	INT		i;
	ER		er;

	if (len < 0 || len > NET_PAYLOAD_MAX) {
		return E_PAR;
	}
	for (i = 0; topic[i] != '\0'; i++) {
		if (i == NET_TOPIC_MAX) {
			return E_PAR;
		}
		msg.topic[i] = topic[i];
	}
	msg.topic[i] = '\0';
	for (i = 0; i < len; i++) {
		msg.payload[i] = src[i];
	}
	msg.qos    = 0;
	msg.retain = retain ? 1 : 0;
	msg.len    = (UH)len;

	/* TMO_POL: never wait.  If the queue is full, drop the message. */
	er = tk_snd_mbf(mbf_tx, &msg, NET_MSG_SIZE(len), TMO_POL);
	if (er < E_OK) {
		tx_refused++;
	}
	return er;
}

EXPORT ER comms_log(const char *text)
{
	COMMS_LOG_TAP	tap = log_tap;	/* read once: it may change meanwhile */
	INT		len = 0;

	while (len < NET_PAYLOAD_MAX && text[len] != '\0') {
		len++;
	}

	if (tap != NULL) {
		tap(text);
	}
	tm_printf((UB*)"%s\n", text);
	return comms_publish(COMMS_DEVICE_TOPIC(TOPIC_LOG), text, len, FALSE);
}

EXPORT void comms_set_log_tap(COMMS_LOG_TAP tap)
{
	log_tap = tap;			/* one store: safe from any task */
}

EXPORT INT comms_format_telemetry(UB *buf, const UD *words, INT count)
{
	INT	len = 0;
	INT	i;

	for (i = 0; i < count && i < TLM_WORDS; i++) {
		if (i > 0) {
			buf[len++] = ' ';
		}
		len += comms_format_u64(&buf[len], words[i]);
	}
	buf[len] = '\0';
	return len;
}

EXPORT ER comms_telemetry(const UD *words, INT count)
{
	NET_INFO	info;
	UB		text[COMMS_TELEMETRY_TEXT_MAX];
	INT		len;

	/* A sample is only worth sending while it is current */
	comms_get_info(&info);
	if (!info.online) {
		return E_TMOUT;
	}

	len = comms_format_telemetry(text, words, count);
	return comms_publish(COMMS_DEVICE_TOPIC(TOPIC_TELEMETRY), text, len,
			     FALSE);
}

EXPORT void comms_get_info(NET_INFO *info)
{
	net_get_info(info);
	info->tx_dropped += tx_refused;
}

EXPORT BOOL comms_topic_is(const COMMS_MSG *msg, const char *topic)
{
	INT	i;

	for (i = 0; msg->topic[i] != '\0' && msg->topic[i] == topic[i]; i++) {
	}
	return msg->topic[i] == topic[i];
}
