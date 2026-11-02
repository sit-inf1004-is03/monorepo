/*
 *	comms_test.c
 *	On-target tests for comms and the network bridge's pure helpers.
 *	Built only with: make NET=lwip TEST=1
 *	Record results in testing/comms/TEST_LOG.md.  IDs: CO-U01, ...
 *
 *	These check the parts that need no network.  Connecting, recovering
 *	and the round trip to the dashboard are system tests (SYS-xx).
 */

#include "comms.h"
#include "../test_report.h"

#if APP_TEST

/* Same sample as the dashboard's test (mqtt-web-interface/src/shared/
   protocol.test.ts): the two sides must agree bit for bit */
#define GOLDEN_WORD	6601995533221430314ULL
#define GOLDEN_TEXT	"6601995533221430314"

LOCAL BOOL parses(const char *text, COMMS_CMD *cmd)
{
	INT	len = 0;

	while (text[len] != '\0') {
		len++;
	}
	return comms_parse_text((const UB*)text, len, cmd) == E_OK;
}

LOCAL BOOL same_text(const UB *a, const char *b)
{
	while (*a != '\0' && *a == (UB)*b) {
		a++;
		b++;
	}
	return *a == (UB)*b;
}

EXPORT void comms_test(void)
{
	COMMS_CMD	cmd;
	NET_BACKOFF	b = NET_BACKOFF_INIT(1000, 8000);
	UB		text[24];
	UD		w = 0;
	UW		d1, d2, d3, d4, d5;

	TEST_INIT();

	/* CO-U01..03: command text -> COMMS_CMD */
	TEST_CHECK("CO-U01", parses("2 60 -20 1500", &cmd) &&
		   cmd.id == 2 && cmd.argc == 3 && cmd.arg[0] == 60 &&
		   cmd.arg[1] == -20 && cmd.arg[2] == 1500 && cmd.arg[3] == 0,
		   "parses \"2 60 -20 1500\"");

	TEST_CHECK("CO-U02", parses(" 1\r\n", &cmd) && cmd.id == 1 &&
		   cmd.argc == 0 && parses("3,2", &cmd) && cmd.arg[0] == 2 &&
		   parses("0 -2147483648 2147483647", &cmd) &&
		   cmd.arg[0] == (W)0x80000000 && cmd.arg[1] == 0x7fffffff,
		   "accepts whitespace, commas and the full W range");

	TEST_CHECK("CO-U03", !parses("", &cmd) && !parses("stop", &cmd) &&
		   !parses("2 6x", &cmd) && !parses("-1", &cmd) &&
		   !parses("70000", &cmd) && !parses("1 2147483648", &cmd) &&
		   !parses("1 2 3 4 5 6", &cmd) && !parses("1 -", &cmd),
		   "rejects text, bad ids, overflow and too many args");

	/* CO-U04: 64-bit numbers -> decimal text */
	comms_format_u64(text, 0);
	d1 = same_text(text, "0");
	comms_format_u64(text, 0xffffffffffffffffULL);
	d2 = same_text(text, "18446744073709551615");
	TEST_CHECK("CO-U04", d1 && d2, "formats 0 and 2^64-1");

	/* CO-U05: telemetry packing matches the dashboard */
	w = tlm_put(w, TLM_SEQ, 42);
	w = tlm_put(w, TLM_NAV_STATE, 4);		/* BYPASS */
	w = tlm_put(w, TLM_NAV_REASON, 0);
	w = tlm_put(w, TLM_FAULTS, 0x84);		/* line lost, Wi-Fi loss */
	w = tlm_put(w, TLM_MOTION, 1);			/* STRAIGHT */
	w = tlm_put(w, TLM_LINE, 2);			/* SEARCHING */
	w = tlm_put(w, TLM_SPEED_LEFT, 345);
	w = tlm_put(w, TLM_SPEED_RIGHT, TLM_SPEED_MAX);	/* 1023 or faster */
	w = tlm_put(w, TLM_OBSTACLE, 123);
	w = tlm_put(w, TLM_RSSI, 55);
	w = tlm_put(w, TLM_VERSION, TLM_LAYOUT);
	comms_format_u64(text, w);
	TEST_CHECK("CO-U05", w == GOLDEN_WORD && same_text(text, GOLDEN_TEXT) &&
		   tlm_get(w, TLM_FAULTS) == 0x84 &&
		   tlm_get(w, TLM_SPEED_RIGHT) == TLM_SPEED_MAX,
		   "telemetry word matches the dashboard's golden sample");

	/* CO-U06/07: backoff doubles up to the limit, jitter stays in
	   [step/2, step], and a reset starts again from the minimum */
	d1 = net_backoff_next(&b, 0);		/* step 1000 -> 500 */
	d2 = net_backoff_next(&b, 0xffffffff);	/* step 2000 */
	d3 = net_backoff_next(&b, 0);		/* step 4000 -> 2000 */
	d4 = net_backoff_next(&b, 1000);	/* step 8000 */
	d5 = net_backoff_next(&b, 0);		/* still 8000 -> 4000 */
	TEST_CHECK("CO-U06", d1 == 500 && d2 >= 1000 && d2 <= 2000 &&
		   d3 == 2000 && d4 == 5000 && d5 == 4000,
		   "backoff doubles to the limit with jitter");
	net_backoff_reset(&b);
	TEST_CHECK("CO-U07", net_backoff_next(&b, 0) == 500,
		   "backoff restarts from the minimum after a reset");

	/* CO-U08: messages over the limits are refused, never cut */
	TEST_CHECK("CO-U08",
		   comms_publish(COMMS_DEVICE_TOPIC("test"), text,
				 NET_PAYLOAD_MAX + 1, FALSE) == E_PAR &&
		   comms_publish("a/topic/that/is/far/longer/than/the/sixty-"
				 "three/byte/topic/limit/allows", text, 1,
				 FALSE) == E_PAR,
		   "an over-long payload or topic is refused");

	/* CO-U09: subscriptions match topics like a broker does */
	TEST_CHECK("CO-U09",
		   net_topic_matches("robocar/a/cmd", "robocar/a/cmd") &&
		   !net_topic_matches("robocar/a/cmd", "robocar/b/cmd") &&
		   net_topic_matches("robocar/+/cmd", "robocar/b/cmd") &&
		   !net_topic_matches("robocar/+/cmd", "robocar/b/log") &&
		   net_topic_matches("robocar/#", "robocar/b/log") &&
		   net_topic_matches("robocar/#", "robocar") &&
		   !net_topic_matches("robocar/a", "robocar/a/cmd"),
		   "topic filters with + and # match as in MQTT");

	TEST_SUMMARY("comms");
}

#endif /* APP_TEST */
