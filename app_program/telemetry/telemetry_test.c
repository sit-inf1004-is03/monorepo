/*
 *	telemetry_test.c
 *	On-target tests for telemetry.  Built only with: make NET=lwip TEST=1
 *	Record unit results in testing/telemetry/TEST_LOG.md and the INT-
 *	results in testing/integration/TEST_LOG.md.
 *
 *	Settings are put straight into mbf_telemetry, exactly as the network
 *	bridge does for a message on robocar/<id>/config.
 */

#include "telemetry.h"
#include "../navigation/navigation.h"
#include "../subsystem2/motion.h"
#include "../test_report.h"

#if APP_TEST

/* Put a text message into a task's buffer, exactly as the network bridge
   does when one arrives from the broker */
LOCAL void deliver(ID mbf, const char *topic, const char *text)
{
	COMMS_MSG	msg = {0};
	INT		i;

	for (i = 0; topic[i] != '\0' && i < NET_TOPIC_MAX; i++) {
		msg.topic[i] = topic[i];
	}
	for (i = 0; text[i] != '\0' && i < NET_PAYLOAD_MAX; i++) {
		msg.payload[i] = (UB)text[i];
	}
	msg.len = (UH)i;
	CHECK(tk_snd_mbf(mbf, &msg, NET_MSG_SIZE(msg.len), TMO_FEVR), "deliver");
}

LOCAL void send(UH id, W value)
{
	UB	text[24];

	tm_sprintf(text, (const UB*)"%d %d", id, value);
	deliver(mbf_telemetry, COMMS_DEVICE_TOPIC(TOPIC_CONFIG), (const char*)text);
}

EXPORT void telemetry_test(void)
{
	UW	before = telemetry_period_ms;
	UW	accepted, rejected;
	UD	w;

	TEST_INIT();

	/* INT-15 (IF-15): a setting reaches the task */
	send(CFG_TELEMETRY_PERIOD, 250);
	tk_dly_tsk(50);
	accepted = telemetry_period_ms;
	TEST_CHECK("INT-15", accepted == 250,
		   "\"1 250\" via mbf_telemetry changes the period");

	/* TL-U01: a period out of range is refused */
	send(CFG_TELEMETRY_PERIOD, 10);
	tk_dly_tsk(50);
	rejected = telemetry_period_ms;
	TEST_CHECK("TL-U01", rejected == 250, "a 10 ms period is refused");

	/* TL-U02: sending can be switched off and on */
	send(CFG_TELEMETRY_ENABLE, 0);
	tk_dly_tsk(50);
	TEST_CHECK("TL-U02", !telemetry_enabled, "CFG_TELEMETRY_ENABLE 0 stops it");
	send(CFG_TELEMETRY_ENABLE, 1);
	send(CFG_TELEMETRY_PERIOD, (W)before);
	tk_dly_tsk(50);

	/* INT-16 (IF-06, IF-09, IF-14): a sample carries what the other
	   subsystems report.  The car may be moving, so check ranges, and
	   that no fault appears that Navigation has not raised (faults only
	   come and go with a remote CLEAR). */
	w = telemetry_sample(7);
	TEST_CHECK("INT-16",
		   tlm_get(w, TLM_SEQ) == 7 &&
		   tlm_get(w, TLM_VERSION) == TLM_LAYOUT &&
		   tlm_get(w, TLM_NAV_STATE) <= NAV_STATE_FAULT &&
		   tlm_get(w, TLM_NAV_REASON) <= NAV_REASON_REMOTE_STOP &&
		   (tlm_get(w, TLM_FAULTS) & ~nav_get_faults()) == 0 &&
		   tlm_get(w, TLM_MOTION) <= MOTION_ACTIVITY_PIVOT &&
		   tlm_get(w, TLM_LINE) <= TLM_LINE_UNKNOWN,
		   "a sample holds Navigation, Motion and line-sensor values");

	TEST_SUMMARY("telemetry");
}

#endif /* APP_TEST */
