/*
 *	remote_test.c
 *	On-target tests for remote.  Built only with: make NET=lwip TEST=1
 *	Record results in testing/remote/TEST_LOG.md.
 *
 *	Commands are put straight into mbf_remote, exactly as the network
 *	bridge does for a message on robocar/<id>/cmd.
 *
 *	No START is sent: in a test build that would drive the car.  START /
 *	STOP / CLEAR reaching Navigation is checked on the bench (SYS-02).
 */

#include "remote.h"
#include "../test_report.h"

#if APP_TEST

LOCAL void deliver(const char *topic, const char *text)
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
	CHECK(tk_snd_mbf(mbf_remote, &msg, NET_MSG_SIZE(msg.len), TMO_FEVR),
	      "deliver");
}

/* Nothing waiting in mbf_remote: the task has taken every message */
LOCAL BOOL all_taken(void)
{
	T_RMBF	r;

	return tk_ref_mbf(mbf_remote, &r) == E_OK && r.frbufsz == COMMS_MSG_MBF_SIZE(8);
}

EXPORT void remote_test(void)
{
	TEST_INIT();

	/* RE-U01: anything arriving is handled, nothing is left behind and
	   the task keeps going: a ping, text, an unknown number, too many
	   arguments, and a broadcast ping */
	deliver(COMMS_DEVICE_TOPIC(TOPIC_CMD), "0 77");
	deliver(COMMS_DEVICE_TOPIC(TOPIC_CMD), "stop");
	deliver(COMMS_DEVICE_TOPIC(TOPIC_CMD), "9");
	deliver(COMMS_DEVICE_TOPIC(TOPIC_CMD), "1 2 3 4 5 6");
	deliver(COMMS_BROADCAST_TOPIC(TOPIC_CMD), "0 78");
	tk_dly_tsk(100);
	TEST_CHECK("RE-U01", all_taken(),
		   "pings, text and unknown commands are all consumed");

	TEST_SUMMARY("remote");
}

#endif /* APP_TEST */
