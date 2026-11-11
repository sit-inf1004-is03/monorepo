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
#include "../subsystem2/drv_encoder.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem4/imu_ramp.h"
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

LOCAL BOOL same_text(const UB *a, const char *b)
{
	while (*a != '\0' && *a == (UB)*b) {
		a++;
		b++;
	}
	return *a == (UB)*b;
}

/* Text of an IMU event, or "" when it is not sent */
LOCAL const UB *event_text(UB *buf, UH imu_event)
{
	if (telemetry_event_text(buf, imu_event) <= 0) {
		buf[0] = '\0';
	}
	return buf;
}

/* Distance from the encoders, as telemetry works it out, to 1 cm */
LOCAL UW encoder_cm(void)
{
	return drv_encoder_pulses_to_mm(
		(drv_encoder_get_count(DRV_ENCODER_LEFT) / 2) +
		(drv_encoder_get_count(DRV_ENCODER_RIGHT) / 2)) / 10;
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
	UW	reads, cm_lo, cm_hi;
	UD	w;
	UB	text[32];
	imu_pack_snapshot_t	snap_lo, snap_hi;

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

	/* TL-U03: only HUMP events are sent, as "0 <mm> <over 127> <deg>" */
	TEST_CHECK("TL-U03",
		   same_text(event_text(text, imu_pack_event(IMU_PACK_EVT_HUMP,
				imu_pack_hump_payload(42, false, 9))),
			     "0 42 0 9") &&
		   same_text(event_text(text, imu_pack_event(IMU_PACK_EVT_HUMP,
				imu_pack_hump_payload(127, true, 31))),
			     "0 127 1 31") &&
		   same_text(event_text(text, imu_pack_event(IMU_PACK_EVT_IMPACT,
				imu_pack_impact_payload(3.0f))), ""),
		   "an IMU HUMP event becomes an EVT_HUMP message");

	/* TL-U04: Line Following's read line gives the letter, nothing else
	   does (the format of barcode.c finish_pass()) */
	TEST_CHECK("TL-U04",
		   telemetry_barcode_in_log("[BC] read *A* -> LEFT") == 'A' &&
		   telemetry_barcode_in_log("[BC] read *7* (reversed) -> NONE")
		   == '7' &&
		   telemetry_barcode_in_log("[BC] no read: 12 bars/gaps, 0"
					    " glitches") == 0 &&
		   telemetry_barcode_in_log("[BC] read **") == 0 &&
		   telemetry_barcode_in_log("remote: pong 1") == 0,
		   "the barcode letter is found only in a read line");

	/* INT-18 (IF-18, IF-17): a read logged through comms_log() reaches
	   word 1, and counts as a new read */
	reads = tlm_get(telemetry_sample_w1(), TLM_W1_BARCODE_COUNT);
	comms_log("[BC] read *C* -> STRAIGHT (telemetry self-test)");
	w = telemetry_sample_w1();
	TEST_CHECK("INT-18", tlm_get(w, TLM_W1_BARCODE) == 'C' &&
		   tlm_get(w, TLM_W1_BARCODE_COUNT) == ((reads + 1) & 0xf),
		   "a barcode read in the log shows in telemetry word 1");

	/* INT-19 (IF-06, IF-S4-1, IF-S4-7): word 1 holds what the IMU and
	   the encoders report.  Both may change while it is built, so it
	   must match the readings just before or just after. */
	cm_lo = encoder_cm();
	imu_pack_unpack_snapshot(imu_task_get_snapshot(), &snap_lo);
	w = telemetry_sample_w1();
	imu_pack_unpack_snapshot(imu_task_get_snapshot(), &snap_hi);
	cm_hi = encoder_cm();
	TEST_CHECK("INT-19",
		   (tlm_get(w, TLM_W1_IMU_MOTION) == snap_lo.motion_class ||
		    tlm_get(w, TLM_W1_IMU_MOTION) == snap_hi.motion_class) &&
		   (tlm_get(w, TLM_W1_IMU_TERRAIN) == snap_lo.terrain_state ||
		    tlm_get(w, TLM_W1_IMU_TERRAIN) == snap_hi.terrain_state) &&
		   tlm_get(w, TLM_W1_IMU_OK) == (imu_is_ok() ? 1 : 0) &&
		   tlm_get(w, TLM_W1_DISTANCE) + 1 >= cm_lo &&
		   tlm_get(w, TLM_W1_DISTANCE) <= cm_hi + 1,
		   "word 1 holds the IMU's motion and terrain, and the distance");

	TEST_SUMMARY("telemetry");
}

#endif /* APP_TEST */
