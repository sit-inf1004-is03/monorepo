/*
 *	telemetry.c
 *	Telemetry -- every telemetry_period_ms, read the car's state from the
 *	other subsystems, pack it into two 64-bit words and queue them for the
 *	network bridge.  In between, every EVENT_POLL_MS, send any new hump
 *	or barcode as an event (telemetry_events.c).
 *
 *	Data flow:  Navigation, Motion, encoders, line sensor, ultrasonic,
 *	            IMU, barcode, Wi-Fi signal -> tlm_put() ->
 *	            comms_telemetry() -> dashboard
 *	Settings:   dashboard -> network bridge -> mbf_telemetry -> here
 *
 *	One blocking call does both jobs: the task waits for a setting, but
 *	only until the next sample or event poll is due.
 *
 *	Sources
 *	-------
 *	Agreed interfaces, used through their headers:
 *	  nav_get_state/_reason/_faults()   IF-14  navigation/navigation.h
 *	  motion_get_activity()             IF-09  subsystem2/motion.h
 *	  drv_encoder_get_speed(), _count() IF-06  subsystem2/drv_encoder.h
 *	  imu_task_get_snapshot()           IF-S4-1 subsystem4/imu_task.h
 *	  imu_task_read_event()             IF-S4-3 subsystem4/imu_task.h
 *	  imu_is_ok()                       IF-S4-7 subsystem4/imu_ramp.h
 *	Barcode reads come from Line Following's log lines (IF-18, see
 *	telemetry_events.c).
 *	Not yet in TEAM.md, so referenced weakly (IF-16, proposed): the field
 *	reads "not reported" until the subsystem that defines the function is
 *	in the build, and nothing here breaks if it is renamed meanwhile:
 *	  line_get_state()                  S3     subsystem3/line_barcode.h
 *	  ultrasonic_last_reading()         S5     subsystem5/drv_ultrasonic.h
 *
 *	APIs shown: tk_rcv_mbf (with timeout), comms_subscribe,
 *	            comms_parse_cmd, comms_telemetry, comms_get_info
 */

#include "telemetry.h"
#include "../navigation/navigation.h"
#include "../subsystem2/motion.h"
#include "../subsystem2/drv_encoder.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem4/imu_ramp.h"

#define PERIOD_MIN_MS	100
#define PERIOD_MAX_MS	60000
#define EVENT_POLL_MS	100	/* the IMU holds 32 events: room for 320 ms
				   of its fastest changes */

/* Weak: NULL when S3 / S5 are not linked in.  Prototypes as in
   line_barcode.h and drv_ultrasonic.h.  line_state_t is an enum, which
   arm-none-eabi makes one byte wide (and AAPCS widens on return). */
IMPORT UB	line_get_state(void) __attribute__((weak));
IMPORT UH	ultrasonic_last_reading(void) __attribute__((weak));

#define ULTRASONIC_DIST_MASK	0x01FF	/* as in drv_ultrasonic.h */
#define ULTRASONIC_NO_ECHO_BIT	0x0200

EXPORT ID		mbf_telemetry;
EXPORT volatile UW	telemetry_period_ms = 1000;
EXPORT volatile BOOL	telemetry_enabled = TRUE;

LOCAL UW now_ms(void)
{
	SYSTIM	t;

	tk_get_otm(&t);
	return t.lo;
}

LOCAL UW speed(UW wheel)
{
	UW	mm_s = drv_encoder_get_speed(wheel);

	return (mm_s > TLM_SPEED_MAX) ? TLM_SPEED_MAX : mm_s;
}

LOCAL UW line_state(void)
{
	return (line_get_state != NULL) ? (UW)line_get_state()
					: TLM_LINE_UNKNOWN;
}

LOCAL UW obstacle_cm(void)
{
	UH	reading;

	if (ultrasonic_last_reading == NULL) {
		return TLM_OBSTACLE_NONE;
	}
	reading = ultrasonic_last_reading();
	if (reading & ULTRASONIC_NO_ECHO_BIT) {
		return TLM_OBSTACLE_NONE;
	}
	return reading & ULTRASONIC_DIST_MASK;
}

/* Mean distance of the two wheels since power-on.  The counts never
   reset (IF-06); a pivot turns both wheels, so it counts too. */
LOCAL UW distance_cm(void)
{
	UD	pulses = ((UD)drv_encoder_get_count(DRV_ENCODER_LEFT) +
			  drv_encoder_get_count(DRV_ENCODER_RIGHT)) / 2;
	UD	cm = pulses * drv_encoder_pulses_to_um(1) / 10000;

	return (cm > TLM_DISTANCE_MAX) ? TLM_DISTANCE_MAX : (UW)cm;
}

EXPORT UD telemetry_sample(UW seq)
{
	NET_INFO	net;
	UD		w = 0;

	comms_get_info(&net);

	w = tlm_put(w, TLM_SEQ,         seq);
	w = tlm_put(w, TLM_NAV_STATE,   nav_get_state());
	w = tlm_put(w, TLM_NAV_REASON,  nav_get_reason());
	w = tlm_put(w, TLM_FAULTS,      nav_get_faults());
	w = tlm_put(w, TLM_MOTION,      motion_get_activity());
	w = tlm_put(w, TLM_LINE,        line_state());
	w = tlm_put(w, TLM_SPEED_LEFT,  speed(DRV_ENCODER_LEFT));
	w = tlm_put(w, TLM_SPEED_RIGHT, speed(DRV_ENCODER_RIGHT));
	w = tlm_put(w, TLM_OBSTACLE,    obstacle_cm());
	w = tlm_put(w, TLM_RSSI,        (net.rssi < 0 && net.rssi > -128)
					? (UW)-net.rssi : 0);
	w = tlm_put(w, TLM_VERSION,     TLM_LAYOUT);
	return w;
}

EXPORT UD telemetry_sample_w1(void)
{
	imu_pack_snapshot_t	imu;
	UW			barcode = telemetry_last_barcode();
	UD			w = 0;

	imu_pack_unpack_snapshot(imu_task_get_snapshot(), &imu);

	w = tlm_put(w, TLM_W1_IMU_MOTION,    imu.motion_class);
	w = tlm_put(w, TLM_W1_IMU_TERRAIN,   imu.terrain_state);
	w = tlm_put(w, TLM_W1_IMU_OK,        imu_is_ok() ? 1 : 0);
	w = tlm_put(w, TLM_W1_BARCODE,       barcode & 0xff);
	w = tlm_put(w, TLM_W1_BARCODE_COUNT, (barcode >> 8) & 0xf);
	w = tlm_put(w, TLM_W1_DISTANCE,      distance_cm());
	return w;
}

LOCAL void apply_setting(const COMMS_MSG *msg)
{
	COMMS_CMD	cmd;
	UB		line[NET_PAYLOAD_MAX];

	if (comms_parse_cmd(msg, &cmd) != E_OK) {
		comms_log("telemetry: ignored a setting: not \"<id> <value>\"");
		return;
	}

	switch (cmd.id) {
	case CFG_TELEMETRY_PERIOD:
		if (cmd.arg[0] < PERIOD_MIN_MS || cmd.arg[0] > PERIOD_MAX_MS) {
			tm_sprintf(line, (const UB*)"telemetry: period %d ms is"
				   " outside %d..%d", cmd.arg[0],
				   PERIOD_MIN_MS, PERIOD_MAX_MS);
			break;
		}
		telemetry_period_ms = (UW)cmd.arg[0];
		tm_sprintf(line, (const UB*)"telemetry: every %d ms", cmd.arg[0]);
		break;

	case CFG_TELEMETRY_ENABLE:
		telemetry_enabled = (cmd.arg[0] != 0);
		tm_sprintf(line, (const UB*)"telemetry: %s",
			   telemetry_enabled ? "on" : "off");
		break;

	default:
		tm_sprintf(line, (const UB*)"telemetry: unknown setting %d",
			   cmd.id);
		break;
	}
	comms_log((const char*)line);
}

LOCAL void telemetry_task(INT stacd, void *exinf)
{
	COMMS_MSG	msg;
	UD		words[TLM_WORDS];
	UW		next = now_ms();
	UW		seq = 0;
	TMO		tmo;
	ER		er;

	(void)stacd; (void)exinf;

	while (1) {
		/* Wait for a setting until the next sample or poll is due */
		tmo = (TMO)(next - now_ms());
		if (tmo < 0) {
			tmo = 0;
		}
		if (tmo > EVENT_POLL_MS) {
			tmo = EVENT_POLL_MS;
		}
		er = tk_rcv_mbf(mbf_telemetry, &msg, tmo);
		if (er >= E_OK) {
			apply_setting(&msg);
			continue;
		}
		if (er != E_TMOUT) {
			CHECK(er, "tk_rcv_mbf");
			continue;
		}

		telemetry_events_poll();
		if ((W)(next - now_ms()) > 0) {
			continue;			/* only a poll was due */
		}

		next += telemetry_period_ms;
		if ((W)(next - now_ms()) < 0) {
			next = now_ms() + telemetry_period_ms;	/* fell behind */
		}

		if (telemetry_enabled) {
			/* Dropped (E_TMOUT) while offline: fine, the next
			   sample will be newer anyway */
			words[0] = telemetry_sample(seq);
			words[1] = telemetry_sample_w1();
			(void)comms_telemetry(words, TLM_WORDS);
			seq = (seq + 1) & 0xff;
		}
	}
}

EXPORT ER telemetry_init(void)
{
	T_CMBF	cmbf = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = COMMS_MSG_MBF_SIZE(4);
	cmbf.maxmsz = sizeof(COMMS_MSG);
	mbf_telemetry = tk_cre_mbf(&cmbf);
	if (mbf_telemetry < E_OK) {
		return mbf_telemetry;
	}
	telemetry_events_init();
	return comms_subscribe(COMMS_DEVICE_TOPIC(TOPIC_CONFIG), mbf_telemetry);
}

EXPORT ER telemetry_start(void)
{
	ID tid = app_start_task(telemetry_task, TELEMETRY_PRI, 0,
				"telemetry_task");
	return (tid < E_OK) ? tid : E_OK;
}
