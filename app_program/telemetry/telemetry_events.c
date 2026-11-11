/*
 *	telemetry_events.c
 *	Telemetry -- events: what happens once rather than all the time.
 *	Each goes out on robocar/<id>/event as "<EVT_*> [arg ...]"
 *	(protocol.h), queued while offline like a log line.
 *
 *	  hump     the IMU's event stream (imu_task_read_event, IF-S4-3),
 *	           drained with TMO_POL by telemetry_events_poll().  Only
 *	           HUMP events are sent; the others are read and dropped,
 *	           as the snapshot word already carries motion and terrain.
 *	  barcode  Line Following has no getter for the barcode it read, but
 *	           it logs every read through comms_log() as
 *	               "[BC] read *A* -> LEFT"
 *	           A log tap (comms_set_log_tap) spots that line on the
 *	           barcode task and keeps the letter; the telemetry task
 *	           sends the event.  When S3 offers a getter (IF-18,
 *	           proposed), read it in telemetry_last_barcode() instead.
 *
 *	The last barcode and a count of reads also go into telemetry word 1,
 *	so the dashboard shows the latest one even if it missed the event.
 *
 *	APIs shown: tk_rcv_mbf (TMO_POL, inside imu_task_read_event),
 *	            comms_set_log_tap, comms_publish
 */

#include "telemetry.h"
#include "../subsystem4/subsystem4.h"

#define BARCODE_PREFIX	"[BC] read *"	/* barcode.c finish_pass() */

/* letter in bits 0-7, reads in bits 8-15; written only by the log tap
   (on the barcode task), with one store, so the telemetry task always
   reads a matching pair */
LOCAL volatile UW	last_barcode;
LOCAL UW		sent_barcode;	/* telemetry task's copy */
LOCAL volatile BOOL	imu_held;

/* "[BC] read *A* -> LEFT" -> 'A'; any other line -> 0 */
EXPORT UB telemetry_barcode_in_log(const char *line)
{
	const char	*p = BARCODE_PREFIX;
	UB		letter;

	while (*p != '\0') {
		if (*line++ != *p++) {
			return 0;
		}
	}
	letter = (UB)line[0];
	if (letter <= ' ' || letter > '~' || letter == '*' || line[1] != '*') {
		return 0;
	}
	return letter;
}

/* Runs on whichever task called comms_log(): quick, never waits */
LOCAL void on_log_line(const char *line)
{
	UB	letter = telemetry_barcode_in_log(line);
	UW	reads;

	if (letter != 0) {
		reads = ((last_barcode >> 8) + 1) & 0xff;
		last_barcode = (reads << 8) | letter;
	}
}

EXPORT UW telemetry_last_barcode(void)
{
	return last_barcode;
}

/* An IMU event -> the text of its EVT_* message; 0 if it is not sent */
EXPORT INT telemetry_event_text(UB *buf, UH imu_event)
{
	imu_pack_event_t	type;
	uint32_t		payload;

	imu_pack_unpack_event(imu_event, &type, &payload);
	if (type != IMU_PACK_EVT_HUMP) {
		return 0;
	}
	/* HUMP payload (imu_pack.h): [6:0] mm, [7] over 127 mm, [12:8] deg */
	return tm_sprintf(buf, (const UB*)"%d %d %d %d", EVT_HUMP,
			  (INT)(payload & 0x7f), (INT)((payload >> 7) & 1),
			  (INT)((payload >> 8) & 0x1f));
}

LOCAL void send_event(const UB *text, INT len)
{
	/* E_TMOUT: the queue is full, the event is lost; the telemetry
	   words still show the latest barcode */
	(void)comms_publish(COMMS_DEVICE_TOPIC(TOPIC_EVENT), text, len, FALSE);
}

EXPORT void telemetry_events_poll(void)
{
	UB	text[32];
	UH	event;
	UW	barcode = last_barcode;
	INT	len;

	if (barcode != sent_barcode) {
		sent_barcode = barcode;
		send_event(text, tm_sprintf(text, (const UB*)"%d %d",
					    EVT_BARCODE, (INT)(barcode & 0xff)));
	}

	while (!imu_held && imu_task_read_event(&event) == E_OK) {
		len = telemetry_event_text(text, event);
		if (len > 0) {
			send_event(text, len);
		}
	}
}

EXPORT void telemetry_events_init(void)
{
	comms_set_log_tap(on_log_line);
}

#if APP_TEST
EXPORT void telemetry_hold_imu_events(BOOL hold)
{
	imu_held = hold;
}
#endif
