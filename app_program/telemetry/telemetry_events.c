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
 *	  barcode  Line Following's last read (barcode_get_last, IF-18): a
 *	           letter and a count of reads in one word.  A new count
 *	           means a new read, even of the same letter.
 *
 *	The last barcode and a count of reads also go into telemetry word 1,
 *	so the dashboard shows the latest one even if it missed the event.
 *
 *	APIs shown: tk_rcv_mbf (TMO_POL, inside imu_task_read_event),
 *	            comms_publish
 */

#include "telemetry.h"
#include "../subsystem3/line_barcode.h"
#include "../subsystem4/subsystem4.h"

LOCAL UW		sent_barcode;	/* last one sent; 0 = none, as S3 starts */
LOCAL volatile BOOL	imu_held;

EXPORT UW telemetry_last_barcode(void)
{
	return barcode_get_last();
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
	UW	barcode = telemetry_last_barcode();
	INT	len;

	if (barcode != sent_barcode) {
		sent_barcode = barcode;
		send_event(text, tm_sprintf(text, (const UB*)"%d %d", EVT_BARCODE,
			   (INT)(barcode & BC_LAST_LETTER_MASK)));
	}

	while (!imu_held && imu_task_read_event(&event) == E_OK) {
		len = telemetry_event_text(text, event);
		if (len > 0) {
			send_event(text, len);
		}
	}
}

#if APP_TEST
EXPORT void telemetry_hold_imu_events(BOOL hold)
{
	imu_held = hold;
}
#endif
