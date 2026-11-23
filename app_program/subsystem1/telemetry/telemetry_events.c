/**
 * @file    telemetry_events.c
 * @brief   Telemetry events: what happens once rather than all the time.
 *          Each goes out on robocar/id/event as an EVT_* message
 *          (protocol.h), queued while offline like a log line.  Coded to
 *          BARR-C:2018.
 *
 *   hump     The IMU's event stream (imu_task_read_event, IF-S4-3), drained
 *            with TMO_POL by telemetry_events_poll().  Only HUMP events are
 *            sent; the others are read and dropped, as the snapshot word
 *            already carries motion and terrain.
 *   barcode  Line Following's last read (barcode_get_last, IF-18): a letter
 *            and a count of reads in one word.  A new count means a new
 *            read, even of the same letter.
 *
 * The last barcode and a count of reads also go into telemetry word 1, so
 * the dashboard shows the latest one even if it missed the event.
 */

#include "telemetry_events.h"
#include "../comms/comms.h"
#include "../../subsystem3/line_barcode.h"
#include "../../subsystem4/subsystem4.h"

/* HUMP event payload (imu_pack.h): [6:0] mm, [7] over 127 mm, [12:8] deg. */
#define HUMP_MM_MASK            (0x7Fu)
#define HUMP_OVER_SHIFT         (7u)
#define HUMP_OVER_MASK          (0x01u)
#define HUMP_TILT_SHIFT         (8u)
#define HUMP_TILT_MASK          (0x1Fu)

static uint32_t      g_sent_barcode = 0u;   /* 0 = none, as S3 starts. */
static bool volatile gb_imu_held    = false;

static void send_event (char const * p_text, uint32_t len);

/**
 * @brief   Send any new barcode read and every new hump.  Called by the
 *          telemetry task every EVENT_POLL_MS.
 */
void
telemetry_events_poll (void)
{
    char           text[TELEMETRY_EVENTS_TEXT_MAX];
    uint32_t const barcode = telemetry_events_last_barcode();

    if (barcode != g_sent_barcode)
    {
        g_sent_barcode = barcode;

        /* Casts: tm_sprintf() takes UB text; ASCII is the same bytes.  It
           returns a small non-negative length here. */
        uint32_t const len = (uint32_t) tm_sprintf((UB *) text,
                                                   (UB const *) "%d %d",
                                                   EVT_BARCODE,
                                                   barcode &
                                                   BC_LAST_LETTER_MASK);

        send_event(text, len);
    }

    uint16_t event  = 0u;
    bool     b_more = !gb_imu_held;

    while (b_more)
    {
        b_more = (E_OK == imu_task_read_event(&event));

        if (b_more)
        {
            uint32_t const len = telemetry_events_text(text, event);

            if (len > 0u)
            {
                send_event(text, len);
            }
        }
    }
}

/**
 * @brief   Line Following's last barcode read (IF-18).
 * @return  The letter in bits 0-7 and the count of reads in bits 8-15.
 */
uint32_t
telemetry_events_last_barcode (void)
{
    return (barcode_get_last());
}

/**
 * @brief   The EVT_* message for an IMU event.
 * @param   p_buf      At least TELEMETRY_EVENTS_TEXT_MAX bytes.
 * @param   imu_event  The event, as imu_task_read_event() gives it.
 * @return  The message's length, or 0 if this event is not sent.
 */
uint32_t
telemetry_events_text (char * p_buf, uint16_t imu_event)
{
    imu_pack_event_t type    = IMU_PACK_EVT_HUMP;
    uint32_t         payload = 0u;
    uint32_t         len     = 0u;

    imu_pack_unpack_event(imu_event, &type, &payload);

    if (IMU_PACK_EVT_HUMP == type)
    {
        /* Casts: tm_sprintf() takes UB text; ASCII is the same bytes.  It
           returns a small non-negative length here. */
        len = (uint32_t) tm_sprintf((UB *) p_buf, (UB const *) "%d %d %d %d",
                                    EVT_HUMP, payload & HUMP_MM_MASK,
                                    (payload >> HUMP_OVER_SHIFT) &
                                    HUMP_OVER_MASK,
                                    (payload >> HUMP_TILT_SHIFT) &
                                    HUMP_TILT_MASK);
    }
    else
    {
        p_buf[0] = '\0';
    }

    return (len);
}

/**
 * @brief   Leave the IMU's events alone, or drain them again.  For test
 *          builds: subsystem4_test() reads them itself (S4-U06 waits for an
 *          IMPACT event), so usermain() holds them around it.
 * @param   b_hold  true to leave them alone.
 */
void
telemetry_events_hold_imu (bool b_hold)
{
    gb_imu_held = b_hold;
}

/**
 * @brief   Queue one event.  If the queue is full the event is lost; the
 *          telemetry words still show the latest barcode.
 * @param   p_text  The message.
 * @param   len     Its length.
 */
static void
send_event (char const * p_text, uint32_t len)
{
    (void) comms_publish(COMMS_TOPIC_EVENT, p_text, len, false);
}

/*** end of file ***/
