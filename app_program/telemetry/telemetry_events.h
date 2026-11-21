/**
 * @file    telemetry_events.h
 * @brief   Telemetry events: humps and barcodes, sent once each on
 *          robocar/id/event as they happen (protocol.h EVT_*).  Coded to
 *          BARR-C:2018.
 */

#ifndef TELEMETRY_EVENTS_H
#define TELEMETRY_EVENTS_H

#include <stdbool.h>
#include <stdint.h>

#define TELEMETRY_EVENTS_TEXT_MAX   (32u)   /* Bytes for one event. */

void     telemetry_events_poll (void);
uint32_t telemetry_events_last_barcode (void);
uint32_t telemetry_events_text (char * p_buf, uint16_t imu_event);
void     telemetry_events_hold_imu (bool b_hold);

#endif /* TELEMETRY_EVENTS_H */

/*** end of file ***/
