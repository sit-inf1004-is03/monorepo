/**
 * @file    telemetry.h
 * @brief   Telemetry: Wi-Fi / MQTT (Rifa).  Samples the car's state from
 *          the other subsystems, packs it into 64-bit words (protocol.h
 *          TLM_* and TLM_W1_*) and publishes them on robocar/id/telemetry.
 *          Humps and barcodes go out as they happen on robocar/id/event
 *          (EVT_*, telemetry_events.h).  Settings (CFG_*) arrive on
 *          robocar/id/config.  Coded to BARR-C:2018.
 *
 * Reads only, never commands: every source is a read-only getter or
 * stream (TEAM.md IF-06, IF-09, IF-14, IF-16, IF-18, IF-S4-1, IF-S4-3,
 * IF-S4-7).
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"

#define TELEMETRY_PRI           (23)    /* Band 22-23 (TEAM.md section 4). */
#define TELEMETRY_MBF_MESSAGES  (4u)    /* Settings its buffer holds. */

ER       telemetry_init (void);
ER       telemetry_start (void);
ID       telemetry_get_mbf (void);
uint32_t telemetry_get_period_ms (void);
bool     telemetry_is_enabled (void);
uint64_t telemetry_sample (uint32_t seq);
uint64_t telemetry_sample_w1 (void);

#endif /* TELEMETRY_H */

/*** end of file ***/
