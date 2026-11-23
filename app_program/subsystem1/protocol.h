/**
 * @file    protocol.h
 * @brief   What the car and the dashboard say to each other over MQTT.
 *          Owner: Wi-Fi / MQTT (Rifa).  TEAM.md interfaces IF-15 .. IF-18.
 *          Coded to BARR-C:2018.
 *
 * THE DASHBOARD HAS A COPY OF THIS FILE:
 *     mqtt-web-interface/src/shared/protocol.ts
 * Change both together, or the two sides will disagree.
 *
 * Topics (id is MQTT_CLIENT_ID from config/wifi_secrets.h):
 *
 *   car to dashboard
 *     robocar/id/status     "online" or "offline" (retained; "offline" is
 *                           the MQTT last will, sent by the broker when
 *                           the car drops off without saying so)
 *     robocar/id/telemetry  unsigned 64-bit integers in decimal, one space
 *                           apart: word 0, then word 1, packed as TLM_*
 *                           and TLM_W1_* (layout 2)
 *     robocar/id/event      something that happened: an EVT_* message
 *     robocar/id/log        one line of plain text
 *
 *   dashboard to car
 *     robocar/id/cmd        a command: a CMD_* identifier and arguments
 *     robocar/all/cmd       the same, for every car at once
 *     robocar/id/config     a setting: a CFG_* identifier and its value
 *
 * A command, setting or event is decimal integers separated by spaces, the
 * first being the identifier, e.g. "1" is CMD_START.  At most
 * COMMS_PAYLOAD_MAX_ARGS arguments follow; missing arguments read as 0.
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

/*
 * Topic leaves: the last part of robocar/id/leaf.
 */
#define TOPIC_STATUS            "status"
#define TOPIC_TELEMETRY         "telemetry"
#define TOPIC_LOG               "log"
#define TOPIC_EVENT             "event"
#define TOPIC_CMD               "cmd"
#define TOPIC_CONFIG            "config"

#define STATUS_ONLINE           "online"
#define STATUS_OFFLINE          "offline"

/*
 * Commands, on robocar/id/cmd and robocar/all/cmd (task: remote).  START,
 * STOP and CLEAR are Navigation's remote commands (IF-13) and keep its
 * numbers (NAV_REMOTE_*, checked in remote.c).
 */
#define CMD_PING                (0u)    /* [token]: reply "pong token". */
#define CMD_START               (1u)    /* IDLE or STOPPED: follow line. */
#define CMD_STOP                (2u)    /* Brake and wait. */
#define CMD_CLEAR               (3u)    /* FAULT: clear faults, to IDLE. */

/*
 * Settings, on robocar/id/config (task: telemetry).
 *
 * The barcode settings set what each barcode character means at the next
 * T, as the directions are only known on the day.  Each answers on the log
 * with the whole map, e.g. "barcodes: A=L K=R" (L left, R right,
 * S straight, U U-turn).  The map is kept until the car restarts, so the
 * dashboard sends it again after that.
 *
 *   CFG_BARCODE_DIR    character direction: the character as its ASCII
 *                      code ('0'-'9', 'A'-'Z'), then 0 left, 1 right,
 *                      2 straight, 3 U-turn or 4 ignore (barcode_cmd_t).
 *   CFG_BARCODE_CLEAR  ignore every character.
 *   CFG_BARCODE_SHOW   only report the map.
 */
#define CFG_TELEMETRY_PERIOD    (1u)    /* ms between samples, 100..60000 */
#define CFG_TELEMETRY_ENABLE    (2u)    /* 0 = stop sending, 1 = send */
#define CFG_BARCODE_DIR         (3u)
#define CFG_BARCODE_CLEAR       (4u)
#define CFG_BARCODE_SHOW        (5u)

/*
 * Events, on robocar/id/event (task: telemetry).  Each is sent once, when
 * it happens, and queued while offline like a log line.
 *
 *   EVT_HUMP     height_mm over_127 tilt_deg: a hump closed
 *                (imu_task_read_event, IF-S4-3).  Tilt is the largest
 *                pitch, or roll when only one rear wheel rode over it.
 *   EVT_BARCODE  letter: a barcode was read, the letter as its ASCII
 *                code (barcode_get_last, IF-18).
 *   EVT_OBSTACLE width_mm front_cm whole: Obstacle Scan (S5) measured an
 *                obstacle with the car stopped; whole is 1 when both
 *                edges were seen, 0 when one was past the sweep ("at
 *                least").  Sent by S5 itself.
 */
#define EVT_HUMP                (0u)
#define EVT_BARCODE             (1u)
#define EVT_OBSTACLE            (2u)

/*
 * Telemetry: 64-bit words, fields packed from bit 0 upwards, sent as
 * TLM_WORDS decimal numbers, one space apart.
 *
 * Word 0 has the layout of version 1, unchanged except that its version
 * field now reads 2: a reader that takes only the first number reads it
 * exactly as a layout 1 word.  Word 1 (TLM_W1_*) was added in layout 2,
 * when word 0 was full.  To add a field, use word 1's spare bits; when
 * those run out, add a word 2 the same way and bump TLM_LAYOUT.
 *
 * Each field is written as "offset, width", so that it can be passed
 * straight to the two helpers below:
 *     word = protocol_tlm_put(word, TLM_NAV_STATE, state);
 * Every field is unsigned.  Sources are the team's interfaces in TEAM.md.
 */

/*      Word 0                  offset, bits    meaning (source) */
#define TLM_SEQ                  0u, 8u  /* Sample count, wraps; a gap
                                            is a lost sample. */
#define TLM_NAV_STATE            8u, 3u  /* NAV_STATE_* (IF-14). */
#define TLM_NAV_REASON          11u, 3u  /* NAV_REASON_* (IF-14). */
#define TLM_FAULTS              14u, 8u  /* Bit n is fault bit n of the
                                            team's map, NAV_FLT_*. */
#define TLM_MOTION              22u, 2u  /* MOTION_ACTIVITY_* (IF-09). */
#define TLM_LINE                24u, 2u  /* line_state_t (IF-16). */
#define TLM_SPEED_LEFT          26u, 10u /* mm/s, TLM_SPEED_MAX = that or
                                            more (IF-06). */
#define TLM_SPEED_RIGHT         36u, 10u /* mm/s, as above. */
#define TLM_OBSTACLE            46u, 9u  /* cm to the nearest obstacle
                                            ahead, TLM_OBSTACLE_NONE = no
                                            echo (IF-16). */
#define TLM_RSSI                55u, 7u  /* Wi-Fi signal as -dBm, 0 =
                                            unknown. */
#define TLM_VERSION             62u, 2u  /* TLM_LAYOUT.  64 of 64 bits
                                            used. */

/*      Word 1                  offset, bits    meaning (source) */
#define TLM_W1_IMU_MOTION        0u, 2u  /* 0 stationary, 1 accelerating,
                                            2 cruising, 3 turning
                                            (IF-S4-1). */
#define TLM_W1_IMU_TERRAIN       2u, 2u  /* 0 level, 1 climbing,
                                            2 descending, 3 hump just
                                            closed (IF-S4-1). */
#define TLM_W1_IMU_OK            4u, 1u  /* 1 = the two above are live
                                            (imu_is_ok, IF-S4-7). */
#define TLM_W1_BARCODE           5u, 7u  /* Last barcode letter, ASCII;
                                            0 = none yet (IF-18). */
#define TLM_W1_BARCODE_COUNT    12u, 4u  /* Reads, wraps: a change is a
                                            new read (IF-18). */
#define TLM_W1_DISTANCE         16u, 20u /* cm since power-on, mean of
                                            both wheels, TLM_DISTANCE_MAX
                                            = that or more (IF-06).  36 of
                                            64 bits used. */

#define TLM_LAYOUT              (2u)    /* Bump when the layout changes. */
#define TLM_WORDS               (2u)    /* Words sent in this layout. */
#define TLM_LINE_UNKNOWN        (3u)
#define TLM_SPEED_MAX           (1023u)
#define TLM_OBSTACLE_NONE       (511u)
#define TLM_DISTANCE_MAX        (1048575u)

/**
 * @brief   Store a value in one field of a telemetry word.
 * @param   word    The word so far.
 * @param   offset  The field's first bit.
 * @param   bits    The field's width, 1 to 32.
 * @param   value   The value; bits above the field's width are dropped.
 * @return  The word with the field set.
 */
static inline uint64_t
protocol_tlm_put (uint64_t word, uint32_t offset, uint32_t bits,
                  uint32_t value)
{
    uint64_t const mask = ((1ull << bits) - 1ull) << offset;

    /* Cast: widening an unsigned value never changes it. */
    return ((word & ~mask) | (((uint64_t)value << offset) & mask));
}

/**
 * @brief   Read one field of a telemetry word back.
 * @param   word    The word.
 * @param   offset  The field's first bit.
 * @param   bits    The field's width, 1 to 32.
 * @return  The field's value.
 */
static inline uint32_t
protocol_tlm_get (uint64_t word, uint32_t offset, uint32_t bits)
{
    /* Cast: the mask leaves at most 32 bits, so the value fits. */
    return ((uint32_t)((word >> offset) & ((1ull << bits) - 1ull)));
}

#endif /* PROTOCOL_H */

/*** end of file ***/
