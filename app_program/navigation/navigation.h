/** @file navigation.h
 *
 * @brief Navigation / Vehicle Controller: holds the mission state and is
 *        the only module that sends commands to Motion (motion.h).
 *
 * Every other module talks to Navigation through the functions below, and
 * none of them waits: each input is stored and Navigation reacts within
 * its next step.
 *   Line Following / Barcode (Wing):  nav_post_barcode(),
 *                                     nav_post_junction(),
 *                                     nav_set_line_state()
 *   Obstacle Scan (Ruien):            nav_post_obstacle(),
 *                                     nav_post_bypass_plan()
 *   Any subsystem, its own bits only: nav_raise_fault()
 *   Wi-Fi / MQTT commands (Rifa):     nav_post_remote()
 *   Everyone (telemetry, scan arm):   nav_get_state(), nav_get_faults(),
 *                                     nav_get_reason()
 * StopTurn and the turn level go straight to Motion (motion_stop_turn(),
 * motion_set_turn_level()), not through Navigation.
 *
 * PROPOSED: the vehicle states, the bypass plan's bit order, the fault
 * stop mask (G10) and the bypass sequence still need team agreement.  The
 * fault bits are the agreed map; they move to the team's event flag and
 * fault header once G1/G2 are settled, and the layouts to msg_formats.h.
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdint.h>

#include "../app.h"

#define NAV_PRIORITY            (10)        // Band 10-11, TEAM.md section 4
                                            // (proposed).

// Vehicle state, 3 bits (nav_get_state()).
//
#define NAV_STATE_IDLE          (0u)    // Waiting for a remote START.
#define NAV_STATE_LINE_FOLLOW   (1u)
#define NAV_STATE_JUNCTION      (2u)    // Barcode read: slowing, turning.
#define NAV_STATE_OBSTACLE      (3u)    // Braked; Obstacle Scan scans now.
#define NAV_STATE_BYPASS        (4u)    // Around the obstacle, back to line.
#define NAV_STATE_STOPPED       (5u)    // Remote STOP.
#define NAV_STATE_FAULT         (6u)    // Braked; needs a remote CLEAR.
#define NAV_STATE_BITS          (3u)

// Barcode command, 2 bits (agreed).
//
#define NAV_BARCODE_LEFT        (0u)    // A
#define NAV_BARCODE_RIGHT       (1u)    // B
#define NAV_BARCODE_STRAIGHT    (2u)    // C
#define NAV_BARCODE_UTURN       (3u)    // D

// Line state, 2 bits (agreed).
//
#define NAV_LINE_ON             (0u)
#define NAV_LINE_LOST           (1u)
#define NAV_LINE_SEARCHING      (2u)

// Bypass side.
//
#define NAV_SIDE_LEFT           (0u)
#define NAV_SIDE_RIGHT          (1u)
#define NAV_BYPASS_CM_MAX       (511u)  // 9-bit distance field.

// Remote commands.
//
#define NAV_REMOTE_START        (1u)    // IDLE or STOPPED: follow the line.
#define NAV_REMOTE_STOP         (2u)    // Brake and wait.
#define NAV_REMOTE_CLEAR        (3u)    // FAULT: clear the faults, to IDLE.

// Fault bits: the team's agreed map.
//
#define NAV_FLT_MOTOR_STALL     (1u << 0)   // Glenn
#define NAV_FLT_ENCODER         (1u << 1)   // Glenn
#define NAV_FLT_LINE_LOST       (1u << 2)   // Wing
#define NAV_FLT_IMU             (1u << 3)   // Andrew
#define NAV_FLT_ESTOP           (1u << 4)   // Andrew
#define NAV_FLT_NO_ECHO         (1u << 5)   // Ruien
#define NAV_FLT_SCAN_ERROR      (1u << 6)   // Ruien
#define NAV_FLT_WIFI_LOSS       (1u << 7)   // Rifa: Degraded only, no stop.
#define NAV_FLT_ALL             (0xFFu)
#define NAV_FLT_STOP_MASK       (NAV_FLT_ALL & ~NAV_FLT_WIFI_LOSS)  // G10

// Why Navigation stopped (nav_get_reason()).
//
#define NAV_REASON_NONE             (0u)
#define NAV_REASON_FAULT_BIT        (1u)    // A bit in NAV_FLT_STOP_MASK.
#define NAV_REASON_MOTION           (2u)    // Motion replied FAULT.
#define NAV_REASON_LINE_NOT_FOUND   (3u)    // After a bypass.
#define NAV_REASON_REMOTE_STOP      (4u)

ER       navigation_init(void);
ER       navigation_start(void);
#if APP_TEST
void     navigation_test(void);
#endif

ER       nav_post_barcode(uint32_t command);
ER       nav_post_junction(void);
ER       nav_set_line_state(uint32_t state);
ER       nav_post_obstacle(void);
ER       nav_post_bypass_plan(uint32_t side, uint32_t distance_cm);
ER       nav_raise_fault(uint32_t bits);
ER       nav_post_remote(uint32_t command);

uint32_t nav_get_state(void);
uint32_t nav_get_faults(void);
uint32_t nav_get_reason(void);

uint32_t nav_plan_pack(uint32_t side, uint32_t distance_cm);
uint32_t nav_plan_side(uint32_t plan);
uint32_t nav_plan_distance_cm(uint32_t plan);

#endif /* NAVIGATION_H */

/*** end of file ***/

