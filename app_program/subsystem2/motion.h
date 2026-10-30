/** @file motion.h
 *
 * @brief Subsystem 2 Motion Control: task_motion and its interface.
 *
 * task_motion is the only task that drives the motors.  It runs every
 * 10 ms and carries out one command at a time from Navigation:
 *   - straight(direction, speed, distance): drive a distance, holding both
 *     wheel speeds with PID and steering by the turn level;
 *   - pivot(direction): turn on the spot at a calibrated speed until
 *     StopTurn.
 * Each command gets exactly one reply: DONE, BRAKED, or FAULT with a
 * reason.  Navigation can brake the running move (motion_brake(), for an
 * obstacle or before a junction turn) and cap the speed of a straight
 * (motion_set_speed_cap(), to slow down approaching a junction).
 *
 * PROPOSED (G10, G1, G2): the command and reply layouts, the turn level
 * and StopTurn are not agreed yet.  Until they are, the layouts below are
 * plain structs private to this interface, StopTurn is a bit of Motion's
 * own event flag (it moves to the team's event flag, bit 8), and faults
 * are reported only in the reply and by motion_get_faults().  The brake
 * command changes the Tribunal 1 design, which had no stop command.
 */

#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

#include "../app.h"

#define MOTION_PRIORITY         (14)        // TEAM.md section 4.
#define MOTION_PERIOD_MSEC      (10u)

// Command types and directions.
//
#define MOTION_CMD_STRAIGHT     (1u)
#define MOTION_CMD_PIVOT        (2u)
#define MOTION_DIR_FORWARDS     (0u)        // MOTION_CMD_STRAIGHT
#define MOTION_DIR_BACKWARDS    (1u)
#define MOTION_DIR_LEFT         (0u)        // MOTION_CMD_PIVOT
#define MOTION_DIR_RIGHT        (1u)

#define MOTION_SPEED_MAX_MM_S   (700u)      // Leaves headroom for steering
                                            // below the ~820 mm/s top speed.
#define MOTION_DISTANCE_MAX_MM  (100000u)   // 100 m: far beyond the track.

// Reply status and fault reasons.
//
#define MOTION_DONE             (0u)
#define MOTION_FAULT            (1u)
#define MOTION_BRAKED           (2u)        // Ended by motion_brake().
#define MOTION_REASON_NONE      (0u)
#define MOTION_REASON_BAD_CMD   (1u)        // Unknown type or out of range.
#define MOTION_REASON_STALL     (2u)        // Fault bit 0.
#define MOTION_REASON_TIMEOUT   (3u)        // Pivot without StopTurn.

// Bits of motion_get_faults(), matching the team's fault bit map.
//
#define MOTION_FAULT_STALL      (1u << 0)

#define MOTION_TURN_LEVEL_MAX   (7)         // Turn level -7..+7 (Wing).

// What task_motion is doing, from motion_get_activity().  A StopTurn
// sender watches for MOTION_ACTIVITY_PIVOT to know a pivot has started.
//
#define MOTION_ACTIVITY_IDLE        (0u)
#define MOTION_ACTIVITY_STRAIGHT    (1u)
#define MOTION_ACTIVITY_PIVOT       (2u)

typedef struct
{
    uint32_t type;          // MOTION_CMD_
    uint32_t direction;     // MOTION_DIR_
    uint32_t speed_mm_s;    // Straight only: 1 to MOTION_SPEED_MAX_MM_S.
    uint32_t distance_mm;   // Straight only: at least 1.
} motion_cmd_t;

typedef struct
{
    uint32_t status;        // MOTION_DONE or MOTION_FAULT.
    uint32_t reason;        // MOTION_REASON_
} motion_reply_t;

ER       motion_init(void);
ER       motion_start(void);
ER       motion_send(motion_cmd_t const * p_cmd, TMO timeout_msec);
ER       motion_wait_reply(motion_reply_t * p_reply, TMO timeout_msec);
void     motion_set_turn_level(int32_t level);
ER       motion_stop_turn(void);
ER       motion_brake(void);
void     motion_set_speed_cap(uint32_t speed_mm_s);
uint32_t motion_get_activity(void);
uint32_t motion_get_faults(void);
uint32_t motion_get_max_loop_usec(void);

#endif /* MOTION_H */

/*** end of file ***/

