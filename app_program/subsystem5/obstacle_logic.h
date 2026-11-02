/** @file obstacle_logic.h
 *
 * @brief Obstacle Scan decision logic (Subsystem 5): pure functions with
 *        no hardware or kernel calls, so subsystem5_test.c can check them.
 *
 * Scan profiles store one distance per angle in 9 bits, packed into one
 * uint64_t (minimum-bit requirement): up to 7 slots.
 *
 * Coding standard: BARR-C:2018.
 */

#ifndef OBSTACLE_LOGIC_H
#define OBSTACLE_LOGIC_H

#include <stdint.h>

// "No echo / out of range" is stored as the largest 9-bit value and
// treated as open space.
//
#define OBS_FAR_CM              (511u)

// Side, 2 bits.  Mapped to NAV_SIDE_ only when posting to Navigation.
//
#define OBS_SIDE_NONE           (0u)
#define OBS_SIDE_LEFT           (1u)
#define OBS_SIDE_RIGHT          (2u)

// Which servo angles look LEFT of the car.  Checked on the car on
// 2026-10-26 (bottle front-left seen at 105-120 deg): 1 is correct.
//
#define OBS_LEFT_IS_HIGH_ANGLE  (1)

// Coarse profile slots: 30, 60, 90, 120, 150 deg.
//
#define OBS_COARSE_COUNT        (5u)
#define OBS_COARSE_CENTRE       (2u)

// Fine profile slots: 15, 30, 45, 60 deg out from the centre.
//
#define OBS_FINE_COUNT          (4u)
#define OBS_FINE_STEP_DEG       (15u)

// Bypass (three legs): pivot out and drive "out" until the car is
// sideways clear of the obstacle; pivot back parallel to the line and
// drive "past" until the whole car is beyond the obstacle; pivot in at
// OBS_REJOIN_STEP and creep at most "back" to find the line.
//
#define OBS_CAR_HALF_WIDTH_CM   (10u)   // TODO: measure the real car.
#define OBS_SIDE_MARGIN_CM      (3u)

// The sensor only sees the obstacle's front face, so its depth is an
// assumption: set it to the deepest obstacle on the track.
//
#define OBS_DEPTH_CM            (8u)    // A bottle.

// Sensor (front of the car) to the back of the car.  The past leg ends
// when the back of the car is past the back of the obstacle.
//
#define OBS_CAR_LENGTH_CM       (20u)   // TODO: measure the real car.

// Half the distance between the two wheel centres: turns a pivot angle
// into wheel travel for the encoders.  If the car turns too little, make
// it larger; too much, smaller.
//
#define OBS_HALF_TRACK_MM       (70u)   // TODO: measure the real car.

// Limits for a small track: the bypass never turns out more than
// OBS_PIVOT_MAX_STEP x 15 deg nor drives the out leg more than
// OBS_OUT_MAX_CM.
//
#define OBS_PIVOT_MAX_STEP      (3u)    // 45 deg.
#define OBS_OUT_MAX_CM          (40u)

// Turn-in angle (x 15 deg): shallow so Line Following has time to see
// the line.  The back leg is the straight-line distance to the line at
// this angle plus OBS_REJOIN_SLACK_CM; if the line is not found by then
// Navigation stops (LINE_NOT_FOUND) instead of wandering onto another
// line.
//
#define OBS_REJOIN_STEP         (2u)    // 30 deg.
#define OBS_REJOIN_SLACK_CM     (15u)

uint16_t obs_median3(uint16_t first, uint16_t second, uint16_t third);
uint16_t obs_prof_get(uint64_t profile, uint16_t slot);
uint64_t obs_prof_set(uint64_t profile, uint16_t slot, uint16_t dist_cm);
uint16_t obs_pick_side(uint64_t coarse, uint16_t clear_cm);
uint16_t obs_find_edge(uint64_t fine, uint16_t clear_cm);
uint16_t obs_lateral_cm(uint16_t front_cm, uint16_t edge_step);
uint16_t obs_pivot_step(uint16_t front_cm, uint16_t lateral_cm);
uint16_t obs_pivot_arc_mm(uint16_t pivot_step);
uint16_t obs_out_cm(uint16_t lateral_cm, uint16_t pivot_step);
uint16_t obs_past_cm(uint16_t front_cm, uint16_t out_cm,
                     uint16_t pivot_step);
uint16_t obs_back_cm(uint16_t out_cm, uint16_t pivot_step);

#endif /* OBSTACLE_LOGIC_H */

/*** end of file ***/
