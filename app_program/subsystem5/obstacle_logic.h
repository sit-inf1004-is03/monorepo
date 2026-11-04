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
#define OBS_CAR_HALF_WIDTH_CM   (8u)    // TODO: measure the real car.
#define OBS_SIDE_MARGIN_CM      (0u)    // Gap: car side to box side.
                                        // 0: shortest out diagonal; the
                                        // half-width limits above still
                                        // leave 0.5-2.5 cm on a 15 cm box.

// The sensor only sees the obstacle's front face, so its depth is an
// assumption: set it to the deepest obstacle on the track.
//
#define OBS_DEPTH_CM            (15u)   // Deepest box on the track.

// Obstacle half-width used for the sideways distance: the measured value
// (front x tan(edge)), limited to MIN..MAX.  Boxes on the track are 10 to
// 15 cm wide.  The sonar beam (~30 deg) makes a box look wider, never
// narrower, so MIN = the widest box (a false "open" ping on a flat face
// cannot make the car clip it) and MAX = the widest box + 2 cm beam slack
// (a box that looks huge does not swing the car 30 cm out).
//
#define OBS_MIN_HALF_WIDTH_CM   (8u)    // 15 cm box.
#define OBS_MAX_HALF_WIDTH_CM   (10u)   // 15 cm box + beam slack.

// Ultrasonic sensor to the wheel axle (the pivot centre).  The past leg
// ends when the axle is OBS_PAST_MARGIN_CM past the obstacle's back: the
// turn in swings the rear of the car away from the obstacle, and the
// diagonal then moves the car along its own length, so its side never
// closes on the obstacle's corner.
//
// The obstacle's back is MEASURED: in the past leg the sensor looks
// sideways at it, and once it is gone the car drives sensor-to-axle +
// margin more.  obs_past_cm() (with OBS_DEPTH_CM) is only the estimate
// used if the side scan never sees the obstacle; Navigation's limit is
// that estimate + OBS_PAST_SCAN_EXTRA_CM, for deeper obstacles.
//
#define OBS_SENSOR_TO_AXLE_CM   (8u)    // TODO: measure the real car.
#define OBS_PAST_MARGIN_CM      (5u)    // Buffer after the axle is past;
                                        // >= 3 cm with no side gap, or the
                                        // turn-in clips the back corner.
#define OBS_PAST_SCAN_EXTRA_CM  (15u)

// Half the distance between the two wheel centres: turns a pivot angle
// into wheel travel for the encoders.  If the car turns too little, make
// it larger; too much, smaller.
//
#define OBS_HALF_TRACK_MM       (70u)   // TODO: measure the real car.

// Pivot-out angle range (x 15 deg).  The steepest turn gives the
// shortest out diagonal for the same sideways distance (the straight is
// measured, so the forward distance lost costs nothing), so MIN = MAX =
// 45 deg.  Lower MIN to let a far obstacle get a gentler turn.  The out
// leg never drives more than OBS_OUT_MAX_CM.
//
#define OBS_PIVOT_MIN_STEP      (3u)    // 45 deg.
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
uint16_t obs_deg_arc_mm(uint16_t angle_deg);
int16_t  obs_search_target_deg(uint16_t step, uint16_t start_deg,
                               uint16_t step_deg);

#endif /* OBSTACLE_LOGIC_H */

/*** end of file ***/
