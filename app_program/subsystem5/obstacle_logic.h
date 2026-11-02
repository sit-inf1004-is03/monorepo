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

#define OBS_CAR_HALF_WIDTH_CM   (10u)   // TODO: measure the real car.
#define OBS_SIDE_MARGIN_CM      (5u)

uint16_t obs_median3(uint16_t first, uint16_t second, uint16_t third);
uint16_t obs_prof_get(uint64_t profile, uint16_t slot);
uint64_t obs_prof_set(uint64_t profile, uint16_t slot, uint16_t dist_cm);
uint16_t obs_pick_side(uint64_t coarse, uint16_t clear_cm);
uint16_t obs_find_edge(uint64_t fine, uint16_t clear_cm);
uint16_t obs_lateral_cm(uint16_t front_cm, uint16_t edge_step);

#endif /* OBSTACLE_LOGIC_H */

/*** end of file ***/
