/** @file obstacle_logic.c
 *
 * @brief Obstacle Scan decision logic; see obstacle_logic.h.
 *
 * Coding standard: BARR-C:2018.
 */

#include <stdbool.h>
#include <stdint.h>

#include "obstacle_logic.h"

#define SLOT_BITS               (9u)
#define SLOT_MASK               (0x1FFu)
#define MAX_SLOTS               (7u)        // 7 x 9 = 63 bits of 64.
#define LATERAL_MAX_CM          (511u)      // 9-bit plan distance.
#define TAN_SCALE               (100u)

// tan(k x 15 deg) x 100, k = 0..4.
//
static uint16_t const g_tan_x100[OBS_FINE_COUNT + 1u] =
{
    0u, 27u, 58u, 100u, 173u
};

/*!
 * @brief  Median of three pings: removes one stray echo.  "Open"
 *         (OBS_FAR_CM) wins only if at least two of the three say so.
 */
uint16_t
obs_median3(uint16_t first, uint16_t second, uint16_t third)
{
    uint16_t median;

    if (((first <= second) && (second <= third)) ||
        ((third <= second) && (second <= first)))
    {
        median = second;
    }
    else if (((second <= first) && (first <= third)) ||
             ((third <= first) && (first <= second)))
    {
        median = first;
    }
    else
    {
        median = third;
    }

    return (median);
}

/*!
 * @brief  Read one 9-bit slot of a packed profile (0 if out of range).
 */
uint16_t
obs_prof_get(uint64_t profile, uint16_t slot)
{
    uint16_t value = 0u;

    if (slot < MAX_SLOTS)
    {
        value = (uint16_t)((profile >> ((uint64_t)slot * SLOT_BITS)) &
                           SLOT_MASK);
    }

    return (value);
}

/*!
 * @brief  Write one 9-bit slot; larger distances saturate, never wrap.
 */
uint64_t
obs_prof_set(uint64_t profile, uint16_t slot, uint16_t dist_cm)
{
    uint64_t const shift = (uint64_t)slot * SLOT_BITS;
    uint64_t       result = profile;

    if (slot < MAX_SLOTS)
    {
        if (dist_cm > SLOT_MASK)
        {
            dist_cm = SLOT_MASK;
        }

        result &= ~((uint64_t)SLOT_MASK << shift);
        result |= ((uint64_t)dist_cm << shift);
    }

    return (result);
}

/*!
 * @brief  Direction Decision (Week 6): the safe and shortest side.
 *
 * A side is a candidate if at least one of its two angles is open
 * (> clear_cm).  Among candidates, prefer the side whose CLOSEST reading
 * is furthest away (less of the obstacle on it, so a shorter way round).
 * Ties go to the more open side, then to LEFT.
 *
 * @return OBS_SIDE_LEFT, OBS_SIDE_RIGHT, or OBS_SIDE_NONE.
 */
uint16_t
obs_pick_side(uint64_t coarse, uint16_t clear_cm)
{
    uint16_t const a30  = obs_prof_get(coarse, 0u);
    uint16_t const a60  = obs_prof_get(coarse, 1u);
    uint16_t const a120 = obs_prof_get(coarse, 3u);
    uint16_t const a150 = obs_prof_get(coarse, 4u);
    bool const     b_is_left_high = (1 == OBS_LEFT_IS_HIGH_ANGLE);
    uint16_t const low_min   = (a30 < a60) ? a30 : a60;
    uint16_t const low_max   = (a30 > a60) ? a30 : a60;
    uint16_t const high_min  = (a120 < a150) ? a120 : a150;
    uint16_t const high_max  = (a120 > a150) ? a120 : a150;
    uint16_t const left_min  = b_is_left_high ? high_min : low_min;
    uint16_t const left_max  = b_is_left_high ? high_max : low_max;
    uint16_t const right_min = b_is_left_high ? low_min : high_min;
    uint16_t const right_max = b_is_left_high ? low_max : high_max;
    bool const     b_is_left_ok  = (left_max > clear_cm);
    bool const     b_is_right_ok = (right_max > clear_cm);
    uint16_t       side;

    if (b_is_left_ok && b_is_right_ok)
    {
        if (left_min != right_min)
        {
            side = (left_min > right_min) ? OBS_SIDE_LEFT : OBS_SIDE_RIGHT;
        }
        else
        {
            side = (left_max >= right_max) ? OBS_SIDE_LEFT : OBS_SIDE_RIGHT;
        }
    }
    else if (b_is_left_ok)
    {
        side = OBS_SIDE_LEFT;
    }
    else if (b_is_right_ok)
    {
        side = OBS_SIDE_RIGHT;
    }
    else
    {
        side = OBS_SIDE_NONE;
    }

    return (side);
}

/*!
 * @brief  First fine step (1..OBS_FINE_COUNT) that looks past the
 *         obstacle's edge.
 * @return Step number, or 0 if the edge was not found.
 */
uint16_t
obs_find_edge(uint64_t fine, uint16_t clear_cm)
{
    uint16_t step;
    uint16_t edge = 0u;

    for (step = 1u; step <= OBS_FINE_COUNT; step++)
    {
        if (obs_prof_get(fine, (uint16_t)(step - 1u)) > clear_cm)
        {
            edge = step;
            break;
        }
    }

    return (edge);
}

/*!
 * @brief  Sideways distance to clear the obstacle:
 *         front x tan(edge angle) + half car width + margin.
 * @param  front_cm   Distance to the obstacle straight ahead.
 * @param  edge_step  Fine step of the edge, 1..OBS_FINE_COUNT.
 * @return Distance in cm, saturated to 9 bits.
 */
uint16_t
obs_lateral_cm(uint16_t front_cm, uint16_t edge_step)
{
    uint16_t const step = (edge_step > OBS_FINE_COUNT) ?
                          (uint16_t)OBS_FINE_COUNT : edge_step;
    uint32_t       dist = (((uint32_t)front_cm * g_tan_x100[step]) /
                           TAN_SCALE) + OBS_CAR_HALF_WIDTH_CM +
                          OBS_SIDE_MARGIN_CM;

    if (dist > LATERAL_MAX_CM)
    {
        dist = LATERAL_MAX_CM;
    }

    return ((uint16_t)dist);
}

/*** end of file ***/
