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
#define PLAN_MAX_CM             (511u)      // 9-bit plan distances.
#define TAN_SCALE               (100u)

// tan(k x 15 deg) x 100, k = 0..4.
//
static uint16_t const g_tan_x100[OBS_FINE_COUNT + 1u] =
{
    0u, 27u, 58u, 100u, 173u
};

// tan((k - 0.5) x 15 deg) x 100, k = 1..4: the middle of the last
// blocked fine step and the first clear one, where the obstacle's edge
// is most likely (the beam is wide, so the first clear step is late).
//
static uint16_t const g_tan_edge_x100[OBS_FINE_COUNT + 1u] =
{
    0u, 13u, 41u, 77u, 130u
};

// sin(k x 15 deg) x 100, k = 0..4.
//
static uint16_t const g_sin_x100[OBS_FINE_COUNT + 1u] =
{
    0u, 26u, 50u, 71u, 87u
};

// cos(k x 15 deg) x 100, k = 0..4.
//
static uint16_t const g_cos_x100[OBS_FINE_COUNT + 1u] =
{
    100u, 97u, 87u, 71u, 50u
};

#define MRAD_PER_STEP           (262u)      // 15 deg = 0.2618 rad.

static uint32_t div_up(uint32_t num, uint32_t den);
static uint16_t clamp_step(uint16_t step);

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
 * @brief  Sideways distance (from the line) the car's centre needs to
 *         clear the obstacle: front x tan(edge) + half car width +
 *         margin, with the edge between the last blocked fine step and
 *         the first clear one.
 * @param  front_cm   Distance to the obstacle straight ahead.
 * @param  edge_step  Fine step of the edge, 1..OBS_FINE_COUNT.
 * @return Distance in cm, saturated to 9 bits.
 */
uint16_t
obs_lateral_cm(uint16_t front_cm, uint16_t edge_step)
{
    uint16_t const step = clamp_step(edge_step);
    uint32_t       dist = div_up((uint32_t)front_cm * g_tan_edge_x100[step],
                                 TAN_SCALE) +
                          OBS_CAR_HALF_WIDTH_CM + OBS_SIDE_MARGIN_CM;

    if (dist > PLAN_MAX_CM)
    {
        dist = PLAN_MAX_CM;
    }

    return ((uint16_t)dist);
}

/*!
 * @brief  Pivot-out angle: the smallest 15-degree step that reaches the
 *         sideways distance before the car reaches the obstacle's front:
 *         front x tan(pivot) >= lateral.
 * @param  front_cm    Distance to the obstacle straight ahead.
 * @param  lateral_cm  From obs_lateral_cm().
 * @return Pivot step 1..OBS_PIVOT_MAX_STEP (x 15 deg); the maximum if
 *         even that is not enough.
 */
uint16_t
obs_pivot_step(uint16_t front_cm, uint16_t lateral_cm)
{
    uint32_t const need = (uint32_t)lateral_cm * TAN_SCALE;
    uint16_t       step = 1u;

    while ((step < OBS_PIVOT_MAX_STEP) &&
           (((uint32_t)front_cm * g_tan_x100[step]) < need))
    {
        step++;
    }

    return (step);
}

/*!
 * @brief  Wheel travel for a pivot: angle (rad) x half the wheel track.
 * @param  pivot_step  Pivot step, 0..4 (x 15 deg).
 * @return Arc length of each wheel, in mm.
 */
uint16_t
obs_pivot_arc_mm(uint16_t pivot_step)
{
    uint16_t const step = (pivot_step > OBS_FINE_COUNT) ?
                          (uint16_t)OBS_FINE_COUNT : pivot_step;

    return ((uint16_t)((((uint32_t)step * MRAD_PER_STEP *
                         OBS_HALF_TRACK_MM) + 500u) / 1000u));
}

/*!
 * @brief  Out leg: drive at the pivot angle until the car is the
 *         sideways distance from the line: lateral / sin(pivot), rounded
 *         up.  No further: going wider only risks other lines.
 * @param  lateral_cm  From obs_lateral_cm().
 * @param  pivot_step  Pivot step, 1..4 (x 15 deg).
 * @return Distance in cm, 1..OBS_OUT_MAX_CM.
 */
uint16_t
obs_out_cm(uint16_t lateral_cm, uint16_t pivot_step)
{
    uint32_t dist = div_up((uint32_t)lateral_cm * TAN_SCALE,
                           g_sin_x100[clamp_step(pivot_step)]);

    if (dist > OBS_OUT_MAX_CM)
    {
        dist = OBS_OUT_MAX_CM;
    }
    else if (0u == dist)
    {
        dist = 1u;
    }
    else
    {
        // In range.
    }

    return ((uint16_t)dist);
}

/*!
 * @brief  Past leg: parallel to the line until the back of the car is
 *         past the back of the obstacle.  From the start the car's front
 *         must go front + depth + car length; the out leg already went
 *         out x cos(pivot) of that (rounded down: the past leg errs
 *         long).
 * @param  front_cm    Distance to the obstacle straight ahead.
 * @param  out_cm      From obs_out_cm().
 * @param  pivot_step  Pivot step, 1..4 (x 15 deg).
 * @return Distance in cm, 1..511 (Navigation needs a move).
 */
uint16_t
obs_past_cm(uint16_t front_cm, uint16_t out_cm, uint16_t pivot_step)
{
    uint32_t const need = (uint32_t)front_cm + OBS_DEPTH_CM +
                          OBS_CAR_LENGTH_CM;
    uint32_t const done = ((uint32_t)out_cm *
                           g_cos_x100[clamp_step(pivot_step)]) / TAN_SCALE;
    uint32_t       dist = 1u;

    if (need > done)
    {
        dist = need - done;
    }

    if (dist > PLAN_MAX_CM)
    {
        dist = PLAN_MAX_CM;
    }

    return ((uint16_t)dist);
}

/*!
 * @brief  Back leg: the most Navigation may creep after pivoting in at
 *         OBS_REJOIN_STEP before it gives up.  The car is out x
 *         sin(pivot) from the line, so it meets it after that /
 *         sin(rejoin); OBS_REJOIN_SLACK_CM more covers pivot error.
 * @param  out_cm      From obs_out_cm().
 * @param  pivot_step  Pivot step, 1..4 (x 15 deg).
 * @return Distance in cm, at most 511.
 */
uint16_t
obs_back_cm(uint16_t out_cm, uint16_t pivot_step)
{
    uint32_t dist = div_up((uint32_t)out_cm *
                           g_sin_x100[clamp_step(pivot_step)],
                           g_sin_x100[clamp_step(OBS_REJOIN_STEP)]) +
                    OBS_REJOIN_SLACK_CM;

    if (dist > PLAN_MAX_CM)
    {
        dist = PLAN_MAX_CM;
    }

    return ((uint16_t)dist);
}

/*!
 * @brief  Limit a 15-degree step to 1..OBS_FINE_COUNT (never 0, so the
 *         sin tables are never divided by 0).
 */
static uint16_t
clamp_step(uint16_t step)
{
    uint16_t result = step;

    if (0u == result)
    {
        result = 1u;
    }
    else if (result > OBS_FINE_COUNT)
    {
        result = OBS_FINE_COUNT;
    }
    else
    {
        // In range.
    }

    return (result);
}

/*!
 * @brief  Integer division rounded up (den is never 0: cos table).
 */
static uint32_t
div_up(uint32_t num, uint32_t den)
{
    return ((num + den - 1u) / den);
}

/*** end of file ***/
