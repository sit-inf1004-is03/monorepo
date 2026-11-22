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

// Width sweep: cos(k x 5 deg) x 1000, and tan(k x 5 + 2.5 deg) x 1000
// (half a step past the last hit: the edge lies between the last hit
// and the first miss), k = 0..12.
//
#define WIDTH_SCALE             (1000u)

static uint16_t const g_wtan_mid_x1000[OBS_WIDTH_STEPS + 1u] =
{
    44u, 132u, 222u, 315u, 414u, 521u, 637u, 767u, 916u, 1091u, 1303u,
    1570u, 1921u
};

static uint16_t const g_wcos_x1000[OBS_WIDTH_STEPS + 1u] =
{
    1000u, 996u, 985u, 966u, 940u, 906u, 866u, 819u, 766u, 707u, 643u,
    574u, 500u
};

#define MRAD_PER_STEP           (262u)      // 15 deg = 0.2618 rad.
#define URAD_PER_DEG            (17453u)    // 1 deg = 0.017453 rad.
#define UM_PER_MM_X_URAD        (1000000u)  // urad x mm -> mm.

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
 *         the first clear one, and front x tan(edge) (the obstacle's
 *         half-width) limited to OBS_MIN_HALF_WIDTH_CM ..
 *         OBS_MAX_HALF_WIDTH_CM.
 * @param  front_cm   Distance to the obstacle straight ahead.
 * @param  edge_step  Fine step of the edge, 1..OBS_FINE_COUNT.
 * @return Distance in cm, saturated to 9 bits.
 */
uint16_t
obs_lateral_cm(uint16_t front_cm, uint16_t edge_step)
{
    uint16_t const step = clamp_step(edge_step);
    uint32_t       dist = div_up((uint32_t)front_cm * g_tan_edge_x100[step],
                                 TAN_SCALE);

    if (dist < OBS_MIN_HALF_WIDTH_CM)
    {
        dist = OBS_MIN_HALF_WIDTH_CM;   // Never narrower than a box.
    }
    else if (dist > OBS_MAX_HALF_WIDTH_CM)
    {
        dist = OBS_MAX_HALF_WIDTH_CM;   // Beam spread, not a wider box.
    }
    else
    {
        // Measured width is believable.
    }

    dist += OBS_CAR_HALF_WIDTH_CM + OBS_SIDE_MARGIN_CM;

    if (dist > PLAN_MAX_CM)
    {
        dist = PLAN_MAX_CM;
    }

    return ((uint16_t)dist);
}

/*!
 * @brief  Pivot-out angle: the smallest 15-degree step, from
 *         OBS_PIVOT_MIN_STEP, that reaches the sideways distance before
 *         the car reaches the obstacle's front: front x tan(pivot) >=
 *         lateral.
 * @param  front_cm    Distance to the obstacle straight ahead.
 * @param  lateral_cm  From obs_lateral_cm().
 * @return Pivot step OBS_PIVOT_MIN_STEP..OBS_PIVOT_MAX_STEP (x 15 deg);
 *         the maximum if
 *         even that is not enough.
 */
uint16_t
obs_pivot_step(uint16_t front_cm, uint16_t lateral_cm)
{
    uint32_t const need = (uint32_t)lateral_cm * TAN_SCALE;
    uint16_t       step = OBS_PIVOT_MIN_STEP;

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
                           g_sin_x100[clamp_step(pivot_step)]) +
                    OBS_OUT_EXTRA_CM;

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
 * @brief  Past leg: parallel to the line until the wheel axle is
 *         OBS_PAST_MARGIN_CM past the back of the obstacle.  From the
 *         start the axle must go front + depth + sensor-to-axle + margin;
 *         the out leg already went out x cos(pivot) of that (rounded
 *         down: the past leg errs long).
 * @param  front_cm    Distance to the obstacle straight ahead.
 * @param  out_cm      From obs_out_cm().
 * @param  pivot_step  Pivot step, 1..4 (x 15 deg).
 * @return Distance in cm, 1..511 (Navigation needs a move).
 */
uint16_t
obs_past_cm(uint16_t front_cm, uint16_t out_cm, uint16_t pivot_step)
{
    uint32_t const need = (uint32_t)front_cm + OBS_DEPTH_CM +
                          OBS_SENSOR_TO_AXLE_CM + OBS_PAST_MARGIN_CM;
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
 * @brief  Wheel travel for a pivot of any angle: angle (rad) x half the
 *         wheel track (the line search is not in 15-degree steps).
 * @param  angle_deg  Pivot angle, 0..360 deg.
 * @return Arc length of each wheel, in mm, rounded.
 */
uint16_t
obs_deg_arc_mm(uint16_t angle_deg)
{
    // 360 x 17453 x 70 = 4.4e8: fits 32 bits.
    return ((uint16_t)((((uint32_t)angle_deg * URAD_PER_DEG *
                         OBS_HALF_TRACK_MM) + (UM_PER_MM_X_URAD / 2u)) /
                       UM_PER_MM_X_URAD));
}

/*!
 * @brief  Line search zig-zag: heading to reach on search step `step`,
 *         relative to the heading the search started at.  Odd steps go
 *         one way (+), even steps the other (-), each 1 x step_deg further
 *         out: +start, -(start + step), +(start + 2 step), ...
 * @param  step       Search step, 1..; 0 gives 0 (the start heading).
 * @param  start_deg  Size of the first target.
 * @param  step_deg   How much further each target is.
 * @return Target heading in degrees, signed (+ = the first direction).
 */
int16_t
obs_search_target_deg(uint16_t step, uint16_t start_deg, uint16_t step_deg)
{
    int16_t target = 0;

    if (step > 0u)
    {
        target = (int16_t)(start_deg + ((step - 1u) * step_deg));

        if (0u == (step % 2u))
        {
            target = (int16_t)(-target);
        }
    }

    return (target);
}

/*!
 * @brief  Width sweep: does a reading at this step still come from the
 *         obstacle's front face?  Its forward part (dist x cos) must be
 *         within OBS_WIDTH_FACE_CM of the reading straight ahead; open
 *         space or something farther back ends the face.
 * @param  dist_cm   Reading at this step (OBS_FAR_CM = no echo).
 * @param  step      Width step, 0..OBS_WIDTH_STEPS (x OBS_WIDTH_STEP_DEG).
 * @param  front_cm  Reading straight ahead.
 */
bool
obs_is_on_face(uint16_t dist_cm, uint16_t step, uint16_t front_cm)
{
    bool b_on_face = false;

    if ((dist_cm < OBS_FAR_CM) && (step <= OBS_WIDTH_STEPS))
    {
        uint32_t const fwd_x1000 = (uint32_t)dist_cm *
                                   g_wcos_x1000[step];

        b_on_face = (fwd_x1000 <= ((uint32_t)(front_cm + OBS_WIDTH_FACE_CM)
                                   * WIDTH_SCALE));
    }

    return (b_on_face);
}

/*!
 * @brief  Half-width from the last hit on the face: its forward
 *         distance (dist x cos) x tan(step angle + half a step), i.e. the
 *         edge is taken midway to the first miss.
 * @param  dist_cm  Last reading on the face (step 0: the centre reading).
 * @param  step     Width step of that reading, 0..OBS_WIDTH_STEPS.
 * @return Distance in mm, rounded.
 */
uint16_t
obs_side_mm(uint16_t dist_cm, uint16_t step)
{
    uint32_t side = 0u;
    uint32_t fwd_mm;

    if ((dist_cm < OBS_FAR_CM) && (step <= OBS_WIDTH_STEPS))
    {
        // 510 cm x 10 x 1000 = 5.1e6, then 5100 mm x 1921: fit 32 bits.
        fwd_mm = (((uint32_t)dist_cm * 10u * g_wcos_x1000[step]) +
                  (WIDTH_SCALE / 2u)) / WIDTH_SCALE;
        side   = ((fwd_mm * g_wtan_mid_x1000[step]) + (WIDTH_SCALE / 2u)) /
                 WIDTH_SCALE;
    }

    return ((uint16_t)side);
}

/*!
 * @brief  Obstacle width from its two half-widths plus the calibration
 *         trim on each side; never below 0.
 * @return Width in mm.
 */
uint16_t
obs_width_mm(uint16_t left_mm, uint16_t right_mm)
{
    int32_t width = (int32_t)left_mm + (int32_t)right_mm +
                    (2 * (int32_t)OBS_WIDTH_TRIM_MM);

    if (width < 0)
    {
        width = 0;
    }

    return ((uint16_t)width);
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
