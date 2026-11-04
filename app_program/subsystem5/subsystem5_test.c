/** @file subsystem5_test.c
 *
 * @brief Subsystem 5 on-target tests (make TEST=1).  IDs match
 *        testing/subsystem5/TEST_LOG.md.
 *
 * S5-U01..U06 check the decision logic and the minimum-bit packing with
 * fixed inputs; they need no hardware.  S5-U07 needs the sensor wired.
 *
 * Coding standard: BARR-C:2018.
 */

#include "subsystem5.h"

#if APP_TEST

#include <stdbool.h>
#include <stdint.h>

#include "../test_report.h"
#include "../navigation/navigation.h"
#include "drv_ultrasonic.h"
#include "obstacle_logic.h"

#define CLEAR_CM        (50u)

static uint64_t make_profile(uint16_t const * p_cm, uint16_t count);

EXPORT void
subsystem5_test(void)
{
    static uint16_t const left_bottle[OBS_COARSE_COUNT] =
        { 511u, 511u, 16u, 16u, 511u };
    static uint16_t const right_bottle[OBS_COARSE_COUNT] =
        { 511u, 16u, 16u, 511u, 511u };
    static uint16_t const wall_right[OBS_COARSE_COUNT] =
        { 40u, 25u, 25u, 25u, 200u };
    static uint16_t const boxed_in[OBS_COARSE_COUNT] =
        { 25u, 25u, 25u, 25u, 25u };
    static uint16_t const fine_left[OBS_FINE_COUNT] =
        { 25u, 25u, 200u, 200u };
    uint64_t             profile;
    ultrasonic_reading_t reading;
    uint32_t             plan;

    TEST_INIT();

    // Distance maths: 0.5 cm rounding, clamp to 9 bits.
    TEST_CHECK("S5-U01", (17u == ultrasonic_width_to_cm(1000u)) &&
               (400u == ultrasonic_width_to_cm(30000u)),
               "echo width to cm, rounded and clamped to 400");

    // Profile packing: 9 bits per slot, saturates, slots independent.
    profile = obs_prof_set(0u, 2u, 600u);
    profile = obs_prof_set(profile, 3u, 123u);
    TEST_CHECK("S5-U02", (511u == obs_prof_get(profile, 2u)) &&
               (123u == obs_prof_get(profile, 3u)) &&
               (0u == obs_prof_get(profile, 1u)),
               "9-bit profile slots saturate and do not overlap");

    // Median of 3: one stray echo is ignored.
    TEST_CHECK("S5-U03", (31u == obs_median3(30u, 175u, 31u)) &&
               (511u == obs_median3(511u, 20u, 511u)),
               "median of 3 removes one stray reading");

    // Direction Decision: go round the side the obstacle is NOT on.
    TEST_CHECK("S5-U04",
               (OBS_SIDE_RIGHT == obs_pick_side(make_profile(left_bottle,
                    OBS_COARSE_COUNT), CLEAR_CM)) &&
               (OBS_SIDE_LEFT == obs_pick_side(make_profile(right_bottle,
                    OBS_COARSE_COUNT), CLEAR_CM)) &&
               (OBS_SIDE_LEFT == obs_pick_side(make_profile(wall_right,
                    OBS_COARSE_COUNT), CLEAR_CM)) &&
               (OBS_SIDE_NONE == obs_pick_side(make_profile(boxed_in,
                    OBS_COARSE_COUNT), CLEAR_CM)),
               "side choice: bottle left/right, wall right, boxed in");

    // Edge and sideways distance: first clear step 45 deg, so the edge
    // is taken at 37.5 deg; 25 cm ahead -> 25 x 0.77 = 20, limited to
    // OBS_MAX_HALF_WIDTH_CM: 10 + 8 + 0 = 18 cm.  Edge at 15 deg (7.5):
    // 25 x 0.13 = 4, raised to OBS_MIN_HALF_WIDTH_CM: 8 + 8 + 0 = 16 cm.
    TEST_CHECK("S5-U05",
               (3u == obs_find_edge(make_profile(fine_left,
                                                 OBS_FINE_COUNT), CLEAR_CM))
               && (18u == obs_lateral_cm(25u, 3u)) &&
               (16u == obs_lateral_cm(25u, 1u)),
               "fine scan edge step and lateral distance");

    // Three legs, 25 cm ahead, 24 cm sideways, 45 deg out:
    // out 24 / 0.71 = 34; past 25 + 15 + 8 + 5 - 34 x 0.71 (24) = 29;
    // back 34 x 0.71 / sin 30 deg (49) + 15 slack = 64.  The out leg is
    // capped at OBS_OUT_MAX_CM; the past leg is never 0.
    TEST_CHECK("S5-U08", (34u == obs_out_cm(24u, 3u)) &&
               (29u == obs_past_cm(25u, 34u, 3u)) &&
               (64u == obs_back_cm(34u, 3u)) &&
               (OBS_OUT_MAX_CM == obs_out_cm(100u, 1u)) &&
               (1u == obs_past_cm(0u, 40u, 1u)),
               "bypass legs: out, past and back lengths, limits");

    // Pivot angle: never below OBS_PIVOT_MIN_STEP (17 cm sideways at
    // 30 cm would allow 30 deg); 33 cm at 25 cm would need 60 deg, capped
    // at OBS_PIVOT_MAX_STEP.  The wheel arc for 45 deg is 0.786 rad x half
    // the track.
    TEST_CHECK("S5-U09", (OBS_PIVOT_MIN_STEP == obs_pivot_step(30u, 17u)) &&
               (OBS_PIVOT_MAX_STEP == obs_pivot_step(25u, 33u)) &&
               (((786u * OBS_HALF_TRACK_MM) + 500u) / 1000u ==
                obs_pivot_arc_mm(3u)),
               "pivot angle clears the car body, encoder arc");

    // Line search: targets +15, -30, +45 ... from the start heading; a
    // 165 deg sweep (+75 -> -90) is 2.88 rad x the half track.
    TEST_CHECK("S5-U10", (0 == obs_search_target_deg(0u, 15u, 15u)) &&
               (15 == obs_search_target_deg(1u, 15u, 15u)) &&
               (-30 == obs_search_target_deg(2u, 15u, 15u)) &&
               (45 == obs_search_target_deg(3u, 15u, 15u)) &&
               (-90 == obs_search_target_deg(6u, 15u, 15u)) &&
               ((((165u * 17453u * OBS_HALF_TRACK_MM) + 500000u) /
                 1000000u) == obs_deg_arc_mm(165u)) &&
               (obs_pivot_arc_mm(3u) == obs_deg_arc_mm(45u)),
               "line search zig-zag targets and pivot arc");

    // Bypass plan through Navigation's packer: 2 + 3 x 9 = 29 bits,
    // values kept.
    plan = nav_plan_pack(NAV_SIDE_RIGHT, 40u, 29u, 64u);
    TEST_CHECK("S5-U06", (plan <= 0x1FFFFFFFu) &&
               (NAV_SIDE_RIGHT == nav_plan_side(plan)) &&
               (40u == nav_plan_out_cm(plan)) &&
               (29u == nav_plan_past_cm(plan)) &&
               (64u == nav_plan_back_cm(plan)),
               "bypass plan fits 29 bits and unpacks unchanged");

    // Hardware: the sensor answers (needs it wired and powered).
    reading = ultrasonic_read();
    TEST_CHECK("S5-U07", !ultrasonic_is_silent(reading),
               "ultrasonic responds (not silent)");

    TEST_SUMMARY("subsystem5");
}

/*!
 * @brief  Pack a test array into a 9-bit-per-slot profile.
 */
static uint64_t
make_profile(uint16_t const * p_cm, uint16_t count)
{
    uint64_t profile = 0u;
    uint16_t slot;

    for (slot = 0u; slot < count; slot++)
    {
        profile = obs_prof_set(profile, slot, p_cm[slot]);
    }

    return (profile);
}

#endif /* APP_TEST */

/*** end of file ***/
