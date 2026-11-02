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
    TEST_CHECK("S5-U03", (30u == obs_median3(30u, 175u, 31u)) &&
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

    // Edge and sideways distance: edge at 45 deg, 25 cm ahead -> 40 cm.
    TEST_CHECK("S5-U05",
               (3u == obs_find_edge(make_profile(fine_left,
                                                 OBS_FINE_COUNT), CLEAR_CM))
               && (40u == obs_lateral_cm(25u, 3u)),
               "fine scan edge step and lateral distance");

    // Bypass plan through Navigation's packer: 2 bytes, values kept.
    plan = nav_plan_pack(NAV_SIDE_RIGHT, 40u);
    TEST_CHECK("S5-U06", (plan <= 0xFFFFu) &&
               (NAV_SIDE_RIGHT == nav_plan_side(plan)) &&
               (40u == nav_plan_distance_cm(plan)),
               "bypass plan fits 16 bits and unpacks unchanged");

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
