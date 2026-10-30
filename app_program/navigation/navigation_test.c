/** @file navigation_test.c
 *
 * @brief On-target tests for Navigation.  Built only with make TEST=1.
 *        Record results in testing/navigation/TEST_LOG.md.
 *
 * NAV-M01 is a scripted bench test.  Until the other subsystems exist, its
 * task plays their parts: Rifa's remote commands, Wing's barcodes,
 * junctions, line state and StopTurns, Ruien's obstacle, bypass plan and
 * StopTurns, and Andrew's ESTOP.  After each input it checks the vehicle
 * state Navigation reaches and prints PASS or FAIL.  Run it with the car on
 * a stand, USB power only, and S2_MANUAL_TEST set to S2_MANUAL_NONE, so no
 * subsystem 2 test drives the motors at the same time.
 *
 * NOTE: Each tm_printf() format is cast to (UB *).  tm_printf() takes UB *
 * and the formats are ASCII string literals, so only the signedness of the
 * character type changes.
 */

#include <stdint.h>
#include <stdbool.h>

#include "navigation.h"

#if APP_TEST

#include "../test_report.h"
#include "../subsystem2/motion.h"

#define NAV_MANUAL_M01      (1)     // 1: run NAV-M01 after the auto tests.

#define M01_PRIORITY        (11)    // Below task_navigation (10).
#define M01_CORE            (TP_PRC1)
#define M01_LEAD_IN_MSEC    (10000u)
#define M01_POLL_MSEC       (10u)
#define M01_STATE_MSEC      (500u)  // Time allowed for a state change.
#define M01_PIVOT_WAIT_MSEC (5000u) // Time allowed for a pivot to start.
#define M01_TURN_MSEC       (1000u) // Pivot time before StopTurn.
#define M01_BYPASS_CM       (30u)

static uint32_t g_m01_pass = 0u;
static uint32_t g_m01_fail = 0u;

static void m01_task(INT start_code, void * p_ext_info);
static void m01_check(char const * p_what, bool b_ok);
static bool m01_wait_state(uint32_t state, uint32_t timeout_msec);
static bool m01_wait_activity(uint32_t activity, uint32_t timeout_msec);
static void m01_expect_state(char const * p_what, uint32_t state);
static void m01_stop_turn_after(char const * p_what, uint32_t turn_msec);


/*!
 * @brief Automatic Navigation tests, then start NAV-M01.  Called from
 *        usermain() in test builds.
 */
void
navigation_test (void)
{
    TEST_INIT();

    // NAV-U01: every input refuses out-of-range values, so a bad caller
    // cannot start anything.
    //
    TEST_CHECK("NAV-U01",
               (E_PAR == nav_post_barcode(4u)) &&
               (E_PAR == nav_set_line_state(3u)) &&
               (E_PAR == nav_post_bypass_plan(2u, 30u)) &&
               (E_PAR == nav_post_bypass_plan(NAV_SIDE_LEFT, 0u)) &&
               (E_PAR == nav_post_bypass_plan(NAV_SIDE_LEFT, 512u)) &&
               (E_PAR == nav_raise_fault(0u)) &&
               (E_PAR == nav_raise_fault(1u << 8)) &&
               (E_PAR == nav_post_remote(0u)) &&
               (E_PAR == nav_post_remote(4u)),
               "Navigation inputs reject out-of-range values");

    // NAV-U02: after boot Navigation waits, with no faults.
    //
    TEST_CHECK("NAV-U02",
               (NAV_STATE_IDLE == nav_get_state()) &&
               (0u == nav_get_faults()) &&
               (NAV_REASON_NONE == nav_get_reason()),
               "Navigation starts IDLE with no faults");

    // NAV-U03: bypass plan golden vectors: min, max, mid, and a distance
    // that must clamp to 511.
    //
    TEST_CHECK("NAV-U03",
               (0x004u == nav_plan_pack(NAV_SIDE_LEFT, 1u)) &&
               (0x7FDu == nav_plan_pack(NAV_SIDE_RIGHT, 511u)) &&
               (0x401u == nav_plan_pack(NAV_SIDE_RIGHT, 256u)) &&
               (0x7FCu == nav_plan_pack(NAV_SIDE_LEFT, 600u)) &&
               (NAV_SIDE_RIGHT == nav_plan_side(0x401u)) &&
               (256u == nav_plan_distance_cm(0x401u)),
               "bypass plan packs and unpacks");

    TEST_SUMMARY("navigation");

#if NAV_MANUAL_M01
    {
        // Cast: the kernel stores every task entry as FP.  TA_HLNG tasks
        // are called with (INT, void *), which is m01_task()'s signature.
        //
        ID const task_id = app_start_task((FP) m01_task, M01_PRIORITY,
                                          M01_CORE, "nav_test_m01_task");

        CHECK(task_id, "nav_test_m01_task");
    }
#endif
}

/*!
 * @brief NAV-M01 script: one mission's worth of inputs.
 *
 * @param[in] start_code  Unused.
 * @param[in] p_ext_info  Unused.
 */
static void
m01_task (INT start_code, void * p_ext_info)
{
    (void) start_code;
    (void) p_ext_info;

    tm_printf((UB *) "[NAV-M01] mission script starts in %u s.  Car on a "
              "stand, USB power only.\n", (M01_LEAD_IN_MSEC / 1000u));
    (void) tk_dly_tsk(M01_LEAD_IN_MSEC);
    (void) nav_set_line_state(NAV_LINE_ON);

    // 1. Start.
    //
    (void) nav_post_remote(NAV_REMOTE_START);
    m01_expect_state("START -> LINE_FOLLOW", NAV_STATE_LINE_FOLLOW);
    (void) tk_dly_tsk(2000u);

    // 2. Barcode B, junction: brake, pivot right until StopTurn.
    //
    (void) nav_post_barcode(NAV_BARCODE_RIGHT);
    m01_expect_state("barcode B -> JUNCTION (slowing)", NAV_STATE_JUNCTION);
    (void) tk_dly_tsk(1000u);
    (void) nav_post_junction();
    m01_stop_turn_after("B: pivot right", M01_TURN_MSEC);
    m01_expect_state("B: back to LINE_FOLLOW", NAV_STATE_LINE_FOLLOW);
    (void) tk_dly_tsk(2000u);

    // 3. Barcode D, junction: U-turn, two pivots.
    //
    (void) nav_post_barcode(NAV_BARCODE_UTURN);
    m01_expect_state("barcode D -> JUNCTION", NAV_STATE_JUNCTION);
    (void) tk_dly_tsk(500u);
    (void) nav_post_junction();
    m01_stop_turn_after("D: U-turn pivot 1", M01_TURN_MSEC);
    m01_stop_turn_after("D: U-turn pivot 2", M01_TURN_MSEC);
    m01_expect_state("D: back to LINE_FOLLOW", NAV_STATE_LINE_FOLLOW);
    (void) tk_dly_tsk(2000u);

    // 4. Barcode C, junction: drive on without stopping.
    //
    (void) nav_post_barcode(NAV_BARCODE_STRAIGHT);
    m01_expect_state("barcode C -> JUNCTION", NAV_STATE_JUNCTION);
    (void) tk_dly_tsk(500u);
    (void) nav_post_junction();
    m01_expect_state("C: LINE_FOLLOW", NAV_STATE_LINE_FOLLOW);
    m01_check("C: still driving, no stop",
              (MOTION_ACTIVITY_STRAIGHT == motion_get_activity()));
    (void) tk_dly_tsk(2000u);

    // 5. Barcode A with no junction: turns after NAV_APPROACH_MAX_MM.
    //
    (void) nav_post_barcode(NAV_BARCODE_LEFT);
    m01_expect_state("barcode A -> JUNCTION", NAV_STATE_JUNCTION);
    m01_stop_turn_after("A: pivot left without a junction", M01_TURN_MSEC);
    m01_expect_state("A: back to LINE_FOLLOW", NAV_STATE_LINE_FOLLOW);
    (void) tk_dly_tsk(2000u);

    // 6. Obstacle: brake, plan, bypass, reacquire, align.
    //
    (void) nav_post_obstacle();
    m01_expect_state("obstacle -> OBSTACLE", NAV_STATE_OBSTACLE);
    m01_check("obstacle: braked",
              m01_wait_activity(MOTION_ACTIVITY_IDLE, M01_STATE_MSEC));
    (void) nav_set_line_state(NAV_LINE_LOST);
    (void) tk_dly_tsk(1000u);
    (void) nav_post_bypass_plan(NAV_SIDE_LEFT, M01_BYPASS_CM);
    m01_expect_state("plan -> BYPASS", NAV_STATE_BYPASS);
    m01_stop_turn_after("bypass: pivot out", 600u);
    m01_stop_turn_after("bypass: pivot back", 600u);
    m01_check("bypass: creeping to the line",
              m01_wait_activity(MOTION_ACTIVITY_STRAIGHT,
                                M01_PIVOT_WAIT_MSEC));
    (void) tk_dly_tsk(1000u);
    (void) nav_set_line_state(NAV_LINE_ON);
    m01_stop_turn_after("bypass: align on the line", 500u);
    m01_expect_state("bypass: back to LINE_FOLLOW", NAV_STATE_LINE_FOLLOW);
    (void) tk_dly_tsk(2000u);

    // 7. Remote STOP, then START again.
    //
    (void) nav_post_remote(NAV_REMOTE_STOP);
    m01_expect_state("STOP -> STOPPED", NAV_STATE_STOPPED);
    m01_check("STOP: braked",
              m01_wait_activity(MOTION_ACTIVITY_IDLE, M01_STATE_MSEC));
    (void) tk_dly_tsk(1000u);
    (void) nav_post_remote(NAV_REMOTE_START);
    m01_expect_state("START again -> LINE_FOLLOW", NAV_STATE_LINE_FOLLOW);
    (void) tk_dly_tsk(1000u);

    // 8. ESTOP: brake and hold until CLEAR; START is refused meanwhile.
    //
    (void) nav_raise_fault(NAV_FLT_ESTOP);
    m01_expect_state("ESTOP -> FAULT", NAV_STATE_FAULT);
    m01_check("ESTOP: braked, bit and reason recorded",
              m01_wait_activity(MOTION_ACTIVITY_IDLE, M01_STATE_MSEC) &&
              (0u != (nav_get_faults() & NAV_FLT_ESTOP)) &&
              (NAV_REASON_FAULT_BIT == nav_get_reason()));
    (void) nav_post_remote(NAV_REMOTE_START);
    (void) tk_dly_tsk(M01_STATE_MSEC);
    m01_check("FAULT: START refused", (NAV_STATE_FAULT == nav_get_state()));
    (void) nav_post_remote(NAV_REMOTE_CLEAR);
    m01_expect_state("CLEAR -> IDLE", NAV_STATE_IDLE);

    tm_printf((UB *) "[NAV-M01] done: %lu passed, %lu failed.  Longest "
              "task_motion step %lu usec.\n", g_m01_pass, g_m01_fail,
              motion_get_max_loop_usec());
    tk_exd_tsk();
}

/*!
 * @brief Print one check's result and count it.
 */
static void
m01_check (char const * p_what, bool b_ok)
{
    if (b_ok)
    {
        g_m01_pass++;
    }
    else
    {
        g_m01_fail++;
    }

    tm_printf((UB *) "[NAV-M01] %s  %s\n", (b_ok ? "PASS" : "FAIL"),
              p_what);
}

/*!
 * @brief Wait until Navigation shows a vehicle state.
 *
 * @return True if it did within the timeout.
 */
static bool
m01_wait_state (uint32_t state, uint32_t timeout_msec)
{
    uint32_t waited = 0u;

    while ((state != nav_get_state()) && (waited < timeout_msec))
    {
        (void) tk_dly_tsk(M01_POLL_MSEC);
        waited += M01_POLL_MSEC;
    }

    return (state == nav_get_state());
}

/*!
 * @brief Wait until Motion shows an activity.
 *
 * @return True if it did within the timeout.
 */
static bool
m01_wait_activity (uint32_t activity, uint32_t timeout_msec)
{
    uint32_t waited = 0u;

    while ((activity != motion_get_activity()) && (waited < timeout_msec))
    {
        (void) tk_dly_tsk(M01_POLL_MSEC);
        waited += M01_POLL_MSEC;
    }

    return (activity == motion_get_activity());
}

/*!
 * @brief Check that Navigation reaches a state within M01_STATE_MSEC.
 */
static void
m01_expect_state (char const * p_what, uint32_t state)
{
    m01_check(p_what, m01_wait_state(state, M01_STATE_MSEC));
}

/*!
 * @brief Play a StopTurn sender: wait for a pivot to start, let it turn,
 *        then send StopTurn.  The check passes if the pivot started; the
 *        state checks that follow show that it ended.
 *
 * @param[in] p_what     Name of the check.
 * @param[in] turn_msec  How long to let the pivot run.
 */
static void
m01_stop_turn_after (char const * p_what, uint32_t turn_msec)
{
    bool const b_ok = m01_wait_activity(MOTION_ACTIVITY_PIVOT,
                                        M01_PIVOT_WAIT_MSEC);

    if (b_ok)
    {
        (void) tk_dly_tsk(turn_msec);
        (void) motion_stop_turn();

        // Let Motion end this pivot (at once) before the next wait, so
        // that wait cannot see this pivot still running.
        //
        (void) tk_dly_tsk(M01_POLL_MSEC * 3u);
    }

    m01_check(p_what, b_ok);
}

#endif /* APP_TEST */

/*** end of file ***/

