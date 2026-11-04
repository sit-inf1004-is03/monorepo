/** @file obstacle_scan.c
 *
 * @brief Obstacle Scan task (Subsystem 5): Week 6 statechart (Figure 4),
 *        joined to Navigation (IF-11, IF-12, IF-14) and Motion (IF-09).
 *
 *   WATCH ---2 pings < 15 cm---> nav_post_obstacle()
 *     |                               |
 *     |                       WAIT_BRAKE (Navigation -> OBSTACLE)
 *     |                               v
 *     |   retry (< 3)          COARSE  5 angles, median of 3 pings
 *     |      +---------------->   |  side found
 *     |      |                    v
 *     |      +-- edge missing  FINE    15 deg steps: edge -> pivot angle
 *     |                           |    and drive-out length ->
 *     |                           v    nav_post_bypass_plan()
 *     |                     BYPASS     follows Navigation's three legs
 *     |                           |    and StopTurns each pivot (leg):
 *     |                           |    OUT      pivot out, the pivot angle
 *     |                           |    PARALLEL pivot back, the same angle;
 *     |                           |             then the servo looks
 *     |                           |             sideways at the obstacle
 *     |                           |             and, once it has gone by
 *     |                           |             + sensor-to-axle + margin,
 *     |                           |             nav_post_bypass_passed()
 *     |                           |    IN       pivot in, OBS_REJOIN_STEP
 *     |                           |    ALIGN    slow pivot the original
 *     |                           |             way until a line sensor
 *     |                           |             sees white then black (the
 *     |                           |             line's own heading, straight
 *     |                           |             or curved), at most
 *     |                           |             ALIGN_MAX_DEG
 *     |                           |    SEARCH   Navigation waits after the
 *     |                           |             align (or when the back limit
 *     |                           |             ran out): line on -> FOUND;
 *     |                           |             else zig-zag pivots to +15,
 *     |                           |             -30, +45 ... deg (encoders),
 *     |                           |             StopTurn at once on the line;
 *     |                           v             past SEARCH_MAX_DEG -> FAILED
 *     +<------- Navigation leaves BYPASS
 *
 *   Pivot angles are measured with the wheel encoders (IF-06): each wheel
 *   travels angle x OBS_HALF_TRACK_MM.
 *
 *   3 failed scans: nav_raise_fault(NAV_FLT_SCAN_ERROR), ERROR state.
 *   ~1 s of a SILENT sensor: nav_raise_fault(NAV_FLT_NO_ECHO).
 *   "No echo" alone (nothing in range) is normal and is not a fault.
 *
 * Minimum-bit representation: the task state is ONE 16-bit word with
 * explicit masks (rule 5.5.b: no bit-fields):
 *
 *   bits 15..14  chosen side (OBS_SIDE_)    2 bits
 *   bits 13..11  WATCH: print divider       3 bits  (0..4)
 *                BYPASS past leg: side watch (PASS_)
 *                BYPASS align/search: search step (0..SEARCH_STEPS,
 *                SEARCH_DONE)               (none of these coexist)
 *   bits 10..9   failed scans               2 bits  (0..3)
 *   bits  8..5   consecutive silent pings   4 bits  (0..10)
 *   bits  4..3   WATCH: "near" pings        2 bits  (0..2)
 *                BYPASS: leg (OUT..ALIGN)   (the two never coexist)
 *   bits  2..0   state                      3 bits  (6 states)
 *
 * Set S5_STANDALONE to 1 for a bench test without Navigation running a
 * mission: the task then scans on its own and only prints its decisions.
 *
 * Coding standard: BARR-C:2018.
 */

#include <stdbool.h>
#include <stdint.h>

#include "obstacle_scan.h"
#include "obstacle_logic.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"
#include "../navigation/navigation.h"
#include "../subsystem2/motion.h"
#include "../subsystem2/drv_encoder.h"
#include "../subsystem3/line_barcode.h"     // line_get_state(), read only.

// Event lines also go to the dashboard's log (robocar/<id>/log) when the
// team's comms module is in the build; otherwise they are only printed.
//
#if defined(__has_include)
#if __has_include("../comms/comms.h")
#include "../comms/comms.h"
#define S5_HAS_COMMS            (1)
#endif
#endif
#ifndef S5_HAS_COMMS
#define S5_HAS_COMMS            (0)
#endif

// 1 = bench test on its own (prints only), 0 = full mission with
// Navigation.
//
#define S5_STANDALONE           (0)

// ---- Tunable settings (Week 6 design, car bench tests) ----
#define WATCH_PERIOD_MS         (100)   // Ping rate while driving.
#define POLL_MS                 (65)    // Wait loops; > 60 ms ping cooldown.
#define PING_GAP_MS             (65)
#define TRIGGER_CM              (15u)   // Obstacle closer than this...
#define CONFIRM_PINGS           (2u)    // ...on this many pings in a row.
#define CLEAR_CM                (50u)   // A direction is open beyond this.
#define SILENT_LIMIT            (10u)   // ~1 s of silence -> NO_ECHO fault.
#define SCAN_RETRY_LIMIT        (3u)
#define PRINT_EVERY             (5u)    // WATCH: print every 5th ping.
#define BRAKE_WAIT_POLLS        (31u)   // ~2 s for Navigation to brake.
#define PIVOT_POLL_MS           (10)    // Encoder check rate in a pivot.
#define PIVOT_MAX_POLLS         (250u)  // 2.5 s bound on one pivot.

// ---- Line search after the bypass (tune on the mat) ----
// Targets, from the heading the search starts at: +START, -(START +
// STEP), +(START + 2 STEP), ... up to +/-SEARCH_MAX_DEG.  "+" is towards
// the original line (the turn-in direction) when SEARCH_FIRST_TOWARD_LINE
// is 1.  The pivot speed is Navigation's NAV_SEARCH_PIVOT_MM_S (60).
//
#define SEARCH_START_DEG        (15u)
#define SEARCH_STEP_DEG         (15u)
#define SEARCH_MAX_DEG          (90u)
#define SEARCH_FIRST_TOWARD_LINE (1)
#define SEARCH_STEPS            ((((SEARCH_MAX_DEG - SEARCH_START_DEG) /  \
                                   SEARCH_STEP_DEG)) + 1u)
#define SEARCH_DONE             (7u)    // Verdict posted (3-bit field).
#define SEARCH_MAX_POLLS        (600u)  // 6 s bound on one slow sweep.


// Align pivot: stops on the line (white, then black) or at this angle.
// A straight line is met at the rejoin angle (30 deg); a line curving
// away later, one curving back earlier.
//
#define ALIGN_MAX_DEG           (75u)

// Parallel pivot: turn back this much more than the out angle, so the
// past leg points slightly towards the line (0 = parallel).
//
#define PARALLEL_IN_DEG         (10u)

#if (SEARCH_STEPS >= SEARCH_DONE)
#error "Line search: too many steps for the 3-bit step field."
#endif
#define SIDE_LOOK_STEP          (6u)    // Past leg: servo 6 x 15 = 90 deg
                                        // off centre, at the obstacle.
#define SIDE_NEAR_CM            (30u)   // Sideways closer = the obstacle.
#define BUFFER_CM               (OBS_SENSOR_TO_AXLE_CM + OBS_PAST_MARGIN_CM)

// ---- Scan geometry ----
#define CENTRE_DEG              (90u)
#define COARSE_START_DEG        (30u)
#define COARSE_STEP_DEG         (30u)
#define LINE_LEN                (112u)

// ---- States (3 bits) ----
#define ST_WATCH                (0u)
#define ST_WAIT_BRAKE           (1u)
#define ST_COARSE               (2u)
#define ST_FINE                 (3u)
#define ST_BYPASS               (4u)
#define ST_ERROR                (5u)

// ---- Bypass legs (2 bits): the pivot Obstacle Scan stops next ----
#define LEG_OUT                 (0u)    // Pivot out: pivot angle.
#define LEG_PARALLEL            (1u)    // After the out leg: pivot angle.
#define LEG_IN                  (2u)    // After the past leg: rejoin angle.
#define LEG_ALIGN               (3u)    // On the line: rejoin angle.

// ---- Side watch in the past leg (3 bits) ----
#define PASS_IDLE               (0u)    // Straight not started yet.
#define PASS_LOOK               (1u)    // Waiting to see the obstacle.
#define PASS_SEEN               (2u)    // Obstacle beside the sensor.
#define PASS_CLEAR_1            (3u)    // One clear ping after it.
#define PASS_BUFFER             (4u)    // Gone: drive axle + margin more.
#define PASS_POSTED             (5u)    // Navigation told; wait.

// ---- Packed state word fields ----
#define F_STATE_SHIFT           (0u)
#define F_STATE_MASK            (0x7u)
#define F_NEAR_SHIFT            (3u)
#define F_NEAR_MASK             (0x3u)
#define F_LEG_SHIFT             (3u)    // Same bits as F_NEAR: BYPASS only.
#define F_LEG_MASK              (0x3u)
#define F_SILENT_SHIFT          (5u)
#define F_SILENT_MASK           (0xFu)
#define F_RETRY_SHIFT           (9u)
#define F_RETRY_MASK            (0x3u)
#define F_PRINT_SHIFT           (11u)
#define F_PRINT_MASK            (0x7u)
#define F_PASS_SHIFT            (11u)   // Same bits as F_PRINT: BYPASS only.
#define F_PASS_MASK             (0x7u)
#define F_SEARCH_SHIFT          (11u)   // Same bits: align / search only.
#define F_SEARCH_MASK           (0x7u)

// ---- track_pivot() results ----
#define TRACK_RUNNING           (0u)
#define TRACK_ANGLE             (1u)    // Stopped at the target angle.
#define TRACK_LINE              (2u)    // Stopped: a line sensor saw black.
#define F_SIDE_SHIFT            (14u)
#define F_SIDE_MASK             (0x3u)

// Event line being built (written only by obstacle_scan_task).
//
static UB g_log[LINE_LEN];

// Task context: on the task's stack, passed by pointer.
//
typedef struct
{
    uint16_t state_word;        // Packed: see the file header.
    uint64_t coarse;            // 5 x 9-bit coarse profile.
    uint32_t start_sum;         // Both encoder counts at pivot start.
    uint16_t pivot_pulses;      // Pivot-out angle, in pulses per wheel;
                                // 0 once the bypass's pivots are done.
    uint16_t past_pulses;       // Estimated past leg (obs_past_cm()),
                                // used if the side scan never sees it.
    uint16_t polls;             // Wait-loop counter.
    bool     b_seen;            // Phase flag: straight seen.
    bool     b_turning;         // Phase flag: tracked pivot started.
    bool     b_requested;       // Search: pivot asked of Navigation.
    bool     b_armed;           // Line stop armed: white seen in pivot.
} scan_ctx_t;

static void     on_watch(scan_ctx_t * p_ctx);
static bool     junction_search(scan_ctx_t * p_ctx);
static void     on_wait_brake(scan_ctx_t * p_ctx);
static void     on_coarse(scan_ctx_t * p_ctx);
static void     on_fine(scan_ctx_t * p_ctx);
static void     on_bypass(scan_ctx_t * p_ctx);
static void     watch_past(scan_ctx_t * p_ctx);
static void     post_passed(uint16_t pass);
static void     on_error(scan_ctx_t * p_ctx);
static void     enter(scan_ctx_t * p_ctx, uint16_t state);
static void     scan_failed(scan_ctx_t * p_ctx);
static void     update_silence(scan_ctx_t * p_ctx,
                               ultrasonic_reading_t reading);
static bool     is_nav_driving(void);
static bool     is_bypass_over(void);
static uint16_t track_pivot(scan_ctx_t * p_ctx, uint32_t target_pulses,
                            bool b_stop_on_line);
static void     on_search(scan_ctx_t * p_ctx);
static uint32_t encoder_sum(void);
static uint32_t rejoin_pulses(void);
static uint32_t align_pulses(void);
static void     log_text(UB const * p_text);
static uint16_t ping_cm(void);
static uint16_t ping_median_cm(void);
static uint64_t coarse_scan(void);
static uint64_t fine_scan(uint16_t side);
static uint8_t  coarse_angle(uint16_t idx);
static uint8_t  fine_angle(uint16_t side, uint16_t step);
static uint16_t fld_get(uint16_t word, uint16_t shift, uint16_t mask);
static uint16_t fld_set(uint16_t word, uint16_t shift, uint16_t mask,
                        uint16_t value);

void
obstacle_scan_task(INT stacd, void * p_exinf)
{
    scan_ctx_t ctx;

    (void)stacd;
    (void)p_exinf;

    ctx.state_word  = ST_WATCH;
    ctx.coarse      = 0u;
    ctx.start_sum    = 0u;
    ctx.pivot_pulses = 0u;
    ctx.past_pulses  = 0u;
    ctx.polls        = 0u;
    ctx.b_seen       = false;
    ctx.b_turning    = false;
    ctx.b_requested  = false;
    ctx.b_armed      = false;

    servo_set_angle_deg((uint8_t)CENTRE_DEG);
    (void)tm_sprintf(g_log,
                     (UB *)"[S5] obstacle scan running (standalone=%d)",
                     S5_STANDALONE);
    log_text(g_log);

    for (;;)
    {
        switch (fld_get(ctx.state_word, F_STATE_SHIFT, F_STATE_MASK))
        {
        case ST_WATCH:
            if (!junction_search(&ctx))
            {
                on_watch(&ctx);
            }
        break;

        case ST_WAIT_BRAKE:
            on_wait_brake(&ctx);
        break;

        case ST_COARSE:
            on_coarse(&ctx);
        break;

        case ST_FINE:
            on_fine(&ctx);
        break;

        case ST_BYPASS:
            on_bypass(&ctx);
        break;

        case ST_ERROR:
            on_error(&ctx);
        break;

        default:
            ctx.state_word = ST_WATCH;  // 6, 7: not used.
        break;
        }
    }
}

/*!
 * @brief  WATCH: ping ahead every 100 ms; report an obstacle after
 *         CONFIRM_PINGS near pings while Navigation is driving.
 */
static void
on_watch(scan_ctx_t * p_ctx)
{
    ultrasonic_reading_t const reading = ultrasonic_read();
    uint16_t const dist = ultrasonic_is_no_echo(reading) ?
                          (uint16_t)OBS_FAR_CM : ultrasonic_dist_cm(reading);
    uint16_t       near = fld_get(p_ctx->state_word, F_NEAR_SHIFT,
                                  F_NEAR_MASK);
    uint16_t       tick = fld_get(p_ctx->state_word, F_PRINT_SHIFT,
                                  F_PRINT_MASK);

    update_silence(p_ctx, reading);


    if (dist < TRIGGER_CM)
    {
        if (near < CONFIRM_PINGS)       // Check before increment: 2 bits.
        {
            near++;
        }
    }
    else
    {
        near = 0u;
    }

    tick++;
    if (tick >= PRINT_EVERY)
    {
        tick = 0u;
        (void)tm_printf((UB *)"[S5] WATCH: front %u cm%s\n", (UINT)dist,
                        (OBS_FAR_CM == dist) ? " (no echo)" : "");
    }

    p_ctx->state_word = fld_set(p_ctx->state_word, F_PRINT_SHIFT,
                                F_PRINT_MASK, tick);
    p_ctx->state_word = fld_set(p_ctx->state_word, F_NEAR_SHIFT,
                                F_NEAR_MASK, near);

    if ((CONFIRM_PINGS == near) && is_nav_driving())
    {
        (void)tm_sprintf(g_log, (UB *)"[S5] OBSTACLE at %u cm", (UINT)dist);
        log_text(g_log);
        p_ctx->state_word = fld_set(p_ctx->state_word, F_NEAR_SHIFT,
                                    F_NEAR_MASK, 0u);
#if (1 == S5_STANDALONE)
        enter(p_ctx, ST_COARSE);
#else
        (void)nav_post_obstacle();
        enter(p_ctx, ST_WAIT_BRAKE);
#endif
    }
    else
    {
        (void)tk_dly_tsk(WATCH_PERIOD_MS);
    }
}

/*!
 * @brief  LOCAL: a T turn ended off the line (Navigation waits in its
 *         search with nav_get_junction_search() set): start the bypass
 *         line search (BYPASS, leg ALIGN).  "+" (the first wiggle) is back
 *         the way the turn came, as after a bypass to that side.
 * @return true if the search was started.
 */
static bool
junction_search(scan_ctx_t * p_ctx)
{
    bool b_started = false;

#if (0 == S5_STANDALONE)
    uint32_t const junc = nav_get_junction_search();

    if ((NAV_JSEARCH_NONE != junc) &&
        (NAV_STATE_BYPASS == nav_get_state()))
    {
        uint16_t const side = (NAV_JSEARCH_LEFT == junc) ?
                              (uint16_t)OBS_SIDE_LEFT :
                              (uint16_t)OBS_SIDE_RIGHT;

        (void)tm_sprintf(g_log, (UB *)"[S5] T turn off the line: search,"
                         " first %s", (OBS_SIDE_LEFT == side) ? "right"
                                                              : "left");
        log_text(g_log);
        p_ctx->state_word = ST_WATCH;   // Clears the near / print bits.
        enter(p_ctx, ST_BYPASS);
        p_ctx->state_word = fld_set(p_ctx->state_word, F_LEG_SHIFT,
                                    F_LEG_MASK, LEG_ALIGN);
        p_ctx->state_word = fld_set(p_ctx->state_word, F_SIDE_SHIFT,
                                    F_SIDE_MASK, side);
        b_started = true;
    }
#else
    (void)p_ctx;
#endif

    return (b_started);
}

/*!
 * @brief  WAIT_BRAKE: scan only once Navigation has braked
 *         (NAV_STATE_OBSTACLE); give up after ~2 s.
 */
static void
on_wait_brake(scan_ctx_t * p_ctx)
{
    if (NAV_STATE_OBSTACLE == nav_get_state())
    {
        enter(p_ctx, ST_COARSE);
    }
    else if (p_ctx->polls >= BRAKE_WAIT_POLLS)
    {
        (void)tm_sprintf(g_log,
                         (UB *)"[S5] Navigation did not brake; watching");
        log_text(g_log);
        enter(p_ctx, ST_WATCH);
    }
    else
    {
        p_ctx->polls++;
        (void)tk_dly_tsk(POLL_MS);
    }
}

/*!
 * @brief  COARSE: scan 5 angles and pick a side, or count a failure.
 */
static void
on_coarse(scan_ctx_t * p_ctx)
{
    uint16_t side;

    p_ctx->coarse = coarse_scan();
    side          = obs_pick_side(p_ctx->coarse, CLEAR_CM);

    if (OBS_SIDE_NONE == side)
    {
        (void)tm_sprintf(g_log, (UB *)"[S5] COARSE: no side clear (> %u cm)",
                         (UINT)CLEAR_CM);
        log_text(g_log);
        scan_failed(p_ctx);
    }
    else
    {
        (void)tm_sprintf(g_log, (UB *)"[S5] COARSE: %s side open",
                         (OBS_SIDE_LEFT == side) ? "LEFT" : "RIGHT");
        log_text(g_log);
        p_ctx->state_word = fld_set(p_ctx->state_word, F_SIDE_SHIFT,
                                    F_SIDE_MASK, side);
        enter(p_ctx, ST_FINE);
    }
}

/*!
 * @brief  FINE: find the obstacle's edge on the chosen side, then send
 *         the three-leg bypass plan (side, out, past and back lengths)
 *         to Navigation.
 */
static void
on_fine(scan_ctx_t * p_ctx)
{
    uint16_t const side = fld_get(p_ctx->state_word, F_SIDE_SHIFT,
                                  F_SIDE_MASK);
    uint16_t const edge = obs_find_edge(fine_scan(side), CLEAR_CM);
    uint16_t       front_cm;
    uint16_t       lateral_cm;
    uint16_t       pivot_step;
    uint16_t       out_cm;
    uint16_t       past_cm;
    uint16_t       back_cm;

    if (0u == edge)
    {
        (void)tm_sprintf(g_log, (UB *)"[S5] FINE: obstacle edge not found");
        log_text(g_log);
        scan_failed(p_ctx);
    }
    else
    {
        front_cm = obs_prof_get(p_ctx->coarse, OBS_COARSE_CENTRE);

        if (front_cm >= OBS_FAR_CM)
        {
            front_cm = TRIGGER_CM;      // Centre missed: assume 15 cm.
        }

        lateral_cm = obs_lateral_cm(front_cm, edge);
        pivot_step = obs_pivot_step(front_cm, lateral_cm);
        out_cm     = obs_out_cm(lateral_cm, pivot_step);
        past_cm    = obs_past_cm(front_cm, out_cm, pivot_step);
        back_cm    = obs_back_cm(out_cm, pivot_step);
        (void)tm_sprintf(g_log, (UB *)"[S5] PLAN: %s, turn %u deg, out %u"
                         " past %u back <= %u cm (front %u, edge %u deg,"
                         " side %u cm)",
                         (OBS_SIDE_LEFT == side) ? "LEFT" : "RIGHT",
                         (UINT)(pivot_step * OBS_FINE_STEP_DEG),
                         (UINT)out_cm, (UINT)past_cm, (UINT)back_cm,
                         (UINT)front_cm, (UINT)(edge * OBS_FINE_STEP_DEG),
                         (UINT)lateral_cm);
        log_text(g_log);
        enter(p_ctx, ST_BYPASS);
        p_ctx->state_word = fld_set(p_ctx->state_word, F_LEG_SHIFT,
                                    F_LEG_MASK, LEG_OUT);
#if (0 == S5_STANDALONE)
        (void)nav_post_bypass_plan((OBS_SIDE_LEFT == side) ?
                                   NAV_SIDE_LEFT : NAV_SIDE_RIGHT,
                                   out_cm,
                                   (uint32_t)past_cm + OBS_PAST_SCAN_EXTRA_CM,
                                   back_cm);
        p_ctx->pivot_pulses = (uint16_t)drv_encoder_mm_to_pulses(
                                  obs_pivot_arc_mm(pivot_step));
        p_ctx->past_pulses  = (uint16_t)drv_encoder_mm_to_pulses(
                                  (uint32_t)past_cm * 10u);
#endif
    }
}

/*!
 * @brief  BYPASS: Navigation runs the legs; StopTurn each of its four
 *         pivots with the encoders.  Every pivot but the first follows a
 *         straight, so wait for that straight first.  Back to WATCH when
 *         Navigation leaves BYPASS (done, or a fault / LINE_NOT_FOUND).
 *         Standalone: nothing moves; wait until the front is clear.
 */
static void
on_bypass(scan_ctx_t * p_ctx)
{
#if (1 == S5_STANDALONE)
    if (ping_median_cm() > CLEAR_CM)
    {
        (void)tm_sprintf(g_log, (UB *)"[S5] front clear, watching");
        log_text(g_log);
        p_ctx->state_word = ST_WATCH;   // Clears every counter and side.
    }
    else
    {
        (void)tk_dly_tsk(POLL_MS);
    }
#else
    uint16_t const leg = fld_get(p_ctx->state_word, F_LEG_SHIFT,
                                 F_LEG_MASK);
    uint32_t const target =
        (LEG_OUT == leg)      ? (uint32_t)p_ctx->pivot_pulses :
        (LEG_PARALLEL == leg) ? ((uint32_t)p_ctx->pivot_pulses +
                                 drv_encoder_mm_to_pulses(
                                     obs_deg_arc_mm(PARALLEL_IN_DEG))) :
        (LEG_IN == leg)       ? rejoin_pulses() : align_pulses();

    if (is_bypass_over())
    {
        if (0u == p_ctx->pivot_pulses)
        {
            (void)tm_sprintf(g_log, (UB *)"[S5] bypass done, watching");
            log_text(g_log);
        }
        else
        {
            (void)tm_sprintf(g_log, (UB *)"[S5] bypass ended at leg %u,"
                             " watching", (UINT)leg);
            log_text(g_log);
        }

        // Clears every counter, the side, the leg and the side watch
        // (which share the near-ping and print bits).
        servo_set_angle_deg((uint8_t)CENTRE_DEG);
        p_ctx->state_word   = ST_WATCH;
        p_ctx->pivot_pulses = 0u;
    }
    else if ((LEG_IN == leg) && p_ctx->b_seen && !p_ctx->b_turning &&
             (MOTION_ACTIVITY_STRAIGHT == motion_get_activity()))
    {
        watch_past(p_ctx);              // The past leg is running.
    }
    else if ((LEG_ALIGN == leg) &&
             (p_ctx->b_requested || nav_is_search_wait() ||
              (0u != fld_get(p_ctx->state_word, F_SEARCH_SHIFT,
                             F_SEARCH_MASK))))
    {
        on_search(p_ctx);               // After the align / back limit.
    }
    else if ((0u == p_ctx->pivot_pulses) ||
             ((LEG_OUT != leg) && !p_ctx->b_seen))
    {
        // Done, or waiting for the straight before this pivot.
        if (!p_ctx->b_seen)
        {
            p_ctx->b_seen = (MOTION_ACTIVITY_STRAIGHT ==
                             motion_get_activity());
        }

        (void)tk_dly_tsk(PIVOT_POLL_MS);
    }
    else if (TRACK_RUNNING != track_pivot(p_ctx, target,
                                          (LEG_ALIGN == leg)))
    {
        p_ctx->polls     = 0u;
        p_ctx->b_seen    = false;
        p_ctx->b_turning = false;

        if (LEG_ALIGN == leg)
        {
            p_ctx->pivot_pulses = 0u;   // All four pivots done.
        }
        else
        {
            if (LEG_PARALLEL == leg)
            {
                // Look at the obstacle while the past leg starts.
                servo_set_angle_deg(fine_angle(
                    (OBS_SIDE_LEFT == fld_get(p_ctx->state_word,
                                              F_SIDE_SHIFT, F_SIDE_MASK)) ?
                    (uint16_t)OBS_SIDE_RIGHT : (uint16_t)OBS_SIDE_LEFT,
                    SIDE_LOOK_STEP));
            }
            else if (LEG_IN == leg)
            {
                // Side watch over: its bits become the search step (0).
                servo_set_angle_deg((uint8_t)CENTRE_DEG);
                p_ctx->state_word = fld_set(p_ctx->state_word,
                                            F_SEARCH_SHIFT, F_SEARCH_MASK,
                                            0u);
            }
            else
            {
                // LEG_OUT: the servo stays centred.
            }

            p_ctx->state_word = fld_set(p_ctx->state_word, F_LEG_SHIFT,
                                        F_LEG_MASK, (uint16_t)(leg + 1u));
        }
    }
    else
    {
        (void)tk_dly_tsk(PIVOT_POLL_MS);
    }
#endif
}

/*!
 * @brief  Past leg: the servo looks sideways at the obstacle.  Wait to
 *         see it (closer than SIDE_NEAR_CM), then for two clear pings in
 *         a row (its back has gone by the sensor), then drive
 *         sensor-to-axle + OBS_PAST_MARGIN_CM more and tell Navigation to
 *         turn in.  Never seen by the estimated past length (the sensor
 *         was already past it): tell Navigation there.
 */
static void
watch_past(scan_ctx_t * p_ctx)
{
    uint16_t       pass   = fld_get(p_ctx->state_word, F_PASS_SHIFT,
                                    F_PASS_MASK);
    uint32_t const driven = (encoder_sum() - p_ctx->start_sum) / 2u;
    uint16_t       dist;

    switch (pass)
    {
    case PASS_IDLE:
        // The servo swung sideways at the end of the parallel pivot:
        // let it settle so no ping is taken mid-swing.
        (void)tk_dly_tsk(SERVO_SETTLE_MS);
        p_ctx->start_sum = encoder_sum();   // Straight start.
        pass = PASS_LOOK;
    break;

    case PASS_LOOK:
        dist = ping_cm();

        if (dist < SIDE_NEAR_CM)
        {
            (void)tm_sprintf(g_log, (UB *)"[S5] PAST: obstacle beside,"
                             " %u cm", (UINT)dist);
            log_text(g_log);
            pass = PASS_SEEN;
        }
        else if (driven >= p_ctx->past_pulses)
        {
            post_passed(pass);          // Sensor was already past it.
            pass = PASS_POSTED;
        }
        else
        {
            // Not beside it yet.
        }
    break;

    case PASS_SEEN:
    case PASS_CLEAR_1:
        dist = ping_cm();

        if (dist < SIDE_NEAR_CM)
        {
            pass = PASS_SEEN;           // Still beside it.
        }
        else if (PASS_SEEN == pass)
        {
            pass = PASS_CLEAR_1;
        }
        else
        {
            (void)tm_sprintf(g_log, (UB *)"[S5] PAST: obstacle ends after"
                             " %u cm; %u cm more",
                             (UINT)(drv_encoder_pulses_to_mm(driven) / 10u),
                             (UINT)BUFFER_CM);
            log_text(g_log);
            p_ctx->start_sum = encoder_sum();   // Buffer starts here.
            pass = PASS_BUFFER;
        }
    break;

    case PASS_BUFFER:
        if (driven >= drv_encoder_mm_to_pulses(BUFFER_CM * 10u))
        {
            post_passed(pass);
            pass = PASS_POSTED;
        }
    break;

    default:
        // PASS_POSTED: Navigation is braking; nothing to do.
    break;
    }

    p_ctx->state_word = fld_set(p_ctx->state_word, F_PASS_SHIFT,
                                F_PASS_MASK, pass);
    (void)tk_dly_tsk(PING_GAP_MS);
}

/*!
 * @brief  End the past leg: centre the servo (ready for WATCH) and tell
 *         Navigation to turn in.
 * @param  pass  Side-watch step that ended it, for the log.
 */
static void
post_passed(uint16_t pass)
{
    servo_set_angle_deg((uint8_t)CENTRE_DEG);
    (void)nav_post_bypass_passed();
    (void)tm_sprintf(g_log, (UB *)"[S5] PAST: passed (%s), turning in",
                     (PASS_LOOK == pass) ? "estimate" : "measured");
    log_text(g_log);
}

/*!
 * @brief  Follow one pivot with the encoders: start counting when Motion
 *         starts pivoting, StopTurn when each wheel has gone
 *         target_pulses, or (align, search) as soon as a line sensor
 *         sees black after it has seen white in this pivot (so it never
 *         stops on the tape it is still over when the pivot starts).
 *         A pivot that takes too long (PIVOT_MAX_POLLS, or
 *         SEARCH_MAX_POLLS for a slow search sweep) raises SCAN_ERROR.
 * @param  p_ctx           Task context (b_turning, start_sum, polls).
 * @param  target_pulses   Pulses per wheel for the angle.
 * @param  b_stop_on_line  Also stop when Line Following reports LINE_ON
 *                         after a not-ON reading.
 * @return TRACK_RUNNING, or TRACK_ANGLE / TRACK_LINE once StopTurn has
 *         been sent.
 */
static uint16_t
track_pivot(scan_ctx_t * p_ctx, uint32_t target_pulses, bool b_stop_on_line)
{
    uint16_t const max_polls = b_stop_on_line ? (uint16_t)SEARCH_MAX_POLLS :
                                                (uint16_t)PIVOT_MAX_POLLS;
    uint16_t       result    = TRACK_RUNNING;
    uint32_t       turned;

    if (!p_ctx->b_turning)
    {
        if (MOTION_ACTIVITY_PIVOT == motion_get_activity())
        {
            p_ctx->b_turning = true;
            p_ctx->start_sum = encoder_sum();
            p_ctx->polls     = 0u;
            p_ctx->b_armed   = false;
        }
    }
    else
    {
        // Both wheels count up in a pivot: the sum is 2 x one wheel.
        turned = (encoder_sum() - p_ctx->start_sum) / 2u;
        p_ctx->polls++;

        if (b_stop_on_line && (LINE_ON != line_get_state()))
        {
            p_ctx->b_armed = true;      // Off the tape: a line may stop it.
        }

        if (b_stop_on_line && p_ctx->b_armed &&
            (LINE_ON == line_get_state()))
        {
            result = TRACK_LINE;
        }
        else if ((turned >= target_pulses) || (p_ctx->polls >= max_polls))
        {
            if (p_ctx->polls >= max_polls)
            {
                (void)nav_raise_fault(NAV_FLT_SCAN_ERROR);
            }

            result = TRACK_ANGLE;
        }
        else
        {
            // Still turning.
        }

        if (TRACK_RUNNING != result)
        {
            (void)motion_stop_turn();
            (void)tm_sprintf(g_log, (UB *)"[S5] StopTurn after %u of %u"
                             " pulses%s", (UINT)turned, (UINT)target_pulses,
                             (TRACK_LINE == result) ? " (line)" : "");
            log_text(g_log);
        }
    }

    return (result);
}

/*!
 * @brief  Line search, in the align leg: runs when Navigation waits for a
 *         verdict (after the align pivot, or when the back limit ran out).
 *         Step 0: line already on -> FOUND.  Otherwise ask Navigation for
 *         one pivot to the next zig-zag target (obs_search_target_deg()),
 *         StopTurn it on the angle or on the line, and log the step.
 *         Line seen -> FOUND; all SEARCH_STEPS done -> FAILED
 *         (Navigation stops with LINE_NOT_FOUND).
 */
static void
on_search(scan_ctx_t * p_ctx)
{
    uint16_t const side = fld_get(p_ctx->state_word, F_SIDE_SHIFT,
                                  F_SIDE_MASK);
    uint16_t       step = fld_get(p_ctx->state_word, F_SEARCH_SHIFT,
                                  F_SEARCH_MASK);
    int16_t        target;
    int16_t        from;
    uint16_t       sweep;
    bool           b_left;
    uint16_t       result;

    // "+" direction: towards the original line = the turn-in pivot, which
    // is RIGHT after a LEFT bypass and LEFT after a RIGHT one.
    bool const     b_plus_is_left = ((OBS_SIDE_RIGHT == side) ==
                                     (1 == SEARCH_FIRST_TOWARD_LINE));

    target = obs_search_target_deg(step, SEARCH_START_DEG, SEARCH_STEP_DEG);
    from   = obs_search_target_deg((uint16_t)((0u == step) ? 0u :
                                              (step - 1u)),
                                   SEARCH_START_DEG, SEARCH_STEP_DEG);
    sweep  = (uint16_t)((target > from) ? (target - from) : (from - target));
    b_left = ((target > from) == b_plus_is_left);

    if (p_ctx->b_requested)
    {
        // A search pivot is running: stop it on the angle or the line.
        result = track_pivot(p_ctx, p_ctx->pivot_pulses, true);

        if (TRACK_RUNNING == result)
        {
            (void)tk_dly_tsk(PIVOT_POLL_MS);
        }
        else
        {
            p_ctx->b_requested = false;
            p_ctx->b_turning   = false;
            p_ctx->polls       = 0u;
            (void)tm_sprintf(g_log, (UB *)"[S5] search: pivot %s %u deg"
                             " (to %d), %s", b_left ? "left" : "right",
                             (UINT)sweep, (INT)target,
                             (TRACK_LINE == result) ? "line found"
                                                    : "no line");
            log_text(g_log);

            if (TRACK_LINE == result)
            {
                (void)nav_post_search(NAV_SEARCH_FOUND);
                p_ctx->pivot_pulses = 0u;   // Logs "bypass done".
                step = SEARCH_DONE;
            }
        }
    }
    else if ((SEARCH_DONE == step) || !nav_is_search_wait())
    {
        (void)tk_dly_tsk(PIVOT_POLL_MS);    // Waiting for Navigation.
    }
    else if ((0u == step) && (LINE_ON == line_get_state()))
    {
        (void)tm_sprintf(g_log, (UB *)"[S5] search: line on, no search");
        log_text(g_log);
        (void)nav_post_search(NAV_SEARCH_FOUND);
        p_ctx->pivot_pulses = 0u;           // Logs "bypass done".
        step = SEARCH_DONE;
    }
    else if (step >= SEARCH_STEPS)
    {
        (void)tm_sprintf(g_log, (UB *)"[S5] search: no line within +/-%u"
                         " deg, LINE_NOT_FOUND", (UINT)SEARCH_MAX_DEG);
        log_text(g_log);
        (void)nav_post_search(NAV_SEARCH_FAILED);
        step = SEARCH_DONE;
    }
    else
    {
        // Next zig-zag target: ask Navigation for the pivot.
        step++;
        target = obs_search_target_deg(step, SEARCH_START_DEG,
                                       SEARCH_STEP_DEG);
        from   = obs_search_target_deg((uint16_t)(step - 1u),
                                       SEARCH_START_DEG, SEARCH_STEP_DEG);
        sweep  = (uint16_t)((target > from) ? (target - from)
                                            : (from - target));
        b_left = ((target > from) == b_plus_is_left);
        p_ctx->pivot_pulses = (uint16_t)drv_encoder_mm_to_pulses(
                                  obs_deg_arc_mm(sweep));
        p_ctx->b_requested  = true;
        p_ctx->b_turning    = false;
        p_ctx->polls        = 0u;
        (void)nav_post_search(b_left ? NAV_SEARCH_LEFT : NAV_SEARCH_RIGHT);
    }

    p_ctx->state_word = fld_set(p_ctx->state_word, F_SEARCH_SHIFT,
                                F_SEARCH_MASK, step);
}

/*!
 * @brief  Send one event line: to the dashboard log when comms is built
 *         in (comms_log() prints it too), else to the console only.
 */
static void
log_text(UB const * p_text)
{
#if (1 == S5_HAS_COMMS)
    (void)comms_log((char const *)p_text);
#else
    (void)tm_printf((UB *)"%s\n", p_text);
#endif
}

/*!
 * @brief  Pulses per wheel for the rejoin angle (OBS_REJOIN_STEP x
 *         15 deg).
 */
static uint32_t
rejoin_pulses(void)
{
    return (drv_encoder_mm_to_pulses(obs_pivot_arc_mm(OBS_REJOIN_STEP)));
}

/*!
 * @brief  Pulses per wheel for the align pivot's limit (ALIGN_MAX_DEG).
 */
static uint32_t
align_pulses(void)
{
    return (drv_encoder_mm_to_pulses(obs_deg_arc_mm(ALIGN_MAX_DEG)));
}

/*!
 * @brief  Sum of both wheels' encoder counts (wraps safely: only the
 *         difference of two sums is used).
 */
static uint32_t
encoder_sum(void)
{
    return (drv_encoder_get_count(DRV_ENCODER_LEFT) +
            drv_encoder_get_count(DRV_ENCODER_RIGHT));
}

/*!
 * @brief  ERROR: hold until the scan-error fault is cleared (remote
 *         CLEAR via Navigation).  Standalone: until the front is clear.
 */
static void
on_error(scan_ctx_t * p_ctx)
{
#if (1 == S5_STANDALONE)
    bool const b_is_cleared = (ping_median_cm() > CLEAR_CM);
#else
    bool const b_is_cleared = (0u == (nav_get_faults() &
                                      NAV_FLT_SCAN_ERROR));
#endif

    if (b_is_cleared)
    {
        (void)tm_sprintf(g_log, (UB *)"[S5] ERROR cleared, watching");
        log_text(g_log);
        p_ctx->state_word = ST_WATCH;
    }
    else
    {
        (void)tk_dly_tsk(POLL_MS);
    }
}

/*!
 * @brief  Change state and reset the wait-loop counters.
 */
static void
enter(scan_ctx_t * p_ctx, uint16_t state)
{
    p_ctx->polls        = 0u;
    p_ctx->pivot_pulses = 0u;
    p_ctx->b_seen       = false;
    p_ctx->b_turning    = false;
    p_ctx->b_requested  = false;
    p_ctx->b_armed      = false;
    p_ctx->state_word   = fld_set(p_ctx->state_word, F_STATE_SHIFT,
                                  F_STATE_MASK, state);
}

/*!
 * @brief  A coarse or fine scan failed: retry, or raise the scan-error
 *         fault after SCAN_RETRY_LIMIT failures (cannot wrap: 0..2 -> 3).
 */
static void
scan_failed(scan_ctx_t * p_ctx)
{
    uint16_t const retries = (uint16_t)(fld_get(p_ctx->state_word,
                                                F_RETRY_SHIFT,
                                                F_RETRY_MASK) + 1u);

    p_ctx->state_word = fld_set(p_ctx->state_word, F_RETRY_SHIFT,
                                F_RETRY_MASK, retries);

    if (SCAN_RETRY_LIMIT == retries)
    {
        (void)tm_sprintf(g_log, (UB *)"[S5] ERROR: %u failed scans",
                         (UINT)retries);
        log_text(g_log);
        (void)nav_raise_fault(NAV_FLT_SCAN_ERROR);
        enter(p_ctx, ST_ERROR);
    }
    else
    {
        (void)tm_sprintf(g_log, (UB *)"[S5] retry %u of %u", (UINT)retries,
                         (UINT)SCAN_RETRY_LIMIT);
        log_text(g_log);
        enter(p_ctx, ST_COARSE);
    }
}

/*!
 * @brief  Count silent pings; raise NAV_FLT_NO_ECHO once after
 *         SILENT_LIMIT in a row (check before increment: 4 bits).
 */
static void
update_silence(scan_ctx_t * p_ctx, ultrasonic_reading_t reading)
{
    uint16_t silent = fld_get(p_ctx->state_word, F_SILENT_SHIFT,
                              F_SILENT_MASK);

    if (ultrasonic_is_silent(reading))
    {
        if (silent < SILENT_LIMIT)
        {
            silent++;

            if (SILENT_LIMIT == silent)
            {
                (void)tm_sprintf(g_log, (UB *)"[S5] sensor silent ~1 s:"
                                 " NO_ECHO fault");
                log_text(g_log);
                (void)nav_raise_fault(NAV_FLT_NO_ECHO);
            }
        }
    }
    else
    {
        silent = 0u;
    }

    p_ctx->state_word = fld_set(p_ctx->state_word, F_SILENT_SHIFT,
                                F_SILENT_MASK, silent);
}

/*!
 * @brief  Is Navigation driving, so an obstacle report is acted on?
 */
static bool
is_nav_driving(void)
{
#if (1 == S5_STANDALONE)
    return (true);
#else
    uint32_t const state = nav_get_state();

    return ((NAV_STATE_LINE_FOLLOW == state) ||
            (NAV_STATE_JUNCTION == state));
#endif
}

/*!
 * @brief  Has Navigation left the obstacle / bypass states?
 */
static bool
is_bypass_over(void)
{
    uint32_t const state = nav_get_state();

    return ((NAV_STATE_BYPASS != state) && (NAV_STATE_OBSTACLE != state));
}

/*!
 * @brief  One ping in cm; nothing measured becomes OBS_FAR_CM.
 */
static uint16_t
ping_cm(void)
{
    ultrasonic_reading_t const reading = ultrasonic_read();
    uint16_t                   dist    = OBS_FAR_CM;

    if ((!ultrasonic_is_no_echo(reading)) &&
        (ultrasonic_dist_cm(reading) < ULTRASONIC_MAX_CM))
    {
        dist = ultrasonic_dist_cm(reading);
    }

    return (dist);
}

/*!
 * @brief  Median of three pings, PING_GAP_MS apart.
 */
static uint16_t
ping_median_cm(void)
{
    uint16_t const first = ping_cm();
    uint16_t       second;
    uint16_t       third;

    (void)tk_dly_tsk(PING_GAP_MS);
    second = ping_cm();
    (void)tk_dly_tsk(PING_GAP_MS);
    third = ping_cm();

    return (obs_median3(first, second, third));
}

/*!
 * @brief  Coarse scan: median distance at 30, 60, 90, 120, 150 deg.
 */
static uint64_t
coarse_scan(void)
{
    uint64_t profile = 0u;
    UB       line[LINE_LEN];
    INT      len;
    uint16_t idx;
    uint16_t dist;

    len = tm_sprintf(line, (UB *)"[S5] COARSE:");

    for (idx = 0u; idx < OBS_COARSE_COUNT; idx++)
    {
        servo_set_angle_deg(coarse_angle(idx));
        (void)tk_dly_tsk(SERVO_SETTLE_MS);
        dist    = ping_median_cm();
        profile = obs_prof_set(profile, idx, dist);
        len    += tm_sprintf(&line[len], (UB *)" %u:%u",
                             (UINT)coarse_angle(idx), (UINT)dist);
    }

    servo_set_angle_deg((uint8_t)CENTRE_DEG);
    (void)tm_sprintf(&line[len], (UB *)" cm (511 = open)");
    log_text(line);

    return (profile);
}

/*!
 * @brief  Fine scan: 15 deg steps out from the centre on one side.
 */
static uint64_t
fine_scan(uint16_t side)
{
    uint64_t profile = 0u;
    UB       line[LINE_LEN];
    INT      len;
    uint16_t step;
    uint16_t dist;

    len = tm_sprintf(line, (UB *)"[S5] FINE:");

    for (step = 1u; step <= OBS_FINE_COUNT; step++)
    {
        servo_set_angle_deg(fine_angle(side, step));
        (void)tk_dly_tsk(SERVO_SETTLE_MS);
        dist    = ping_median_cm();
        profile = obs_prof_set(profile, (uint16_t)(step - 1u), dist);
        len    += tm_sprintf(&line[len], (UB *)" %u:%u",
                             (UINT)fine_angle(side, step), (UINT)dist);
    }

    servo_set_angle_deg((uint8_t)CENTRE_DEG);
    (void)tm_sprintf(&line[len], (UB *)" cm");
    log_text(line);

    return (profile);
}

static uint8_t
coarse_angle(uint16_t idx)
{
    return ((uint8_t)(COARSE_START_DEG + (idx * COARSE_STEP_DEG)));
}

/*!
 * @brief  Servo angle for fine step 1..OBS_FINE_COUNT on one side.
 */
static uint8_t
fine_angle(uint16_t side, uint16_t step)
{
    bool const     b_is_high = ((OBS_SIDE_LEFT == side) ==
                                (1 == OBS_LEFT_IS_HIGH_ANGLE));
    uint16_t const offset    = (uint16_t)(step * OBS_FINE_STEP_DEG);

    return ((uint8_t)(b_is_high ? (CENTRE_DEG + offset)
                                : (CENTRE_DEG - offset)));
}

static uint16_t
fld_get(uint16_t word, uint16_t shift, uint16_t mask)
{
    return ((uint16_t)((word >> shift) & mask));
}

/*!
 * @brief  Write a packed field; values larger than the field saturate.
 */
static uint16_t
fld_set(uint16_t word, uint16_t shift, uint16_t mask, uint16_t value)
{
    uint16_t const clamped = (value > mask) ? mask : value;

    return ((uint16_t)((word & (uint16_t)~(uint16_t)(mask << shift)) |
                       (uint16_t)(clamped << shift)));
}

/*** end of file ***/
