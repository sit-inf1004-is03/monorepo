/** @file obstacle_scan.c
 *
 * @brief Obstacle Scan task (Subsystem 5): Week 6 statechart (Figure 4),
 *        joined to Navigation (IF-11, IF-12, IF-14) and Motion (IF-09).
 *
 *   WATCH ---2 pings < 30 cm---> nav_post_obstacle()
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
 *     |                           |    PARALLEL pivot back, the same angle
 *     |                           |    IN       pivot in, OBS_REJOIN_STEP
 *     |                           |    ALIGN    re-align on the line, the
 *     |                           v             rejoin angle again
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
 *   bits 13..11  print divider              3 bits  (0..4)
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

// 1 = bench test on its own (prints only), 0 = full mission with
// Navigation.
//
#define S5_STANDALONE           (0)

// ---- Tunable settings (Week 6 design, car bench tests) ----
#define WATCH_PERIOD_MS         (100)   // Ping rate while driving.
#define POLL_MS                 (65)    // Wait loops; > 60 ms ping cooldown.
#define PING_GAP_MS             (65)
#define TRIGGER_CM              (30u)   // Obstacle closer than this...
#define CONFIRM_PINGS           (2u)    // ...on this many pings in a row.
#define CLEAR_CM                (50u)   // A direction is open beyond this.
#define SILENT_LIMIT            (10u)   // ~1 s of silence -> NO_ECHO fault.
#define SCAN_RETRY_LIMIT        (3u)
#define PRINT_EVERY             (5u)    // WATCH: print every 5th ping.
#define BRAKE_WAIT_POLLS        (31u)   // ~2 s for Navigation to brake.
#define PIVOT_POLL_MS           (10)    // Encoder check rate in a pivot.
#define PIVOT_MAX_POLLS         (250u)  // 2.5 s bound on one pivot.

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
#define F_SIDE_SHIFT            (14u)
#define F_SIDE_MASK             (0x3u)

// Task context: on the task's stack, passed by pointer.
//
typedef struct
{
    uint16_t state_word;        // Packed: see the file header.
    uint64_t coarse;            // 5 x 9-bit coarse profile.
    uint32_t start_sum;         // Both encoder counts at pivot start.
    uint16_t pivot_pulses;      // Pivot-out angle, in pulses per wheel;
                                // 0 once the bypass's pivots are done.
    uint8_t  polls;             // Wait-loop counter (0..255).
    bool     b_seen;            // Phase flag: straight seen.
    bool     b_turning;         // Phase flag: tracked pivot started.
} scan_ctx_t;

static void     on_watch(scan_ctx_t * p_ctx);
static void     on_wait_brake(scan_ctx_t * p_ctx);
static void     on_coarse(scan_ctx_t * p_ctx);
static void     on_fine(scan_ctx_t * p_ctx);
static void     on_bypass(scan_ctx_t * p_ctx);
static void     on_error(scan_ctx_t * p_ctx);
static void     enter(scan_ctx_t * p_ctx, uint16_t state);
static void     scan_failed(scan_ctx_t * p_ctx);
static void     update_silence(scan_ctx_t * p_ctx,
                               ultrasonic_reading_t reading);
static bool     is_nav_driving(void);
static bool     is_bypass_over(void);
static bool     track_pivot(scan_ctx_t * p_ctx, uint32_t target_pulses);
static uint32_t encoder_sum(void);
static uint32_t rejoin_pulses(void);
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
    ctx.polls        = 0u;
    ctx.b_seen       = false;
    ctx.b_turning    = false;

    servo_set_angle_deg((uint8_t)CENTRE_DEG);
    (void)tm_printf((UB *)"[S5] obstacle scan running (standalone=%d)\n",
                    S5_STANDALONE);

    for (;;)
    {
        switch (fld_get(ctx.state_word, F_STATE_SHIFT, F_STATE_MASK))
        {
        case ST_WATCH:
            on_watch(&ctx);
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
        (void)tm_printf((UB *)"[S5] OBSTACLE at %u cm\n", (UINT)dist);
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
        (void)tm_printf((UB *)"[S5] Navigation did not brake; watching\n");
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
        (void)tm_printf((UB *)"[S5] COARSE: no side clear (> %u cm)\n",
                        (UINT)CLEAR_CM);
        scan_failed(p_ctx);
    }
    else
    {
        (void)tm_printf((UB *)"[S5] COARSE: %s side open\n",
                        (OBS_SIDE_LEFT == side) ? "LEFT" : "RIGHT");
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
        (void)tm_printf((UB *)"[S5] FINE: obstacle edge not found\n");
        scan_failed(p_ctx);
    }
    else
    {
        front_cm = obs_prof_get(p_ctx->coarse, OBS_COARSE_CENTRE);

        if (front_cm >= OBS_FAR_CM)
        {
            front_cm = TRIGGER_CM;      // Centre missed: assume 30 cm.
        }

        lateral_cm = obs_lateral_cm(front_cm, edge);
        pivot_step = obs_pivot_step(front_cm, lateral_cm);
        out_cm     = obs_out_cm(lateral_cm, pivot_step);
        past_cm    = obs_past_cm(front_cm, out_cm, pivot_step);
        back_cm    = obs_back_cm(out_cm, pivot_step);
        (void)tm_printf((UB *)"[S5] PLAN: %s, turn %u deg, out %u past %u"
                        " back <= %u cm (front %u, edge %u deg,"
                        " side %u cm)\n",
                        (OBS_SIDE_LEFT == side) ? "LEFT" : "RIGHT",
                        (UINT)(pivot_step * OBS_FINE_STEP_DEG),
                        (UINT)out_cm, (UINT)past_cm, (UINT)back_cm,
                        (UINT)front_cm, (UINT)(edge * OBS_FINE_STEP_DEG),
                        (UINT)lateral_cm);
        enter(p_ctx, ST_BYPASS);
        p_ctx->state_word = fld_set(p_ctx->state_word, F_LEG_SHIFT,
                                    F_LEG_MASK, LEG_OUT);
#if (0 == S5_STANDALONE)
        (void)nav_post_bypass_plan((OBS_SIDE_LEFT == side) ?
                                   NAV_SIDE_LEFT : NAV_SIDE_RIGHT,
                                   out_cm, past_cm, back_cm);
        p_ctx->pivot_pulses = (uint16_t)drv_encoder_mm_to_pulses(
                                  obs_pivot_arc_mm(pivot_step));
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
        (void)tm_printf((UB *)"[S5] front clear, watching\n");
        p_ctx->state_word = ST_WATCH;   // Clears every counter and side.
    }
    else
    {
        (void)tk_dly_tsk(POLL_MS);
    }
#else
    uint16_t const leg = fld_get(p_ctx->state_word, F_LEG_SHIFT,
                                 F_LEG_MASK);
    uint32_t const target = (leg <= LEG_PARALLEL) ?
                            (uint32_t)p_ctx->pivot_pulses : rejoin_pulses();

    if (is_bypass_over())
    {
        if (0u == p_ctx->pivot_pulses)
        {
            (void)tm_printf((UB *)"[S5] bypass done, watching\n");
        }
        else
        {
            (void)tm_printf((UB *)"[S5] bypass ended at leg %u, watching\n",
                            (UINT)leg);
        }

        // Clears every counter, the side and the leg (which shares the
        // near-ping bits).
        p_ctx->state_word   = ST_WATCH;
        p_ctx->pivot_pulses = 0u;
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
    else if (track_pivot(p_ctx, target))
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
 * @brief  Follow one pivot with the encoders: start counting when Motion
 *         starts pivoting, StopTurn when each wheel has gone
 *         target_pulses (or after PIVOT_MAX_POLLS: SCAN_ERROR fault).
 * @param  p_ctx          Task context (b_turning, start_sum, polls).
 * @param  target_pulses  Pulses per wheel for the angle.
 * @return true once StopTurn has been sent.
 */
static bool
track_pivot(scan_ctx_t * p_ctx, uint32_t target_pulses)
{
    bool     b_done = false;
    uint32_t turned;

    if (!p_ctx->b_turning)
    {
        if (MOTION_ACTIVITY_PIVOT == motion_get_activity())
        {
            p_ctx->b_turning = true;
            p_ctx->start_sum = encoder_sum();
            p_ctx->polls     = 0u;
        }
    }
    else
    {
        // Both wheels count up in a pivot: the sum is 2 x one wheel.
        turned = (encoder_sum() - p_ctx->start_sum) / 2u;
        p_ctx->polls++;

        if ((turned >= target_pulses) || (p_ctx->polls >= PIVOT_MAX_POLLS))
        {
            if (p_ctx->polls >= PIVOT_MAX_POLLS)
            {
                (void)nav_raise_fault(NAV_FLT_SCAN_ERROR);
            }

            (void)motion_stop_turn();
            (void)tm_printf((UB *)"[S5] StopTurn after %u of %u pulses\n",
                            (UINT)turned, (UINT)target_pulses);
            b_done = true;
        }
    }

    return (b_done);
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
        (void)tm_printf((UB *)"[S5] ERROR cleared, watching\n");
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
        (void)tm_printf((UB *)"[S5] ERROR: %u failed scans\n",
                        (UINT)retries);
        (void)nav_raise_fault(NAV_FLT_SCAN_ERROR);
        enter(p_ctx, ST_ERROR);
    }
    else
    {
        (void)tm_printf((UB *)"[S5] retry %u of %u\n", (UINT)retries,
                        (UINT)SCAN_RETRY_LIMIT);
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
                (void)tm_printf((UB *)"[S5] sensor silent ~1 s:"
                                " NO_ECHO fault\n");
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
    (void)tm_printf((UB *)"%s cm (511 = open)\n", line);

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
    (void)tm_printf((UB *)"%s cm\n", line);

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
