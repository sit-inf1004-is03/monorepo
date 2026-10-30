/** @file task_navigation.c
 *
 * @brief Navigation / Vehicle Controller task (see navigation.h).
 *
 * The other modules' inputs arrive as event bits on Navigation's event
 * flag, each with its data in a word that only that module writes.  The
 * task wakes on any of them, or every NAV_POLL_MSEC to collect Motion's
 * replies, and steps its state machine.
 *
 * Mission flow (PROPOSED, see navigation.h):
 *   - Line following: one long straight; Line Following steers it through
 *     Motion's turn level.
 *   - Barcode read: cap the speed (slow down) and wait for the junction.
 *     At the junction: C drives on at full speed; A, B and D brake, then
 *     pivot until StopTurn (D pivots NAV_UTURN_PIVOTS times).
 *   - Obstacle: brake and wait for Obstacle Scan's bypass plan, then pivot
 *     out (StopTurn from Obstacle Scan), drive the plan's distance, pivot
 *     back (StopTurn from Obstacle Scan), creep forward until the line is
 *     on, brake, and pivot onto it (StopTurn from Line Following).
 *   - A stop fault brakes and holds until a remote CLEAR.
 *
 * Navigation sends Motion one command at a time and always waits for its
 * reply before the next, so every command gets exactly one reply.
 *
 * NOTE: Each tm_printf() format is cast to (UB *).  tm_printf() takes UB *
 * and the formats are ASCII string literals, so only the signedness of the
 * character type changes.
 */

#include <stdint.h>
#include <stdbool.h>

#include "navigation.h"
#include "../subsystem2/motion.h"
#include "../subsystem2/drv_encoder.h"

#define NAV_CORE            (TP_PRC2)   // Core 1, with task_motion.
#define NAV_POLL_MSEC       (10)        // Motion reply poll period.

// Mission tuning.  PROVISIONAL until the track tests.
//
#define NAV_CRUISE_MM_S     (300u)
#define NAV_JUNCTION_MM_S   (120u)      // Speed cap after a barcode.
#define NAV_APPROACH_MAX_MM (300u)      // Act anyway this far after a
                                        // barcode with no junction.
#define NAV_BYPASS_MM_S     (200u)
#define NAV_REACQUIRE_MM_S  (120u)
#define NAV_REACQUIRE_MAX_MM (600u)     // Line not found by then: fault.
#define NAV_UTURN_PIVOTS    (2u)        // StopTurns in a U-turn; check on
                                        // the track's junctions.
#define NAV_UTURN_DIR       (MOTION_DIR_LEFT)
#define MM_PER_CM           (10u)

// Event bits.  Bits 0-7 are the fault bits (navigation.h); bit 8 is kept
// free for StopTurn, as in the team's map.
//
#define EVT_BARCODE         (1u << 9)
#define EVT_JUNCTION        (1u << 10)
#define EVT_OBSTACLE        (1u << 11)
#define EVT_PLAN            (1u << 12)
#define EVT_REMOTE          (1u << 13)
#define EVT_ALL             (EVT_BARCODE | EVT_JUNCTION | EVT_OBSTACLE |    \
                             EVT_PLAN | EVT_REMOTE)
#define WAIT_ALL            (EVT_ALL | NAV_FLT_ALL)

// Bypass plan, uint16 in a uint32_t (PROPOSED bit order, Ruien owns it):
// bits 0-1 side, bits 2-10 distance in cm (0-511), bits 11-14 angle
// (unused since Tribunal 1, always 0), bit 15 reserved (0).
//
#define FIELD_MASK(bits)    ((1u << (bits)) - 1u)
#define PLAN_SIDE_SHIFT     (0u)
#define PLAN_SIDE_BITS      (2u)
#define PLAN_DIST_SHIFT     (2u)
#define PLAN_DIST_BITS      (9u)

// Internal phases.  Several phases share one vehicle state.
//
#define P_IDLE              (0u)
#define P_FOLLOW            (1u)
#define P_APPROACH          (2u)    // Barcode read, slowing.
#define P_BRAKING           (3u)    // Brake sent; then after_brake.
#define P_TURNING           (4u)
#define P_WAIT_PLAN         (5u)
#define P_BYPASS_OUT        (6u)
#define P_BYPASS_PAST       (7u)
#define P_BYPASS_BACK       (8u)
#define P_REACQUIRE         (9u)
#define P_ALIGN             (10u)
#define P_HALTING           (11u)   // Brake sent; then P_HALTED.
#define P_HALTED            (12u)

typedef struct
{
    uint32_t phase;
    uint32_t after_brake;       // Phase to enter when the brake is done.
    bool     b_motion_busy;     // A command was sent; its reply is due.
    uint32_t barcode;           // Command waiting for the junction.
    uint32_t turn_dir;          // MOTION_DIR_ of the junction pivots.
    uint32_t turns_left;        // Pivots still to run at this junction.
    uint32_t start_left;        // Encoder counts when the barcode was read.
    uint32_t start_right;
    uint32_t plan;              // Bypass plan being run.
    bool     b_barcode;         // Inputs not yet handled.
    bool     b_junction;
    bool     b_obstacle;
    bool     b_plan;
    bool     b_remote;
} nav_ctx_t;

static ID                g_flg_id   = 0;

// Each written by one module only (navigation.h), read by the task.
//
static uint32_t volatile g_barcode_in = 0u;     // Wing
static uint32_t volatile g_line_state = NAV_LINE_ON;    // Wing
static uint32_t volatile g_plan_in    = 0u;     // Ruien
static uint32_t volatile g_remote_in  = 0u;     // Rifa

// Written by the Navigation task only.
//
static uint32_t volatile g_state  = NAV_STATE_IDLE;
static uint32_t volatile g_faults = 0u;
static uint32_t volatile g_reason = NAV_REASON_NONE;

// Used by the Navigation task only.  Static storage starts zeroed:
// P_IDLE, nothing pending.
//
static nav_ctx_t         g_ctx;

static void nav_task(INT start_code, void * p_ext_info);
static void nav_take_events(UINT pattern);
static void nav_poll_motion(void);
static void nav_on_reply(motion_reply_t const * p_reply);
static void nav_on_move_done(void);
static void nav_process(void);
static void nav_on_remote(uint32_t command);
static void nav_at_junction(void);
static bool nav_approach_too_long(void);
static void nav_enter(uint32_t phase);
static void nav_brake_then(uint32_t phase, uint32_t state);
static void nav_halt(uint32_t state, uint32_t reason);
static void nav_set_state(uint32_t state);
static void nav_drive_line(void);
static void nav_send_straight(uint32_t speed_mm_s, uint32_t distance_mm);
static void nav_send_pivot(uint32_t direction);
static void nav_send(motion_cmd_t const * p_cmd);
static ER   nav_post(uint32_t event);


/*!
 * @brief Create Navigation's event flag.  Call from usermain(), after
 *        subsystem2_init() (Motion's objects).
 *
 * @return E_OK, or the error from tk_cre_flg().
 */
ER
navigation_init (void)
{
    T_CFLG cflg = { 0 };

    cflg.flgatr  = (TA_TFIFO | TA_WSGL);    // Only the task waits.
    cflg.iflgptn = 0u;
    g_flg_id     = tk_cre_flg(&cflg);

    return ((g_flg_id < E_OK) ? g_flg_id : E_OK);
}

/*!
 * @brief Start the Navigation task.  It stays IDLE until a remote START.
 *
 * @return E_OK, or the error from creating or starting the task.
 */
ER
navigation_start (void)
{
    // Cast: the kernel stores every task entry as FP.  TA_HLNG tasks are
    // called with (INT, void *), which is nav_task()'s signature.
    //
    ID const task_id = app_start_task((FP) nav_task, NAV_PRIORITY, NAV_CORE,
                                      "task_navigation");

    return ((task_id < E_OK) ? task_id : E_OK);
}

/*!
 * @brief A barcode was decoded (Wing).  Acted on while following the line.
 *
 * @param[in] command  NAV_BARCODE_LEFT (A) to NAV_BARCODE_UTURN (D).
 *
 * @return E_OK, E_PAR for a bad command, or the error from tk_set_flg().
 */
ER
nav_post_barcode (uint32_t command)
{
    ER result = E_PAR;

    if (command <= NAV_BARCODE_UTURN)
    {
        g_barcode_in = command;     // One store, then the event.
        result       = nav_post(EVT_BARCODE);
    }

    return (result);
}

/*!
 * @brief The car has reached a junction (Wing).  Acted on only after a
 *        barcode.
 *
 * @return E_OK, or the error from tk_set_flg().
 */
ER
nav_post_junction (void)
{
    return (nav_post(EVT_JUNCTION));
}

/*!
 * @brief Report the line state (Wing), on every change.
 *
 * @param[in] state  NAV_LINE_ON, NAV_LINE_LOST or NAV_LINE_SEARCHING.
 *
 * @return E_OK, or E_PAR for a bad state.
 */
ER
nav_set_line_state (uint32_t state)
{
    ER result = E_PAR;

    if (state <= NAV_LINE_SEARCHING)
    {
        g_line_state = state;
        result       = E_OK;
    }

    return (result);
}

/*!
 * @brief An obstacle is close ahead (Ruien).  Acted on while following
 *        the line; Navigation brakes and goes to NAV_STATE_OBSTACLE, which
 *        is Obstacle Scan's cue to scan.
 *
 * @return E_OK, or the error from tk_set_flg().
 */
ER
nav_post_obstacle (void)
{
    return (nav_post(EVT_OBSTACLE));
}

/*!
 * @brief The scan's bypass plan (Ruien), in NAV_STATE_OBSTACLE.  A failed
 *        scan raises NAV_FLT_SCAN_ERROR instead.
 *
 * @param[in] side         NAV_SIDE_LEFT or NAV_SIDE_RIGHT.
 * @param[in] distance_cm  How far to drive past the obstacle, 1 to
 *                         NAV_BYPASS_CM_MAX.
 *
 * @return E_OK, E_PAR for a bad side or distance, or the error from
 *         tk_set_flg().
 */
ER
nav_post_bypass_plan (uint32_t side, uint32_t distance_cm)
{
    ER result = E_PAR;

    if ((side <= NAV_SIDE_RIGHT) && (distance_cm >= 1u) &&
        (distance_cm <= NAV_BYPASS_CM_MAX))
    {
        g_plan_in = nav_plan_pack(side, distance_cm);
        result    = nav_post(EVT_PLAN);
    }

    return (result);
}

/*!
 * @brief Raise fault bits (any subsystem, its own bits only).  A bit in
 *        NAV_FLT_STOP_MASK brakes the car; Wi-Fi loss is only recorded.
 *        Bits stay raised in nav_get_faults() until a remote CLEAR.
 *
 * @param[in] bits  One or more NAV_FLT_ bits.
 *
 * @return E_OK, E_PAR for bits outside NAV_FLT_ALL, or the error from
 *         tk_set_flg().
 */
ER
nav_raise_fault (uint32_t bits)
{
    ER result = E_PAR;

    if ((0u != bits) && (0u == (bits & ~NAV_FLT_ALL)))
    {
        result = tk_set_flg(g_flg_id, bits);
    }

    return (result);
}

/*!
 * @brief A command from the laptop (Rifa's MQTT handler).
 *
 * @param[in] command  NAV_REMOTE_START, _STOP or _CLEAR.
 *
 * @return E_OK, E_PAR for a bad command, or the error from tk_set_flg().
 */
ER
nav_post_remote (uint32_t command)
{
    ER result = E_PAR;

    if ((command >= NAV_REMOTE_START) && (command <= NAV_REMOTE_CLEAR))
    {
        g_remote_in = command;
        result      = nav_post(EVT_REMOTE);
    }

    return (result);
}

/*!
 * @brief Read the vehicle state (NAV_STATE_), 3 bits.
 */
uint32_t
nav_get_state (void)
{
    return (g_state);
}

/*!
 * @brief Read the fault bits raised since the last remote CLEAR.
 */
uint32_t
nav_get_faults (void)
{
    return (g_faults);
}

/*!
 * @brief Read why Navigation last stopped (NAV_REASON_).
 */
uint32_t
nav_get_reason (void)
{
    return (g_reason);
}

/*!
 * @brief Pack a bypass plan (layout above).  Clamps instead of wrapping.
 *
 * @param[in] side         NAV_SIDE_; above NAV_SIDE_RIGHT counts as right.
 * @param[in] distance_cm  Clamped to NAV_BYPASS_CM_MAX.
 *
 * @return The packed plan; angle and reserved bits are 0.
 */
uint32_t
nav_plan_pack (uint32_t side, uint32_t distance_cm)
{
    uint32_t const s = ((side > NAV_SIDE_RIGHT) ? NAV_SIDE_RIGHT : side);
    uint32_t const d = ((distance_cm > NAV_BYPASS_CM_MAX) ?
                        NAV_BYPASS_CM_MAX : distance_cm);

    return (((s & FIELD_MASK(PLAN_SIDE_BITS)) << PLAN_SIDE_SHIFT) |
            ((d & FIELD_MASK(PLAN_DIST_BITS)) << PLAN_DIST_SHIFT));
}

/*!
 * @brief Side field of a packed bypass plan.
 */
uint32_t
nav_plan_side (uint32_t plan)
{
    return ((plan >> PLAN_SIDE_SHIFT) & FIELD_MASK(PLAN_SIDE_BITS));
}

/*!
 * @brief Distance field of a packed bypass plan, in cm.
 */
uint32_t
nav_plan_distance_cm (uint32_t plan)
{
    return ((plan >> PLAN_DIST_SHIFT) & FIELD_MASK(PLAN_DIST_BITS));
}

/*!
 * @brief Task body: wait for an input or the poll period, then step.
 *
 * @param[in] start_code  Unused.
 * @param[in] p_ext_info  Unused.
 */
static void
nav_task (INT start_code, void * p_ext_info)
{
    (void) start_code;
    (void) p_ext_info;

    for (;;)
    {
        UINT pattern = 0u;

        if (E_OK == tk_wai_flg(g_flg_id, WAIT_ALL, (TWF_ORW | TWF_BITCLR),
                               &pattern, NAV_POLL_MSEC))
        {
            nav_take_events(pattern);
        }

        nav_poll_motion();
        nav_process();
    }
}

/*!
 * @brief Latch new faults, and note which inputs arrived.  A new stop
 *        fault brakes at once.
 *
 * @param[in] pattern  The event bits that woke the task.
 */
static void
nav_take_events (UINT pattern)
{
    // Cast: UINT and uint32_t are both 32 bits on this target.
    //
    uint32_t const bits   = (uint32_t) pattern;
    uint32_t const faults = bits & NAV_FLT_ALL;
    uint32_t const fresh  = faults & ~g_faults;

    g_faults = g_faults | faults;   // This task is the only writer.

    g_ctx.b_barcode  = (g_ctx.b_barcode || (0u != (bits & EVT_BARCODE)));
    g_ctx.b_junction = (g_ctx.b_junction || (0u != (bits & EVT_JUNCTION)));
    g_ctx.b_obstacle = (g_ctx.b_obstacle || (0u != (bits & EVT_OBSTACLE)));
    g_ctx.b_plan     = (g_ctx.b_plan || (0u != (bits & EVT_PLAN)));
    g_ctx.b_remote   = (g_ctx.b_remote || (0u != (bits & EVT_REMOTE)));

    if ((0u != (fresh & NAV_FLT_STOP_MASK)) &&
        (NAV_STATE_FAULT != g_state))
    {
        nav_halt(NAV_STATE_FAULT, NAV_REASON_FAULT_BIT);
    }
}

/*!
 * @brief Collect Motion's reply to the command in progress, if it is in.
 */
static void
nav_poll_motion (void)
{
    if (g_ctx.b_motion_busy)
    {
        motion_reply_t reply = { 0u, 0u };

        if (E_OK == motion_wait_reply(&reply, TMO_POL))
        {
            g_ctx.b_motion_busy = false;
            nav_on_reply(&reply);
        }
    }
}

/*!
 * @brief React to Motion's reply.
 *
 * @param[in] p_reply  The reply.
 */
static void
nav_on_reply (motion_reply_t const * p_reply)
{
#if APP_TEST
    if (MOTION_FAULT == p_reply->status)
    {
        tm_printf((UB *) "[NAV] Motion FAULT, reason %lu (1 bad command, "
                  "2 stall, 3 pivot timeout)\n", p_reply->reason);
    }
#endif

    if ((MOTION_FAULT == p_reply->status) &&
        (MOTION_REASON_STALL == p_reply->reason))
    {
        g_faults = g_faults | NAV_FLT_MOTOR_STALL;
    }

    if (P_HALTING == g_ctx.phase)
    {
        g_ctx.phase = P_HALTED;
    }
    else if (MOTION_FAULT == p_reply->status)
    {
        nav_halt(NAV_STATE_FAULT, NAV_REASON_MOTION);
    }
    else if (P_BRAKING == g_ctx.phase)
    {
        nav_enter(g_ctx.after_brake);
    }
    else
    {
        nav_on_move_done();
    }
}

/*!
 * @brief A move ended normally (DONE, or BRAKED by someone else): start
 *        the next step of the current phase.
 */
static void
nav_on_move_done (void)
{
    switch (g_ctx.phase)
    {
        case P_FOLLOW:
        case P_APPROACH:
            nav_drive_line();       // The long straight ran out: go on.
            break;

        case P_TURNING:
            if (g_ctx.turns_left > 1u)
            {
                g_ctx.turns_left--;
                nav_send_pivot(g_ctx.turn_dir);
            }
            else
            {
                nav_enter(P_FOLLOW);
            }
            break;

        case P_BYPASS_OUT:
            nav_enter(P_BYPASS_PAST);
            break;

        case P_BYPASS_PAST:
            nav_enter(P_BYPASS_BACK);
            break;

        case P_BYPASS_BACK:
            nav_enter(P_REACQUIRE);
            break;

        case P_REACQUIRE:
            nav_halt(NAV_STATE_FAULT, NAV_REASON_LINE_NOT_FOUND);
            break;

        case P_ALIGN:
            nav_enter(P_FOLLOW);
            break;

        default:
            // No move belongs to this phase.
            break;
    }
}

/*!
 * @brief Act on the inputs that arrived, as far as the phase allows, and
 *        run the checks that do not wait for an input.
 */
static void
nav_process (void)
{
    // Remote first: a STOP must not be undone by an input below.
    //
    if (g_ctx.b_remote)
    {
        g_ctx.b_remote = false;
        nav_on_remote(g_remote_in);
    }

    if (g_ctx.b_obstacle)
    {
        g_ctx.b_obstacle = false;

        // An obstacle wins over a barcode waiting for its junction
        // (design assumption: the two do not happen together).
        //
        if ((P_FOLLOW == g_ctx.phase) || (P_APPROACH == g_ctx.phase))
        {
            motion_set_speed_cap(0u);
            nav_brake_then(P_WAIT_PLAN, NAV_STATE_OBSTACLE);
        }
    }

    if (g_ctx.b_barcode)
    {
        g_ctx.b_barcode = false;

        if ((P_FOLLOW == g_ctx.phase) || (P_APPROACH == g_ctx.phase))
        {
            g_ctx.barcode = g_barcode_in;
            nav_enter(P_APPROACH);
        }
    }

    if (g_ctx.b_junction)
    {
        g_ctx.b_junction = false;

        if (P_APPROACH == g_ctx.phase)
        {
            nav_at_junction();
        }
    }
    else if ((P_APPROACH == g_ctx.phase) && nav_approach_too_long())
    {
        nav_at_junction();
    }
    else
    {
        // No junction yet.
    }

    // A plan that arrives while the obstacle brake finishes is kept.
    //
    if (g_ctx.b_plan)
    {
        if (P_WAIT_PLAN == g_ctx.phase)
        {
            g_ctx.b_plan = false;
            g_ctx.plan   = g_plan_in;
            nav_enter(P_BYPASS_OUT);
        }
        else if (!((P_BRAKING == g_ctx.phase) &&
                   (P_WAIT_PLAN == g_ctx.after_brake)))
        {
            g_ctx.b_plan = false;
        }
        else
        {
            // Wait for the brake.
        }
    }

    if ((P_REACQUIRE == g_ctx.phase) && (NAV_LINE_ON == g_line_state))
    {
        nav_brake_then(P_ALIGN, NAV_STATE_BYPASS);
    }
}

/*!
 * @brief Act on a remote command.
 *
 * @param[in] command  NAV_REMOTE_.
 */
static void
nav_on_remote (uint32_t command)
{
    bool const b_halted = ((P_IDLE == g_ctx.phase) ||
                           (P_HALTED == g_ctx.phase));

    if (NAV_REMOTE_START == command)
    {
        if (b_halted && (NAV_STATE_FAULT != g_state))
        {
            g_reason = NAV_REASON_NONE;
            nav_enter(P_FOLLOW);
        }
    }
    else if (NAV_REMOTE_STOP == command)
    {
        if ((!b_halted) && (P_HALTING != g_ctx.phase))
        {
            nav_halt(NAV_STATE_STOPPED, NAV_REASON_REMOTE_STOP);
        }
    }
    else
    {
        // CLEAR.  A fault that is still there must be raised again by its
        // owner.
        //
        if ((NAV_STATE_FAULT == g_state) && (P_HALTED == g_ctx.phase))
        {
            g_faults    = 0u;
            g_reason    = NAV_REASON_NONE;
            g_ctx.phase = P_IDLE;
            nav_set_state(NAV_STATE_IDLE);
        }
    }
}

/*!
 * @brief At the junction: drive on for C, else brake and turn.
 */
static void
nav_at_junction (void)
{
    if (NAV_BARCODE_STRAIGHT == g_ctx.barcode)
    {
        nav_enter(P_FOLLOW);
    }
    else
    {
        g_ctx.turns_left = 1u;

        if (NAV_BARCODE_LEFT == g_ctx.barcode)
        {
            g_ctx.turn_dir = MOTION_DIR_LEFT;
        }
        else if (NAV_BARCODE_RIGHT == g_ctx.barcode)
        {
            g_ctx.turn_dir = MOTION_DIR_RIGHT;
        }
        else
        {
            g_ctx.turn_dir   = NAV_UTURN_DIR;
            g_ctx.turns_left = NAV_UTURN_PIVOTS;
        }

        nav_brake_then(P_TURNING, NAV_STATE_JUNCTION);
    }
}

/*!
 * @brief True once the car has driven NAV_APPROACH_MAX_MM since the
 *        barcode without reaching a junction.
 */
static bool
nav_approach_too_long (void)
{
    uint32_t const left  = drv_encoder_get_count(DRV_ENCODER_LEFT) -
                           g_ctx.start_left;
    uint32_t const right = drv_encoder_get_count(DRV_ENCODER_RIGHT) -
                           g_ctx.start_right;

    return (((left / 2u) + (right / 2u)) >=
            drv_encoder_mm_to_pulses(NAV_APPROACH_MAX_MM));
}

/*!
 * @brief Enter a phase: set the vehicle state and start the phase's move.
 *
 * @param[in] phase  P_ phase.
 */
static void
nav_enter (uint32_t phase)
{
    uint32_t const side = nav_plan_side(g_ctx.plan);
    uint32_t const out  = ((NAV_SIDE_LEFT == side) ? MOTION_DIR_LEFT :
                                                     MOTION_DIR_RIGHT);
    uint32_t const back = ((NAV_SIDE_LEFT == side) ? MOTION_DIR_RIGHT :
                                                     MOTION_DIR_LEFT);

    g_ctx.phase = phase;

    switch (phase)
    {
        case P_FOLLOW:
            nav_set_state(NAV_STATE_LINE_FOLLOW);
            motion_set_speed_cap(0u);
            nav_drive_line();
            break;

        case P_APPROACH:
            nav_set_state(NAV_STATE_JUNCTION);
            g_ctx.start_left  = drv_encoder_get_count(DRV_ENCODER_LEFT);
            g_ctx.start_right = drv_encoder_get_count(DRV_ENCODER_RIGHT);
            motion_set_speed_cap(NAV_JUNCTION_MM_S);
            nav_drive_line();
            break;

        case P_TURNING:
            nav_set_state(NAV_STATE_JUNCTION);
            motion_set_speed_cap(0u);
            nav_send_pivot(g_ctx.turn_dir);
            break;

        case P_WAIT_PLAN:
            nav_set_state(NAV_STATE_OBSTACLE);
            break;

        case P_BYPASS_OUT:
            nav_set_state(NAV_STATE_BYPASS);
            nav_send_pivot(out);
            break;

        case P_BYPASS_PAST:
            nav_send_straight(NAV_BYPASS_MM_S,
                              (nav_plan_distance_cm(g_ctx.plan) * MM_PER_CM));
            break;

        case P_BYPASS_BACK:
            nav_send_pivot(back);
            break;

        case P_REACQUIRE:
            nav_send_straight(NAV_REACQUIRE_MM_S, NAV_REACQUIRE_MAX_MM);
            break;

        case P_ALIGN:
            nav_send_pivot(out);    // The line runs the original way.
            break;

        default:
            // P_IDLE, P_BRAKING, P_HALTING, P_HALTED: set by the callers.
            break;
    }
}

/*!
 * @brief Brake the running move, then enter a phase.  Without a running
 *        move the phase is entered at once.
 *
 * @param[in] phase  P_ phase to enter after the brake.
 * @param[in] state  Vehicle state to show while braking.
 */
static void
nav_brake_then (uint32_t phase, uint32_t state)
{
    if (g_ctx.b_motion_busy)
    {
        (void) motion_brake();
        g_ctx.after_brake = phase;
        g_ctx.phase       = P_BRAKING;
        nav_set_state(state);
    }
    else
    {
        nav_enter(phase);
    }
}

/*!
 * @brief Stop the mission: brake, drop every pending input, and hold.
 *
 * @param[in] state   NAV_STATE_STOPPED or NAV_STATE_FAULT.
 * @param[in] reason  NAV_REASON_.
 */
static void
nav_halt (uint32_t state, uint32_t reason)
{
    motion_set_speed_cap(0u);
    g_reason         = reason;
    g_ctx.b_barcode  = false;
    g_ctx.b_junction = false;
    g_ctx.b_obstacle = false;
    g_ctx.b_plan     = false;
    nav_set_state(state);

    if (g_ctx.b_motion_busy)
    {
        (void) motion_brake();
        g_ctx.phase = P_HALTING;
    }
    else
    {
        g_ctx.phase = P_HALTED;
    }
}

/*!
 * @brief Publish a new vehicle state (one store), and log it in test
 *        builds.
 */
static void
nav_set_state (uint32_t state)
{
#if APP_TEST
    static char const * const p_names[] =
    {
        "IDLE", "LINE_FOLLOW", "JUNCTION", "OBSTACLE", "BYPASS",
        "STOPPED", "FAULT", "?"
    };

    if (state != g_state)
    {
        tm_printf((UB *) "[NAV] state %s (reason %lu, faults 0x%02lx)\n",
                  p_names[state & FIELD_MASK(NAV_STATE_BITS)], g_reason,
                  g_faults);
    }
#endif

    g_state = state;
}

/*!
 * @brief Keep a line-following straight running (sent only if Motion is
 *        free; the speed cap and turn level act on it as it runs).
 */
static void
nav_drive_line (void)
{
    if (!g_ctx.b_motion_busy)
    {
        nav_send_straight(NAV_CRUISE_MM_S, MOTION_DISTANCE_MAX_MM);
    }
}

/*!
 * @brief Send a forward straight.
 */
static void
nav_send_straight (uint32_t speed_mm_s, uint32_t distance_mm)
{
    motion_cmd_t const cmd = { MOTION_CMD_STRAIGHT, MOTION_DIR_FORWARDS,
                               speed_mm_s, distance_mm };

    nav_send(&cmd);
}

/*!
 * @brief Send a pivot; StopTurn from Line Following or Obstacle Scan ends
 *        it.
 */
static void
nav_send_pivot (uint32_t direction)
{
    motion_cmd_t const cmd = { MOTION_CMD_PIVOT, direction, 0u, 0u };

    nav_send(&cmd);
}

/*!
 * @brief Send a command to Motion.  The callers send only when Motion is
 *        free, so the command buffer always has room.
 */
static void
nav_send (motion_cmd_t const * p_cmd)
{
    if (E_OK == motion_send(p_cmd, TMO_POL))
    {
        g_ctx.b_motion_busy = true;
    }
    else
    {
        nav_halt(NAV_STATE_FAULT, NAV_REASON_MOTION);
    }
}

/*!
 * @brief Set one input event bit.
 */
static ER
nav_post (uint32_t event)
{
    return (tk_set_flg(g_flg_id, event));
}

/*** end of file ***/

