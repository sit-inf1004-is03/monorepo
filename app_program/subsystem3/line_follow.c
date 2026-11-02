/**
 * @file  line_follow.c
 * @brief Analogue CENTRE + RIGHT line following -> Motion turn level (Wing).
 *        Coded to BARR-C:2018.
 *
 *   - reads GP26/GP27 through the kernel "adca" device (channels 0 and 1);
 *   - keeps a forward STRAIGHT running (re-issued when Motion goes idle)
 *     while LINE_SELF_DRIVE is set, and steers with motion_set_turn_level;
 *   - per-sensor calibration (white ~175 both; black C~1540, R~3000).
 *
 * Error: e = (r + c - 1), -1..+1.  centred->0, line-right->+, line-left->-.
 * turn level = clamp(KP*e, -7..+7).  +level = steer RIGHT (Glenn's motion).
 * Flip LINE_STEER_SIGN if it steers the wrong way wheels-up.
 */
#include "line_barcode.h"
#include "../subsystem2/motion.h"
#include "../navigation/navigation.h"

#define LINE_C_CHAN     (0U)      /* GP26 centre = ADC0 */
#define LINE_R_CHAN     (1U)      /* GP27 right  = ADC1 */

/* ---- calibration (measured; re-check on the track) ------------------- */
#define WHITE_LEVEL_C   (180)
#define BLACK_LEVEL_C   (3833)
#define WHITE_LEVEL_R   (152)
#define BLACK_LEVEL_R   (3921)
#define LINE_LOST_FRAC  (0.15f)   /* below this on BOTH sensors => lost */
#define LINE_BOTH_BLACK (0.80f)   /* above this on BOTH sensors => junction/
                                   * on-centre: go STRAIGHT, don't hard-turn */

/* ---- control --------------------------------------------------------- */
#define LINE_LOOP_MS    (20)      /* 20 ms loop (NFR1) */
#define LINE_KP         (4.0f)    /* was 8: saturated at e>=0.6 (bang-bang) */
#define LINE_SPEED_MMS  (40u)    /* forward speed (of 700 max) */
#define LINE_STEER_SIGN (-1)      /* flipped: centre-black now steers left */

/* Set to 1 to run CALIBRATION MODE: the task only reads the sensors and
 * prints raw + running min/max, so you can sweep white/black and read the
 * levels.  It does NOT drive the car.  Set back to 0 for normal running. */
#define LINE_CALIBRATE  (0)

/* Trim: the error value when the car sits CENTRED on the line.  If the car
 * drifts one way on a straight, read the "e=" debug while centred and put
 * that number here so centred => 0.  Positive trim cancels a left drift. */
#define LINE_TRIM       (0.0f)

/* Weak-side gain: the centre+right layout corrects one direction weaker
 * than the other.  >1.0 boosts the under-correcting side.  Start at 1.0,
 * raise toward ~2.0 if one direction still corrects too little. */
#define LINE_KP_WEAK    (1.0f)    /* 1.0 = both sides equal; raise if one is weak */

/* ---- smoothing: stops the shake on straights ----------------------- */
#define LINE_KD         (4.0f)    /* damping: resists fast swings; 0 = off */
#define LINE_FILTER     (0.6f)    /* 0..1 error smoothing; lower = smoother */
#define LINE_DEADBAND   (0.10f)  /* |error| below this => drive straight  */
#define LINE_LEVEL_STEP (1)       /* max turn-level change per 20 ms loop  */
#define LINE_LOST_SIDE  (0.20f)   /* |error| at loss above this => that side */
#define LINE_LOST_LEVEL (2)       /* turn level used to swing back to line */
#define LINE_JUNC_SAMPLES (3U)    /* LOCAL: both black this many loops in a
                                   * row after a barcode = the T junction */
#define LINE_JUNC_BLACK (0.65f)   /* LOCAL: "black" for the T check (both
                                   * sensors); lower = easier to trigger */
#define LINE_JUNC_SEEN  (0.35f)   /* LOCAL debug: both sensors above this
                                   * = something across the line; its
                                   * darkest point is printed after */
#define LINE_JUNC_PRINT (10U)     /* LOCAL: after a barcode, print the two
                                   * sensors every 10 loops (200 ms) */

static ID           g_adc_dd     = -1;
static line_state_t g_state      = LINE_SEARCHING;
static int32_t      g_last_level = 0;
static float        g_err_filt   = 0.0f;   /* smoothed error   */
static float        g_err_prev   = 0.0f;   /* last loop's one  */
static bool         g_b_resync   = true;   /* LOCAL: next ON sample has no
                                            * valid previous error: skip D */
static bool         g_b_was_follow = false;  /* LOCAL: hand-back edge */
static uint8_t      g_both_count   = 0U;     /* LOCAL: both-black loops */
static bool         g_b_in_pivot   = false;  /* LOCAL: junction pivot on */
static bool         g_b_junc_hold  = false;  /* LOCAL: T posted: drive
                                              * straight until the pivot */
static bool         g_b_pivot_armed = false; /* LOCAL: white seen in it */

static uint16_t line_read(uint32_t chan)
{
    UW  val   = 0U;
    SZ  asize = 0;
    ER  er    = tk_srea_dev(g_adc_dd, (W)chan, &val, 1, &asize);

    return (er < E_OK) ? 0U : (uint16_t)(val & 0x0FFFU);
}

/* raw -> blackness 0..1 (1 = black), per-sensor calibration. */
static float blackness(uint16_t raw, int32_t white_lvl, int32_t black_lvl)
{
    float f = ((float)white_lvl - (float)raw) /
              ((float)white_lvl - (float)black_lvl);

    if (f < 0.0f)      { f = 0.0f; }
    else if (f > 1.0f) { f = 1.0f; }
    return f;
}

#if LINE_SELF_DRIVE
static void drive_forward(void)
{
    motion_cmd_t cmd;
    cmd.type        = MOTION_CMD_STRAIGHT;
    cmd.direction   = MOTION_DIR_FORWARDS;
    cmd.speed_mm_s  = LINE_SPEED_MMS;
    cmd.distance_mm = MOTION_DISTANCE_MAX_MM;
    (void)motion_send(&cmd, 100);
}
#endif

/* Tell Navigation whether the line is seen (needed to rejoin after a
 * bypass), and steer only while Navigation is following the line, so the
 * held turn level does not bend the bypass straights. */
static void line_output(int32_t level)
{
    static line_state_t s_sent = LINE_SEARCHING;

    if (g_state != s_sent)
    {
        s_sent = g_state;
        (void)nav_set_line_state((LINE_ON == g_state) ? NAV_LINE_ON
                                                      : NAV_LINE_LOST);
    }

#if LINE_SELF_DRIVE
    motion_set_turn_level(LINE_STEER_SIGN * level);
#else
    {
        uint32_t const nav = nav_get_state();

        if (g_b_junc_hold)
        {
            /* LOCAL: creeping over the T's crossbar: no steering, so the
             * pivot starts square to it. */
            motion_set_turn_level(0);
        }
        else if ((NAV_STATE_LINE_FOLLOW == nav) ||
                 (NAV_STATE_JUNCTION == nav))
        {
            motion_set_turn_level(LINE_STEER_SIGN * level);
        }
        else
        {
            motion_set_turn_level(0);
        }
    }
#endif
}

ER line_init(void)
{
    g_adc_dd = tk_opn_dev((UB*)"adca", TD_READ);   /* ADC unit 0 = "adca" */

#if LINE_DEBUG
    tm_printf((UB*)"\n[line] adc open -> dd=%d\n", (int)g_adc_dd);
#endif

    g_state      = LINE_SEARCHING;
    g_last_level = 0;
    return (g_adc_dd < E_OK) ? (ER)g_adc_dd : E_OK;
}

line_state_t line_get_state(void)
{
    return g_state;
}

EXPORT void line_task(INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;

#if LINE_SELF_DRIVE
    drive_forward();
#endif

    for (;;)
    {
#if LINE_CALIBRATE
        {
            static uint16_t c_min = 4095, c_max = 0, r_min = 4095, r_max = 0;
            uint16_t cc = line_read(LINE_C_CHAN);
            uint16_t rr = line_read(LINE_R_CHAN);
            if (cc < c_min) c_min = cc;   if (cc > c_max) c_max = cc;
            if (rr < r_min) r_min = rr;   if (rr > r_max) r_max = rr;
            tm_printf((UB*)"[CAL] C=%4u (min %4u max %4u)  R=%4u (min %4u max %4u)\n",
                      (unsigned)cc, (unsigned)c_min, (unsigned)c_max,
                      (unsigned)rr, (unsigned)r_min, (unsigned)r_max);
            tk_dly_tsk(100);              /* 10 Hz, easy to read */
            continue;                    /* skip driving in calibration mode */
        }
#endif
#if LINE_SELF_DRIVE
        /* Re-issue the straight ONLY if Motion has actually stopped (fault/
         * done) -- NOT on every idle, which made the speed ramp creep up. */
        if (motion_get_activity() == MOTION_ACTIVITY_IDLE)
        {
            static int idle_count = 0;
            if (++idle_count >= 10)           /* ~200 ms genuinely stopped */
            {
                drive_forward();
                idle_count = 0;
            }
        }
        else
        {
            /* reset the idle counter while it's driving */
        }
#endif

        /* LOCAL: Navigation hands steering back (e.g. after a bypass):
         * forget what was read while not steering (the diagonal line
         * crossing), so only what is under the sensors now decides. */
        {
            uint32_t const nav = nav_get_state();
            bool const b_follow = ((NAV_STATE_LINE_FOLLOW == nav) ||
                                   (NAV_STATE_JUNCTION == nav));

            if (b_follow && !g_b_was_follow)
            {
                g_err_filt   = 0.0f;
                g_err_prev   = 0.0f;
                g_last_level = 0;
                g_b_resync   = true;
            }
            g_b_was_follow = b_follow;
        }

        uint16_t raw_c = line_read(LINE_C_CHAN);
        uint16_t raw_r = line_read(LINE_R_CHAN);
        float    c     = blackness(raw_c, WHITE_LEVEL_C, BLACK_LEVEL_C);
        float    r     = blackness(raw_r, WHITE_LEVEL_R, BLACK_LEVEL_R);

        /* LOCAL: barcode junctions.  Navigation is JUNCTION after a
         * barcode.  (1) While it still drives the line, both sensors
         * black for LINE_JUNC_SAMPLES loops = the T's crossbar: report
         * the junction once.  (2) In its turn pivot, StopTurn when a
         * sensor sees the line after having seen white (so not on the
         * line the pivot starts on). */
        {
            bool const b_junc  = (NAV_STATE_JUNCTION == nav_get_state());
            uint32_t const act = motion_get_activity();
            bool const b_on    = ((c >= LINE_LOST_FRAC) ||
                                  (r >= LINE_LOST_FRAC));

            static uint8_t s_print = 0U;

            if (b_junc && (MOTION_ACTIVITY_STRAIGHT == act))
            {
                s_print++;

                if (s_print >= LINE_JUNC_PRINT)
                {
                    s_print = 0U;
                    tm_printf((UB*)"[LINE] waiting for T: c=%d r=%d"
                              " (both > %d)\n", (int)(c * 100.0f),
                              (int)(r * 100.0f),
                              (int)(LINE_JUNC_BLACK * 100.0f));
                }
            }

            /* Debug, always: how dark did BOTH sensors get together? */
            {
                static int32_t s_peak = -1;     /* -1 = nothing across */
                float const    both   = (c < r) ? c : r;

                if (both >= LINE_JUNC_SEEN)
                {
                    if ((int32_t)(both * 100.0f) > s_peak)
                    {
                        s_peak = (int32_t)(both * 100.0f);
                    }
                }
                else if (s_peak >= 0)
                {
                    tm_printf((UB*)"[LINE] crossbar? both sensors reached"
                              " %d (T needs > %d)%s\n", (int)s_peak,
                              (int)(LINE_JUNC_BLACK * 100.0f),
                              b_junc ? "" : " [no barcode: ignored]");
                    s_peak = -1;
                }
                else
                {
                    /* Nothing across the line. */
                }
            }

            if ((c > LINE_JUNC_BLACK) && (r > LINE_JUNC_BLACK))
            {
                if (g_both_count < LINE_JUNC_SAMPLES)
                {
                    g_both_count++;

                    if (LINE_JUNC_SAMPLES == g_both_count)
                    {
                        if (b_junc && (MOTION_ACTIVITY_STRAIGHT == act))
                        {
                            (void)nav_post_junction();
                            g_b_junc_hold = true;
                            tm_printf((UB*)"[LINE] T junction\n");
                        }
                        else
                        {
                            tm_printf((UB*)"[LINE] T seen, no barcode:"
                                      " straight on\n");
                        }
                    }
                }
            }
            else
            {
                g_both_count = 0U;
            }

            if ((!b_junc) || (MOTION_ACTIVITY_PIVOT == act))
            {
                g_b_junc_hold = false;  /* Pivot started, or not a T. */
            }

            if (b_junc && (MOTION_ACTIVITY_PIVOT == act))
            {
                if (!g_b_in_pivot)
                {
                    g_b_in_pivot    = true;
                    g_b_pivot_armed = false;
                }

                if (!b_on)
                {
                    g_b_pivot_armed = true;
                }
                else if (g_b_pivot_armed)
                {
                    (void)motion_stop_turn();
                    g_b_pivot_armed = false;    /* Once per pivot. */
                    g_b_resync      = true;
                    tm_printf((UB*)"[LINE] turn: line found\n");
                }
                else
                {
                    /* Still on the line it started on. */
                }
            }
            else
            {
                g_b_in_pivot = false;
            }
        }

        int32_t level;

        if ((c < LINE_LOST_FRAC) && (r < LINE_LOST_FRAC))
        {
            g_state    = LINE_LOST;
            g_b_resync = true;    /* LOCAL: no D kick when the line returns */

            /* Lost: turn towards the side the line was last seen on.
             * Near the middle (line between the sensors on a straight):
             * drive straight instead of holding an old turn. */
            if (g_err_filt > LINE_LOST_SIDE)
            {
                level = LINE_LOST_LEVEL;
            }
            else if (g_err_filt < -LINE_LOST_SIDE)
            {
                level = -LINE_LOST_LEVEL;
            }
            else
            {
                level = 0;
            }

            if (level > (g_last_level + LINE_LEVEL_STEP))
            {
                level = g_last_level + LINE_LEVEL_STEP;
            }
            else if (level < (g_last_level - LINE_LEVEL_STEP))
            {
                level = g_last_level - LINE_LEVEL_STEP;
            }
            g_last_level = level;
        }
        else
        {
            g_state = LINE_ON;

            /* Both sensors black (junction, or dead-centre on a wide line):
             * treat as straight instead of a hard turn. */
            if ((c > LINE_BOTH_BLACK) && (r > LINE_BOTH_BLACK))
            {
                /* LOCAL: ease back to straight at LINE_LEVEL_STEP per loop
                 * instead of snapping to 0 (that jerked on straights). */
                level = 0;
                if (g_last_level > LINE_LEVEL_STEP)
                {
                    level = g_last_level - LINE_LEVEL_STEP;
                }
                else if (g_last_level < -LINE_LEVEL_STEP)
                {
                    level = g_last_level + LINE_LEVEL_STEP;
                }
                g_last_level = level;
                g_b_resync   = true;  /* LOCAL: no D kick afterwards */
                line_output(level);
#if LINE_DEBUG
                tm_printf((UB*)"C=%4u R=%4u e=%d lvl=%d BOTH-BLK\n",
                          (unsigned)raw_c, (unsigned)raw_r,
                          (int)((r - c) * 100.0f),
                          (int)(LINE_STEER_SIGN * level));
#endif
                tk_dly_tsk(LINE_LOOP_MS);
                continue;
            }

            /* Difference error: centre and right steer OPPOSITE ways.
             *   right black  -> error +  (one direction)
             *   centre black -> error -  (the other direction)
             * This is required because, on your layout, centre-black and
             * right-black must correct in opposite directions. */
            float error = (r - c) - LINE_TRIM;

            /* Asymmetric gain: the weak side (negative error on a
             * centre+right layout) gets a stronger push so both
             * directions correct more evenly. */
            float kp = (error < 0.0f) ? (LINE_KP * LINE_KP_WEAK) : LINE_KP;

            /* 1. Smooth the error: one noisy reading can't jerk the car. */
            g_err_filt += LINE_FILTER * (error - g_err_filt);

            /* LOCAL: first sample after LOST / both-black / hand-back:
             * the previous error is stale, so no derivative this time. */
            if (g_b_resync)
            {
                g_err_prev = g_err_filt;
                g_b_resync = false;
            }

            /* 2. Small errors on a straight: just go straight. */
            float p_err = g_err_filt;
            if ((p_err < LINE_DEADBAND) && (p_err > -LINE_DEADBAND))
            {
                p_err = 0.0f;
            }

            /* 3. P + D: D pushes back against a fast swing (overshoot). */
            float out = (kp * p_err) + (LINE_KD * (g_err_filt - g_err_prev));
            g_err_prev = g_err_filt;
            level = (int32_t)(out + ((out >= 0.0f) ? 0.5f : -0.5f));
            if (level > MOTION_TURN_LEVEL_MAX)       { level = MOTION_TURN_LEVEL_MAX;  }
            else if (level < -MOTION_TURN_LEVEL_MAX) { level = -MOTION_TURN_LEVEL_MAX; }

            /* 4. Change the turn gradually, one step per loop. */
            if (level > (g_last_level + LINE_LEVEL_STEP))
            {
                level = g_last_level + LINE_LEVEL_STEP;
            }
            else if (level < (g_last_level - LINE_LEVEL_STEP))
            {
                level = g_last_level - LINE_LEVEL_STEP;
            }
            g_last_level = level;
        }

        line_output(level);

#if LINE_DEBUG
        tm_printf((UB*)"C=%4u R=%4u e=%d lvl=%d %s\n",
                  (unsigned)raw_c, (unsigned)raw_r,
                  (int)((r - c) * 100.0f),
                  (int)(LINE_STEER_SIGN * level),
                  (g_state == LINE_LOST) ? "LOST" : "on");
#endif

        tk_dly_tsk(LINE_LOOP_MS);
    }
}

/*** end of file ***/