/**
 * @file  barcode.c
 * @brief Code 39 barcode reader on the analog IR sensor (GP28 / ADC2,
 *        Grove port 7).  Coded to BARR-C:2018.
 *
 * LOCAL (s5-local-test): rewritten from Wing's frame-of-9 reader with the
 * pass-based decoder of the team's bare-SDK sample (barcode.c/.h), moved
 * onto the kernel's shared "adca" device (the line task reads channels 0
 * and 1 of the same device; the driver's spinlock serialises the two).
 *
 * Sampling (barcode_task, every BC_SAMPLE_MS):
 *   Higher ADC = blacker (BC_BAR_IS_HIGH).  A slow average follows the
 *   white floor, a peak that decays slowly follows the black bars.  The
 *   FIRST bar must cross their midpoint (+ hysteresis), so noise on the
 *   plain floor never starts a pass.  Inside a pass, edges are swings
 *   (LOCAL, more sensitive): black -> white when the reading falls
 *   BC_SWING_PCT of the span below the bar's peak, white -> black when
 *   it rises that much above the gap's valley.  A thin gap that the
 *   sensor spot blurs (never back to white) still counts, as long as
 *   the reading dips.  Each element's peak / valley is kept for the
 *   debug print ("[BC] levels").
 *
 * Pass: starts on the first black edge, collects the widths bar, gap,
 *   bar, ... (in 100 us units, 16 bits), and ends when no edge has come
 *   for BC_END_GAP_MS.  Pulses shorter than BC_GLITCH_MS are noise.
 *
 * Decode: Code 39 = 9 elements per character (5 bars, 4 gaps, 3 thick),
 *   one thin gap between characters, so a pass is 10 x chars - 1 long
 *   (29 for "*A*").  First try 2 thick of the 5 bars + 1 thick of the 4
 *   gaps (sorted apart, so a threshold that makes bars look wider than
 *   gaps does no harm); else 3 thick of all 9.  Thick must be >= 1.5 x
 *   thin.  Must be framed by '*'; tried forwards, then reversed.  Thick /
 *   thin is a ratio, so the car's speed does not matter (only that it is
 *   steady across one barcode).
 *
 * Command: first data letter, A = LEFT, B = RIGHT, C = STRAIGHT,
 *   D = U-TURN -> nav_post_barcode() (Navigation acts on it only while
 *   following the line).
 *
 * Beep (BC_BEEP 1): the Robo Pico buzzer on GP22 (PWM slice 3, 2 kHz)
 *   beeps once per letter number, A = 1 ... D = 4; a read with no
 *   command gives one long beep.  Driven from the sampling loop, so
 *   sampling never stops.
 *
 * This file deliberately does NOT include <string.h> (the kernel's
 * syslib.h size_t typedef conflicts with newlib's).
 */
#include "line_barcode.h"           /* kernel/app headers FIRST (app.h) */
#include "../gpio_irq.h"            /* gpio_irq_get_time_usec() */
#include "../navigation/navigation.h"
#include "../comms/comms.h"         /* comms_log(): dashboard log */

#include <stdint.h>
#include <stdbool.h>

#if (0 == BARCODE_ANALOG)
#error "barcode.c (LOCAL) supports only the analog front-end (GP28)."
#endif

#define ADC_FULL_SCALE      (4095U)
#define CHAR_ELEMENTS       (9U)
#define CHAR_STRIDE         (10U)   /* 9 elements + 1 gap between chars */
#define BAR_COUNT           (5U)
#define GAP_COUNT           (4U)
#define THICK_BARS          (2U)
#define THICK_GAPS          (1U)
#define THICK_PER_CHAR      (3U)
#define MIN_RATIO_X10       (12U)   /* thick >= 1.2 x thin (blur) */
#define WIDTH_UNIT_US       (100U)  /* widths stored in 100 us units */
#define WIDTH_MAX           (0xFFFFU)
#define WHITE_SHIFT         (8U)    /* floor average ~256 samples */
#define BLACK_DECAY_SHIFT   (13U)   /* bar peak decays over ~8 s */
#define LEVEL_FRAC_SHIFT    (8U)    /* levels kept x256 */
#define LEARN_SAMPLES       (64U)   /* floor readings at start-up */
#define LEVELS_EVERY        (1000U) /* debug: levels every ~1 s */
#define LOG_LEN             (96U)

/* Buzzer: 1 PWM tick = 1 us (125 MHz / 125), wrap 500 = 2 kHz, 50 %. */
#define BUZZER_SYS_CLK_HZ   (125000000U)
#define BUZZER_CLK_DIV      (BUZZER_SYS_CLK_HZ / 1000000U)
#define BUZZER_WRAP         (BUZZER_TONE_PERIOD_US - 1U)
#define BUZZER_TONE_PERIOD_US (500U)
#define BUZZER_ON_CC        (BUZZER_TONE_PERIOD_US / 2U)
#define BUZZER_PWM_BASE     (0x40050000U)
#define BUZZER_PWM_STRIDE   (0x14U)
#define BUZZER_DIV_OFFSET   (0x04U)
#define BUZZER_DIV_SHIFT    (4U)
#define BUZZER_SLICE        ((BC_BUZZER_PIN >> 1) & 0x07U)
#define BUZZER_DIV_REG      (BUZZER_PWM_BASE + BUZZER_DIV_OFFSET + \
                             (BUZZER_SLICE * BUZZER_PWM_STRIDE))
#define BUZZER_RESET_DONE   (RESETS_RESET + 0x08U)
#define BEEP_LONG_MS        (400U)

/* Code 39: 9 elements, first element = bit 8, 1 = thick. */
typedef struct
{
    char     symbol;
    uint16_t pattern;
} code39_t;

static const code39_t g_code39[] =
{
    {'0', 0x034U}, {'1', 0x121U}, {'2', 0x061U}, {'3', 0x160U},
    {'4', 0x031U}, {'5', 0x130U}, {'6', 0x070U}, {'7', 0x025U},
    {'8', 0x124U}, {'9', 0x064U}, {'A', 0x109U}, {'B', 0x049U},
    {'C', 0x148U}, {'D', 0x019U}, {'E', 0x118U}, {'F', 0x058U},
    {'G', 0x00DU}, {'H', 0x10CU}, {'I', 0x04CU}, {'J', 0x01CU},
    {'K', 0x103U}, {'L', 0x043U}, {'M', 0x142U}, {'N', 0x013U},
    {'O', 0x112U}, {'P', 0x052U}, {'Q', 0x007U}, {'R', 0x106U},
    {'S', 0x046U}, {'T', 0x016U}, {'U', 0x181U}, {'V', 0x0C1U},
    {'W', 0x1C0U}, {'X', 0x091U}, {'Y', 0x190U}, {'Z', 0x0D0U},
    {'-', 0x085U}, {'.', 0x184U}, {' ', 0x0C4U}, {'$', 0x0A8U},
    {'/', 0x0A2U}, {'+', 0x08AU}, {'%', 0x02AU}, {'*', 0x094U}
};

#define CODE39_COUNT  (sizeof(g_code39) / sizeof(g_code39[0]))

/* Sampler + pass state: barcode_task only. */
static ID       g_adc_dd       = -1;
static uint32_t g_white_q      = 0U;    /* floor level x256 */
static uint32_t g_black_q      = 0U;    /* bar level x256 */
static uint16_t g_raw          = 0U;
static bool     g_b_black      = false;

static uint16_t g_widths[BC_MAX_ELEMENTS];
static uint16_t g_count        = 0U;
static bool     g_b_in_pass    = false;
static bool     g_b_last_black = false;
static bool     g_b_too_long   = false;
static uint32_t g_last_edge_us = 0U;
static uint16_t g_glitches     = 0U;
static uint16_t g_tried        = 0U;    /* count at the last early try */
static uint16_t g_extreme      = 0U;    /* peak (bar) / valley (gap) */
static uint16_t g_levels[BC_MAX_ELEMENTS];  /* each element's extreme */

static UB       g_log[LOG_LEN];

/* Beep pattern: halves left (on, off, on ...) and ms left in this half. */
static uint8_t  g_beep_halves  = 0U;
static uint16_t g_beep_ms      = 0U;
static uint16_t g_beep_on_ms   = 0U;

static uint16_t      read_ao(void);
static void          learn_floor(void);
static void          sample(void);
static void          on_edge(bool b_black, uint32_t now_us,
                             uint16_t level);
static void          finish_pass(void);
static bool          mark_widest(uint16_t const * p_w, uint16_t first,
                                 uint16_t step, uint16_t n,
                                 uint16_t count, bool * p_thick);
static char          lookup(bool const * p_thick);
static char          decode_char(uint16_t const * p_w);
static bool          decode(uint16_t const * p_w, uint16_t n,
                            char * p_letter, bool b_reverse);
static barcode_cmd_t letter_to_cmd(char letter);
static void          log_line(void);
static void          buzzer_init(void);
static void          beep_start(uint8_t count, uint16_t on_ms);
static void          beep_tick(void);

/* ===================================================================== *
 *  Sampling
 * ===================================================================== */

/* One conversion of GP28, as "higher = blacker", 0..4095. */
static uint16_t read_ao(void)
{
    UW       val   = 0U;
    SZ       asize = 0;
    uint16_t v     = 0U;
    ER const er    = tk_srea_dev(g_adc_dd, (W)BC_ADC_CHANNEL, &val, 1,
                                 &asize);

    if (er >= E_OK)
    {
        v = (uint16_t)(val & ADC_FULL_SCALE);
    }

#if (1 == BC_BAR_IS_HIGH)
    return (v);
#else
    return ((uint16_t)(ADC_FULL_SCALE - v));
#endif
}

/* Floor level: the sensor should be over white when the task starts. */
static void learn_floor(void)
{
    uint32_t sum = 0U;

    for (uint32_t i = 0U; i < LEARN_SAMPLES; i++)
    {
        sum += read_ao();
        (void)tk_dly_tsk(BC_SAMPLE_MS);
    }

    g_white_q = (sum / LEARN_SAMPLES) << LEVEL_FRAC_SHIFT;
    g_black_q = g_white_q +
                ((uint32_t)BC_MIN_CONTRAST << LEVEL_FRAC_SHIFT);
    g_b_black = false;
}

/* One sample through the adaptive threshold; an edge goes to on_edge(). */
static void sample(void)
{
    uint16_t const v   = read_ao();
    uint32_t const v_q = (uint32_t)v << LEVEL_FRAC_SHIFT;
    uint32_t       white;
    uint32_t       black;
    uint32_t       span;
    uint32_t       mid;
    uint32_t       hyst;
    uint32_t       swing;

    g_raw = v;

    /* Bar level: jump up to any higher reading, then decay slowly back
     * towards the floor, so it follows the paper and the lighting. */
    if (v_q > g_black_q)
    {
        g_black_q = v_q;
    }
    else if (g_black_q > g_white_q)
    {
        g_black_q -= (g_black_q - g_white_q) >> BLACK_DECAY_SHIFT;
    }
    else
    {
        /* At the floor already. */
    }

    white = g_white_q >> LEVEL_FRAC_SHIFT;
    black = g_black_q >> LEVEL_FRAC_SHIFT;
    span  = (black > white) ? (black - white) : 0U;

    if (span < BC_MIN_CONTRAST)
    {
        span = BC_MIN_CONTRAST;     /* Plain floor: ignore small noise. */
    }

    mid   = white + (span / 2U);
    hyst  = (span * BC_HYST_PCT) / 100U;
    swing = (span * BC_SWING_PCT) / 100U;

    if (swing < BC_MIN_SWING)
    {
        swing = BC_MIN_SWING;
    }

    if (!g_b_in_pass)
    {
        /* Plain floor: only a real bar (past the midpoint) starts a pass. */
        if (v > (mid + hyst))
        {
            g_b_black = true;
            g_extreme = v;
            on_edge(true, gpio_irq_get_time_usec(), 0U);
        }
        else
        {
            g_b_black = false;
        }
    }
    else if (g_b_black)
    {
        if (v > g_extreme)
        {
            g_extreme = v;                  /* Bar peak so far. */
        }
        else if ((uint32_t)v + swing < g_extreme)
        {
            g_b_black = false;              /* Fell off the bar. */
            on_edge(false, gpio_irq_get_time_usec(), g_extreme);
            g_extreme = v;
        }
        else
        {
            /* Still on the bar. */
        }
    }
    else
    {
        if (v < g_extreme)
        {
            g_extreme = v;                  /* Gap valley so far. */
        }
        else if (v > ((uint32_t)g_extreme + swing))
        {
            g_b_black = true;               /* Up onto the next bar. */
            on_edge(true, gpio_irq_get_time_usec(), g_extreme);
            g_extreme = v;
        }
        else
        {
            /* Still in the gap. */
        }
    }

    /* Floor level: slow average of the plain floor between barcodes. */
    if ((!g_b_in_pass) && (!g_b_black))
    {
        if (v_q >= g_white_q)
        {
            g_white_q += (v_q - g_white_q) >> WHITE_SHIFT;
        }
        else
        {
            g_white_q -= (g_white_q - v_q) >> WHITE_SHIFT;
        }
    }
}

/* One black <-> white edge: start a pass, or store the width just ended. */
static void on_edge(bool b_black, uint32_t now_us, uint16_t level)
{
    uint32_t dt;

    if (!g_b_in_pass)
    {
        if (b_black)                /* The first bar starts a pass. */
        {
            g_b_in_pass    = true;
            g_tried        = 0U;
            g_count        = 0U;
            g_glitches     = 0U;
            g_b_too_long   = false;
            g_b_last_black = true;
            g_last_edge_us = now_us;
        }
    }
    else if (b_black != g_b_last_black)
    {
        dt = (now_us - g_last_edge_us) / WIDTH_UNIT_US;

        if ((dt * WIDTH_UNIT_US) < (BC_GLITCH_MS * 1000U))
        {
            /* Noise: undo the previous edge and skip this one. */
            g_glitches++;

            if (g_count > 0U)
            {
                g_count--;
                g_last_edge_us -= (uint32_t)g_widths[g_count] *
                                  WIDTH_UNIT_US;
                g_b_last_black  = b_black;
            }
            else
            {
                g_b_in_pass = false;
            }
        }
        else
        {
            if (g_count < BC_MAX_ELEMENTS)
            {
                g_widths[g_count] = (dt > WIDTH_MAX) ? (uint16_t)WIDTH_MAX
                                                     : (uint16_t)dt;
                g_levels[g_count] = level;
                g_count++;
            }
            else
            {
                g_b_too_long = true;
            }

            g_last_edge_us = now_us;
            g_b_last_black = b_black;
        }
    }
    else
    {
        /* Same level again: ignore. */
    }
}

/* ===================================================================== *
 *  Decoding
 * ===================================================================== */

/* Mark the count widest of p_w[first], p_w[first + step], ... (n of
 * them) thick.  False if thick and thin are not clearly different. */
static bool mark_widest(uint16_t const * p_w, uint16_t first,
                        uint16_t step, uint16_t n, uint16_t count,
                        bool * p_thick)
{
    uint16_t const end            = (uint16_t)(first + (n * step));
    uint16_t       thinnest_thick = WIDTH_MAX;
    uint16_t       widest_thin    = 0U;

    for (uint16_t k = 0U; k < count; k++)
    {
        int32_t best = -1;

        for (uint16_t i = first; i < end; i += step)
        {
            if ((!p_thick[i]) &&
                ((best < 0) || (p_w[i] > p_w[best])))
            {
                best = (int32_t)i;
            }
        }

        if (best >= 0)
        {
            p_thick[best] = true;
        }
    }

    for (uint16_t i = first; i < end; i += step)
    {
        if (p_thick[i])
        {
            if (p_w[i] < thinnest_thick)
            {
                thinnest_thick = p_w[i];
            }
        }
        else if (p_w[i] > widest_thin)
        {
            widest_thin = p_w[i];
        }
        else
        {
            /* Narrower thin element. */
        }
    }

    return (((uint32_t)thinnest_thick * 10U) >=
            ((uint32_t)widest_thin * MIN_RATIO_X10));
}

static char lookup(bool const * p_thick)
{
    uint16_t pattern = 0U;
    char     symbol  = INVALID_PATTERN;

    for (uint16_t i = 0U; i < CHAR_ELEMENTS; i++)
    {
        if (p_thick[i])
        {
            pattern |= (uint16_t)(1U << (CHAR_ELEMENTS - 1U - i));
        }
    }

    for (uint32_t i = 0U; i < CODE39_COUNT; i++)
    {
        if (g_code39[i].pattern == pattern)
        {
            symbol = g_code39[i].symbol;
            break;
        }
    }

    return (symbol);
}

/* One 9-element character: 2 of 5 bars + 1 of 4 gaps, else 3 of 9. */
static char decode_char(uint16_t const * p_w)
{
    bool thick[CHAR_ELEMENTS];
    char c = INVALID_PATTERN;

    for (uint16_t i = 0U; i < CHAR_ELEMENTS; i++)
    {
        thick[i] = false;
    }

    if (mark_widest(p_w, 0U, 2U, BAR_COUNT, THICK_BARS, thick) &&
        mark_widest(p_w, 1U, 2U, GAP_COUNT, THICK_GAPS, thick))
    {
        c = lookup(thick);
    }

    if (INVALID_PATTERN == c)
    {
        for (uint16_t i = 0U; i < CHAR_ELEMENTS; i++)
        {
            thick[i] = false;
        }

        if (mark_widest(p_w, 0U, 1U, CHAR_ELEMENTS, THICK_PER_CHAR,
                        thick))
        {
            c = lookup(thick);
        }
    }

    return (c);
}

/* Decode "*X...*" in the given direction; *p_letter = first data char. */
static bool decode(uint16_t const * p_w, uint16_t n, char * p_letter,
                   bool b_reverse)
{
    uint16_t buf[CHAR_ELEMENTS];
    uint16_t chars;
    char     c;
    bool     b_ok = true;

    if ((n < ((3U * CHAR_STRIDE) - 1U)) ||
        (0U != ((n + 1U) % CHAR_STRIDE)))
    {
        b_ok = false;               /* Not 29, 39, 49 ... elements. */
    }
    else
    {
        chars = (uint16_t)((n + 1U) / CHAR_STRIDE);

        for (uint16_t ch = 0U; (ch < chars) && b_ok; ch++)
        {
            for (uint16_t i = 0U; i < CHAR_ELEMENTS; i++)
            {
                uint16_t const k = (uint16_t)((ch * CHAR_STRIDE) + i);

                buf[i] = b_reverse ? p_w[n - 1U - k] : p_w[k];
            }

            c = decode_char(buf);

            if (INVALID_PATTERN == c)
            {
                b_ok = false;
            }
            else if ((0U == ch) || ((chars - 1U) == ch))
            {
                b_ok = ('*' == c);  /* Must be framed by '*'. */
            }
            else if (1U == ch)
            {
                *p_letter = c;
                b_ok      = ('*' != c);
            }
            else
            {
                b_ok = ('*' != c);  /* A later data character. */
            }
        }
    }

    return (b_ok);
}

static barcode_cmd_t letter_to_cmd(char letter)
{
    barcode_cmd_t cmd;

    switch (letter)
    {
        case 'A':
            cmd = CMD_A_LEFT;
            break;

        case 'B':
            cmd = CMD_B_RIGHT;
            break;

        case 'C':
            cmd = CMD_C_STRAIGHT;
            break;

        case 'D':
            cmd = CMD_D_UTURN;
            break;

        default:
            cmd = CMD_NONE;
            break;
    }

    return (cmd);
}

/* A pass has ended: decode it, log it, tell Navigation. */
static void finish_pass(void)
{
    char          letter     = INVALID_PATTERN;
    bool          b_reversed = false;
    bool          b_ok       = false;
    barcode_cmd_t cmd        = CMD_NONE;

    if (!g_b_too_long)
    {
        b_ok = decode(g_widths, g_count, &letter, false);

        if (!b_ok)
        {
            b_ok       = decode(g_widths, g_count, &letter, true);
            b_reversed = b_ok;
        }
    }

    if (b_ok)
    {
        cmd = letter_to_cmd(letter);
        (void)tm_sprintf(g_log, (UB *)"[BC] read *%c*%s -> %s",
                         (INT)letter, b_reversed ? " (reversed)" : "",
                         barcode_cmd_name(cmd));
        log_line();

        if (CMD_NONE != cmd)
        {
            (void)nav_post_barcode((uint32_t)cmd);
            beep_start((uint8_t)((uint8_t)cmd + 1U), BC_BEEP_MS);
        }
        else
        {
            beep_start(1U, BEEP_LONG_MS);   /* Read, but no command. */
        }
    }
    else if (g_count >= CHAR_ELEMENTS)
    {
        /* Worth a look: a barcode-sized pass that did not decode. */
        (void)tm_sprintf(g_log, (UB *)"[BC] no read: %u bars/gaps%s,"
                         " %u glitches", (UINT)g_count,
                         g_b_too_long ? " (too many)" : "",
                         (UINT)g_glitches);
        log_line();
    }
    else
    {
        /* A short blob (the line, a speck): not a barcode. */
    }

#if (1 == BARCODE_DEBUG)
    if (g_count >= CHAR_ELEMENTS)
    {
        tm_printf((UB *)"[BC] widths x100us:");

        for (uint16_t i = 0U; i < g_count; i++)
        {
            tm_printf((UB *)" %u", (UINT)g_widths[i]);
        }

        tm_printf((UB *)"\n");
        tm_printf((UB *)"[BC] levels (bar peak / gap valley):");

        for (uint16_t i = 0U; i < g_count; i++)
        {
            tm_printf((UB *)" %u", (UINT)g_levels[i]);
        }

        tm_printf((UB *)"\n");
    }
#endif

    g_b_in_pass = false;
    g_count     = 0U;
}

static void log_line(void)
{
    (void)comms_log((char const *)g_log);   /* Also prints it. */
}

/* ===================================================================== *
 *  Buzzer (GP22)
 * ===================================================================== */

static void buzzer_init(void)
{
#if (1 == BC_BEEP)
    /* The PWM block may still be in reset (motor / servo init may come
     * later); clearing a cleared bit is harmless. */
    clr_w(RESETS_RESET, RESETS_RESET_PWM);

    while (0U == (in_w(BUZZER_RESET_DONE) & RESETS_RESET_PWM))
    {
        /* Wait for the PWM block to leave reset. */
    }

    (void)pwm_set_pin(BC_BUZZER_PIN);
    out_w(BUZZER_DIV_REG, (BUZZER_CLK_DIV << BUZZER_DIV_SHIFT));
    (void)pwm_set_wrap(BC_BUZZER_PIN, BUZZER_WRAP);
    (void)pwm_set_cc(BC_BUZZER_PIN, 0U);            /* Silent. */
    (void)pwm_set_enabled(BC_BUZZER_PIN, TRUE);
#endif
}

/* count beeps of on_ms each, on_ms apart; does not wait. */
static void beep_start(uint8_t count, uint16_t on_ms)
{
#if (1 == BC_BEEP)
    g_beep_on_ms  = on_ms;
    g_beep_halves = (uint8_t)((count * 2U) - 1U);
    g_beep_ms     = on_ms;
    (void)pwm_set_cc(BC_BUZZER_PIN, BUZZER_ON_CC);
#else
    (void)count;
    (void)on_ms;
#endif
}

/* Called every sample (~1 ms): step the beep pattern. */
static void beep_tick(void)
{
#if (1 == BC_BEEP)
    if (0U != g_beep_halves)
    {
        if (g_beep_ms > (uint16_t)BC_SAMPLE_MS)
        {
            g_beep_ms -= (uint16_t)BC_SAMPLE_MS;
        }
        else
        {
            g_beep_halves--;
            g_beep_ms = g_beep_on_ms;

            /* Odd halves left = a tone is next (on, off, on ...). */
            (void)pwm_set_cc(BC_BUZZER_PIN,
                             (0U != (g_beep_halves & 1U)) ? BUZZER_ON_CC
                                                          : 0U);

            if (0U == g_beep_halves)
            {
                (void)pwm_set_cc(BC_BUZZER_PIN, 0U);
            }
        }
    }
#endif
}

/* ===================================================================== *
 *  Public
 * ===================================================================== */

ER barcode_init(void)
{
    g_adc_dd = tk_opn_dev((UB *)"adca", TD_READ);  /* Shared with line. */

    g_b_in_pass = false;
    g_count     = 0U;
    g_b_black   = false;
    buzzer_init();

    return ((g_adc_dd < E_OK) ? (ER)g_adc_dd : E_OK);
}

/* Sample every BC_SAMPLE_MS; a pass ends after BC_END_GAP_MS without an
 * edge, and is decoded then. */
void barcode_task(INT stacd, void * exinf)
{
#if (1 == BARCODE_DEBUG)
    uint16_t n_levels = 0U;
#endif

    (void)stacd;
    (void)exinf;

    learn_floor();
    beep_start(1U, BC_BEEP_MS);     /* Ready: proves the buzzer works. */

    for (;;)
    {
        sample();
        beep_tick();

        if (g_b_in_pass &&
            ((gpio_irq_get_time_usec() - g_last_edge_us) >
             (BC_END_GAP_MS * 1000U)))
        {
            finish_pass();
        }
        else if (g_b_in_pass && (!g_b_black) && (g_count != g_tried) &&
                 (g_count >= ((3U * CHAR_STRIDE) - 1U)) &&
                 (0U == ((g_count + 1U) % CHAR_STRIDE)))
        {
            /* LOCAL: a bar just ended with a whole number of characters:
             * if it is already a valid "*X*", decode now instead of
             * waiting BC_END_GAP_MS (the T's crossbar may come first). */
            char letter = INVALID_PATTERN;

            g_tried = g_count;

            if (decode(g_widths, g_count, &letter, false) ||
                decode(g_widths, g_count, &letter, true))
            {
                finish_pass();
            }
        }
        else
        {
            /* Pass still running, or none. */
        }

#if (1 == BARCODE_DEBUG)
        n_levels++;

        if (n_levels >= LEVELS_EVERY)
        {
            n_levels = 0U;
            tm_printf((UB *)"[BC] ao=%u white=%u black=%u\n",
                      (UINT)g_raw,
                      (UINT)(g_white_q >> LEVEL_FRAC_SHIFT),
                      (UINT)(g_black_q >> LEVEL_FRAC_SHIFT));
        }
#endif

        (void)tk_dly_tsk(BC_SAMPLE_MS);
    }
}

const char * barcode_cmd_name(barcode_cmd_t cmd)
{
    const char * p_name;

    switch (cmd)
    {
        case CMD_A_LEFT:
            p_name = "LEFT";
            break;

        case CMD_B_RIGHT:
            p_name = "RIGHT";
            break;

        case CMD_C_STRAIGHT:
            p_name = "STRAIGHT";
            break;

        case CMD_D_UTURN:
            p_name = "U-TURN";
            break;

        default:
            p_name = "NONE";
            break;
    }

    return (p_name);
}

/*** end of file ***/
