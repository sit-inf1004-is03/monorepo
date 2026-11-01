/**
 * @file  barcode.c
 * @brief Code 39 barcode reader. Shared decoder + two selectable front-ends.
 *
 * Coded to BARR-C:2018.
 *
 * Decode (unchanged for both front-ends):
 *   Code 39 uses 9 bars per character, exactly 3 of them wide. Of the 9
 *   measured widths the 3 widest are "wide" (1), the rest narrow (0).
 *   A full barcode is '*' (start), one data letter, '*' (stop). The data
 *   letter maps to a command: A=Left B=Right C=Straight D=U-Turn. We keep
 *   forward and reversed patterns so either scan direction decodes.
 *
 * Front-ends (chosen by BARCODE_ANALOG in line_barcode.h):
 *   ANALOG  (1): a task samples GP28/ADC2 through the kernel "adc" device,
 *                decides black/white in software with an adaptive threshold
 *                + hysteresis, and times each run.
 *   DIGITAL (0): the IR module's comparator drives GP4; a shared-dispatcher
 *                edge interrupt times each bar.
 * Either way the 9 widths land in g_widths[] and feed the same decoder.
 *
 * NOTE: this file deliberately does NOT include <string.h>.  The kernel's
 * syslib.h typedefs size_t as SZ, which conflicts with newlib's <string.h>.
 * We compare 9-char patterns with a local pattern_eq() instead of strcmp().
 */
#include "line_barcode.h"          /* kernel/app headers FIRST (via app.h) */
#include "../gpio_irq.h"           /* gpio_irq_get_time_usec(), register */

#include <stdint.h>
#include <stdbool.h>

#define BARS_PER_CHAR   (9U)
#define WIDE_COUNT      (3U)
#define WIDTH_MAX       (0xFFFFU)
#define PATTERN_LEN     (BARS_PER_CHAR + 1U)
#define CHARS_PER_FRAME (3U)
#define ADC_FULL_SCALE  (4095U)

/* ---- shared capture + frame state ----------------------------------- */
static volatile uint16_t g_widths[BARS_PER_CHAR];
static volatile uint8_t  g_bar_index  = 0U;
static volatile bool     g_char_ready = false;
static uint8_t           g_char_count = 0U;
static barcode_cmd_t     g_pending    = CMD_NONE;

/*
 * Code 39 lookup. '1' = wide, '0' = narrow. forward / reverse patterns.
 * VERIFY against the actual printed barcodes before track use.
 */
typedef struct
{
    const char *  p_forward;
    const char *  p_reverse;
    char          letter;
} code39_char_t;

static const code39_char_t g_chars[] =
{
    { "010010100", "001010010", '*' },   /* start/stop delimiter */
    { "100001001", "100100001", 'A' },
    { "001001001", "100100100", 'B' },
    { "101001000", "000100101", 'C' },
    { "000011001", "100110000", 'D' }
};

#define CHARS_LEN  (sizeof(g_chars) / sizeof(g_chars[0]))

static barcode_cmd_t letter_to_cmd(char letter)
{
    barcode_cmd_t cmd;
    switch (letter)
    {
        case 'A':  cmd = CMD_A_LEFT;     break;
        case 'B':  cmd = CMD_B_RIGHT;    break;
        case 'C':  cmd = CMD_C_STRAIGHT; break;
        case 'D':  cmd = CMD_D_UTURN;    break;
        default:   cmd = CMD_NONE;       break;
    }
    return cmd;
}

/* Store one bar width; mark the character ready on the 9th. Shared by both
 * front-ends.  Returns true exactly when this width completes a character. */
static bool bc_store_width(uint32_t dt)
{
    bool completed = false;

    if (g_bar_index < BARS_PER_CHAR)
    {
        g_widths[g_bar_index] = (dt > WIDTH_MAX) ? (uint16_t)WIDTH_MAX
                                                 : (uint16_t)dt;
        g_bar_index++;

        if (g_bar_index == BARS_PER_CHAR)
        {
            g_char_ready = true;
            completed    = true;
        }
    }

    return completed;
}

/* Mark the 3 widest widths as '1', the rest '0'. */
static void classify(const uint16_t * p_widths, char * p_out)
{
    bool wide[BARS_PER_CHAR];

    for (uint32_t i = 0U; i < BARS_PER_CHAR; i++)
    {
        wide[i] = false;
    }

    for (uint32_t k = 0U; k < WIDE_COUNT; k++)
    {
        int32_t  best     = -1;
        uint16_t best_val = 0U;

        for (uint32_t i = 0U; i < BARS_PER_CHAR; i++)
        {
            if ((!wide[i]) && (p_widths[i] >= best_val))
            {
                best_val = p_widths[i];
                best     = (int32_t)i;
            }
        }

        if (best >= 0)
        {
            wide[best] = true;
        }
    }

    for (uint32_t i = 0U; i < BARS_PER_CHAR; i++)
    {
        p_out[i] = wide[i] ? '1' : '0';
    }
    p_out[BARS_PER_CHAR] = '\0';
}

/* Compare two 9-char patterns. Returns true if equal (replaces strcmp,
 * so this file needs no <string.h>). */
static bool pattern_eq(const char * a, const char * b)
{
    for (uint32_t i = 0U; i < BARS_PER_CHAR; i++)
    {
        if (a[i] != b[i])
        {
            return false;
        }
    }
    return true;
}

static char pattern_to_letter(const char * p_pattern)
{
    char letter = INVALID_PATTERN;

    for (uint32_t i = 0U; i < CHARS_LEN; i++)
    {
        if (pattern_eq(p_pattern, g_chars[i].p_forward) ||
            pattern_eq(p_pattern, g_chars[i].p_reverse))
        {
            letter = g_chars[i].letter;
            break;
        }
    }
    return letter;
}

static void frame_reset(void)
{
    g_char_count = 0U;
    g_pending    = CMD_NONE;
}

/* Decoder step: snapshot 9 widths, classify, advance the *-frame. Returns
 * true with *p_cmd set when a full '*' data '*' frame has decoded. */
bool barcode_poll(barcode_cmd_t * p_cmd)
{
    if (!g_char_ready)
    {
        return false;
    }

    uint16_t snapshot[BARS_PER_CHAR];
    for (uint32_t i = 0U; i < BARS_PER_CHAR; i++)
    {
        snapshot[i] = g_widths[i];
    }

    char pattern[PATTERN_LEN];
    classify(snapshot, pattern);
    char letter = pattern_to_letter(pattern);

#if BARCODE_DEBUG
    /* %c may not be in every tm_printf; use %d (int)letter if needed. */
    tm_printf((UB*)"  pattern=%s letter=%c (char %u of frame)\n",
              pattern, letter, (unsigned)(g_char_count + 1U));
#endif

    /* Re-arm capture for the next character. */
    g_bar_index  = 0U;
    g_char_ready = false;

    bool have_command = false;
    g_char_count++;

    if (g_char_count == 1U)
    {
        if (letter != '*') { frame_reset(); }        /* no start delimiter */
    }
    else if (g_char_count == 2U)
    {
        g_pending = letter_to_cmd(letter);
        if (g_pending == CMD_NONE) { frame_reset(); }/* unknown data char */
    }
    else /* 3rd char */
    {
        if (letter == '*')
        {
            *p_cmd       = g_pending;
            have_command = true;
        }
        frame_reset();
    }

    return have_command;
}

const char * barcode_cmd_name(barcode_cmd_t cmd)
{
    const char * p_name;
    switch (cmd)
    {
        case CMD_A_LEFT:     p_name = "LEFT";     break;
        case CMD_B_RIGHT:    p_name = "RIGHT";    break;
        case CMD_C_STRAIGHT: p_name = "STRAIGHT"; break;
        case CMD_D_UTURN:    p_name = "U-TURN";   break;
        default:             p_name = "NONE";     break;
    }
    return p_name;
}

/* ===================================================================== *
 *  ANALOG FRONT-END  (BARCODE_ANALOG == 1): sample GP28 via "adc" device
 * ===================================================================== */
#if BARCODE_ANALOG

static ID       g_adc_dd       = -1;            /* "adc" device descriptor  */
static uint16_t g_lo           = ADC_FULL_SCALE; /* adaptive dark envelope  */
static uint16_t g_hi           = 0U;             /* adaptive light envelope */
static bool     g_cur_black    = false;
static uint32_t g_run_start_us = 0U;

/* One synchronous conversion of our channel; returns 0..4095. */
static uint16_t bc_read_adc(void)
{
    UW  val   = 0U;
    SZ  asize = 0;
    ER  er    = tk_srea_dev(g_adc_dd, (W)BC_ADC_CHANNEL, &val, 1, &asize);

    if (er < E_OK)
    {
        return 0U;      /* read failed: treat as dark, next sample retries */
    }
    return (uint16_t)(val & ADC_FULL_SCALE);
}

/*
 * Feed one raw sample through the adaptive comparator. Returns true when a
 * detected transition completes a character's 9 widths.
 */
static bool bc_feed(uint16_t raw)
{
    if (raw < g_lo)                               { g_lo = raw; }
    else if ((uint32_t)g_lo + BC_ENV_RELEASE <= ADC_FULL_SCALE) { g_lo += BC_ENV_RELEASE; }

    if (raw > g_hi)                               { g_hi = raw; }
    else if (g_hi >= BC_ENV_RELEASE)              { g_hi -= BC_ENV_RELEASE; }

    uint16_t span = (g_hi > g_lo) ? (uint16_t)(g_hi - g_lo) : 0U;
    if (span < BC_MIN_CONTRAST)
    {
        return false;           /* quiet zone / not enough contrast yet */
    }

    uint16_t mid = (uint16_t)(((uint32_t)g_lo + g_hi) / 2U);
    uint16_t hys = (uint16_t)(span / 8U);
    bool     is_black = g_cur_black;

#if BC_BLACK_IS_LOW
    if (raw < (uint16_t)(mid - hys))      { is_black = true;  }
    else if (raw > (uint16_t)(mid + hys)) { is_black = false; }
#else
    if (raw > (uint16_t)(mid + hys))      { is_black = true;  }
    else if (raw < (uint16_t)(mid - hys)) { is_black = false; }
#endif

    bool completed = false;
    if (is_black != g_cur_black)                  /* a run just ended */
    {
        uint32_t now = gpio_irq_get_time_usec();
        uint32_t dt  = now - g_run_start_us;      /* = one bar width (us) */
        g_run_start_us = now;
        g_cur_black    = is_black;
        completed      = bc_store_width(dt);
    }
    return completed;
}

ER barcode_init(void)
{
    g_adc_dd = tk_opn_dev((UB*)"adca", TD_READ);   /* driver auto-created at boot */
    if (g_adc_dd < E_OK)
    {
        return (ER)g_adc_dd;
    }

    g_lo = ADC_FULL_SCALE;
    g_hi = 0U;
    g_cur_black    = false;
    g_run_start_us = gpio_irq_get_time_usec();
    g_bar_index    = 0U;
    g_char_ready   = false;
    g_char_count   = 0U;
    g_pending      = CMD_NONE;
    return E_OK;
}

/* Sample until one character completes, then run the decoder once. */
bool barcode_next(barcode_cmd_t * p_cmd)
{
    for (;;)
    {
        if (bc_feed(bc_read_adc()))
        {
            return barcode_poll(p_cmd);
        }
        tk_dly_tsk(BC_SAMPLE_MS);
    }
}

/* ===================================================================== *
 *  DIGITAL FRONT-END  (BARCODE_ANALOG == 0): time edges on GP4
 * ===================================================================== */
#else /* digital */

#define DEBOUNCE_US         (120U)
#define FLG_BARCODE_CHAR    (1U << 0)

static ID                g_barcode_flg  = 0;
static volatile uint32_t g_last_edge_us = 0U;

static void barcode_edge_isr(uint32_t gpio, uint32_t events, uint32_t now_usec)
{
    (void)gpio;
    (void)events;

    uint32_t dt = now_usec - g_last_edge_us;      /* uint32_t: wrap-safe */
    if (dt < DEBOUNCE_US)
    {
        return;
    }
    g_last_edge_us = now_usec;

    if (bc_store_width(dt))                        /* 9th bar: wake the task */
    {
        (void)tk_set_flg(g_barcode_flg, FLG_BARCODE_CHAR);
    }
}

ER barcode_init(void)
{
    T_CFLG cflg = { 0 };
    ER     er;

    er = gpio_set_pin(BARCODE_PIN, GPIO_MODE_IN);
    if (er < E_OK)
    {
        return er;
    }

    cflg.flgatr   = TA_TFIFO | TA_WMUL;
    g_barcode_flg = tk_cre_flg(&cflg);
    if (g_barcode_flg < E_OK)
    {
        return g_barcode_flg;
    }

    g_bar_index    = 0U;
    g_char_ready   = false;
    g_last_edge_us = gpio_irq_get_time_usec();
    g_char_count   = 0U;
    g_pending      = CMD_NONE;

    return gpio_irq_register(BARCODE_PIN,
                             (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL),
                             barcode_edge_isr);
}

bool barcode_next(barcode_cmd_t * p_cmd)
{
    UINT flgptn = 0U;
    (void)tk_wai_flg(g_barcode_flg, FLG_BARCODE_CHAR,
                     (TWF_ANDW | TWF_CLR), &flgptn, TMO_FEVR);
    return barcode_poll(p_cmd);
}

#endif /* BARCODE_ANALOG */

/*** end of file ***/
