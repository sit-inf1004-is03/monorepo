/**
 * @file  line_barcode.h
 * @brief Buddy 3: IR line following + Code 39 barcode interface (Wing).
 *        Coded to BARR-C:2018.
 *
 * Front-end switches (see below): BARCODE_ANALOG, LINE_SELF_DRIVE, debug.
 *
 * Pins:
 *   LINE_C  = GP26 / ADC0  analogue  (centre)   -- ADC "adca" channel 0
 *   LINE_R  = GP27 / ADC1  analogue  (right)    -- ADC "adca" channel 1
 *   BARCODE = GP28 / ADC2  analogue  (if BARCODE_ANALOG 1)  -- channel 2
 *          or GP4          digital   (if BARCODE_ANALOG 0)
 *   -> enable each ADC channel you use in
 *      device/adc/sysdepend/rp2040/adc_cnf_sysdep.h:
 *      DEVCONF_ADC_PIN_INIT_0 (GP26), _1 (GP27), _2 (GP28) = TRUE.
 */
#ifndef LINE_BARCODE_H
#define LINE_BARCODE_H

#include <stdint.h>
#include <stdbool.h>

#include "../app.h"          /* ID, ER, INT and the kernel/bsp API */

/* ===== build switches ================================================= */
#define LINE_SELF_DRIVE     (0)   /* 1 = line task drives the car itself
                                   * (bring-up).  SET TO 0 once Navigation
                                   * issues the straights, or they fight.  */
#define LINE_DEBUG          (0)   /* 1 = print line readings to serial     */
#define BARCODE_ANALOG      (1)   /* 1 = analog GP28, 0 = digital GP4       */
#define BARCODE_DEBUG       (1)   /* 1 = print barcode patterns/commands   */

/* ===== barcode tunables (analog GP28, LOCAL) -- calibrate on the mat == */
#define BC_ADC_CHANNEL      (2U)      /* GP28 = ADC2, Grove port 7 */
#define BC_BAR_IS_HIGH      (1)       /* 1 = higher ADC over black (as the
                                       * line sensors); 0 if the [BC] ao
                                       * print shows the opposite */
#define BC_SAMPLE_MS        (1)       /* ~1 kHz sampling */
#define BC_MIN_CONTRAST     (150U)    /* ADC counts black above white (was
                                       * 250; 150 + MIN_SWING 80 still decode
                                       * a 6 mm blurred spot in sim) */
#define BC_HYST_PCT         (10U)     /* hysteresis around the midpoint */
#define BC_SWING_PCT        (12U)     /* in a pass: rise / dip of this % of
                                       * the white-black span = an edge;
                                       * lower = more sensitive */
#define BC_MIN_SWING        (80U)     /* ...but at least this many counts */
#define BC_GLITCH_MS        (3U)      /* shorter pulses are noise */
#define BC_END_GAP_MS       (1000U)   /* no edge this long = pass over */
#define BC_MAX_ELEMENTS     (49U)     /* up to *XYZ* (5 characters) */
#define BC_WIDTH_BY_DISTANCE (1)      /* LOCAL: 1 = bar widths from wheel
                                       * travel (L+R encoder counts), so
                                       * the car's jerks, slow-downs and
                                       * stops on a bar do not stretch it;
                                       * 0 = widths from time only */
#define BC_MIN_PASS_COUNTS  (60U)     /* LOCAL: a pass with less travel
                                       * (L+R counts, ~10 mm) was swiped by
                                       * hand: use its time widths */
#define BC_HOLD_MIN_ELEMENTS (3U)     /* LOCAL: barcode_is_scanning(): a
                                       * pass with this many bars / gaps */
#define BC_HOLD_EDGE_MS     (400U)    /* LOCAL: ...and an edge this recent
                                       * (bars come every few mm; 400 ms =
                                       * 16 mm at 40 mm/s) */
#define BC_BEEP             (1)       /* 1 = beep on a read (GP22) */
#define BC_BUZZER_PIN       (22U)     /* Robo Pico buzzer, PWM slice 3 */
#define BC_BEEP_MS          (80U)     /* each beep; A = 1 ... D = 4 */

/* ===== line state / nav command enums ================================= */
typedef enum
{
    LINE_ON        = 0,
    LINE_LOST      = 1,
    LINE_SEARCHING = 2
} line_state_t;

typedef enum
{
    CMD_A_LEFT     = 0,
    CMD_B_RIGHT    = 1,
    CMD_C_STRAIGHT = 2,
    CMD_D_UTURN    = 3,
    CMD_NONE       = 4
} barcode_cmd_t;

#define FAULT_BIT_LINE_LOST  (1U << 2U)
#define INVALID_PATTERN      ('#')

/* ===== last barcode read (IF-18) ======================================
 * barcode_get_last() returns one 32-bit word, written by barcode_task
 * with a single store on every successful read, so a reader on either
 * core always sees a matching letter and count:
 *   bits  0-7   letter read, ASCII ('A'...); 0 = none since power-on
 *   bits  8-15  reads since power-on, wraps at 256: a change means a new
 *               read, even of the same letter
 *   bits 16-31  0
 * Any task may call it at any time; it never waits.                     */
#define BC_LAST_LETTER_MASK  (0xFFU)
#define BC_LAST_READS_SHIFT  (8U)
#define BC_LAST_READS_MASK   (0xFFU)

/* ===== prototypes ===================================================== */
/* Line following. */
ER           line_init(void);
void         line_task(INT stacd, void * exinf);
line_state_t line_get_state(void);

/* Barcode. */
ER           barcode_init(void);
void         barcode_task(INT stacd, void * exinf);
const char * barcode_cmd_name(barcode_cmd_t cmd);
uint32_t     barcode_get_last(void);                        /* IF-18 */
void         barcode_beep(uint8_t count, uint16_t on_ms);   /* LOCAL */
bool         barcode_is_scanning(void);                     /* LOCAL */

/* LOCAL: barcode map, set from the dashboard (protocol.h CFG_BARCODE_*):
 * what each character '0'-'9', 'A'-'Z' means at the next T.  Defaults
 * A-D = LEFT, RIGHT, STRAIGHT, U-TURN; others ignored (CMD_NONE).
 * barcode_set_cmd() and barcode_clear_cmds(): telemetry task only. */
bool          barcode_set_cmd(char letter, barcode_cmd_t cmd);
barcode_cmd_t barcode_get_cmd(char letter);
void          barcode_clear_cmds(void);
#define BC_BEEP_T_MS        (500U)    /* LOCAL: one long beep = T reached
                                       * with a barcode command: turning */

#endif /* LINE_BARCODE_H */
