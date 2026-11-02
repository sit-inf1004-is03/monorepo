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
#define BC_MIN_CONTRAST     (250U)    /* ADC counts black above white */
#define BC_HYST_PCT         (10U)     /* hysteresis around the midpoint */
#define BC_SWING_PCT        (12U)     /* in a pass: rise / dip of this % of
                                       * the white-black span = an edge;
                                       * lower = more sensitive */
#define BC_MIN_SWING        (150U)    /* ...but at least this many counts */
#define BC_GLITCH_MS        (3U)      /* shorter pulses are noise */
#define BC_END_GAP_MS       (1000U)   /* no edge this long = pass over */
#define BC_MAX_ELEMENTS     (49U)     /* up to *XYZ* (5 characters) */
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

/* ===== prototypes ===================================================== */
/* Line following. */
ER           line_init(void);
void         line_task(INT stacd, void * exinf);
line_state_t line_get_state(void);

/* Barcode. */
ER           barcode_init(void);
void         barcode_task(INT stacd, void * exinf);
const char * barcode_cmd_name(barcode_cmd_t cmd);

#endif /* LINE_BARCODE_H */
