/** @file drv_ultrasonic.c
 *
 * @brief HC-SR04 ultrasonic driver; see drv_ultrasonic.h.
 *
 * Minimum-bit changes from the Week 6 version:
 *  - ISR -> task: rise time, fall time and a captured flag (96 bits) are
 *    replaced by ONE 16-bit word written by the handler in a single store
 *    (bit 15 captured, bits 14..0 width in us) plus one bool.  The rise
 *    time is private to the handler.
 *  - Task -> tasks: a float with -1.0f for "no echo" is replaced by the
 *    packed 16-bit reading (9-bit distance + no-echo + silent bits).
 *  - Integer distance maths: the RP2040 has no FPU.
 *
 * Coding standard: BARR-C:2018.
 */

#include <stdbool.h>
#include <stdint.h>

#include "drv_ultrasonic.h"
#include "../gpio_irq.h"

// distance_cm = width_us x 0.0343 / 2 = width_us x 343 / 20000
//
#define CM_PER_US_NUM             (343u)
#define CM_PER_US_DEN             (20000u)

#define ECHO_TIMEOUT_US           (30000u)
#define MIN_TRIGGER_INTERVAL_US   (60000u)  // HC-SR04 settle time.
#define TRIG_PULSE_US             (10u)
#define ECHO_POLL_MS              (1)

// Shortest real echo: the HC-SR04 cannot measure under ~2 cm (~116 us).
// Shorter pulses are the TRIG pulse coupling onto a floating ECHO line
// (sensor unpowered or a loose wire), so they count as silence.
//
#define ECHO_MIN_WIDTH_US         (100u)

// Handler -> task echo word.
//
#define ECHO_CAPTURED_BIT         (0x8000u)
#define ECHO_WIDTH_MASK           (0x7FFFu)

// Written by the echo handler (one 16-bit store); cleared by the task
// BEFORE it fires TRIG, so the two never write it at the same time.
//
static volatile uint16_t g_echo_word = 0u;

// Set by the handler on a rising edge; cleared by the task before TRIG.
//
static volatile bool gb_rise_seen = false;

// Handler-private: written and read only inside echo_handler().
//
static uint32_t g_isr_rise_us = 0u;

// Task-private.  Full timer width: compared with the 32-bit timer.
//
static uint32_t g_last_trigger_us = 0u;

// Shared task -> tasks.  Single writer: ultrasonic_read().
//
static volatile ultrasonic_reading_t g_last_reading = ULTRASONIC_NO_ECHO_BIT;

static void busy_wait_us(uint32_t duration_us);
static void echo_handler(uint32_t gpio, uint32_t events, uint32_t now_usec);

ER
ultrasonic_init(void)
{
    (void)gpio_set_pin(ULTRASONIC_TRIG_PIN, GPIO_MODE_OUT);
    (void)gpio_set_val(ULTRASONIC_TRIG_PIN, 0u);
    (void)gpio_set_pin(ULTRASONIC_ECHO_PIN, GPIO_MODE_IN);

    g_echo_word  = 0u;
    gb_rise_seen = false;

    return (gpio_irq_register(ULTRASONIC_ECHO_PIN,
                              (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL),
                              echo_handler));
}

ultrasonic_reading_t
ultrasonic_read(void)
{
    ultrasonic_reading_t result       = g_last_reading;
    bool const           b_is_cooling =
        ((0u != g_last_trigger_us) &&
         ((gpio_irq_get_time_usec() - g_last_trigger_us) <
          MIN_TRIGGER_INTERVAL_US));
    uint32_t             start_us;
    uint16_t             echo_snapshot;

    // Called again too soon: hand back the last reading instead of firing
    // another pulse.  Bounds the trigger (and interrupt) rate.
    //
    if (!b_is_cooling)
    {
        g_last_trigger_us = gpio_irq_get_time_usec();
        g_echo_word       = 0u;     // Task writes only BEFORE TRIG.
        gb_rise_seen      = false;

        (void)gpio_set_val(ULTRASONIC_TRIG_PIN, 1u);
        busy_wait_us(TRIG_PULSE_US);
        (void)gpio_set_val(ULTRASONIC_TRIG_PIN, 0u);

        start_us = gpio_irq_get_time_usec();

        for (;;)
        {
            echo_snapshot = g_echo_word;        // One 16-bit load.

            if (0u != (echo_snapshot & ECHO_CAPTURED_BIT))
            {
                if ((echo_snapshot & ECHO_WIDTH_MASK) < ECHO_MIN_WIDTH_US)
                {
                    result = (ULTRASONIC_NO_ECHO_BIT | ULTRASONIC_SILENT_BIT);
                }
                else
                {
                    result = ultrasonic_width_to_cm(echo_snapshot &
                                                    ECHO_WIDTH_MASK);
                }
                break;
            }
            else if ((gpio_irq_get_time_usec() - start_us) > ECHO_TIMEOUT_US)
            {
                // ECHO went high but no echo came back: nothing in range.
                // ECHO never went high: the sensor is silent.
                //
                result = gb_rise_seen ? ULTRASONIC_NO_ECHO_BIT :
                         (ULTRASONIC_NO_ECHO_BIT | ULTRASONIC_SILENT_BIT);
                break;
            }
            else
            {
                (void)tk_dly_tsk(ECHO_POLL_MS);
            }
        }

        g_last_reading = result;                // One 16-bit store.
    }

    return (result);
}

ultrasonic_reading_t
ultrasonic_last_reading(void)
{
    return (g_last_reading);
}

uint16_t
ultrasonic_width_to_cm(uint32_t width_us)
{
    uint32_t dist_cm = ((width_us * CM_PER_US_NUM) + (CM_PER_US_DEN / 2u))
                       / CM_PER_US_DEN;

    if (dist_cm > ULTRASONIC_MAX_CM)
    {
        dist_cm = ULTRASONIC_MAX_CM;            // Clamp, never wrap.
    }

    return ((uint16_t)dist_cm);
}

/*!
 * @brief  Busy-wait a few microseconds (TRIG pulse only).
 *
 * WARNING: Spins the CPU.  Never use it for task-level delays.
 */
static void
busy_wait_us(uint32_t duration_us)
{
    uint32_t const start_us = gpio_irq_get_time_usec();

    while ((gpio_irq_get_time_usec() - start_us) < duration_us)
    {
        // Intentionally empty: waiting for the timer to advance.
    }
}

/*!
 * @brief  ECHO edge handler, called by the shared dispatcher in interrupt
 *         context.  Short and non-blocking (gpio_irq.h rules).
 */
static void
echo_handler(uint32_t gpio, uint32_t events, uint32_t now_usec)
{
    uint32_t width_us;

    (void)gpio;

    if (0u != (events & GPIO_IRQ_EDGE_RISE))
    {
        g_isr_rise_us = now_usec;
        gb_rise_seen  = true;
    }

    if (0u != (events & GPIO_IRQ_EDGE_FALL))
    {
        width_us = now_usec - g_isr_rise_us;

        if (width_us > ECHO_WIDTH_MASK)
        {
            width_us = ECHO_WIDTH_MASK;         // Clamp, never wrap.
        }

        g_echo_word = (uint16_t)(ECHO_CAPTURED_BIT | width_us);
    }
}

/*** end of file ***/
