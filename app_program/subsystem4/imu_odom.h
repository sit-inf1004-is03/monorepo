/**
 * @file    imu_odom.h
 * @brief   Distance travelled per sample, from Motion Control's encoder
 *          words, for the hump-height estimate.
 *
 * Interface agreed in Tribunal 1 (2.3 and 2.4, "Encoder distance for IMU
 * hump height"):
 *   - Each wheel's count lives in its own 32-bit word, written ONLY by the
 *     encoder interrupt (Buddy 2).  The IMU reads it, never writes it.
 *   - The count is 12 bits wide and wraps, so the IMU takes a modulo
 *     difference of the masked values.  Taking the difference every 10 ms
 *     means a wrap is harmless (at most a few pulses per sample).
 *   - Buddy 2 never resets the counter between moves.
 *
 * ASSUMPTIONS (to confirm with Buddy 2, gap G4):
 *   - The count is in the LOW 12 bits of the word (IMU_ODOM_COUNT_MASK).
 *   - IMU_ODOM_MM_PER_PULSE: 210 mm wheel circumference / 20 disc slots.
 *   - Single-channel encoders count up in both directions, so reversing
 *     over a hump would be integrated as forward travel.  The car only
 *     reverses during an obstacle "reverse and reattempt", on flat ground.
 *
 * If Motion Control has not registered its words (e.g. bench tests before
 * integration), distance falls back to IMU_ODOM_FALLBACK_MM_PER_S while the
 * IMU sees the car moving, and the snapshot's "no encoder" bit is set.
 */

#ifndef IMU_ODOM_H
#define IMU_ODOM_H

#include <stdint.h>
#include <stdbool.h>

#include "imu_math.h"

#define IMU_ODOM_COUNT_MASK         (0x00000FFFu)
#define IMU_ODOM_MM_PER_PULSE       (10.5f)
#define IMU_ODOM_MAX_PULSES         (20u)   /* per 10 ms; more = glitch */
#define IMU_ODOM_IDLE_SAMPLES       (20u)   /* 200 ms without pulses    */
#define IMU_ODOM_FALLBACK_MM_PER_S  (150.0f)
#define IMU_ODOM_SETTLE_SAMPLES     (40u)   /* 400 ms of steady movement */

/** Encoder words read this sample. */
typedef struct
{
    uint32_t left_count;
    uint32_t right_count;
    bool     b_has_left;
    bool     b_has_right;
} imu_odom_counts_t;

/** Odometry state.  Treat as opaque outside imu_odom.c. */
typedef struct
{
    uint32_t last_left;
    uint32_t last_right;
    uint32_t idle_samples;
    uint32_t moving_samples;
    uint32_t glitch_count;
    bool     b_primed;
    bool     b_moving;
    bool     b_using_fallback;
} imu_odom_t;

void      imu_odom_init (imu_odom_t * p_odom);
uint32_t  imu_odom_delta (uint32_t now_count, uint32_t last_count);
float32_t imu_odom_update (imu_odom_t * p_odom,
                           imu_odom_counts_t const * p_counts,
                           float32_t dt_s, bool b_vibration_moving);
bool      imu_odom_is_moving (imu_odom_t const * p_odom);
bool      imu_odom_is_settled (imu_odom_t const * p_odom);
bool      imu_odom_is_fallback (imu_odom_t const * p_odom);
uint32_t  imu_odom_glitches (imu_odom_t const * p_odom);

#endif /* IMU_ODOM_H */

/*** end of file ***/
