/**
 * @file    imu_odom.h
 * @brief   Distance travelled per sample, from Motion Control's encoder
 *          words, for the hump-height estimate.
 *
 * Interface: Buddy 2's encoder driver, IF-06 (subsystem2/drv_encoder.h).
 * Resolved against Buddy 2's shipped getter on 2026-11-03 (supersedes the
 * Tribunal 1 2.3/2.4 raw-pointer proposal):
 *   - imu_task reads each wheel with drv_encoder_get_count(DRV_ENCODER_*),
 *     a 32-bit cumulative pulse count written only by Buddy 2's ISR.  The
 *     IMU reads it, never writes it.
 *   - The count is 32 bits wide and only wraps at 2^32 (~1380 km at
 *     0.33 mm/pulse), so the per-sample modulo difference is always exact.
 *   - Buddy 2 never resets the counter between moves.
 *   - IMU_ODOM_MM_PER_PULSE = 210 mm wheel circumference / 637 pulses per
 *     wheel revolution (Buddy 2 S2-M01, IF-06) = 0.330 mm/pulse.
 *   - Single-channel encoders count up in both directions, so reversing
 *     over a hump would be integrated as forward travel.  The car only
 *     reverses during an obstacle "reverse and reattempt", on flat ground.
 *
 * If no encoder count is available (host tests, or a bench build without
 * Buddy 2), distance falls back to IMU_ODOM_FALLBACK_MM_PER_S while the IMU
 * sees the car moving, and the snapshot's "no encoder" bit is set.
 */

#ifndef IMU_ODOM_H
#define IMU_ODOM_H

#include <stdint.h>
#include <stdbool.h>

#include "imu_math.h"

#define IMU_ODOM_COUNT_MASK         (0xFFFFFFFFu)  /* 32-bit cumulative (IF-06) */
#define IMU_ODOM_MM_PER_PULSE       (0.330f)       /* 210 mm / 637 pulses per rev */
#define IMU_ODOM_MAX_PULSES         (637u)  /* 1 wheel rev / 10 ms; more = glitch */
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
