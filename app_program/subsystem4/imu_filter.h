/**
 * @file    imu_filter.h
 * @brief   Sensor calibration and complementary-filter attitude estimate.
 *
 * Body frame used everywhere in the IMU subsystem (right-handed):
 *   +X forward, +Y to the left, +Z up.
 *   pitch   > 0  nose up          (rotation about -Y)
 *   roll    > 0  left side up     (rotation about +X)
 *   yaw rate > 0  turning left    (rotation about +Z)
 *
 * Pitch uses a complementary filter (Tribunal 1, Buddy 4 "useful AI idea"),
 * so forward acceleration is not mistaken for climbing:
 *   pitch = a * (pitch + rate * dt) + (1 - a) * pitch_from_accel
 * with a = IMU_FILTER_ALPHA, and the accelerometer correction per sample
 * limited to IMU_FILTER_MAX_CORR_DPS (enough to cancel gyro drift, too slow
 * for a short acceleration to tilt the estimate).  The correction is used
 * only when the accelerometer is believable:
 *   - |accel| is within IMU_FILTER_ACCEL_TRUST_G of 1 g (no bump), and
 *   - the accelerometer angle agrees with the gyro-propagated angle within
 *     IMU_FILTER_AGREE_DEG, judged on the disagreement averaged over about
 *     10 samples so vibration noise does not flip the decision.  Forward
 *     acceleration tilts the gravity vector
 *     WITHOUT any rotation, so the two disagree and the gyro carries the
 *     estimate; on a real slope the gyro rotates too and they agree
 *     (host tests H-U28, H-U31).
 * Two ways back from a large error (e.g. the gyro clipped at 500 deg/s):
 *   - STATIONARY: the accelerometer is always trusted (gain
 *     IMU_FILTER_ALPHA_STILL, still rate-limited), so every stop re-levels.
 *   - Re-sync: a steady 1 g accelerometer that disagrees for
 *     IMU_FILTER_RESYNC_SAMPLES (1.5 s) cannot be linear acceleration on
 *     this car (it would mean a 1.5 m/s speed change at 0.1 g), so it is
 *     trusted at up to IMU_FILTER_RESYNC_DPS (host test H-I09).
 *
 * ACCELEROMETER-ONLY MODE (LSM303D / GY-511, no gyroscope; chosen
 * automatically when the sample has b_has_gyro false):
 *   - pitch and roll are the accelerometer angles through a first-order
 *     low-pass (time constant IMU_FILTER_TILT_TAU_S).  This is only right
 *     while the car is stopped or cruising: forward acceleration a tilts
 *     the estimate by about atan(a / 1 g) (0.1 g -> 5.7 deg).  The terrain
 *     region is therefore only allowed to open a hump once the car has
 *     moved steadily for IMU_ODOM_SETTLE_SAMPLES (imu_monitor.c), which
 *     hides the start-up acceleration;
 *   - forward acceleration (lin_fwd_g) is measured against a much slower
 *     pitch reference (IMU_FILTER_SLOW_TAU_S), so short accelerations still
 *     show up for the motion region and impact detection;
 *   - yaw rate and heading come from the tilt-compensated compass instead
 *     of the gyro.  The heading's RATE, not its absolute value, drives
 *     TURNING, so a constant distortion from the motors matters less.  The
 *     hard-iron offsets IMU_MAG_OFFSET_* must still be measured on the car
 *     (spin it a full turn, take (max + min) / 2 per axis; test S4-M01).
 *
 * ASSUMPTION: the car stands still on LEVEL ground during calibration; the
 * mean accelerometer direction then defines zero pitch and zero roll, which
 * also removes any tilt in how the board is mounted.  The mean |a| also
 * becomes the 1 g reference, which removes the sensor's sensitivity error
 * (the GY-511 tested read 0.95 g).
 *
 * The filter state stays in float (internal computation state, Tribunal 1
 * 2.6: "The complementary filter stays as a float").
 */

#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include <stdint.h>
#include <stdbool.h>

#include "imu_math.h"
#include "drv_imu.h"

#define IMU_FILTER_AXES                 (3u)
#define IMU_FILTER_FWD                  (0u)
#define IMU_FILTER_LEFT                 (1u)
#define IMU_FILTER_UP                   (2u)

/*
 * Mounting: body axis <- sign * sensor axis.  Default: sensor X forward,
 * Y left, Z up (chip face up, X arrow to the front).  TODO: confirm on the
 * car with test S4-M01 (tilt by hand) and change only these six lines.
 */
#define IMU_MOUNT_FWD_AXIS              (0u)
#define IMU_MOUNT_FWD_SIGN              (1.0f)
#define IMU_MOUNT_LEFT_AXIS             (1u)
#define IMU_MOUNT_LEFT_SIGN             (1.0f)
#define IMU_MOUNT_UP_AXIS               (2u)
#define IMU_MOUNT_UP_SIGN               (1.0f)

/*
 * Compass hard-iron offsets in raw sensor counts (sensor axes), subtracted
 * before the mounting table.  Measured 28 Oct 2026 on the car (board
 * upside down): (max + min) / 2 of `mag raw` over a flat full turn, X
 * -3490..1308, Y -5067..1878 (testing/subsystem4/logs/
 * 2026-10-28_compass_spin_and_tap.txt).  Z cannot be found from a flat turn
 * (it never points sideways), so it stays 0; on a level car Z only enters
 * the heading through the small pitch and roll terms.
 */
#define IMU_MAG_OFFSET_X                (-1091.0f)
#define IMU_MAG_OFFSET_Y                (-1594.0f)
#define IMU_MAG_OFFSET_Z                (0.0f)

#define IMU_FILTER_ALPHA                (0.98f)
#define IMU_FILTER_ALPHA_STILL          (0.90f)
#define IMU_FILTER_ACCEL_TRUST_G        (0.15f)
#define IMU_FILTER_AGREE_DEG            (3.0f)
#define IMU_FILTER_MAX_CORR_DPS         (2.0f)
#define IMU_FILTER_ERROR_GAIN           (0.1f)    /* ~10-sample average */
#define IMU_FILTER_RESYNC_SAMPLES       (150u)    /* 1.5 s; <= 255 */
#define IMU_FILTER_RESYNC_DPS           (20.0f)
#define IMU_FILTER_CALIB_SAMPLES        (100u)    /* 1 s at 100 Hz */
#define IMU_FILTER_CALIB_MAX_ATTEMPTS   (5u)
#define IMU_FILTER_CALIB_GYRO_MAX_DPS   (15.0f)
/* Calibration stillness: |a| only has to be plausible (the GY-511 tested
 * read 0.87 g upside down), but every sample must stay within
 * IMU_FILTER_CALIB_STEADY_G of the collection's first sample (host tests
 * H-A11, H-A12).
 */
#define IMU_FILTER_CALIB_ACCEL_TOL_G    (0.30f)
#define IMU_FILTER_CALIB_STEADY_G       (0.05f)
#define IMU_FILTER_BIAS_GAIN            (0.005f)
#define IMU_FILTER_BIAS_STILL_DPS       (2.0f)

/* Accelerometer-only mode. */
#define IMU_FILTER_TILT_TAU_S           (0.05f)
#define IMU_FILTER_SLOW_TAU_S           (1.0f)
#define IMU_FILTER_MAG_GAIN             (0.2f)    /* ~5-sample average  */
#define IMU_FILTER_YAW_RATE_GAIN        (0.1f)    /* ~10-sample average */
#define IMU_FILTER_MAG_MIN_COUNTS       (100.0f)  /* weaker = no heading */

/** One sample in body axes and physical units. */
typedef struct
{
    float32_t accel_g[IMU_FILTER_AXES];
    float32_t gyro_dps[IMU_FILTER_AXES];
    float32_t mag[IMU_FILTER_AXES];     /**< Offset-corrected raw counts. */
    bool      b_has_gyro;
    bool      b_has_mag;
} imu_filter_input_t;

/** Attitude and derived quantities after one update. */
typedef struct
{
    float32_t pitch_deg;
    float32_t roll_deg;
    float32_t yaw_rate_dps;
    float32_t heading_deg;      /**< Integrated yaw, 0..360, drifts slowly. */
    float32_t accel_mag_g;      /**< |a|; 1.0 when still.                   */
    float32_t lin_fwd_g;        /**< Forward acceleration, gravity removed. */
    bool      b_accel_trusted;  /**< false while |a| is far from 1 g.       */
} imu_filter_attitude_t;

/** Result of feeding one calibration sample. */
typedef enum
{
    IMU_FILTER_CALIB_BUSY = 0,  /**< Still collecting.                      */
    IMU_FILTER_CALIB_RESTART,   /**< Car moved; collection restarted.       */
    IMU_FILTER_CALIB_DONE,      /**< Calibrated from still samples.         */
    IMU_FILTER_CALIB_FORCED     /**< Max attempts used; calibrated anyway.  */
} imu_filter_calib_t;

/** Filter state.  Treat as opaque outside imu_filter.c. */
typedef struct
{
    float32_t             gyro_bias_dps[IMU_FILTER_AXES];
    float32_t             pitch_offset_deg;
    float32_t             roll_offset_deg;
    float32_t             gyro_sum[IMU_FILTER_AXES];
    float32_t             accel_sum[IMU_FILTER_AXES];
    float32_t             calib_ref_g[IMU_FILTER_AXES]; /**< 1st sample. */
    uint32_t              calib_count;
    uint32_t              calib_attempts;
    float32_t             error_pitch;      /**< Smoothed accel - gyro.   */
    float32_t             error_roll;
    uint8_t               disagree_pitch;   /**< Samples of disagreement. */
    uint8_t               disagree_roll;
    float32_t             one_g;            /**< Calibrated |a| at rest.  */
    float32_t             pitch_slow_deg;   /**< Accel-only reference.    */
    float32_t             mag_filt[IMU_FILTER_AXES];
    float32_t             yaw_prev_deg;
    float32_t             yaw_ref_deg;
    bool                  b_yaw_primed;
    bool                  b_calibrated;
    imu_filter_attitude_t attitude;
} imu_filter_t;

void imu_filter_init (imu_filter_t * p_filter);
void imu_filter_scale_raw (drv_imu_sample_t const * p_raw,
                           imu_filter_input_t * p_input);
imu_filter_calib_t imu_filter_calibrate (imu_filter_t * p_filter,
                                         imu_filter_input_t const * p_input);
bool imu_filter_update (imu_filter_t * p_filter,
                        imu_filter_input_t const * p_input, float32_t dt_s,
                        bool b_still);
void imu_filter_track_bias (imu_filter_t * p_filter,
                            imu_filter_input_t const * p_input);
bool imu_filter_is_calibrated (imu_filter_t const * p_filter);
imu_filter_attitude_t const *
     imu_filter_get_attitude (imu_filter_t const * p_filter);

#endif /* IMU_FILTER_H */

/*** end of file ***/
