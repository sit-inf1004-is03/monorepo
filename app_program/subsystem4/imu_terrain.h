/**
 * @file    imu_terrain.h
 * @brief   Terrain-classification region of the IMU statechart (Week 6
 *          Figure 5, right region), hump peak-height estimate and tilt
 *          detection.
 *
 *   LEVEL        pitch within the level band
 *   CLIMBING     pitch above +IMU_TERRAIN_CLIMB_DEG while the car moves;
 *                opens a hump measurement
 *   DESCENDING   pitch below -IMU_TERRAIN_CLIMB_DEG, past the crest
 *   HUMP_CLOSED  pitch back within +/-IMU_TERRAIN_LEVEL_DEG; peak finalised
 *                and reported, then the region returns to LEVEL
 * Each transition needs IMU_TERRAIN_PERSIST_SAMPLES consecutive samples.
 *
 * CLIMBING -> HUMP_CLOSED without DESCENDING: on the real car a short hump
 * often never tips the nose down (the nose goes up, then straight back to
 * level as the rear axle follows).  So CLIMBING also closes once pitch has
 * stayed level for IMU_TERRAIN_WHEELBASE_MM of travel (both axles at the
 * same height again), or for IMU_TERRAIN_TOP_LEVEL_SAMPLES when no distance
 * is measured (no encoders, or the car stopped).  The level crest of a
 * rounded hump is shorter than one wheelbase, so a normal hump still goes
 * through DESCENDING (host tests H-I04, H-U33).  The peak is already
 * complete when the front axle reaches the top, so it is not cut short.
 *
 * Peak height (FR3, NFR4 +/-20%) from pitch and encoder distance, with no
 * double integration of acceleration (Week 6 Table 15):
 *
 *   The car touches the ground at a front point and a rear point one
 *   wheelbase (W) apart, so  sin(pitch) = (h_front - h_rear) / W.  The rear
 *   point is where the front point was W millimetres of travel ago, so
 *       h_front(s) = h_front(s - W) + W * sin(pitch(s))
 *   A ring buffer of h_front, indexed by distance in IMU_TERRAIN_BIN_MM bins,
 *   supplies h_front(s - W).  The hump peak is the largest h_front.  The
 *   model runs whenever pitch leaves the +/-IMU_TERRAIN_LEVEL_DEG band, so
 *   the rise before the climb threshold is not lost.
 *
 *   That needs the rear wheels to climb the hump too.  On the course they
 *   do not: the ramp is narrower than the rear wheels, which pass either
 *   side, and only the front ball rides over it (measured 2026-11-04: 7.7
 *   deg peak, no nose-down).  The rear stays at ground level, so
 *       h_front = W * sin(pitch)
 *   with pitch taken as the median of IMU_TERRAIN_PEAK_WINDOW samples so
 *   that a jolt does not set the peak.  IMU_TERRAIN_REAR_CLIMBS selects
 *   between the two.
 *
 * Detection limit (by design): a hump whose steepest slope is below about
 * 6 deg never tilts the car past IMU_TERRAIN_CLIMB_DEG and is treated as
 * level ground.  Host test H-I01 covers 15-120 mm humps with slopes of
 * 6-35 deg at 100-300 mm/s: worst peak error 5.7 % (NFR4 allows 20 %).
 *
 *   Integrating sin(pitch) * ds instead only gives the AVERAGE height over
 *   one wheelbase, which under-reads a hump shorter than about 2 W (by about
 *   18 % for a hump twice the wheelbase long).  Setting
 *   IMU_TERRAIN_WHEELBASE_MM to 0 selects that simpler method.
 *
 * The published peak is 7 bits, 0..127 mm, plus a saturation flag (Tribunal
 * 1, 2.6); IMU_TERRAIN_HEIGHT_GAIN is the calibration factor to be set from
 * the end-to-end hump test (T4) against measured humps.
 */

#ifndef IMU_TERRAIN_H
#define IMU_TERRAIN_H

#include <stdint.h>
#include <stdbool.h>

#include "imu_math.h"

#define IMU_TERRAIN_CLIMB_DEG           (4.0f)
#define IMU_TERRAIN_LEVEL_DEG           (2.0f)
#define IMU_TERRAIN_PERSIST_SAMPLES     (5u)      /* <= 7 (3-bit counter) */
#define IMU_TERRAIN_TOP_LEVEL_SAMPLES   (100u)    /* 1 s; <= 127 (7 bits) */
#define IMU_TERRAIN_MAX_HUMP_MM         (1500.0f) /* travel; then close   */
#define IMU_TERRAIN_WHEELBASE_MM        (75.0f)   /* measured on the car */
#define IMU_TERRAIN_BIN_MM              (5.0f)
#define IMU_TERRAIN_HISTORY_BINS        (48u)     /* wheelbase <= 240 mm  */
#define IMU_TERRAIN_HEIGHT_GAIN         (1.0f)    /* set from test T4     */
#define IMU_TERRAIN_REAR_CLIMBS         (0)       /* 1 = full-width hump  */

/*
 * Rear-on-flat calibration, from three car runs over the measured 10 mm
 * ramp on 2026-11-04 (13.6, 12.1, 12.4 mm): the car reads high at speed on
 * the ramp.  One ramp only; check against a second known height.
 */
#define IMU_TERRAIN_FLAT_GAIN           (0.8f)
#define IMU_TERRAIN_PEAK_WINDOW         (9u)      /* samples: 90 ms       */
#define IMU_TERRAIN_PEAK_MAX_MM         (127u)    /* 7-bit field          */
#define IMU_TERRAIN_TILT_PITCH_DEG      (25.0f)
#define IMU_TERRAIN_TILT_ROLL_DEG       (20.0f)

/** Terrain state, 2 bits. */
typedef enum
{
    IMU_TERRAIN_LEVEL       = 0,
    IMU_TERRAIN_CLIMBING    = 1,
    IMU_TERRAIN_DESCENDING  = 2,
    IMU_TERRAIN_HUMP_CLOSED = 3
} imu_terrain_state_t;

/** Result of one closed hump. */
typedef struct
{
    float32_t peak_mm;          /**< Full-precision estimate (after gain). */
    uint8_t   peak_mm_packed;   /**< Rounded and clamped to 0..127.        */
    bool      b_saturated;      /**< true if the estimate exceeded 127 mm. */
    uint8_t   max_pitch_deg;    /**< Steepest climb, whole degrees, 0..31. */
    bool      b_cut_short;      /**< Closed by distance limit or impact.   */
} imu_terrain_hump_t;

/** Terrain state.  Treat as opaque outside imu_terrain.c. */
typedef struct
{
    float32_t          history_mm[IMU_TERRAIN_HISTORY_BINS];
    float32_t          front_mm;
    float32_t          peak_mm;
    float32_t          max_pitch_deg;
    float32_t          travel_mm;
    float32_t          bin_fill_mm;
    float32_t          run_peak_mm;
    float32_t          level_travel_mm; /**< Level travel while CLIMBING. */
    float32_t          pitch_window[IMU_TERRAIN_PEAK_WINDOW];
    uint32_t           window_head;
    uint32_t           window_count;
    uint32_t           history_head;
    uint32_t           history_bins;
    uint32_t           hump_count;
    uint16_t           state_bits;  /**< [1:0] state [4:2] count
                                         [7:5] tilt count [8] tilted
                                         [15:9] level count (CLIMBING) */
    bool               b_rear_climbs;   /**< Which height model runs.     */
    bool               b_result_ready;
    imu_terrain_hump_t result;
} imu_terrain_t;

void                imu_terrain_init (imu_terrain_t * p_terrain);
void                imu_terrain_reset_region (imu_terrain_t * p_terrain);
bool                imu_terrain_update (imu_terrain_t * p_terrain,
                                        float32_t pitch_deg,
                                        float32_t roll_deg,
                                        float32_t distance_mm,
                                        bool b_moving);
bool                imu_terrain_force_close (imu_terrain_t * p_terrain);
bool                imu_terrain_take_result (imu_terrain_t * p_terrain,
                                             imu_terrain_hump_t * p_hump);
imu_terrain_state_t imu_terrain_get_state (imu_terrain_t const * p_terrain);
bool                imu_terrain_is_tilted (imu_terrain_t const * p_terrain);
float32_t           imu_terrain_get_run_peak_mm (
                        imu_terrain_t const * p_terrain);
uint32_t            imu_terrain_get_hump_count (
                        imu_terrain_t const * p_terrain);

#endif /* IMU_TERRAIN_H */

/*** end of file ***/
