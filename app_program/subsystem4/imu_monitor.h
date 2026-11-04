/**
 * @file    imu_monitor.h
 * @brief   One processing step of the IMU statechart (Week 6 Figure 5),
 *          free of kernel and hardware calls so it can be tested on a PC.
 *
 *   CALIBRATING --done--> MONITORING --shock--> IMPACT --Recovered--+
 *                          (motion region || terrain region)         |
 *                              ^-------------------------------------+
 *
 * MONITORING runs the motion and terrain regions in parallel on every
 * sample.  IMPACT sits outside Monitoring: both regions are frozen (an open
 * hump is closed with the peak measured so far) and re-enter at their
 * initial states, STATIONARY and LEVEL, on "Recovered".  The attitude
 * filter keeps running in every state after calibration.
 *
 * The kernel-facing part (I2C, sensor faults, publishing,
 * "Course completed") is in imu_task.c.
 */

#ifndef IMU_MONITOR_H
#define IMU_MONITOR_H

#include <stdint.h>
#include <stdbool.h>

#include "imu_math.h"
#include "drv_imu.h"
#include "imu_filter.h"
#include "imu_odom.h"
#include "imu_motion.h"
#include "imu_terrain.h"
#include "imu_impact.h"
#include "imu_pack.h"

/** Top-level mode inside the running IMU task. */
typedef enum
{
    IMU_MONITOR_CALIBRATING = 0,
    IMU_MONITOR_MONITORING  = 1,
    IMU_MONITOR_IMPACT      = 2
} imu_monitor_mode_t;

/** What happened during one step. */
typedef struct
{
    bool                b_calibrated;       /**< Calibration just ended.  */
    bool                b_calib_forced;     /**< ...without a still car.  */
    bool                b_impact;           /**< New impact (telemetry).  */
    bool                b_recovered;        /**< Impact recovered.        */
    bool                b_motion_changed;
    bool                b_terrain_changed;
    bool                b_hump_closed;      /**< hump holds the result.   */
    bool                b_filter_fault;     /**< Non-finite filter value. */
    imu_motion_class_t  motion_class;
    imu_terrain_state_t terrain_state;
    imu_terrain_hump_t  hump;
    float32_t           impact_peak_g;
    float32_t           distance_mm;
} imu_monitor_output_t;

/** Monitor state.  Treat as opaque outside imu_monitor.c. */
typedef struct
{
    imu_filter_t       filter;
    imu_odom_t         odom;
    imu_motion_t       motion;
    imu_terrain_t      terrain;
    imu_impact_t       impact;
    imu_monitor_mode_t mode;
} imu_monitor_t;

void imu_monitor_init (imu_monitor_t * p_monitor);
void imu_monitor_recalibrate (imu_monitor_t * p_monitor);
bool imu_monitor_resume (imu_monitor_t * p_monitor);
bool imu_monitor_close_hump (imu_monitor_t * p_monitor,
                             imu_terrain_hump_t * p_hump);
void imu_monitor_step (imu_monitor_t * p_monitor,
                       drv_imu_sample_t const * p_raw,
                       imu_odom_counts_t const * p_counts, float32_t dt_s,
                       imu_monitor_output_t * p_out);
void imu_monitor_fill_publish (imu_monitor_t const * p_monitor,
                               imu_pack_snapshot_t * p_snap,
                               imu_pack_aux_t * p_aux);
imu_monitor_mode_t imu_monitor_get_mode (imu_monitor_t const * p_monitor);
imu_filter_attitude_t const *
     imu_monitor_get_attitude (imu_monitor_t const * p_monitor);
float32_t imu_monitor_get_run_peak_mm (imu_monitor_t const * p_monitor);
uint32_t  imu_monitor_get_hump_count (imu_monitor_t const * p_monitor);

#endif /* IMU_MONITOR_H */

/*** end of file ***/
