/**
 * @file    imu_monitor.c
 * @brief   One processing step of the IMU statechart (pure C, testable on
 *          a PC through host_test/).
 */

#include <stddef.h>

#include "imu_monitor.h"
#include "../common/bitpack.h"

static void clear_output (imu_monitor_output_t * p_out);
static void enter_monitoring (imu_monitor_t * p_monitor);
static void run_regions (imu_monitor_t * p_monitor,
                         imu_filter_input_t const * p_input,
                         float32_t distance_mm,
                         imu_monitor_output_t * p_out);

/**
 * @brief   Full reset: calibration, regions and run statistics.
 * @param   p_monitor  Monitor state.
 */
void
imu_monitor_init (imu_monitor_t * p_monitor)
{
    if (NULL != p_monitor)
    {
        imu_terrain_init(&p_monitor->terrain);
        imu_monitor_recalibrate(p_monitor);
    }
}

/**
 * @brief   Restart from calibration but keep the run statistics (highest
 *          hump peak, hump count), e.g. after an I2C fault was recovered.
 * @param   p_monitor  Monitor state.
 */
void
imu_monitor_recalibrate (imu_monitor_t * p_monitor)
{
    if (NULL != p_monitor)
    {
        imu_filter_init(&p_monitor->filter);
        imu_odom_init(&p_monitor->odom);
        imu_motion_init(&p_monitor->motion);
        imu_terrain_reset_region(&p_monitor->terrain);
        imu_impact_init(&p_monitor->impact);
        p_monitor->mode = IMU_MONITOR_CALIBRATING;
    }
}

/**
 * @brief   Process one 10 ms sample.
 * @param   p_monitor  Monitor state.
 * @param   p_raw      Raw sensor sample.
 * @param   p_counts   Encoder words read this sample.
 * @param   dt_s       Time since the previous sample, in seconds.
 * @param   p_out      Receives what happened.
 */
void
imu_monitor_step (imu_monitor_t * p_monitor,
                  drv_imu_sample_t const * p_raw,
                  imu_odom_counts_t const * p_counts, float32_t dt_s,
                  imu_monitor_output_t * p_out)
{
    if ((NULL != p_monitor) && (NULL != p_raw) && (NULL != p_counts) &&
        (NULL != p_out))
    {
        imu_filter_input_t input;

        clear_output(p_out);
        imu_filter_scale_raw(p_raw, &input);

        if (IMU_MONITOR_CALIBRATING == p_monitor->mode)
        {
            imu_filter_calib_t const result =
                imu_filter_calibrate(&p_monitor->filter, &input);

            if ((IMU_FILTER_CALIB_DONE == result) ||
                (IMU_FILTER_CALIB_FORCED == result))
            {
                p_out->b_calibrated   = true;
                p_out->b_calib_forced = (IMU_FILTER_CALIB_FORCED == result);
                enter_monitoring(p_monitor);
            }
        }
        else if (!imu_filter_update(&p_monitor->filter, &input, dt_s,
                                    (IMU_MOTION_STATIONARY ==
                                     imu_motion_get_class(&p_monitor->motion))))
        {
            /* Filter re-seeded itself; skip classification this sample. */
            p_out->b_filter_fault = true;
        }
        else
        {
            float32_t const distance = imu_odom_update(
                &p_monitor->odom, p_counts, dt_s,
                imu_motion_is_vibrating(&p_monitor->motion));

            p_out->distance_mm = distance;
            run_regions(p_monitor, &input, distance, p_out);
        }

        p_out->motion_class  = imu_motion_get_class(&p_monitor->motion);
        p_out->terrain_state = imu_terrain_get_state(&p_monitor->terrain);
    }
}

/**
 * @brief   Fill the published fields this module knows (everything except
 *          the IMU-fault copy, which imu_task.c reads from the shared fault
 *          flag).
 * @param   p_monitor  Monitor state.
 * @param   p_snap     Receives the snapshot fields.
 * @param   p_aux      Receives the aux fields.
 */
void
imu_monitor_fill_publish (imu_monitor_t const * p_monitor,
                          imu_pack_snapshot_t * p_snap,
                          imu_pack_aux_t * p_aux)
{
    if ((NULL != p_monitor) && (NULL != p_snap) && (NULL != p_aux))
    {
        imu_filter_attitude_t const * const p_att =
            imu_filter_get_attitude(&p_monitor->filter);
        bool b_saturated = false;

        p_snap->pitch_q          = imu_pack_quantize_angle(p_att->pitch_deg);
        p_snap->peak_mm          = imu_pack_quantize_peak(
            imu_terrain_get_run_peak_mm(&p_monitor->terrain), &b_saturated);
        p_snap->b_peak_saturated = b_saturated;
        /* Casts: the enum constants are 0..3, non-negative. */
        p_snap->motion_class     = (uint32_t)imu_motion_get_class(
                                       &p_monitor->motion);
        p_snap->terrain_state    = (uint32_t)imu_terrain_get_state(
                                       &p_monitor->terrain);
        p_snap->b_imu_fault      = false;
        p_snap->b_tilted         = imu_terrain_is_tilted(&p_monitor->terrain);
        p_snap->b_no_encoder     = imu_odom_is_fallback(&p_monitor->odom);
        p_snap->yaw_rate_q       = imu_pack_quantize_yaw_rate(
                                       p_att->yaw_rate_dps);

        p_aux->roll_q      = imu_pack_quantize_angle(p_att->roll_deg);
        p_aux->heading_deg = imu_pack_quantize_heading(p_att->heading_deg);
        p_aux->hump_count  = bitpack_clamp_u32(
            imu_terrain_get_hump_count(&p_monitor->terrain),
            IMU_PACK_HUMP_COUNT_MAX);
    }
}

/**
 * @brief   Current top-level mode.
 * @param   p_monitor  Monitor state.
 * @return  The mode.
 */
imu_monitor_mode_t
imu_monitor_get_mode (imu_monitor_t const * p_monitor)
{
    return ((NULL != p_monitor) ? p_monitor->mode : IMU_MONITOR_CALIBRATING);
}

/**
 * @brief   Latest attitude (read-only).
 * @param   p_monitor  Monitor state.
 * @return  Pointer into @p p_monitor, or NULL.
 */
imu_filter_attitude_t const *
imu_monitor_get_attitude (imu_monitor_t const * p_monitor)
{
    return ((NULL != p_monitor) ?
            imu_filter_get_attitude(&p_monitor->filter) : NULL);
}

/**
 * @brief   Highest hump peak of the run, full precision.
 * @param   p_monitor  Monitor state.
 * @return  Peak in mm.
 */
float32_t
imu_monitor_get_run_peak_mm (imu_monitor_t const * p_monitor)
{
    return ((NULL != p_monitor) ?
            imu_terrain_get_run_peak_mm(&p_monitor->terrain) : 0.0f);
}

/**
 * @brief   Number of humps closed in the run.
 * @param   p_monitor  Monitor state.
 * @return  Hump count (saturates at 255).
 */
uint32_t
imu_monitor_get_hump_count (imu_monitor_t const * p_monitor)
{
    return ((NULL != p_monitor) ?
            imu_terrain_get_hump_count(&p_monitor->terrain) : 0u);
}

/**
 * @brief   Clear every output field.
 * @param   p_out  Output to clear.
 */
static void
clear_output (imu_monitor_output_t * p_out)
{
    p_out->b_calibrated         = false;
    p_out->b_calib_forced       = false;
    p_out->b_impact             = false;
    p_out->b_recovered          = false;
    p_out->b_motion_changed     = false;
    p_out->b_terrain_changed    = false;
    p_out->b_hump_closed        = false;
    p_out->b_filter_fault       = false;
    p_out->motion_class         = IMU_MOTION_STATIONARY;
    p_out->terrain_state        = IMU_TERRAIN_LEVEL;
    p_out->hump.peak_mm         = 0.0f;
    p_out->hump.peak_mm_packed  = 0u;
    p_out->hump.b_saturated     = false;
    p_out->hump.max_pitch_deg   = 0u;
    p_out->hump.b_cut_short     = false;
    p_out->impact_peak_g        = 0.0f;
    p_out->distance_mm          = 0.0f;
}

/**
 * @brief   Enter MONITORING at the initial sub-states of both regions.
 * @param   p_monitor  Monitor state.
 */
static void
enter_monitoring (imu_monitor_t * p_monitor)
{
    imu_motion_init(&p_monitor->motion);
    imu_terrain_reset_region(&p_monitor->terrain);
    p_monitor->mode = IMU_MONITOR_MONITORING;
}

/**
 * @brief   MONITORING / IMPACT processing after a good filter update.
 * @param   p_monitor    Monitor state.
 * @param   p_input      Sample in body axes (gyro-bias tracking; whether
 *                       the sensor has a gyroscope).
 * @param   distance_mm  Distance travelled this sample.
 * @param   p_out        Receives what happened.
 */
static void
run_regions (imu_monitor_t * p_monitor, imu_filter_input_t const * p_input,
             float32_t distance_mm, imu_monitor_output_t * p_out)
{
    imu_filter_attitude_t const * const p_att =
        imu_filter_get_attitude(&p_monitor->filter);

    if (IMU_MONITOR_IMPACT == p_monitor->mode)
    {
        if (imu_impact_check_recovered(&p_monitor->impact,
                                       p_att->accel_mag_g))
        {
            p_out->b_recovered = true;
            enter_monitoring(p_monitor);
        }
    }
    else if (imu_impact_detect(&p_monitor->impact, p_att->accel_mag_g,
                               p_att->lin_fwd_g,
                               (IMU_TERRAIN_DESCENDING ==
                                imu_terrain_get_state(&p_monitor->terrain))))
    {
        /* Shock: leave Monitoring at once.  Keep any hump measured so far. */
        p_out->b_impact      = true;
        p_out->impact_peak_g = imu_impact_get_peak_g(&p_monitor->impact);
        p_monitor->mode      = IMU_MONITOR_IMPACT;

        if (imu_terrain_force_close(&p_monitor->terrain))
        {
            p_out->b_terrain_changed = true;
        }
    }
    else
    {
        imu_motion_inputs_t const motion_in =
        {
            p_att->accel_mag_g, p_att->lin_fwd_g, p_att->yaw_rate_dps,
            imu_odom_is_moving(&p_monitor->odom)
        };

        p_out->b_motion_changed = imu_motion_update(&p_monitor->motion,
                                                    &motion_in);

        if (IMU_MOTION_STATIONARY == imu_motion_get_class(&p_monitor->motion))
        {
            imu_filter_track_bias(&p_monitor->filter, p_input);
        }

        /* Without a gyroscope, the start-up acceleration reads as a nose-up
         * tilt, so a hump may only open once the car moves steadily.
         */
        bool const b_terrain_moving = p_input->b_has_gyro ?
            imu_odom_is_moving(&p_monitor->odom) :
            imu_odom_is_settled(&p_monitor->odom);

        p_out->b_terrain_changed = imu_terrain_update(
            &p_monitor->terrain, p_att->pitch_deg, p_att->roll_deg,
            distance_mm, b_terrain_moving);
    }

    if (imu_terrain_take_result(&p_monitor->terrain, &p_out->hump))
    {
        p_out->b_hump_closed = true;
        imu_impact_start_holdoff(&p_monitor->impact);
    }
}

/*** end of file ***/
