/**
 * @file    drv_mpu9250.h
 * @brief   MPU-9250 accelerometer + gyroscope driver over I2C1.
 *
 * Only the 6-axis MPU-6500 core is used.  The AK8963 magnetometer is left
 * off: using it for heading was rejected as an unrealistic idea in the
 * Tribunal 1 pre-Zoom review (Buddy 4 row, weak idea 3).
 *
 * Configuration (MPU-9250 Register Map rev 1.6):
 *   accelerometer  +/-8 g,      4096 LSB/g,   44.8 Hz digital low-pass
 *   gyroscope      +/-500 dps,  65.5 LSB/dps, 41 Hz digital low-pass
 *   output rate    200 Hz (task reads every 10 ms, so data is always fresh)
 * +/-8 g leaves head-room for impact detection; raw samples stay 16-bit,
 * the sensor's native width (Tribunal 1, 2.3 "Raw ... IMU (16-bit)").
 */

#ifndef DRV_MPU9250_H
#define DRV_MPU9250_H

#include <stdint.h>

#include "drv_imu.h"

/* The common scale in drv_imu.h is this configuration's native one. */

/** Result of a sensor operation. */
typedef enum
{
    DRV_MPU9250_OK = 0,
    DRV_MPU9250_ERR_BUS,        /**< I2C timeout, NACK or stuck bus.        */
    DRV_MPU9250_ERR_WHO_AM_I,   /**< Device answered with an unknown ID.    */
    DRV_MPU9250_ERR_CONFIG,     /**< Read-back of a setting did not match.  */
    DRV_MPU9250_ERR_DATA        /**< Sample is not plausible (all zero).    */
} drv_mpu9250_status_t;

drv_mpu9250_status_t drv_mpu9250_init (void);
drv_mpu9250_status_t drv_mpu9250_read (drv_imu_sample_t * p_sample);
uint8_t              drv_mpu9250_who_am_i (void);
uint8_t              drv_mpu9250_address (void);

#endif /* DRV_MPU9250_H */

/*** end of file ***/
