/**
 * @file    drv_imu.h
 * @brief   Sensor-independent front end for the IMU on I2C1: finds which
 *          supported sensor is fitted and reads it in one common format.
 *
 * Supported sensors, probed in this order:
 *   1. MPU-9250 / MPU-9255 / MPU-6500 (drv_mpu9250): accelerometer + gyro.
 *   2. LSM303D on the GY-511 board (drv_lsm303d): accelerometer + compass,
 *      NO gyroscope.
 *
 * Common sample format (sensor axes X, Y, Z, native 16-bit counts):
 *   accel  4096 LSB/g.  MPU at +/-8 g is exactly this; the LSM303D at
 *          +/-8 g is 0.244 mg/LSB (4098 LSB/g), 0.05 % away, well inside
 *          the part's own sensitivity tolerance, so it is used as is.
 *   gyro   65.5 LSB/dps; all zero and b_has_gyro false on the LSM303D.
 *   mag    raw counts (only ratios are used, for heading); all zero and
 *          b_has_mag false on the MPU (its AK8963 is left off).
 */

#ifndef DRV_IMU_H
#define DRV_IMU_H

#include <stdint.h>
#include <stdbool.h>

#define DRV_IMU_AXES                (3u)
#define DRV_IMU_ACCEL_LSB_PER_G     (4096.0f)
#define DRV_IMU_GYRO_LSB_PER_DPS    (65.5f)

/** Result of a sensor operation. */
typedef enum
{
    DRV_IMU_OK = 0,
    DRV_IMU_ERR_BUS,            /**< I2C timeout, NACK or stuck bus.        */
    DRV_IMU_ERR_WHO_AM_I,       /**< Device answered with an unknown ID.    */
    DRV_IMU_ERR_CONFIG,         /**< Read-back of a setting did not match.  */
    DRV_IMU_ERR_DATA            /**< Sample is not plausible (all zero).    */
} drv_imu_status_t;

/** Which sensor drv_imu_init() found. */
typedef enum
{
    DRV_IMU_KIND_NONE    = 0,
    DRV_IMU_KIND_MPU     = 1,
    DRV_IMU_KIND_LSM303D = 2
} drv_imu_kind_t;

/** One raw sample in sensor axes. */
typedef struct
{
    int16_t accel[DRV_IMU_AXES];
    int16_t gyro[DRV_IMU_AXES];
    int16_t mag[DRV_IMU_AXES];
    int16_t temperature;
    bool    b_has_gyro;
    bool    b_has_mag;
} drv_imu_sample_t;

drv_imu_status_t drv_imu_init (void);
drv_imu_status_t drv_imu_read (drv_imu_sample_t * p_sample);
drv_imu_kind_t   drv_imu_kind (void);
uint8_t          drv_imu_address (void);
uint8_t          drv_imu_who_am_i (void);

#endif /* DRV_IMU_H */

/*** end of file ***/
