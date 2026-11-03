/**
 * @file    drv_lsm303d.h
 * @brief   LSM303D accelerometer + magnetometer driver over I2C1 (GY-511
 *          board).  The LSM303D has NO gyroscope.
 *
 * Configuration (ST LSM303D datasheet DocID023312 rev 3):
 *   accelerometer  +/-8 g, 0.244 mg/LSB, 200 Hz, 50 Hz anti-alias filter,
 *                  block data update (the two bytes of an axis always come
 *                  from the same sample)
 *   magnetometer   +/-4 gauss, 0.160 mgauss/LSB, 100 Hz, high resolution,
 *                  continuous conversion
 * The task reads every 10 ms, so the 200 Hz accelerometer data is always
 * fresh.  Both sensors share one I2C address: 0x1E (SA0 high, the GY-511
 * default) or 0x1D (SA0 low); WHO_AM_I (0x0F) reads 0x49.
 */

#ifndef DRV_LSM303D_H
#define DRV_LSM303D_H

#include <stdint.h>

#include "drv_imu.h"

drv_imu_status_t drv_lsm303d_init (void);
drv_imu_status_t drv_lsm303d_read (drv_imu_sample_t * p_sample);
uint8_t          drv_lsm303d_who_am_i (void);
uint8_t          drv_lsm303d_address (void);

#endif /* DRV_LSM303D_H */

/*** end of file ***/
