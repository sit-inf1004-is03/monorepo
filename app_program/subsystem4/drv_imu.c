/**
 * @file    drv_imu.c
 * @brief   Sensor-independent front end: probes the MPU first, then the
 *          LSM303D, and forwards reads to whichever answered.
 *
 * Called only from the IMU task (sole owner of I2C1).
 */

#include <stddef.h>

#include "drv_imu.h"
#include "drv_mpu9250.h"
#include "drv_lsm303d.h"

static drv_imu_kind_t g_kind = DRV_IMU_KIND_NONE;

static drv_imu_status_t from_mpu_status (drv_mpu9250_status_t status);

/**
 * @brief   Find a supported sensor and configure it.
 *
 * If neither answers, the more informative error wins: an MPU or LSM303D
 * that answered with a bad ID or configuration is reported over "nobody
 * answered".
 *
 * @return  DRV_IMU_OK, or the error of the failed probe.
 */
drv_imu_status_t
drv_imu_init (void)
{
    drv_imu_status_t const mpu = from_mpu_status(drv_mpu9250_init());
    drv_imu_status_t       status = mpu;

    if (DRV_IMU_OK == mpu)
    {
        g_kind = DRV_IMU_KIND_MPU;
    }
    else
    {
        drv_imu_status_t const lsm = drv_lsm303d_init();

        if (DRV_IMU_OK == lsm)
        {
            g_kind = DRV_IMU_KIND_LSM303D;
            status = DRV_IMU_OK;
        }
        else
        {
            g_kind = DRV_IMU_KIND_NONE;
            status = (DRV_IMU_ERR_BUS != mpu) ? mpu : lsm;
        }
    }

    return (status);
}

/**
 * @brief   Read one sample from the sensor found by drv_imu_init().
 * @param   p_sample  Receives the raw sample.
 * @return  DRV_IMU_OK or an error (DRV_IMU_ERR_BUS if no sensor is set up).
 */
drv_imu_status_t
drv_imu_read (drv_imu_sample_t * p_sample)
{
    drv_imu_status_t status = DRV_IMU_ERR_BUS;

    if (NULL == p_sample)
    {
        status = DRV_IMU_ERR_DATA;
    }
    else if (DRV_IMU_KIND_MPU == g_kind)
    {
        status = from_mpu_status(drv_mpu9250_read(p_sample));

        for (uint32_t axis = 0u; axis < DRV_IMU_AXES; axis++)
        {
            p_sample->mag[axis] = 0;
        }

        p_sample->b_has_gyro = true;
        p_sample->b_has_mag  = false;
    }
    else if (DRV_IMU_KIND_LSM303D == g_kind)
    {
        status = drv_lsm303d_read(p_sample);
    }
    else
    {
        status = DRV_IMU_ERR_BUS;
    }

    return (status);
}

/**
 * @brief   Which sensor is in use.
 * @return  The kind, or DRV_IMU_KIND_NONE before a successful init.
 */
drv_imu_kind_t
drv_imu_kind (void)
{
    return (g_kind);
}

/**
 * @brief   I2C address of the sensor in use (diagnostics).
 * @return  The 7-bit address, or 0 if none.
 */
uint8_t
drv_imu_address (void)
{
    uint8_t address = 0u;

    if (DRV_IMU_KIND_MPU == g_kind)
    {
        address = drv_mpu9250_address();
    }
    else if (DRV_IMU_KIND_LSM303D == g_kind)
    {
        address = drv_lsm303d_address();
    }
    else
    {
        address = 0u;
    }

    return (address);
}

/**
 * @brief   ID register value of the sensor in use (diagnostics).
 * @return  WHO_AM_I, or 0 if none.
 */
uint8_t
drv_imu_who_am_i (void)
{
    uint8_t who = 0u;

    if (DRV_IMU_KIND_MPU == g_kind)
    {
        who = drv_mpu9250_who_am_i();
    }
    else if (DRV_IMU_KIND_LSM303D == g_kind)
    {
        who = drv_lsm303d_who_am_i();
    }
    else
    {
        who = 0u;
    }

    return (who);
}

/**
 * @brief   Map the MPU driver's status onto the common one.
 * @param   status  MPU driver status.
 * @return  Equivalent drv_imu status.
 */
static drv_imu_status_t
from_mpu_status (drv_mpu9250_status_t status)
{
    drv_imu_status_t result = DRV_IMU_ERR_BUS;

    switch (status)
    {
        case DRV_MPU9250_OK:
            result = DRV_IMU_OK;
        break;

        case DRV_MPU9250_ERR_WHO_AM_I:
            result = DRV_IMU_ERR_WHO_AM_I;
        break;

        case DRV_MPU9250_ERR_CONFIG:
            result = DRV_IMU_ERR_CONFIG;
        break;

        case DRV_MPU9250_ERR_DATA:
            result = DRV_IMU_ERR_DATA;
        break;

        default:
            result = DRV_IMU_ERR_BUS;
        break;
    }

    return (result);
}

/*** end of file ***/
