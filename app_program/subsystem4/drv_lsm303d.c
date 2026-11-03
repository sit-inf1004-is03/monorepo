/**
 * @file    drv_lsm303d.c
 * @brief   LSM303D accelerometer + magnetometer driver over I2C1.
 *
 * Reference: ST "LSM303D Ultra compact high performance e-compass 3D
 * accelerometer and 3D magnetometer module", DocID023312 rev 3 (register
 * numbers and bit fields below).  Called only from the IMU task (sole
 * owner of I2C1).
 */

#include <stdbool.h>
#include <tk/tkernel.h>

#include "drv_lsm303d.h"
#include "drv_i2c1.h"

/* GY-511 straps SA0 high (0x1E); 0x1D is tried as well. */
#define ADDRESS_SA0_HIGH        (0x1Eu)
#define ADDRESS_SA0_LOW         (0x1Du)

#define REG_OUT_X_L_M           (0x08u)
#define REG_WHO_AM_I            (0x0Fu)
#define REG_CTRL0               (0x1Fu)
#define REG_CTRL1               (0x20u)
#define REG_CTRL2               (0x21u)
#define REG_CTRL3               (0x22u)
#define REG_CTRL4               (0x23u)
#define REG_CTRL5               (0x24u)
#define REG_CTRL6               (0x25u)
#define REG_CTRL7               (0x26u)
#define REG_OUT_X_L_A           (0x28u)

/* Setting bit 7 of the sub-address auto-increments it (datasheet 6.1.1). */
#define SUB_AUTO_INCREMENT      (0x80u)

#define WHO_AM_I_LSM303D        (0x49u)

#define CTRL0_DEFAULT           (0x00u)     /* no FIFO, no high-pass       */
#define CTRL1_200HZ_BDU_XYZ     (0x7Fu)     /* AODR 0111, BDU, X/Y/Z on    */
#define CTRL2_ABW50_8G          (0xD8u)     /* ABW 11 (50 Hz), AFS 011     */
#define CTRL_INT_OFF            (0x00u)     /* CTRL3 / CTRL4: no INT pins  */
#define CTRL5_MAG_HIRES_100HZ   (0x74u)     /* M_RES 11, M_ODR 101         */
#define CTRL6_MAG_4GAUSS        (0x20u)     /* MFS 01                      */
#define CTRL7_MAG_CONTINUOUS    (0x00u)     /* MD 00, normal accel path    */

#define START_DELAY_MS          (20u)       /* first samples after config  */

#define AXIS_BYTES              (6u)
#define BYTES_PER_AXIS          (2u)

#define UINT16_SIGN_BIT         (0x8000u)
#define UINT16_MODULUS          (65536)

static uint8_t g_address  = ADDRESS_SA0_HIGH;
static uint8_t g_who_am_i = 0u;

static drv_imu_status_t write_register (uint8_t reg, uint8_t value);
static drv_imu_status_t probe_address (uint8_t address);
static int16_t          to_int16_le (uint8_t const * p_bytes);

/**
 * @brief   Find the sensor and apply the configuration.
 *
 * Blocks for about 20 ms (first conversions), so it is called only while
 * the IMU is not yet monitoring.
 *
 * @return  DRV_IMU_OK or the first error met.
 */
drv_imu_status_t
drv_lsm303d_init (void)
{
    drv_imu_status_t status = probe_address(ADDRESS_SA0_HIGH);

    if (DRV_IMU_ERR_BUS == status)
    {
        status = probe_address(ADDRESS_SA0_LOW);
    }

    uint8_t const reg_list[] =
    {
        REG_CTRL0, REG_CTRL1, REG_CTRL2, REG_CTRL3, REG_CTRL4, REG_CTRL5,
        REG_CTRL6, REG_CTRL7
    };
    uint8_t const value_list[] =
    {
        CTRL0_DEFAULT, CTRL1_200HZ_BDU_XYZ, CTRL2_ABW50_8G, CTRL_INT_OFF,
        CTRL_INT_OFF, CTRL5_MAG_HIRES_100HZ, CTRL6_MAG_4GAUSS,
        CTRL7_MAG_CONTINUOUS
    };
    uint32_t const reg_count = sizeof(reg_list) / sizeof(reg_list[0]);

    /* Each step runs only if every earlier step succeeded. */
    for (uint32_t index = 0u;
         (DRV_IMU_OK == status) && (index < reg_count); index++)
    {
        status = write_register(reg_list[index], value_list[index]);
    }

    /* Read back the settings the scaling depends on: CTRL1, CTRL2 and,
     * one auto-incremented read later, CTRL5..CTRL6.
     */
    if (DRV_IMU_OK == status)
    {
        uint8_t accel_rb[2] = {0u, 0u};
        uint8_t mag_rb[2]   = {0u, 0u};

        if ((DRV_I2C1_OK != drv_i2c1_read_reg(g_address,
                                              REG_CTRL1 | SUB_AUTO_INCREMENT,
                                              accel_rb, sizeof(accel_rb))) ||
            (DRV_I2C1_OK != drv_i2c1_read_reg(g_address,
                                              REG_CTRL5 | SUB_AUTO_INCREMENT,
                                              mag_rb, sizeof(mag_rb))))
        {
            status = DRV_IMU_ERR_BUS;
        }
        else if ((CTRL1_200HZ_BDU_XYZ != accel_rb[0]) ||
                 (CTRL2_ABW50_8G != accel_rb[1]) ||
                 (CTRL5_MAG_HIRES_100HZ != mag_rb[0]) ||
                 (CTRL6_MAG_4GAUSS != mag_rb[1]))
        {
            status = DRV_IMU_ERR_CONFIG;
        }
        else
        {
            (void)tk_dly_tsk(START_DELAY_MS);
            status = DRV_IMU_OK;
        }
    }

    return (status);
}

/**
 * @brief   Read one accelerometer + magnetometer sample.
 * @param   p_sample  Receives the raw sample in sensor axes; gyro and
 *                    temperature are zero, b_has_gyro false.
 * @return  DRV_IMU_OK, DRV_IMU_ERR_BUS, or DRV_IMU_ERR_DATA if all six
 *          accelerometer bytes are zero (sensor powered down).
 */
drv_imu_status_t
drv_lsm303d_read (drv_imu_sample_t * p_sample)
{
    drv_imu_status_t status = DRV_IMU_ERR_BUS;
    uint8_t          accel_raw[AXIS_BYTES];
    uint8_t          mag_raw[AXIS_BYTES];

    if (NULL == p_sample)
    {
        status = DRV_IMU_ERR_DATA;
    }
    else if ((DRV_I2C1_OK == drv_i2c1_read_reg(g_address,
                                               REG_OUT_X_L_A |
                                               SUB_AUTO_INCREMENT,
                                               accel_raw, AXIS_BYTES)) &&
             (DRV_I2C1_OK == drv_i2c1_read_reg(g_address,
                                               REG_OUT_X_L_M |
                                               SUB_AUTO_INCREMENT,
                                               mag_raw, AXIS_BYTES)))
    {
        uint32_t nonzero = 0u;

        for (uint32_t axis = 0u; axis < DRV_IMU_AXES; axis++)
        {
            uint32_t const offset = axis * BYTES_PER_AXIS;

            p_sample->accel[axis] = to_int16_le(&accel_raw[offset]);
            p_sample->mag[axis]   = to_int16_le(&mag_raw[offset]);
            p_sample->gyro[axis]  = 0;
        }

        p_sample->temperature = 0;
        p_sample->b_has_gyro  = false;
        p_sample->b_has_mag   = true;

        for (uint32_t index = 0u; index < AXIS_BYTES; index++)
        {
            nonzero |= accel_raw[index];
        }

        status = (0u == nonzero) ? DRV_IMU_ERR_DATA : DRV_IMU_OK;
    }
    else
    {
        status = DRV_IMU_ERR_BUS;
    }

    return (status);
}

/**
 * @brief   WHO_AM_I value seen by the last successful probe (diagnostics).
 * @return  0x49, or 0 if no device was found yet.
 */
uint8_t
drv_lsm303d_who_am_i (void)
{
    return (g_who_am_i);
}

/**
 * @brief   I2C address the sensor answered on.
 * @return  0x1E or 0x1D.
 */
uint8_t
drv_lsm303d_address (void)
{
    return (g_address);
}

/**
 * @brief   Write one register.
 * @param   reg    Register number.
 * @param   value  Value to write.
 * @return  DRV_IMU_OK or DRV_IMU_ERR_BUS.
 */
static drv_imu_status_t
write_register (uint8_t reg, uint8_t value)
{
    uint8_t const frame[2] = {reg, value};

    return ((DRV_I2C1_OK == drv_i2c1_write(g_address, frame, sizeof(frame)))
            ? DRV_IMU_OK : DRV_IMU_ERR_BUS);
}

/**
 * @brief   Read WHO_AM_I at one address and remember the address if valid.
 * @param   address  7-bit address to try.
 * @return  DRV_IMU_OK, ERR_BUS (no answer) or ERR_WHO_AM_I.
 */
static drv_imu_status_t
probe_address (uint8_t address)
{
    drv_imu_status_t status = DRV_IMU_ERR_BUS;
    uint8_t          who    = 0u;

    if (DRV_I2C1_OK == drv_i2c1_read_reg(address, REG_WHO_AM_I, &who, 1u))
    {
        if (WHO_AM_I_LSM303D == who)
        {
            g_address  = address;
            g_who_am_i = who;
            status     = DRV_IMU_OK;
        }
        else
        {
            status = DRV_IMU_ERR_WHO_AM_I;
        }
    }

    return (status);
}

/**
 * @brief   Combine two little-endian bytes into a signed 16-bit value.
 *
 * Done arithmetically, without converting an out-of-range unsigned value to
 * a signed type (implementation-defined in C99).
 *
 * @param   p_bytes  Low byte, then high byte.
 * @return  The signed value.
 */
static int16_t
to_int16_le (uint8_t const * p_bytes)
{
    /* Cast: widening uint8_t to uint32_t is lossless. */
    uint32_t const unsigned_value = ((uint32_t)p_bytes[1] << 8) | p_bytes[0];
    int32_t        value          = 0;

    if (unsigned_value >= UINT16_SIGN_BIT)
    {
        /* Cast: unsigned_value < 65536, so it fits in int32_t. */
        value = (int32_t)unsigned_value - UINT16_MODULUS;
    }
    else
    {
        /* Cast: unsigned_value < 32768, so it fits in int32_t. */
        value = (int32_t)unsigned_value;
    }

    /* Cast: value is in [-32768, 32767], the int16_t range. */
    return ((int16_t)value);
}

/*** end of file ***/
