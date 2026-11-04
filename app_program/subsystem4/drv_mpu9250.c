/**
 * @file    drv_mpu9250.c
 * @brief   MPU-9250 accelerometer + gyroscope driver over I2C1.
 *
 * References: InvenSense "MPU-9250 Register Map and Descriptions" rev 1.6
 * (register numbers below) and "MPU-9250 Product Specification" rev 1.1
 * (start-up times).  Called only from the IMU task (sole owner of I2C1).
 */

#include <stdbool.h>
#include <tk/tkernel.h>

#include "drv_mpu9250.h"
#include "drv_i2c1.h"

/* The Grove/GY-9250 boards strap AD0 low (0x68); 0x69 is tried as well. */
#define ADDRESS_AD0_LOW         (0x68u)
#define ADDRESS_AD0_HIGH        (0x69u)

/* Register numbers. */
#define REG_SMPLRT_DIV          (0x19u)
#define REG_CONFIG              (0x1Au)
#define REG_GYRO_CONFIG         (0x1Bu)
#define REG_ACCEL_CONFIG        (0x1Cu)
#define REG_ACCEL_CONFIG2       (0x1Du)
#define REG_ACCEL_XOUT_H        (0x3Bu)
#define REG_USER_CTRL           (0x6Au)
#define REG_PWR_MGMT_1          (0x6Bu)
#define REG_PWR_MGMT_2          (0x6Cu)
#define REG_WHO_AM_I            (0x75u)

/* Register values. */
#define PWR1_RESET              (0x80u)
#define PWR1_CLK_PLL            (0x01u)     /* auto-select PLL clock      */
#define PWR2_ALL_AXES_ON        (0x00u)
#define USER_CTRL_DEFAULT       (0x00u)     /* no FIFO, no aux I2C master */
#define CONFIG_DLPF_41HZ        (0x03u)     /* gyro 41 Hz, 1 kHz internal */
#define SMPLRT_DIV_200HZ        (0x04u)     /* 1 kHz / (1 + 4) = 200 Hz   */
#define GYRO_FS_500DPS          (0x08u)
#define ACCEL_FS_8G             (0x10u)
#define ACCEL_DLPF_45HZ         (0x03u)

/* Accepted WHO_AM_I values: MPU-9250, MPU-9255 and the bare MPU-6500. */
#define WHO_AM_I_MPU9250        (0x71u)
#define WHO_AM_I_MPU9255        (0x73u)
#define WHO_AM_I_MPU6500        (0x70u)

#define RESET_DELAY_MS          (100u)
#define CLOCK_SETTLE_MS         (10u)

/* Burst read: accel X/Y/Z, temperature, gyro X/Y/Z, big-endian 16-bit. */
#define BURST_LENGTH            (14u)
#define OFFSET_ACCEL            (0u)
#define OFFSET_TEMP             (6u)
#define OFFSET_GYRO             (8u)
#define BYTES_PER_AXIS          (2u)

#define UINT16_SIGN_BIT         (0x8000u)
#define UINT16_MODULUS          (65536)

static uint8_t g_address  = ADDRESS_AD0_LOW;
static uint8_t g_who_am_i = 0u;

static drv_mpu9250_status_t write_register (uint8_t reg, uint8_t value);
static drv_mpu9250_status_t probe_address (uint8_t address);
static bool                 is_known_device (uint8_t who_am_i);
static int16_t              to_int16 (uint8_t const * p_bytes);

/**
 * @brief   Find the sensor, reset it and apply the configuration.
 *
 * Blocks for about 120 ms (reset and clock start-up), so it is called only
 * while the IMU is not yet monitoring.
 *
 * @return  DRV_MPU9250_OK or the first error met.
 */
drv_mpu9250_status_t
drv_mpu9250_init (void)
{
    drv_mpu9250_status_t status = probe_address(ADDRESS_AD0_LOW);

    if (DRV_MPU9250_ERR_BUS == status)
    {
        status = probe_address(ADDRESS_AD0_HIGH);
    }

    if (DRV_MPU9250_OK == status)
    {
        status = write_register(REG_PWR_MGMT_1, PWR1_RESET);
        (void)tk_dly_tsk(RESET_DELAY_MS);
    }

    /* Each step runs only if every earlier step succeeded. */
    if (DRV_MPU9250_OK == status)
    {
        status = write_register(REG_PWR_MGMT_1, PWR1_CLK_PLL);
        (void)tk_dly_tsk(CLOCK_SETTLE_MS);
    }

    uint8_t const reg_list[] =
    {
        REG_PWR_MGMT_2, REG_USER_CTRL, REG_CONFIG, REG_SMPLRT_DIV,
        REG_GYRO_CONFIG, REG_ACCEL_CONFIG, REG_ACCEL_CONFIG2
    };
    uint8_t const value_list[] =
    {
        PWR2_ALL_AXES_ON, USER_CTRL_DEFAULT, CONFIG_DLPF_41HZ,
        SMPLRT_DIV_200HZ, GYRO_FS_500DPS, ACCEL_FS_8G, ACCEL_DLPF_45HZ
    };
    uint32_t const reg_count = sizeof(reg_list) / sizeof(reg_list[0]);

    for (uint32_t index = 0u;
         (DRV_MPU9250_OK == status) && (index < reg_count); index++)
    {
        status = write_register(reg_list[index], value_list[index]);
    }

    /* Read back the two full-scale settings the scaling depends on. */
    if (DRV_MPU9250_OK == status)
    {
        uint8_t readback[2] = {0u, 0u};

        if (DRV_I2C1_OK != drv_i2c1_read_reg(g_address, REG_GYRO_CONFIG,
                                             readback, sizeof(readback)))
        {
            status = DRV_MPU9250_ERR_BUS;
        }
        else if ((GYRO_FS_500DPS != readback[0]) ||
                 (ACCEL_FS_8G != readback[1]))
        {
            status = DRV_MPU9250_ERR_CONFIG;
        }
        else
        {
            status = DRV_MPU9250_OK;
        }
    }

    return (status);
}

/**
 * @brief   Read one accelerometer + temperature + gyroscope sample.
 * @param   p_sample  Receives the raw sample in sensor axes.
 * @return  DRV_MPU9250_OK, DRV_MPU9250_ERR_BUS, or DRV_MPU9250_ERR_DATA if
 *          all 14 bytes are zero (sensor asleep or held in reset).
 */
drv_mpu9250_status_t
drv_mpu9250_read (drv_imu_sample_t * p_sample)
{
    drv_mpu9250_status_t status = DRV_MPU9250_ERR_BUS;
    uint8_t              raw[BURST_LENGTH];

    if (NULL == p_sample)
    {
        status = DRV_MPU9250_ERR_DATA;
    }
    else if (DRV_I2C1_OK == drv_i2c1_read_reg(g_address, REG_ACCEL_XOUT_H,
                                              raw, BURST_LENGTH))
    {
        uint32_t nonzero = 0u;

        for (uint32_t axis = 0u; axis < DRV_IMU_AXES; axis++)
        {
            uint32_t const offset = axis * BYTES_PER_AXIS;

            p_sample->accel[axis] = to_int16(&raw[OFFSET_ACCEL + offset]);
            p_sample->gyro[axis]  = to_int16(&raw[OFFSET_GYRO + offset]);
        }

        p_sample->temperature = to_int16(&raw[OFFSET_TEMP]);

        for (uint32_t index = 0u; index < BURST_LENGTH; index++)
        {
            nonzero |= raw[index];
        }

        status = (0u == nonzero) ? DRV_MPU9250_ERR_DATA : DRV_MPU9250_OK;
    }
    else
    {
        status = DRV_MPU9250_ERR_BUS;
    }

    return (status);
}

/**
 * @brief   WHO_AM_I value seen by the last successful probe (diagnostics).
 * @return  0x71, 0x73, 0x70, or 0 if no device was found yet.
 */
uint8_t
drv_mpu9250_who_am_i (void)
{
    return (g_who_am_i);
}

/**
 * @brief   I2C address the sensor answered on.
 * @return  0x68 or 0x69.
 */
uint8_t
drv_mpu9250_address (void)
{
    return (g_address);
}

/**
 * @brief   Write one register.
 * @param   reg    Register number.
 * @param   value  Value to write.
 * @return  DRV_MPU9250_OK or DRV_MPU9250_ERR_BUS.
 */
static drv_mpu9250_status_t
write_register (uint8_t reg, uint8_t value)
{
    uint8_t const frame[2] = {reg, value};

    return ((DRV_I2C1_OK == drv_i2c1_write(g_address, frame, sizeof(frame)))
            ? DRV_MPU9250_OK : DRV_MPU9250_ERR_BUS);
}

/**
 * @brief   Read WHO_AM_I at one address and remember the address if valid.
 * @param   address  7-bit address to try.
 * @return  DRV_MPU9250_OK, ERR_BUS (no answer) or ERR_WHO_AM_I.
 */
static drv_mpu9250_status_t
probe_address (uint8_t address)
{
    drv_mpu9250_status_t status = DRV_MPU9250_ERR_BUS;
    uint8_t              who    = 0u;

    if (DRV_I2C1_OK == drv_i2c1_read_reg(address, REG_WHO_AM_I, &who, 1u))
    {
        if (is_known_device(who))
        {
            g_address  = address;
            g_who_am_i = who;
            status     = DRV_MPU9250_OK;
        }
        else
        {
            status = DRV_MPU9250_ERR_WHO_AM_I;
        }
    }

    return (status);
}

/**
 * @brief   Check a WHO_AM_I value against the supported parts.
 * @param   who_am_i  Value read from register 0x75.
 * @return  true for a supported device.
 */
static bool
is_known_device (uint8_t who_am_i)
{
    return ((WHO_AM_I_MPU9250 == who_am_i) ||
            (WHO_AM_I_MPU9255 == who_am_i) ||
            (WHO_AM_I_MPU6500 == who_am_i));
}

/**
 * @brief   Combine two big-endian bytes into a signed 16-bit value.
 *
 * Done arithmetically, without converting an out-of-range unsigned value to
 * a signed type (implementation-defined in C99).
 *
 * @param   p_bytes  High byte, then low byte.
 * @return  The signed value.
 */
static int16_t
to_int16 (uint8_t const * p_bytes)
{
    /* Cast: widening uint8_t to uint32_t is lossless. */
    uint32_t const unsigned_value = ((uint32_t)p_bytes[0] << 8) | p_bytes[1];
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
