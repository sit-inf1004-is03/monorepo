/**
 * @file    drv_i2c1.h
 * @brief   Polled I2C1 master for the IMU (GP2 = SDA, GP3 = SCL, Grove 2).
 *
 * Ownership: the IMU task is the ONLY caller (Week 6 Table 4, Tribunal 1
 * "I2C1 owned only by the IMU task").  The driver therefore has no locking.
 *
 * Why this driver instead of the kernel's device/i2c driver:
 *   - The kernel driver is disabled (config/config_device.h DEVCNF_USE_IIC 0)
 *     and enabling it changes shared configuration and claims GP6/GP7 for
 *     I2C1 by default; GP7 is now a wheel-encoder pin (Tribunal 1, G7).
 *   - Every transfer here has a hard timeout and the bus can be recovered
 *     (nine SCL pulses + STOP) when a slave holds SDA low (gap G5), so an
 *     I2C fault can never hang the IMU task and silently disable ESTOP.
 *
 * Timing: 400 kHz fast mode.  A 14-byte sensor read takes about 0.4 ms of
 * polling (Tribunal 1, 2.2.4).  Worst case per call is bounded by
 * DRV_I2C1_TIMEOUT_BASE_US + DRV_I2C1_TIMEOUT_PER_BYTE_US * bytes.
 */

#ifndef DRV_I2C1_H
#define DRV_I2C1_H

#include <stdint.h>

#define DRV_I2C1_SDA_PIN                (2u)
#define DRV_I2C1_SCL_PIN                (3u)
#define DRV_I2C1_TIMEOUT_BASE_US        (500u)
#define DRV_I2C1_TIMEOUT_PER_BYTE_US    (100u)
#define DRV_I2C1_MAX_TRANSFER           (32u)

/** Result of an I2C1 operation. */
typedef enum
{
    DRV_I2C1_OK = 0,
    DRV_I2C1_ERR_PARAM,         /**< Bad argument.                         */
    DRV_I2C1_ERR_TIMEOUT,       /**< Bus did not finish in time.           */
    DRV_I2C1_ERR_NACK,          /**< Address or data not acknowledged.     */
    DRV_I2C1_ERR_BUS_STUCK      /**< SCL or SDA held low; recovery failed. */
} drv_i2c1_status_t;

drv_i2c1_status_t drv_i2c1_init (void);
drv_i2c1_status_t drv_i2c1_write (uint8_t address, uint8_t const * p_data,
                                  uint32_t length);
drv_i2c1_status_t drv_i2c1_read_reg (uint8_t address, uint8_t reg,
                                     uint8_t * p_buffer, uint32_t length);
drv_i2c1_status_t drv_i2c1_recover_bus (void);
uint32_t          drv_i2c1_last_abort_source (void);

#endif /* DRV_I2C1_H */

/*** end of file ***/
