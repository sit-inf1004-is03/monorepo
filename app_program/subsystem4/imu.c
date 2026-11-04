/*!
 * @file    imu.c
 * @brief   LOCAL TEST (s5-local-test, Subsystem 4 is Andrew's):
 *          accelerometer tilt for ramp detection.  Auto-detects MPU6050/
 *          6500/9250, ADXL345, MMA845x, LSM303D/LSM9DS0 (the board on the
 *          car answers at 0x1D, not the MPU's 0x68/0x69).
 *
 * I2C1 through the kernel's "iicb" device (GP2 SDA, GP3 SCL, fast mode).
 * Start-up: find the chip at 0x68 or 0x69 (WHO_AM_I), reset, wake, +/-2 g,
 * accelerometer low-pass 10 Hz, then average the gravity vector for
 * IMU_CAL_SAMPLES (the car stands flat at boot).  Every IMU_PERIOD_MS:
 * read the accelerometer, smooth it, and take the angle to the boot
 * gravity vector as the tilt.  Ramp on at IMU_RAMP_ON_DEG for
 * IMU_ON_SAMPLES samples, off at IMU_RAMP_OFF_DEG for IMU_OFF_SAMPLES.
 * IMU_FAIL_LIMIT failed reads in a row raise NAV_FLT_IMU (degrade only:
 * the car keeps driving, never "on a ramp"); a good read clears it.
 */

#include "imu.h"
#include "../navigation/navigation.h"
#include "../subsystem3/line_barcode.h"     // barcode_beep()

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "../../device/include/dev_i2c.h"

#define IMU_DEV_NAME        "iicb"
#define IMU_ADDR_LOW        (0x68u)     // AD0 low.
#define IMU_ADDR_HIGH       (0x69u)     // AD0 high.
#define REG_ACCEL_XOUT_H    (0x3Bu)
#define REG_CONFIG          (0x1Au)
#define REG_ACCEL_CONFIG    (0x1Cu)
#define REG_ACCEL_CONFIG2   (0x1Du)
#define REG_PWR_MGMT_1      (0x6Bu)
#define REG_WHO_AM_I        (0x75u)
#define PWR_RESET           (0x80u)
#define PWR_CLK_PLL         (0x01u)
#define ACCEL_2G            (0x00u)
#define ACCEL_DLPF_10HZ     (0x05u)

// ADXL345.
#define ADXL_DEVID          (0x00u)     // = 0xE5
#define ADXL_BW_RATE        (0x2Cu)
#define ADXL_POWER_CTL      (0x2Du)
#define ADXL_DATA_FORMAT    (0x31u)
#define ADXL_DATAX0         (0x32u)
#define ADXL_RATE_50HZ      (0x09u)
#define ADXL_MEASURE        (0x08u)
#define ADXL_FULL_RES_2G    (0x08u)

// MMA8451 / 8452 / 8453.
#define MMA_OUT_X_MSB       (0x01u)
#define MMA_WHO_AM_I        (0x0Du)     // = 0x1A, 0x2A, 0x3A
#define MMA_XYZ_DATA_CFG    (0x0Eu)
#define MMA_CTRL_REG1       (0x2Au)
#define MMA_ACTIVE_50HZ     (0x21u)     // DR = 50 Hz, ACTIVE.

// LSM303D / LSM9DS0 accelerometer.
#define LSM_WHO_AM_I        (0x0Fu)     // = 0x49
#define LSM_CTRL1           (0x20u)
#define LSM_CTRL2           (0x21u)
#define LSM_OUT_X_L_A       (0x28u)
#define LSM_AUTO_INC        (0x80u)
#define LSM_50HZ_XYZ        (0x57u)

#define CHIP_NONE           (0u)
#define CHIP_MPU            (1u)
#define CHIP_ADXL345        (2u)
#define CHIP_MMA845X        (3u)
#define CHIP_LSM303D        (4u)
#define IMU_PROBES          (8u)

typedef struct
{
    uint8_t addr;
    uint8_t id_reg;
    uint8_t id_a;
    uint8_t id_b;
    uint8_t id_c;
    uint8_t id_d;
    uint8_t chip;
} imu_probe_t;

static imu_probe_t const g_probe[IMU_PROBES] =
{
    { 0x68u, REG_WHO_AM_I, 0x71u, 0x73u, 0x70u, 0x68u, CHIP_MPU     },
    { 0x69u, REG_WHO_AM_I, 0x71u, 0x73u, 0x70u, 0x68u, CHIP_MPU     },
    { 0x1Du, ADXL_DEVID,   0xE5u, 0xE5u, 0xE5u, 0xE5u, CHIP_ADXL345 },
    { 0x53u, ADXL_DEVID,   0xE5u, 0xE5u, 0xE5u, 0xE5u, CHIP_ADXL345 },
    { 0x1Du, MMA_WHO_AM_I, 0x1Au, 0x2Au, 0x3Au, 0x1Au, CHIP_MMA845X },
    { 0x1Cu, MMA_WHO_AM_I, 0x1Au, 0x2Au, 0x3Au, 0x1Au, CHIP_MMA845X },
    { 0x1Du, LSM_WHO_AM_I, 0x49u, 0x49u, 0x49u, 0x49u, CHIP_LSM303D },
    { 0x1Eu, LSM_WHO_AM_I, 0x49u, 0x49u, 0x49u, 0x49u, CHIP_LSM303D },
};

#define IMU_PERIOD_MS       (20)
#define IMU_CAL_SAMPLES     (50u)       // 1 s of gravity at boot.
#define IMU_ON_SAMPLES      (3u)
#define IMU_OFF_SAMPLES     (5u)
#define IMU_FAIL_LIMIT      (5u)        // 100 ms of bad reads: restart.
#define IMU_RETRY_MS        (200)
#define IMU_SCAN_EVERY      (50u)       // Print + bus scan every 10 s.
#define IMU_SDA_PIN         (2u)        // = DEVCNF_I2C1_SDA_PIN
#define IMU_SCL_PIN         (3u)        // = DEVCNF_I2C1_SCL_PIN
#define IMU_PRINT_SAMPLES   (50u)       // Tilt print every 1 s when > 2 deg.
#define IMU_SMOOTH          (0.3f)      // 0..1, lower = smoother.
#define RAD_TO_DEG_X10      (572.9578f)

static bool     volatile g_b_ok    = false;
static bool     volatile g_b_ramp  = false;
static uint32_t volatile g_tilt_x10 = 0u;

static ID       g_dd   = 0;
static ER       g_last_er = E_OK;       // Last I2C error (diagnostics).
static uint8_t  g_addr = IMU_ADDR_LOW;
static uint8_t  g_chip = CHIP_NONE;
static uint32_t g_addr_flips = 0u;      // LSM303D address changes seen.

static bool     imu_write(uint8_t reg, uint8_t value);
static bool     imu_read(uint8_t reg, uint8_t * p_buf, uint32_t size);
static bool     imu_read_accel(float * p_acc);
static bool     imu_setup(void);
static void     imu_report_fail(uint32_t attempt);
static void     imu_bus_recover(void);
static void     imu_track(float const * p_grav, float * p_filt,
                          float * p_acc);

// Chip set-up prints: on at boot, off for repeated recoveries.
//
static bool     g_b_verbose = true;
static float    imu_sqrt(float x);
static uint32_t imu_angle_x10(float const * p_a, float const * p_g);

/*!
 * @brief  True while the IMU answers.
 */
bool
imu_is_ok(void)
{
    return (g_b_ok);
}

/*!
 * @brief  True while the car is tilted (on a ramp), with hysteresis.
 */
bool
imu_on_ramp(void)
{
    return (g_b_ramp);
}

/*!
 * @brief  Tilt from the boot attitude, in 0.1 deg.
 */
uint32_t
imu_get_tilt_x10(void)
{
    return (g_tilt_x10);
}

/*!
 * @brief  IMU task: set up, calibrate, then track the tilt for ever.
 */
void
imu_task(INT stacd, void * p_exinf)
{
    // Reference "flat" gravity.  Default = this car's mounting (flat run
    // 2026-11-04: x -16 y 4 z -951 mg), used until a level reading works.
    //
    float    grav[3] = { 0.0f, 0.0f, -1.0f };
    bool     b_leveled = false;
    float    filt[3];
    float    acc[3];
    uint32_t attempt = 0u;
    uint32_t starts  = 0u;

    (void)stacd;
    (void)p_exinf;

    g_dd = tk_opn_dev((UB *)IMU_DEV_NAME, TD_UPDATE);

    // Self-healing: (1) set the chip up, (2) take the level reading once,
    // (3) track the tilt.  IMU_FAIL_LIMIT bad reads in a row in (3) ->
    // back to (1): the chip is set up again (it may have been reset by a
    // power dip, or moved to its other address), the level is kept.
    //
    for (;;)
    {
        // (1) Set up, retrying every IMU_RETRY_MS, freeing the bus first.
        //
        while ((g_dd < E_OK) || !imu_setup())
        {
            imu_report_fail(attempt);
            attempt++;
            g_b_verbose = (attempt <= 3u);
            (void)nav_raise_fault(NAV_FLT_IMU);
            (void)tk_dly_tsk(IMU_RETRY_MS);
            imu_bus_recover();
        }
        g_b_verbose = false;
        attempt     = 0u;
        starts++;

        // (2) Level reading, first start only (later the car may be on a
        // slope).  Two tries; then the default mounting is used.
        //
        for (uint32_t tries = 0u; (!b_leveled) && (tries < 2u); tries++)
        {
            uint32_t good   = 0u;
            float    sum[3] = { 0.0f, 0.0f, 0.0f };

            for (uint32_t i = 0u; i < IMU_CAL_SAMPLES; i++)
            {
                if (imu_read_accel(acc))
                {
                    good++;
                    for (uint32_t k = 0u; k < 3u; k++)
                    {
                        sum[k] += acc[k];
                    }
                }
                (void)tk_dly_tsk(IMU_PERIOD_MS);
            }

            if (good >= (IMU_CAL_SAMPLES / 2u))
            {
                for (uint32_t k = 0u; k < 3u; k++)
                {
                    grav[k] = sum[k] / (float)good;
                }
                b_leveled = true;
                tm_printf((UB *)"[IMU] level: gravity x %d y %d z %d (mg);"
                          " ramp at %u deg\n", (int)(grav[0] * 1000.0f),
                          (int)(grav[1] * 1000.0f),
                          (int)(grav[2] * 1000.0f),
                          (unsigned)IMU_RAMP_ON_DEG);
            }
            else
            {
                tm_printf((UB *)"[IMU] level: only %u/%u good reads (error"
                          " %d)\n", (unsigned)good,
                          (unsigned)IMU_CAL_SAMPLES, (int)g_last_er);
                imu_bus_recover();
                (void)imu_setup();
            }
        }

        if ((!b_leveled) && (1u == starts))
        {
            tm_printf((UB *)"[IMU] level reading failed: using the default"
                      " (IMU flat, z down)\n");
        }

        if (1u == starts)
        {
            barcode_beep(2U, BC_BEEP_MS);       // 2 beeps: IMU ready.
        }
        else
        {
            tm_printf((UB *)"[IMU] running again (restart %u, addr 0x%x)"
                      "\n", (unsigned)(starts - 1u), (unsigned)g_addr);
        }

        g_b_ok = true;
        (void)nav_clear_fault(NAV_FLT_IMU);
        imu_track(grav, filt, acc);

        // imu_track() returned: the IMU stopped answering.
        //
        g_b_ok     = false;
        g_b_ramp   = false;
        g_tilt_x10 = 0u;
        tm_printf((UB *)"[IMU] stopped answering (error %d): setting it up"
                  " again (car keeps driving)\n", (int)g_last_er);
        (void)nav_raise_fault(NAV_FLT_IMU);
        imu_bus_recover();
    }
}

/*!
 * @brief  Track the tilt until IMU_FAIL_LIMIT reads in a row fail.
 */
static void
imu_track(float const * p_grav, float * p_filt, float * p_acc)
{
    uint32_t fails   = 0u;
    uint32_t on_cnt  = 0u;
    uint32_t off_cnt = 0u;
    uint32_t print   = 0u;
    bool     b_first = true;

    while (fails < IMU_FAIL_LIMIT)
    {
        if (imu_read_accel(p_acc))
        {
            fails = 0u;

            for (uint32_t k = 0u; k < 3u; k++)
            {
                if (b_first)
                {
                    p_filt[k] = p_acc[k];
                }
                else
                {
                    p_filt[k] += IMU_SMOOTH * (p_acc[k] - p_filt[k]);
                }
            }
            b_first = false;

            uint32_t const tilt = imu_angle_x10(p_filt, p_grav);

            g_tilt_x10 = tilt;

            if (tilt >= (IMU_RAMP_ON_DEG * 10u))
            {
                off_cnt = 0u;
                if ((!g_b_ramp) && (++on_cnt >= IMU_ON_SAMPLES))
                {
                    g_b_ramp = true;
                    tm_printf((UB *)"[IMU] ramp: tilt %u.%u deg\n",
                              (unsigned)(tilt / 10u),
                              (unsigned)(tilt % 10u));
                }
            }
            else if (tilt <= (IMU_RAMP_OFF_DEG * 10u))
            {
                on_cnt = 0u;
                if (g_b_ramp && (++off_cnt >= IMU_OFF_SAMPLES))
                {
                    g_b_ramp = false;
                    tm_printf((UB *)"[IMU] flat again\n");
                }
            }
            else
            {
                // Between the thresholds: keep the state.
            }

            print++;
            if ((print >= IMU_PRINT_SAMPLES) && (tilt > 20u))
            {
                print = 0u;
                tm_printf((UB *)"[IMU] tilt %u.%u deg%s (addr 0x%x, %u"
                          " flips)\n", (unsigned)(tilt / 10u),
                          (unsigned)(tilt % 10u),
                          g_b_ramp ? " (ramp)" : "", (unsigned)g_addr,
                          (unsigned)g_addr_flips);
            }
        }
        else
        {
            fails++;
        }

        (void)tk_dly_tsk(IMU_PERIOD_MS);
    }
}

/*!
 * @brief  Find the chip and configure it.  Tries, in order: MPU6050/
 *         6500/9250 (0x68, 0x69), ADXL345 (0x53, 0x1D), MMA8451/2/3
 *         (0x1C, 0x1D), LSM303D / LSM9DS0 accelerometer (0x1E, 0x1D).
 *         Each is told apart by its ID register.
 * @return true if a known chip answered.
 */
static bool
imu_setup(void)
{
    uint8_t  id   = 0u;
    bool     b_ok = false;
    uint32_t i;

    g_chip = CHIP_NONE;

    for (i = 0u; (i < IMU_PROBES) && (CHIP_NONE == g_chip); i++)
    {
        g_addr = g_probe[i].addr;

        if (imu_read(g_probe[i].id_reg, &id, 1u))
        {
            if (g_b_verbose)
            {
                tm_printf((UB *)"[IMU] 0x%x answers: reg 0x%x = 0x%x\n",
                          (unsigned)g_addr, (unsigned)g_probe[i].id_reg,
                          (unsigned)id);
            }

            if ((id == g_probe[i].id_a) || (id == g_probe[i].id_b) ||
                (id == g_probe[i].id_c) || (id == g_probe[i].id_d))
            {
                g_chip = g_probe[i].chip;
            }
        }
    }

    switch (g_chip)
    {
        case CHIP_MPU:
            if (g_b_verbose)
            {
                tm_printf((UB *)"[IMU] MPU6050/6500/9250 at 0x%x (id 0x%x)\n",
                          (unsigned)g_addr, (unsigned)id);
            }
            (void)imu_write(REG_PWR_MGMT_1, PWR_RESET);
            (void)tk_dly_tsk(100);
            b_ok = imu_write(REG_PWR_MGMT_1, PWR_CLK_PLL) &&
                   imu_write(REG_CONFIG, ACCEL_DLPF_10HZ) &&
                   imu_write(REG_ACCEL_CONFIG, ACCEL_2G) &&
                   imu_write(REG_ACCEL_CONFIG2, ACCEL_DLPF_10HZ);
            break;

        case CHIP_ADXL345:
            if (g_b_verbose)
            {
                tm_printf((UB *)"[IMU] ADXL345 at 0x%x\n", (unsigned)g_addr);
            }
            b_ok = imu_write(ADXL_BW_RATE, ADXL_RATE_50HZ) &&
                   imu_write(ADXL_DATA_FORMAT, ADXL_FULL_RES_2G) &&
                   imu_write(ADXL_POWER_CTL, ADXL_MEASURE);
            break;

        case CHIP_MMA845X:
            if (g_b_verbose)
            {
                tm_printf((UB *)"[IMU] MMA845x at 0x%x (id 0x%x)\n",
                          (unsigned)g_addr, (unsigned)id);
            }
            b_ok = imu_write(MMA_CTRL_REG1, 0x00u) &&       // standby
                   imu_write(MMA_XYZ_DATA_CFG, 0x00u) &&    // +/-2 g
                   imu_write(MMA_CTRL_REG1, MMA_ACTIVE_50HZ);
            break;

        case CHIP_LSM303D:
            if (g_b_verbose)
            {
                tm_printf((UB *)"[IMU] LSM303D/LSM9DS0 at 0x%x\n",
                          (unsigned)g_addr);
            }
            b_ok = imu_write(LSM_CTRL2, 0x00u) &&            // +/-2 g
                   imu_write(LSM_CTRL1, LSM_50HZ_XYZ);
            break;

        default:
            if (g_b_verbose)
            {
                tm_printf((UB *)"[IMU] no known accelerometer found\n");
            }
            b_ok = false;
            break;
    }

    (void)tk_dly_tsk(50);

    return (b_ok);
}

/*!
 * @brief  Write one register.
 */
static bool
imu_write(uint8_t reg, uint8_t value)
{
    return (E_OK <= i2c_write_reg(g_dd, (UW)g_addr, (UW)reg, (UB)value));
}

/*!
 * @brief  Read size registers from reg on (auto-increment).
 */
static bool
imu_read(uint8_t reg, uint8_t * p_buf, uint32_t size)
{
    T_I2C_EXEC exec;
    UB         start = (UB)reg;
    SZ         asize = 0;

    exec.sadr     = (UW)g_addr;
    exec.snd_size = 1;
    exec.snd_data = &start;
    exec.rcv_size = (SZ)size;
    exec.rcv_data = (UB *)p_buf;

    g_last_er = tk_swri_dev(g_dd, TDN_I2C_EXEC, &exec, (SZ)sizeof(exec),
                            &asize);

    // LSM303D: its address pin (SA0) floats on this board, so the chip
    // flips between 0x1D and 0x1E (seen in the logs).  No ACK -> try the
    // other address once and stay there if it answers.
    //
    if ((E_OK > g_last_er) && (CHIP_LSM303D == g_chip))
    {
        uint8_t const other = (0x1Du == g_addr) ? 0x1Eu : 0x1Du;

        exec.sadr = (UW)other;
        g_last_er = tk_swri_dev(g_dd, TDN_I2C_EXEC, &exec,
                                (SZ)sizeof(exec), &asize);
        if (E_OK <= g_last_er)
        {
            g_addr = other;
            g_addr_flips++;
        }
    }

    return (E_OK <= g_last_er);
}

/*!
 * @brief  Say why the IMU is not answering.  First failure (and every
 *         IMU_SCAN_EVERY after): scan every 7-bit address for anything
 *         that ACKs.  -57 (E_IO) = no ACK: nothing at that address, or
 *         SDA/SCL not connected.  -50 (E_TMOUT) = the bus is stuck (no
 *         pull-ups, SDA/SCL swapped or shorted): no scan then.
 */
static void
imu_report_fail(uint32_t attempt)
{
    int32_t const code    = (int32_t)g_last_er;
    bool const    b_print = ((attempt < 3u) ||
                             (0u == (attempt % IMU_SCAN_EVERY)));

    if (b_print)
    {
        tm_printf((UB *)"[IMU] no accelerometer set up on I2C1 GP2 SDA /"
                  " GP3 SCL (try %u, error %d%s); retrying\n",
                  (unsigned)attempt, (int)code,
                  (-57 == code) ? " = no ACK" :
                  ((-50 == code) ? " = bus stuck/timeout" : ""));
    }

    if ((g_dd >= E_OK) && (-50 != code) &&
        (0u == (attempt % IMU_SCAN_EVERY)))
    {
        uint32_t found = 0u;

        for (uint32_t addr = 0x08u; addr < 0x78u; addr++)
        {
            UB byte = 0u;

            if (E_OK <= i2c_read_reg(g_dd, (UW)addr, 0u, &byte))
            {
                tm_printf((UB *)"[IMU] scan: device ACKs at 0x%x\n",
                          (unsigned)addr);
                found++;
            }
        }
        tm_printf((UB *)"[IMU] scan done: %u device(s) on GP2/GP3%s\n",
                  (unsigned)found, (0u == found) ?
                  " (check the Grove 2 cable / power)" : "");
    }
}

/*!
 * @brief  Accelerometer in g.  Every chip is scaled to 16384 counts per g
 *         (MPU +/-2 g; MMA845x left-justified; LSM303D 0.061 mg; ADXL345
 *         full-res 3.9 mg = 256 per g, x 64).  The tilt only uses the
 *         direction, so the scale need not be exact.
 */
static bool
imu_read_accel(float * p_acc)
{
    uint8_t raw[6];
    bool    b_ok   = false;
    bool    b_big  = true;      // Big-endian (high byte first).
    float   per_g  = 16384.0f;

    switch (g_chip)
    {
        case CHIP_MPU:
            b_ok = imu_read(REG_ACCEL_XOUT_H, raw, 6u);
            break;

        case CHIP_ADXL345:
            b_ok  = imu_read(ADXL_DATAX0, raw, 6u);
            b_big = false;
            per_g = 256.0f;
            break;

        case CHIP_MMA845X:
            b_ok = imu_read(MMA_OUT_X_MSB, raw, 6u);
            break;

        case CHIP_LSM303D:
            b_ok  = imu_read(LSM_OUT_X_L_A | LSM_AUTO_INC, raw, 6u);
            b_big = false;
            break;

        default:
            b_ok = false;
            break;
    }

    if (b_ok)
    {
        float mag2 = 0.0f;

        for (uint32_t k = 0u; k < 3u; k++)
        {
            uint32_t const hi = b_big ? (2u * k) : ((2u * k) + 1u);
            uint32_t const lo = b_big ? ((2u * k) + 1u) : (2u * k);
            int16_t const  v  = (int16_t)(((uint16_t)raw[hi] << 8) |
                                          (uint16_t)raw[lo]);

            p_acc[k] = (float)v / per_g;
            mag2    += p_acc[k] * p_acc[k];
        }

        // Not near 1 g (0.5 .. 1.5): a glitch, or the chip was reset by
        // a power dip and is powered down (reads all zero).  Bad sample.
        //
        if ((mag2 < 0.25f) || (mag2 > 2.25f))
        {
            b_ok      = false;
            g_last_er = E_OBJ;
        }
    }

    return (b_ok);
}

/*!
 * @brief  Square root (Newton); no maths library in this build.
 */
static float
imu_sqrt(float x)
{
    float r = (x > 1.0f) ? x : 1.0f;

    if (x <= 0.0f)
    {
        r = 0.0f;
    }
    else
    {
        for (uint32_t i = 0u; i < 20u; i++)
        {
            r = 0.5f * (r + (x / r));
        }
    }

    return (r);
}

/*!
 * @brief  Angle between two vectors, 0.1 deg, from 0 to 90 deg.
 *         acos(c) ~ sqrt(1 - c) * (1.5707288 - 0.2121144 c + 0.0742610 c^2
 *         - 0.0187293 c^3) for 0 <= c <= 1 (error < 0.01 deg).
 */
static uint32_t
imu_angle_x10(float const * p_a, float const * p_g)
{
    float const dot = (p_a[0] * p_g[0]) + (p_a[1] * p_g[1]) +
                      (p_a[2] * p_g[2]);
    float const na  = imu_sqrt((p_a[0] * p_a[0]) + (p_a[1] * p_a[1]) +
                               (p_a[2] * p_a[2]));
    float const ng  = imu_sqrt((p_g[0] * p_g[0]) + (p_g[1] * p_g[1]) +
                               (p_g[2] * p_g[2]));
    float       c   = 0.0f;
    float       rad;

    if ((na > 0.1f) && (ng > 0.1f))
    {
        c = dot / (na * ng);
    }

    if (c > 1.0f)
    {
        c = 1.0f;
    }
    else if (c < 0.0f)
    {
        c = 0.0f;           // 90 deg or more: report 90.
    }
    else
    {
        // In range.
    }

    rad = imu_sqrt(1.0f - c) *
          (1.5707288f + (c * (-0.2121144f + (c * (0.0742610f +
                                                  (c * -0.0187293f))))));

    return ((uint32_t)((rad * RAD_TO_DEG_X10) + 0.5f));
}

/*!
 * @brief  Free a stuck I2C bus.  A motor spike in the middle of a read can
 *         leave the chip holding SDA low, waiting for clocks that never
 *         come; then every later transfer fails.  Take the pins as GPIO,
 *         give 9 clocks on SCL, send a STOP, and hand them back to I2C.
 *         Open drain by hand: output value 0, enable = pull low, disable =
 *         let the pull-up raise the line.
 */
static void
imu_bus_recover(void)
{
    uint32_t const sda = (1uL << IMU_SDA_PIN);
    uint32_t const scl = (1uL << IMU_SCL_PIN);

    out_w(GPIO_OE_CLR, sda | scl);              // Both released (high).
    out_w(GPIO_OUT_CLR, sda | scl);
    out_w(GPIO_CTRL(IMU_SDA_PIN), GPIO_CTRL_FUNCSEL_SIO);
    out_w(GPIO_CTRL(IMU_SCL_PIN), GPIO_CTRL_FUNCSEL_SIO);
    (void)tk_dly_tsk(1);

    for (uint32_t i = 0u; i < 9u; i++)
    {
        out_w(GPIO_OE_SET, scl);                // SCL low.
        (void)tk_dly_tsk(1);
        out_w(GPIO_OE_CLR, scl);                // SCL high.
        (void)tk_dly_tsk(1);
    }

    out_w(GPIO_OE_SET, sda);                    // STOP: SDA low ...
    (void)tk_dly_tsk(1);
    out_w(GPIO_OE_CLR, sda);                    // ... then high, SCL high.
    (void)tk_dly_tsk(1);

    // Reset the RP2040's I2C1 block as well (after a 1 s timeout it can
    // stay wedged), then open the device again so the driver programs
    // it from scratch.  Only this task uses I2C1.
    //
    if (g_dd >= E_OK)
    {
        (void)tk_cls_dev(g_dd, 0);
    }
    set_w(RESETS_RESET, RESETS_RESET_I2C1);
    clr_w(RESETS_RESET, RESETS_RESET_I2C1);
    while (0u == (in_w(RESETS_RESET_DONE) & RESETS_RESET_I2C1))
    {
        // Wait for the reset to finish (a few clocks).
    }

    out_w(GPIO_CTRL(IMU_SDA_PIN), GPIO_CTRL_FUNCSEL_I2C);
    out_w(GPIO_CTRL(IMU_SCL_PIN), GPIO_CTRL_FUNCSEL_I2C);
    g_dd = tk_opn_dev((UB *)IMU_DEV_NAME, TD_UPDATE);
}

