/**
 * @file    drv_i2c1.c
 * @brief   Polled RP2040 I2C1 master with timeouts and bus recovery.
 *
 * Register-level driver for the Synopsys DW_apb_i2c block (RP2040 datasheet,
 * section 4.3).  The transfer sequences follow pico-sdk's
 * i2c_write_blocking_internal() / i2c_read_blocking_internal() (BSD-3), with
 * every busy-wait bounded by a microsecond deadline AND a poll-count cap, so
 * a stopped timer cannot hang the task either.  No interrupt is used.
 */

#include <stdbool.h>
#include <tk/tkernel.h>

#include "drv_i2c1.h"
#include "drv_us_timer.h"

/* I2C1 register addresses (datasheet 4.3.17). */
#define I2C1_BASE_ADDR          (0x40048000u)
#define IC_CON                  (I2C1_BASE_ADDR + 0x00u)
#define IC_TAR                  (I2C1_BASE_ADDR + 0x04u)
#define IC_DATA_CMD             (I2C1_BASE_ADDR + 0x10u)
#define IC_FS_SCL_HCNT          (I2C1_BASE_ADDR + 0x1Cu)
#define IC_FS_SCL_LCNT          (I2C1_BASE_ADDR + 0x20u)
#define IC_INTR_MASK            (I2C1_BASE_ADDR + 0x30u)
#define IC_RAW_INTR_STAT        (I2C1_BASE_ADDR + 0x34u)
#define IC_RX_TL                (I2C1_BASE_ADDR + 0x38u)
#define IC_TX_TL                (I2C1_BASE_ADDR + 0x3Cu)
#define IC_CLR_TX_ABRT          (I2C1_BASE_ADDR + 0x54u)
#define IC_CLR_STOP_DET         (I2C1_BASE_ADDR + 0x60u)
#define IC_ENABLE               (I2C1_BASE_ADDR + 0x6Cu)
#define IC_TXFLR                (I2C1_BASE_ADDR + 0x74u)
#define IC_RXFLR                (I2C1_BASE_ADDR + 0x78u)
#define IC_SDA_HOLD             (I2C1_BASE_ADDR + 0x7Cu)
#define IC_TX_ABRT_SOURCE       (I2C1_BASE_ADDR + 0x80u)
#define IC_FS_SPKLEN            (I2C1_BASE_ADDR + 0xA0u)

/* Register bits. */
#define CON_MASTER_MODE         (0x00000001u)
#define CON_SPEED_FAST          (0x00000004u)
#define CON_RESTART_EN          (0x00000020u)
#define CON_SLAVE_DISABLE       (0x00000040u)
#define CON_TX_EMPTY_CTRL       (0x00000100u)
#define DATA_CMD_READ           (0x00000100u)
#define DATA_CMD_STOP           (0x00000200u)
#define DATA_CMD_RESTART        (0x00000400u)
#define DATA_CMD_BYTE_MASK      (0x000000FFu)
#define RAW_TX_EMPTY            (0x00000010u)
#define RAW_STOP_DET            (0x00000200u)
#define ENABLE_BIT              (0x00000001u)
#define TX_FIFO_DEPTH           (16u)
#define ADDRESS_7BIT_MAX        (0x7Fu)

/*
 * 400 kHz timing for clk_peri = 125 MHz (CLK_PERI_FREQ in sysdef.h), worked
 * out as in pico-sdk i2c_set_baudrate():
 *   period = (125 MHz + 200 kHz) / 400 kHz = 313 clocks
 *   low    = period * 3 / 5 = 187,  high = period - low = 126
 *   spike  = low / 16 = 11,  SDA hold = 125 MHz * 300 ns + 1 = 38
 * These match the kernel's own driver (device/i2c/sysdepend/rp2040).
 */
#define SCL_HCNT_400K           (126u)
#define SCL_LCNT_400K           (187u)
#define SPKLEN_400K             (11u)
#define SDA_HOLD_400K           (38u)

/* Bit-banged recovery (NXP UM10204, section 3.1.16 "Bus clear"). */
#define RECOVERY_PULSES         (9u)
#define RECOVERY_HALF_PERIOD_US (5u)
#define RECOVERY_STRETCH_US     (1000u)

/* Poll-count cap: about 8 polls per microsecond is far more than the loop
 * can actually do at 125 MHz, so the time deadline normally fires first.
 */
#define POLLS_PER_US            (8u)
#define RESET_DONE_POLLS        (10000u)

/** Deadline for one transfer. */
typedef struct
{
    uint32_t start_us;
    uint32_t budget_us;
    uint32_t polls;
} deadline_t;

static uint32_t volatile g_last_abort_source = 0u;

static void              deadline_start (deadline_t * p_deadline,
                                         uint32_t bytes);
static bool              deadline_expired (deadline_t * p_deadline);
static void              select_target (uint8_t address);
static bool              take_abort (void);
static void              wait_stop_detect (deadline_t * p_deadline);
static drv_i2c1_status_t write_bytes (uint8_t const * p_data,
                                      uint32_t length, bool b_send_stop,
                                      deadline_t * p_deadline);
static void              pins_to_i2c (void);
static bool              line_is_high (uint32_t pin);
static void              line_release (uint32_t pin);
static void              line_drive_low (uint32_t pin);

/**
 * @brief   Reset the I2C1 block, claim GP2/GP3 and configure 400 kHz master.
 * @return  DRV_I2C1_OK, or DRV_I2C1_ERR_TIMEOUT if the reset never ended.
 */
drv_i2c1_status_t
drv_i2c1_init (void)
{
    drv_i2c1_status_t status = DRV_I2C1_OK;
    uint32_t          polls  = 0u;

    set_w(RESETS_RESET, RESETS_RESET_I2C1);
    clr_w(RESETS_RESET, RESETS_RESET_I2C1);

    while ((0u == (in_w(RESETS_RESET_DONE) & RESETS_RESET_I2C1)) &&
           (polls < RESET_DONE_POLLS))
    {
        polls++;
    }

    if (polls >= RESET_DONE_POLLS)
    {
        status = DRV_I2C1_ERR_TIMEOUT;
    }
    else
    {
        pins_to_i2c();

        out_w(IC_ENABLE, 0u);
        out_w(IC_CON, CON_MASTER_MODE | CON_SPEED_FAST | CON_RESTART_EN |
                      CON_SLAVE_DISABLE | CON_TX_EMPTY_CTRL);
        out_w(IC_TX_TL, 0u);
        out_w(IC_RX_TL, 0u);
        out_w(IC_FS_SCL_HCNT, SCL_HCNT_400K);
        out_w(IC_FS_SCL_LCNT, SCL_LCNT_400K);
        out_w(IC_FS_SPKLEN, SPKLEN_400K);
        out_w(IC_SDA_HOLD, SDA_HOLD_400K);
        out_w(IC_INTR_MASK, 0u);
        out_w(IC_ENABLE, ENABLE_BIT);
    }

    return (status);
}

/**
 * @brief   Write bytes to a slave, ending with STOP.
 * @param   address  7-bit slave address.
 * @param   p_data   Bytes to send (e.g. register address then value).
 * @param   length   1 to DRV_I2C1_MAX_TRANSFER.
 * @return  DRV_I2C1_OK or an error code.
 */
drv_i2c1_status_t
drv_i2c1_write (uint8_t address, uint8_t const * p_data, uint32_t length)
{
    drv_i2c1_status_t status = DRV_I2C1_ERR_PARAM;

    if ((NULL != p_data) && (length > 0u) &&
        (length <= DRV_I2C1_MAX_TRANSFER) && (address <= ADDRESS_7BIT_MAX))
    {
        deadline_t deadline;

        deadline_start(&deadline, length);
        select_target(address);
        status = write_bytes(p_data, length, true, &deadline);
    }

    return (status);
}

/**
 * @brief   Read consecutive registers: write @p reg, repeated START, read.
 * @param   address   7-bit slave address.
 * @param   reg       First register to read.
 * @param   p_buffer  Receives @p length bytes.
 * @param   length    1 to DRV_I2C1_MAX_TRANSFER.
 * @return  DRV_I2C1_OK or an error code.
 */
drv_i2c1_status_t
drv_i2c1_read_reg (uint8_t address, uint8_t reg, uint8_t * p_buffer,
                   uint32_t length)
{
    drv_i2c1_status_t status = DRV_I2C1_ERR_PARAM;

    if ((NULL != p_buffer) && (length > 0u) &&
        (length <= DRV_I2C1_MAX_TRANSFER) && (address <= ADDRESS_7BIT_MAX))
    {
        deadline_t deadline;

        deadline_start(&deadline, length + 1u);
        select_target(address);
        status = write_bytes(&reg, 1u, false, &deadline);

        for (uint32_t index = 0u;
             (DRV_I2C1_OK == status) && (index < length); index++)
        {
            uint32_t cmd = DATA_CMD_READ;

            if (0u == index)
            {
                cmd |= DATA_CMD_RESTART;
            }

            if ((index + 1u) == length)
            {
                cmd |= DATA_CMD_STOP;
            }

            while ((in_w(IC_TXFLR) >= TX_FIFO_DEPTH) &&
                   !deadline_expired(&deadline))
            {
                /* Wait for room in the command FIFO. */
            }

            out_w(IC_DATA_CMD, cmd);

            bool b_abort = false;

            while ((0u == in_w(IC_RXFLR)) && !b_abort &&
                   !deadline_expired(&deadline))
            {
                b_abort = take_abort();
            }

            if (b_abort)
            {
                wait_stop_detect(&deadline);
                status = DRV_I2C1_ERR_NACK;
            }
            else if (0u == in_w(IC_RXFLR))
            {
                status = DRV_I2C1_ERR_TIMEOUT;
            }
            else
            {
                /* Cast: the mask keeps only the 8 data bits. */
                p_buffer[index] = (uint8_t)(in_w(IC_DATA_CMD) &
                                            DATA_CMD_BYTE_MASK);
            }
        }

        if (DRV_I2C1_OK == status)
        {
            wait_stop_detect(&deadline);
        }
        else
        {
            out_w(IC_ENABLE, 0u);
        }
    }

    return (status);
}

/**
 * @brief   Free a bus whose slave holds SDA low, then re-initialise I2C1.
 *
 * Switches GP2/GP3 to software control, clocks SCL up to nine times until
 * the slave releases SDA, sends a STOP, and restores the I2C function.
 *
 * @return  DRV_I2C1_OK if both lines are high afterwards, otherwise
 *          DRV_I2C1_ERR_BUS_STUCK.
 */
drv_i2c1_status_t
drv_i2c1_recover_bus (void)
{
    drv_i2c1_status_t status = DRV_I2C1_OK;

    out_w(IC_ENABLE, 0u);

    /* Open-drain emulation: output value 0, toggle only output-enable. */
    out_w(GPIO_OUT_CLR, (1u << DRV_I2C1_SDA_PIN) | (1u << DRV_I2C1_SCL_PIN));
    line_release(DRV_I2C1_SDA_PIN);
    line_release(DRV_I2C1_SCL_PIN);
    out_w(GPIO_CTRL(DRV_I2C1_SDA_PIN), GPIO_CTRL_FUNCSEL_SIO);
    out_w(GPIO_CTRL(DRV_I2C1_SCL_PIN), GPIO_CTRL_FUNCSEL_SIO);
    drv_us_timer_delay(RECOVERY_HALF_PERIOD_US);

    for (uint32_t pulse = 0u;
         (pulse < RECOVERY_PULSES) && !line_is_high(DRV_I2C1_SDA_PIN);
         pulse++)
    {
        line_drive_low(DRV_I2C1_SCL_PIN);
        drv_us_timer_delay(RECOVERY_HALF_PERIOD_US);
        line_release(DRV_I2C1_SCL_PIN);

        uint32_t const start = drv_us_timer_now();

        while (!line_is_high(DRV_I2C1_SCL_PIN) &&
               (drv_us_timer_elapsed(start) < RECOVERY_STRETCH_US))
        {
            /* Slave may stretch the clock; wait for SCL to rise. */
        }

        drv_us_timer_delay(RECOVERY_HALF_PERIOD_US);
    }

    /* STOP condition: SDA rises while SCL is high. */
    line_drive_low(DRV_I2C1_SCL_PIN);
    drv_us_timer_delay(RECOVERY_HALF_PERIOD_US);
    line_drive_low(DRV_I2C1_SDA_PIN);
    drv_us_timer_delay(RECOVERY_HALF_PERIOD_US);
    line_release(DRV_I2C1_SCL_PIN);
    drv_us_timer_delay(RECOVERY_HALF_PERIOD_US);
    line_release(DRV_I2C1_SDA_PIN);
    drv_us_timer_delay(RECOVERY_HALF_PERIOD_US);

    if (!line_is_high(DRV_I2C1_SDA_PIN) || !line_is_high(DRV_I2C1_SCL_PIN))
    {
        status = DRV_I2C1_ERR_BUS_STUCK;
    }

    drv_i2c1_status_t const init_status = drv_i2c1_init();

    if (DRV_I2C1_OK == status)
    {
        status = init_status;
    }

    return (status);
}

/**
 * @brief   Abort-source register captured at the last NACK (diagnostics).
 * @return  IC_TX_ABRT_SOURCE value, 0 if no abort has happened.
 */
uint32_t
drv_i2c1_last_abort_source (void)
{
    return (g_last_abort_source);
}

/**
 * @brief   Start a deadline sized for a transfer of @p bytes.
 * @param   p_deadline  Deadline to initialise.
 * @param   bytes       Number of bytes on the bus.
 */
static void
deadline_start (deadline_t * p_deadline, uint32_t bytes)
{
    p_deadline->start_us  = drv_us_timer_now();
    p_deadline->budget_us = DRV_I2C1_TIMEOUT_BASE_US +
                            (bytes * DRV_I2C1_TIMEOUT_PER_BYTE_US);
    p_deadline->polls     = 0u;
}

/**
 * @brief   Count one poll and report whether the deadline has passed.
 * @param   p_deadline  Running deadline.
 * @return  true once the time budget or the poll cap is exceeded.
 */
static bool
deadline_expired (deadline_t * p_deadline)
{
    p_deadline->polls++;

    return ((drv_us_timer_elapsed(p_deadline->start_us) >
             p_deadline->budget_us) ||
            (p_deadline->polls > (p_deadline->budget_us * POLLS_PER_US)));
}

/**
 * @brief   Set the slave address (the block must be disabled to do this).
 * @param   address  7-bit slave address.
 */
static void
select_target (uint8_t address)
{
    out_w(IC_ENABLE, 0u);
    out_w(IC_TAR, address);
    out_w(IC_ENABLE, ENABLE_BIT);
}

/**
 * @brief   Check for and clear a transmit abort (NACK, arbitration loss).
 * @return  true if the transfer was aborted.
 */
static bool
take_abort (void)
{
    uint32_t const source  = in_w(IC_TX_ABRT_SOURCE);
    bool           b_abort = false;

    if (0u != source)
    {
        g_last_abort_source = source;

        /* Reading IC_CLR_TX_ABRT clears the abort and releases the FIFO. */
        (void)in_w(IC_CLR_TX_ABRT);
        b_abort = true;
    }

    return (b_abort);
}

/**
 * @brief   Wait (bounded) for the STOP condition and clear its status bit.
 * @param   p_deadline  Running deadline.
 */
static void
wait_stop_detect (deadline_t * p_deadline)
{
    while ((0u == (in_w(IC_RAW_INTR_STAT) & RAW_STOP_DET)) &&
           !deadline_expired(p_deadline))
    {
        /* The controller sends STOP after the last byte or an abort. */
    }

    (void)in_w(IC_CLR_STOP_DET);
}

/**
 * @brief   Push bytes into the transmit FIFO one at a time.
 * @param   p_data       Bytes to send.
 * @param   length       Number of bytes, >= 1.
 * @param   b_send_stop  true to end the last byte with STOP.
 * @param   p_deadline   Running deadline.
 * @return  DRV_I2C1_OK, DRV_I2C1_ERR_NACK or DRV_I2C1_ERR_TIMEOUT.
 */
static drv_i2c1_status_t
write_bytes (uint8_t const * p_data, uint32_t length, bool b_send_stop,
             deadline_t * p_deadline)
{
    drv_i2c1_status_t status = DRV_I2C1_OK;

    for (uint32_t index = 0u; (DRV_I2C1_OK == status) && (index < length);
         index++)
    {
        bool const b_last = ((index + 1u) == length);
        uint32_t   cmd    = p_data[index];

        if (b_last && b_send_stop)
        {
            cmd |= DATA_CMD_STOP;
        }

        out_w(IC_DATA_CMD, cmd);

        /* With TX_EMPTY_CTRL set, TX_EMPTY means the byte has left. */
        while ((0u == (in_w(IC_RAW_INTR_STAT) & RAW_TX_EMPTY)) &&
               !deadline_expired(p_deadline))
        {
            /* Wait for the byte to be clocked out. */
        }

        if (take_abort())
        {
            wait_stop_detect(p_deadline);
            status = DRV_I2C1_ERR_NACK;
        }
        else if (0u == (in_w(IC_RAW_INTR_STAT) & RAW_TX_EMPTY))
        {
            status = DRV_I2C1_ERR_TIMEOUT;
        }
        else if (b_last && b_send_stop)
        {
            wait_stop_detect(p_deadline);
        }
        else
        {
            /* More bytes, or a repeated START follows. */
        }
    }

    if (DRV_I2C1_ERR_TIMEOUT == status)
    {
        out_w(IC_ENABLE, 0u);
    }

    return (status);
}

/**
 * @brief   Give GP2/GP3 to the I2C block with pull-ups and Schmitt inputs.
 */
static void
pins_to_i2c (void)
{
    uint32_t const pad = GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT;

    out_w(GPIO(DRV_I2C1_SDA_PIN), pad);
    out_w(GPIO(DRV_I2C1_SCL_PIN), pad);
    out_w(GPIO_CTRL(DRV_I2C1_SDA_PIN), GPIO_CTRL_FUNCSEL_I2C);
    out_w(GPIO_CTRL(DRV_I2C1_SCL_PIN), GPIO_CTRL_FUNCSEL_I2C);
}

/**
 * @brief   Read a bus line while it is under software control.
 * @param   pin  GPIO number.
 * @return  true if the line is high.
 */
static bool
line_is_high (uint32_t pin)
{
    return (0u != (in_w(GPIO_IN) & (1u << pin)));
}

/**
 * @brief   Release a line (output disabled; the pull-up takes it high).
 * @param   pin  GPIO number.
 */
static void
line_release (uint32_t pin)
{
    out_w(GPIO_OE_CLR, 1u << pin);
}

/**
 * @brief   Pull a line low (output value is already 0).
 * @param   pin  GPIO number.
 */
static void
line_drive_low (uint32_t pin)
{
    out_w(GPIO_OE_SET, 1u << pin);
}

/*** end of file ***/
