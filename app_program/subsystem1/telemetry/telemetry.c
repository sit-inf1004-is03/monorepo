/**
 * @file    telemetry.c
 * @brief   Telemetry: every telemetry period, read the car's state from the
 *          other subsystems, pack it into two 64-bit words and queue them
 *          for the network bridge.  In between, every EVENT_POLL_MS, send
 *          any new hump or barcode as an event (telemetry_events.c).
 *          Coded to BARR-C:2018.
 *
 * Data flow: Navigation, Motion, encoders, line sensor, ultrasonic, IMU,
 * barcode and Wi-Fi signal, into protocol_tlm_put(), comms_telemetry(),
 * the dashboard.  Settings: dashboard, network bridge, the telemetry
 * message buffer, apply_setting().
 *
 * One blocking call does both jobs: the task waits for a setting, but only
 * until the next sample or event poll is due.
 *
 * Timing: a sample every PERIOD_DEFAULT_MS (500 ms) until the dashboard
 * sets another period.  Each time the car comes online, at start-up or
 * after a dropped connection, the first sample is sent FIRST_SAMPLE_MS
 * (300 ms) later instead of whenever the period comes round, so the
 * dashboard has fresh values soon after every reconnect.
 *
 * Sources, the team's interfaces, used through their headers:
 *   nav_get_state(), _reason(), _faults()  IF-14  navigation/navigation.h
 *   motion_get_activity()                  IF-09  subsystem2/motion.h
 *   drv_encoder_get_speed(), _count()      IF-06  subsystem2/drv_encoder.h
 *   line_get_state()                       IF-16  subsystem3/line_barcode.h
 *   ultrasonic_last_reading()              IF-16  subsystem5/drv_ultrasonic.h
 *   imu_task_get_snapshot()                IF-S4-1 subsystem4/imu_task.h
 *   imu_is_ok()                            IF-S4-7 subsystem4/imu_ramp.h
 *   barcode_get_last(), imu_task_read_event()  in telemetry_events.c
 */

#include "telemetry.h"
#include "telemetry_events.h"
#include "../comms/comms.h"
#include "../../navigation/navigation.h"
#include "../../subsystem2/motion.h"
#include "../../subsystem2/drv_encoder.h"
#include "../../subsystem3/line_barcode.h"
#include "../../subsystem4/subsystem4.h"
#include "../../subsystem4/imu_ramp.h"
#include "../../subsystem5/drv_ultrasonic.h"

#define PERIOD_MIN_MS           (100)
#define PERIOD_MAX_MS           (60000)
#define PERIOD_DEFAULT_MS       (500u)
#define FIRST_SAMPLE_MS         (300u)  /* After coming online. */
#define EVENT_POLL_MS           (100)   /* The IMU holds 32 events: room
                                           for 320 ms of its fastest
                                           changes. */
#define SEQ_MASK                (0xFFu)
#define RSSI_UNKNOWN_DBM        (-128)  /* Weaker cannot be reported. */
#define UM_PER_CM               (10000u)
#define BARCODE_LETTER_MASK     (0xFFu) /* barcode_get_last() layout. */
#define BARCODE_READS_SHIFT     (8u)
#define ASCII_MAX               (127)
#define MAP_ENTRY_LEN           (4u)    /* " K=L" */
#define MAP_MORE_LEN            (5u)    /* " ..." and the NUL. */

/* The barcode map's characters, and the letter for each barcode_cmd_t. */
static char const g_map_chars[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
static char const g_map_dirs[]  = "LRSU";

static ID                g_mbf = 0;
static uint32_t volatile g_period_ms = PERIOD_DEFAULT_MS;
static bool volatile     gb_enabled = true;

static void     telemetry_task (INT stacd, void * p_exinf);
static uint32_t now_ms (void);
static uint32_t wheel_speed (uint32_t wheel);
static uint32_t line_state (void);
static uint32_t obstacle_cm (void);
static uint32_t distance_cm (void);
static uint32_t rssi_field (int32_t rssi);
static void     apply_setting (comms_msg_t const * p_msg);
static void     apply_barcode_setting (comms_payload_cmd_t const * p_cmd,
                                       char * p_line);
static bool     set_barcode (comms_payload_cmd_t const * p_cmd);
static void     barcode_map_text (char * p_line, uint32_t size);

/**
 * @brief   Create the settings buffer and subscribe it to robocar/id/config.
 * @return  E_OK, or the first error.
 */
ER
telemetry_init (void)
{
    T_CMBF cmbf = {0};
    ER     result = E_OK;

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = comms_mbf_size(TELEMETRY_MBF_MESSAGES);
    cmbf.maxmsz = sizeof(comms_msg_t);
    g_mbf       = tk_cre_mbf(&cmbf);

    if (g_mbf < E_OK)
    {
        result = g_mbf;
    }
    else
    {
        result = comms_subscribe(COMMS_TOPIC_CONFIG, g_mbf);
    }

    return (result);
}

/**
 * @brief   Start telemetry_task.
 * @return  E_OK, or the error from creating it.
 */
ER
telemetry_start (void)
{
    ER       result = E_OK;
    ID const tid    = app_start_task(telemetry_task, TELEMETRY_PRI, 0u,
                                     "telemetry_task");

    if (tid < E_OK)
    {
        result = tid;
    }

    return (result);
}

/**
 * @brief   The settings buffer, for the self-tests to put settings in as the
 *          network bridge does.
 * @return  Its ID.
 */
ID
telemetry_get_mbf (void)
{
    return (g_mbf);
}

/**
 * @brief   The time between samples, as last set.
 * @return  The period in ms.
 */
uint32_t
telemetry_get_period_ms (void)
{
    return (g_period_ms);
}

/**
 * @brief   Is telemetry being sent?
 * @return  false after CFG_TELEMETRY_ENABLE 0, until it is set to 1.
 */
bool
telemetry_is_enabled (void)
{
    return (gb_enabled);
}

/**
 * @brief   Sample the car into telemetry word 0 (protocol.h TLM_*).
 * @param   seq  The sample count; its low 8 bits are sent.
 * @return  The packed word.
 */
uint64_t
telemetry_sample (uint32_t seq)
{
    net_bridge_info_t net;
    uint64_t          word = 0u;

    comms_get_info(&net);

    word = protocol_tlm_put(word, TLM_SEQ,         seq);
    word = protocol_tlm_put(word, TLM_NAV_STATE,   nav_get_state());
    word = protocol_tlm_put(word, TLM_NAV_REASON,  nav_get_reason());
    word = protocol_tlm_put(word, TLM_FAULTS,      nav_get_faults());
    word = protocol_tlm_put(word, TLM_MOTION,      motion_get_activity());
    word = protocol_tlm_put(word, TLM_LINE,        line_state());
    word = protocol_tlm_put(word, TLM_SPEED_LEFT,  wheel_speed(
                                                   DRV_ENCODER_LEFT));
    word = protocol_tlm_put(word, TLM_SPEED_RIGHT, wheel_speed(
                                                   DRV_ENCODER_RIGHT));
    word = protocol_tlm_put(word, TLM_OBSTACLE,    obstacle_cm());
    word = protocol_tlm_put(word, TLM_RSSI,        rssi_field(net.rssi));
    word = protocol_tlm_put(word, TLM_VERSION,     TLM_LAYOUT);

    return (word);
}

/**
 * @brief   Sample the car into telemetry word 1 (protocol.h TLM_W1_*).
 * @return  The packed word.
 */
uint64_t
telemetry_sample_w1 (void)
{
    imu_pack_snapshot_t imu;
    uint32_t const      barcode = telemetry_events_last_barcode();
    uint64_t            word    = 0u;

    imu_pack_unpack_snapshot(imu_task_get_snapshot(), &imu);

    word = protocol_tlm_put(word, TLM_W1_IMU_MOTION,  imu.motion_class);
    word = protocol_tlm_put(word, TLM_W1_IMU_TERRAIN, imu.terrain_state);
    word = protocol_tlm_put(word, TLM_W1_IMU_OK,      imu_is_ok() ? 1u : 0u);
    word = protocol_tlm_put(word, TLM_W1_BARCODE,
                            barcode & BARCODE_LETTER_MASK);
    word = protocol_tlm_put(word, TLM_W1_BARCODE_COUNT,
                            barcode >> BARCODE_READS_SHIFT);
    word = protocol_tlm_put(word, TLM_W1_DISTANCE, distance_cm());

    return (word);
}

/**
 * @brief   Take settings as they come, poll for events, and send a sample
 *          every period.
 * @param   stacd    Unused start code.
 * @param   p_exinf  Unused.
 */
static void
telemetry_task (INT stacd, void * p_exinf)
{
    comms_msg_t       msg;
    net_bridge_info_t net;
    uint64_t          words[TLM_WORDS];
    uint32_t          next         = now_ms();
    uint32_t          seq          = 0u;
    bool              b_was_online = false;

    (void) stacd;
    (void) p_exinf;

    for (;;)
    {
        /* Just came online: the first sample soon, not a period later.
           Seen within EVENT_POLL_MS, the longest wait below. */
        comms_get_info(&net);

        if ((net.b_online) && (!b_was_online))
        {
            next = now_ms() + FIRST_SAMPLE_MS;
        }

        b_was_online = net.b_online;

        /* Cast: the difference of two wrapping 32-bit times, read as
           signed, is the time left; it is clamped to 0..EVENT_POLL_MS. */
        TMO wait = (TMO)(next - now_ms());

        if (wait < 0)
        {
            wait = 0;
        }
        else if (wait > EVENT_POLL_MS)
        {
            wait = EVENT_POLL_MS;
        }
        else
        {
            /* Wait the time left. */
        }

        /* Wait for a setting until the next sample or poll is due. */
        INT const received = tk_rcv_mbf(g_mbf, &msg, wait);

        if (received >= E_OK)
        {
            apply_setting(&msg);
        }
        else if (E_TMOUT != received)
        {
            CHECK(received, "tk_rcv_mbf");
        }
        else
        {
            telemetry_events_poll();

            /* Cast: as above, the time left, now 0 or less if due. */
            if ((int32_t)(next - now_ms()) <= 0)
            {
                next += g_period_ms;

                /* Cast: as above.  Fell behind: start again from now. */
                if ((int32_t)(next - now_ms()) < 0)
                {
                    next = now_ms() + g_period_ms;
                }

                if (gb_enabled)
                {
                    /* Dropped (E_TMOUT) while offline: fine, the next
                       sample will be newer anyway. */
                    words[0] = telemetry_sample(seq);
                    words[1] = telemetry_sample_w1();
                    (void) comms_telemetry(words, TLM_WORDS);
                    seq = (seq + 1u) & SEQ_MASK;
                }
            }
        }
    }
}

/**
 * @brief   The kernel's clock in ms; wraps every 49 days.
 * @return  Its low 32 bits.
 */
static uint32_t
now_ms (void)
{
    SYSTIM time;

    (void) tk_get_otm(&time);

    return (time.lo);
}

/**
 * @brief   One wheel's speed, limited to what the field holds.
 * @param   wheel  DRV_ENCODER_LEFT or DRV_ENCODER_RIGHT.
 * @return  mm/s, at most TLM_SPEED_MAX.
 */
static uint32_t
wheel_speed (uint32_t wheel)
{
    uint32_t const mm_s = drv_encoder_get_speed(wheel);

    return ((mm_s > TLM_SPEED_MAX) ? TLM_SPEED_MAX : mm_s);
}

/**
 * @brief   Line Following's line state.
 * @return  LINE_ON, LINE_LOST or LINE_SEARCHING.
 */
static uint32_t
line_state (void)
{
    /* Cast: line_state_t values are 0..2. */
    return ((uint32_t) line_get_state());
}

/**
 * @brief   The nearest obstacle ahead, from Obstacle Scan's last reading.
 * @return  cm, or TLM_OBSTACLE_NONE for no echo.
 */
static uint32_t
obstacle_cm (void)
{
    ultrasonic_reading_t const reading = ultrasonic_last_reading();
    uint32_t                   result  = TLM_OBSTACLE_NONE;

    if (!ultrasonic_is_no_echo(reading))
    {
        result = ultrasonic_dist_cm(reading);
    }

    return (result);
}

/**
 * @brief   Mean distance of the two wheels since power-on.  The counts never
 *          reset (IF-06); a pivot turns both wheels, so it counts too.
 * @return  cm, at most TLM_DISTANCE_MAX.
 */
static uint32_t
distance_cm (void)
{
    /* Cast: widening first, so the sum of two 32-bit counts cannot
       wrap. */
    uint64_t const left   = (uint64_t) drv_encoder_get_count(
                                DRV_ENCODER_LEFT);
    uint64_t const pulses = (left + drv_encoder_get_count(
                                DRV_ENCODER_RIGHT)) / 2u;
    uint64_t const cm     = (pulses * drv_encoder_pulses_to_um(1u)) /
                            UM_PER_CM;
    uint32_t       result = TLM_DISTANCE_MAX;

    if (cm < TLM_DISTANCE_MAX)
    {
        /* Cast: cm is below TLM_DISTANCE_MAX here. */
        result = (uint32_t) cm;
    }

    return (result);
}

/**
 * @brief   The Wi-Fi signal as the RSSI field holds it.
 * @param   rssi  dBm from the bridge; 0 = unknown.
 * @return  -dBm, or 0 when unknown or too weak to report.
 */
static uint32_t
rssi_field (int32_t rssi)
{
    uint32_t result = 0u;

    if ((rssi < 0) && (rssi > RSSI_UNKNOWN_DBM))
    {
        /* Cast: -rssi is 1..127 here. */
        result = (uint32_t)(-rssi);
    }

    return (result);
}

/**
 * @brief   Apply a setting from robocar/id/config and log the outcome.
 * @param   p_msg  The message.
 */
static void
apply_setting (comms_msg_t const * p_msg)
{
    comms_payload_cmd_t cmd;
    char                line[NET_PAYLOAD_MAX];

    if (E_OK != comms_payload_parse_msg(p_msg, &cmd))
    {
        (void) comms_log("telemetry: ignored a setting: not"
                         " \"<id> <value>\"");
    }
    else
    {
        switch (cmd.msg_id)
        {
            case CFG_TELEMETRY_PERIOD:
                if ((cmd.arg[0] < PERIOD_MIN_MS) ||
                    (cmd.arg[0] > PERIOD_MAX_MS))
                {
                    /* Casts: tm_sprintf() takes UB text; ASCII is the same
                       bytes. */
                    (void) tm_sprintf((UB *) line, (UB const *) "telemetry:"
                                      " period %d ms is outside %d..%d",
                                      cmd.arg[0], PERIOD_MIN_MS,
                                      PERIOD_MAX_MS);
                }
                else
                {
                    /* Cast: the period is within 100..60000, checked. */
                    g_period_ms = (uint32_t) cmd.arg[0];

                    /* Casts: ASCII text, as above. */
                    (void) tm_sprintf((UB *) line, (UB const *) "telemetry:"
                                      " every %d ms", cmd.arg[0]);
                }
            break;

            case CFG_TELEMETRY_ENABLE:
                gb_enabled = (0 != cmd.arg[0]);

                /* Casts: ASCII text, as above. */
                (void) tm_sprintf((UB *) line, (UB const *) "telemetry: %s",
                                  gb_enabled ? "on" : "off");
            break;

            case CFG_BARCODE_DIR:
            case CFG_BARCODE_CLEAR:
            case CFG_BARCODE_SHOW:
                apply_barcode_setting(&cmd, line);
            break;

            default:
                /* Casts: ASCII text, as above. */
                (void) tm_sprintf((UB *) line, (UB const *) "telemetry:"
                                  " unknown setting %d", cmd.msg_id);
            break;
        }

        (void) comms_log(line);
    }
}

/**
 * @brief   Apply one of the barcode map settings (protocol.h CFG_BARCODE_*)
 *          and describe the map that results.  The telemetry task is the
 *          map's only writer.
 * @param   p_cmd   The setting.
 * @param   p_line  At least NET_PAYLOAD_MAX bytes, for the log line.
 */
static void
apply_barcode_setting (comms_payload_cmd_t const * p_cmd, char * p_line)
{
    bool b_ok = true;

    if (CFG_BARCODE_DIR == p_cmd->msg_id)
    {
        b_ok = set_barcode(p_cmd);
    }
    else if (CFG_BARCODE_CLEAR == p_cmd->msg_id)
    {
        barcode_clear_cmds();
    }
    else
    {
        /* CFG_BARCODE_SHOW: only report. */
    }

    if (b_ok)
    {
        barcode_map_text(p_line, NET_PAYLOAD_MAX);
    }
    else
    {
        /* Casts: tm_sprintf() takes UB text; ASCII is the same bytes. */
        (void) tm_sprintf((UB *) p_line, (UB const *) "telemetry: barcode"
                          " setting needs <'0'-'9' or 'A'-'Z' as ASCII"
                          " code> <0-4>");
    }
}

/**
 * @brief   Set what one barcode character means, from CFG_BARCODE_DIR.
 * @param   p_cmd  The setting: the character's ASCII code, then a
 *                 barcode_cmd_t (CMD_NONE = ignore it).
 * @return  true if the map changed; false for a missing or bad argument.
 */
static bool
set_barcode (comms_payload_cmd_t const * p_cmd)
{
    int32_t const letter = p_cmd->arg[0];
    int32_t const dir    = p_cmd->arg[1];
    bool          b_set  = false;

    if ((p_cmd->arg_count >= 2u) && (letter >= 0) && (letter <= ASCII_MAX) &&
        (dir >= 0) && (dir <= (int32_t) CMD_NONE))  /* Cast: small enum. */
    {
        /* Casts: letter is 0..127, so it fits a char; dir is 0..CMD_NONE,
           a barcode_cmd_t value. */
        b_set = barcode_set_cmd((char) letter, (barcode_cmd_t) dir);
    }

    return (b_set);
}

/**
 * @brief   Describe the barcode map: "barcodes: A=L B=R K=S", every
 *          character that is not ignored, cut with " ..." to fit.
 * @param   p_line  At least size bytes.
 * @param   size    The room, including the NUL.
 */
static void
barcode_map_text (char * p_line, uint32_t size)
{
    uint32_t len   = 0u;
    uint32_t shown = 0u;
    bool     b_cut = false;

    /* Casts: tm_sprintf() takes UB text and returns a small length. */
    len = (uint32_t) tm_sprintf((UB *) p_line, (UB const *) "barcodes:");

    for (uint32_t idx = 0u; ('\0' != g_map_chars[idx]) && (!b_cut); idx++)
    {
        barcode_cmd_t const cmd = barcode_get_cmd(g_map_chars[idx]);

        if (CMD_NONE == cmd)
        {
            /* Ignored: not listed. */
        }
        else if ((len + MAP_ENTRY_LEN + MAP_MORE_LEN) > size)
        {
            /* Casts: as above; the text fits the room kept for it. */
            len += (uint32_t) tm_sprintf((UB *) &p_line[len],
                                         (UB const *) " ...");
            b_cut = true;
        }
        else
        {
            p_line[len]      = ' ';
            p_line[len + 1u] = g_map_chars[idx];
            p_line[len + 2u] = '=';
            p_line[len + 3u] = g_map_dirs[cmd];
            len += MAP_ENTRY_LEN;
            shown++;
        }
    }

    p_line[len] = '\0';

    if (0u == shown)
    {
        /* Casts: as above. */
        (void) tm_sprintf((UB *) &p_line[len], (UB const *) " none (all"
                          " ignored)");
    }
}

/*** end of file ***/
