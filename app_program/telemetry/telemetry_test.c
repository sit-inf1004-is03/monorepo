/**
 * @file    telemetry_test.c
 * @brief   On-target tests of telemetry.  Built only with make TEST=1 (with
 *          or without NET=lwip).  Record unit results in testing/telemetry/
 *          TEST_LOG.md and the INT- results in testing/integration/
 *          TEST_LOG.md.  Coded to BARR-C:2018.
 *
 * Settings are put straight into the telemetry message buffer, exactly as
 * the network bridge does for a message on robocar/id/config.
 */

#include "telemetry_test.h"
#include "telemetry.h"
#include "telemetry_events.h"
#include "../comms/comms.h"
#include "../navigation/navigation.h"
#include "../subsystem2/motion.h"
#include "../subsystem2/drv_encoder.h"
#include "../subsystem3/line_barcode.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem4/imu_ramp.h"
#include "../test_report.h"

#if APP_TEST

#define SETTLE_MS               (50)    /* Time for the task to apply it. */
#define TEST_PERIOD_MS          (250)
#define TOO_SHORT_MS            (10)
#define TEST_SEQ                (7u)
#define SETTING_TEXT_MAX        (24u)
#define COUNT_FIELD_MASK        (0xFu)  /* TLM_W1_BARCODE_COUNT width. */
#define HUMP_TEST_MM            (42u)
#define HUMP_TEST_DEG           (9u)
#define HUMP_HIGH_MM            (127u)
#define HUMP_STEEP_DEG          (31u)
#define IMPACT_TEST_G           (3.0f)

static void     deliver (char const * p_text);
static void     send_setting (uint16_t cfg_id, int32_t value);
static bool     same_text (char const * p_a, char const * p_b);
static uint32_t encoder_cm (void);
static bool     barcode_matches (uint64_t word, uint32_t barcode);
static void     test_settings_and_word0 (void);
static void     test_word1 (void);

/**
 * @brief   Run every telemetry test.
 */
void
telemetry_test (void)
{
    test_settings_and_word0();
    test_word1();
}

/**
 * @brief   Put a text message into the telemetry buffer, as the bridge does
 *          when one arrives on robocar/id/config.
 * @param   p_text  The payload, NUL-terminated.
 */
static void
deliver (char const * p_text)
{
    comms_msg_t        msg     = {0};
    char const * const p_topic = COMMS_TOPIC_CONFIG;
    uint32_t           idx     = 0u;

    for (idx = 0u; ('\0' != p_topic[idx]) && (idx < NET_TOPIC_MAX); idx++)
    {
        msg.topic[idx] = p_topic[idx];
    }

    for (idx = 0u; ('\0' != p_text[idx]) && (idx < NET_PAYLOAD_MAX); idx++)
    {
        /* Cast: ASCII text is stored as the same bytes. */
        msg.payload[idx] = (uint8_t) p_text[idx];
    }

    /* Cast: idx is at most NET_PAYLOAD_MAX, which fits 16 bits. */
    msg.len = (uint16_t) idx;
    CHECK(tk_snd_mbf(telemetry_get_mbf(), &msg,
                     net_bridge_msg_size(msg.len), TMO_FEVR), "deliver");
}

/**
 * @brief   Send a setting as the dashboard does.
 * @param   cfg_id  CFG_*.
 * @param   value   Its value.
 */
static void
send_setting (uint16_t cfg_id, int32_t value)
{
    char text[SETTING_TEXT_MAX];

    /* Casts: tm_sprintf() takes UB text; ASCII is the same bytes. */
    (void) tm_sprintf((UB *) text, (UB const *) "%d %d", cfg_id, value);
    deliver(text);
}

/**
 * @brief   Are two NUL-terminated texts the same?
 * @param   p_a  One text.
 * @param   p_b  The other.
 * @return  true if they are.
 */
static bool
same_text (char const * p_a, char const * p_b)
{
    uint32_t idx = 0u;

    while (('\0' != p_a[idx]) && (p_a[idx] == p_b[idx]))
    {
        idx++;
    }

    return (p_a[idx] == p_b[idx]);
}

/**
 * @brief   Distance from the encoders, as telemetry works it out, to 1 cm.
 * @return  cm.
 */
static uint32_t
encoder_cm (void)
{
    return (drv_encoder_pulses_to_mm(
                (drv_encoder_get_count(DRV_ENCODER_LEFT) / 2u) +
                (drv_encoder_get_count(DRV_ENCODER_RIGHT) / 2u)) / 10u);
}

/**
 * @brief   Does word 1 hold this barcode_get_last() reading?
 * @param   word     Telemetry word 1.
 * @param   barcode  The reading.
 * @return  true if the letter and the count's low bits match.
 */
static bool
barcode_matches (uint64_t word, uint32_t barcode)
{
    return ((protocol_tlm_get(word, TLM_W1_BARCODE) ==
             (barcode & BC_LAST_LETTER_MASK)) &&
            (protocol_tlm_get(word, TLM_W1_BARCODE_COUNT) ==
             ((barcode >> BC_LAST_READS_SHIFT) & COUNT_FIELD_MASK)));
}

/**
 * @brief   INT-15, TL-U01, TL-U02, TL-U05, INT-16: settings, and word 0.
 */
static void
test_settings_and_word0 (void)
{
    uint32_t const before = telemetry_get_period_ms();

    TEST_INIT();

    /* INT-15 (IF-15): a setting reaches the task. */
    send_setting(CFG_TELEMETRY_PERIOD, TEST_PERIOD_MS);
    (void) tk_dly_tsk(SETTLE_MS);
    TEST_CHECK("INT-15", (TEST_PERIOD_MS == telemetry_get_period_ms()),
               "\"1 250\" via the telemetry buffer changes the period");

    /* TL-U01: a period out of range is refused. */
    send_setting(CFG_TELEMETRY_PERIOD, TOO_SHORT_MS);
    (void) tk_dly_tsk(SETTLE_MS);
    TEST_CHECK("TL-U01", (TEST_PERIOD_MS == telemetry_get_period_ms()),
               "a 10 ms period is refused");

    /* TL-U02: sending can be switched off and on. */
    send_setting(CFG_TELEMETRY_ENABLE, 0);
    (void) tk_dly_tsk(SETTLE_MS);
    TEST_CHECK("TL-U02", (!telemetry_is_enabled()),
               "CFG_TELEMETRY_ENABLE 0 stops it");
    send_setting(CFG_TELEMETRY_ENABLE, 1);

    /* Cast: the period was within 100..60000, so it fits. */
    send_setting(CFG_TELEMETRY_PERIOD, (int32_t) before);
    (void) tk_dly_tsk(SETTLE_MS);

    /* TL-U05: the dashboard sets what a barcode means.  "3 75 1" makes K
       turn right; A keeps its default; a bad direction is refused; "4"
       ignores everything.  Ends with the defaults, A-D, back. */
    deliver("3 75 1");
    deliver("3 75 9");
    (void) tk_dly_tsk(SETTLE_MS);

    bool const b_set = (CMD_B_RIGHT == barcode_get_cmd('K')) &&
                       (CMD_A_LEFT == barcode_get_cmd('A'));

    deliver("4");
    (void) tk_dly_tsk(SETTLE_MS);

    bool const b_cleared = (CMD_NONE == barcode_get_cmd('K')) &&
                           (CMD_NONE == barcode_get_cmd('A')) &&
                           (CMD_NONE == barcode_get_cmd('*'));

    (void) barcode_set_cmd('A', CMD_A_LEFT);
    (void) barcode_set_cmd('B', CMD_B_RIGHT);
    (void) barcode_set_cmd('C', CMD_C_STRAIGHT);
    (void) barcode_set_cmd('D', CMD_D_UTURN);
    TEST_CHECK("TL-U05", (b_set && b_cleared),
               "barcode map: set from config, cleared, bad values refused");

    /* INT-16 (IF-06, IF-09, IF-14): a sample carries what the other
       subsystems report.  The car may be moving, so check ranges, and that
       no fault appears that Navigation has not raised (faults only come
       and go with a remote CLEAR). */
    uint64_t const word = telemetry_sample(TEST_SEQ);
    bool const     b_int16 =
        (TEST_SEQ == protocol_tlm_get(word, TLM_SEQ)) &&
        (TLM_LAYOUT == protocol_tlm_get(word, TLM_VERSION)) &&
        (protocol_tlm_get(word, TLM_NAV_STATE) <= NAV_STATE_FAULT) &&
        (protocol_tlm_get(word, TLM_NAV_REASON) <= NAV_REASON_REMOTE_STOP) &&
        (0u == (protocol_tlm_get(word, TLM_FAULTS) & ~nav_get_faults())) &&
        (protocol_tlm_get(word, TLM_MOTION) <= MOTION_ACTIVITY_PIVOT) &&
        (protocol_tlm_get(word, TLM_LINE) <= TLM_LINE_UNKNOWN);

    TEST_CHECK("INT-16", b_int16,
               "a sample holds Navigation, Motion and line-sensor values");
    TEST_SUMMARY("telemetry settings");
}

/**
 * @brief   TL-U03, INT-20, INT-19: events, and word 1.
 */
static void
test_word1 (void)
{
    char text[TELEMETRY_EVENTS_TEXT_MAX];

    TEST_INIT();

    /* TL-U03: only HUMP events are sent, as "0 mm over_127 deg". */
    (void) telemetry_events_text(text, imu_pack_event(IMU_PACK_EVT_HUMP,
        imu_pack_hump_payload(HUMP_TEST_MM, false, HUMP_TEST_DEG)));

    bool b_u03 = same_text(text, "0 42 0 9");

    (void) telemetry_events_text(text, imu_pack_event(IMU_PACK_EVT_HUMP,
        imu_pack_hump_payload(HUMP_HIGH_MM, true, HUMP_STEEP_DEG)));
    b_u03 = b_u03 && (same_text(text, "0 127 1 31"));
    b_u03 = b_u03 &&
            (0u == telemetry_events_text(text, imu_pack_event(
                IMU_PACK_EVT_IMPACT, imu_pack_impact_payload(IMPACT_TEST_G))));
    TEST_CHECK("TL-U03", b_u03,
               "an IMU HUMP event becomes an EVT_HUMP message");

    /* INT-20 (IF-18): word 1 holds Line Following's last read, letter and
       count.  A barcode may be read meanwhile, so it must match the
       reading just before or just after. */
    uint32_t const bc_before = barcode_get_last();
    uint64_t const word_bc   = telemetry_sample_w1();
    uint32_t const bc_after  = barcode_get_last();

    TEST_CHECK("INT-20", ((barcode_matches(word_bc, bc_before)) ||
                          (barcode_matches(word_bc, bc_after))),
               "word 1 holds the last barcode letter and read count");

    /* INT-19 (IF-06, IF-S4-1, IF-S4-7): word 1 holds what the IMU and the
       encoders report.  Both may change while it is built, so it must
       match the readings just before or just after. */
    imu_pack_snapshot_t snap_before;
    imu_pack_snapshot_t snap_after;
    uint32_t const      cm_before = encoder_cm();

    imu_pack_unpack_snapshot(imu_task_get_snapshot(), &snap_before);

    uint64_t const word = telemetry_sample_w1();

    imu_pack_unpack_snapshot(imu_task_get_snapshot(), &snap_after);

    uint32_t const cm_after = encoder_cm();
    uint32_t const motion   = protocol_tlm_get(word, TLM_W1_IMU_MOTION);
    uint32_t const terrain  = protocol_tlm_get(word, TLM_W1_IMU_TERRAIN);
    uint32_t const distance = protocol_tlm_get(word, TLM_W1_DISTANCE);
    bool const     b_int19  =
        ((motion == snap_before.motion_class) ||
         (motion == snap_after.motion_class)) &&
        ((terrain == snap_before.terrain_state) ||
         (terrain == snap_after.terrain_state)) &&
        (protocol_tlm_get(word, TLM_W1_IMU_OK) == (imu_is_ok() ? 1u : 0u)) &&
        ((distance + 1u) >= cm_before) && (distance <= (cm_after + 1u));

    TEST_CHECK("INT-19", b_int19,
               "word 1 holds the IMU's motion and terrain, and the distance");
    TEST_SUMMARY("telemetry word 1");
}

#endif /* APP_TEST */

/*** end of file ***/
