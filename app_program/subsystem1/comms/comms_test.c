/**
 * @file    comms_test.c
 * @brief   On-target tests of comms and the network bridge's pure helpers.
 *          Built only with make TEST=1 (with or without NET=lwip).
 *          Record results in testing/subsystem1/comms/TEST_LOG.md.
 *          IDs: CO-U01, ...
 *          Coded to BARR-C:2018.
 *
 * These check the parts that need no network.  Connecting, recovering and
 * the round trip to the dashboard are system tests (SYS-xx).
 */

#include "comms_test.h"
#include "comms.h"
#include "../net/net_backoff.h"
#include "../net/net_topic.h"
#include "../../test_report.h"

#if APP_TEST

/* The same sample as the dashboard's test (mqtt-web-interface/src/shared/
   protocol.test.ts): the two sides must agree bit for bit. */
#define GOLDEN_WORD         (11213681551648818218ull)
#define GOLDEN_TEXT         "11213681551648818218"
#define GOLDEN_W1           (8090835030ull)
#define GOLDEN_PAYLOAD      "11213681551648818218 8090835030"

#define BACKOFF_MIN_MS      (1000u)
#define BACKOFF_MAX_MS      (8000u)
#define INT32_SMALLEST      (-2147483647 - 1)

static bool parses (char const * p_text, comms_payload_cmd_t * p_cmd);
static bool same_text (char const * p_a, char const * p_b);
static void test_parsing (void);
static void test_formatting (void);
static void test_backoff_and_topics (void);

/**
 * @brief   Run every comms test.
 */
void
comms_test (void)
{
    test_parsing();
    test_formatting();
    test_backoff_and_topics();
}

/**
 * @brief   Does a NUL-terminated text parse as a command?
 * @param   p_text  The text.
 * @param   p_cmd   Where to put the result.
 * @return  true if comms_payload_parse_text() accepts it.
 */
static bool
parses (char const * p_text, comms_payload_cmd_t * p_cmd)
{
    uint32_t len = 0u;

    while ('\0' != p_text[len])
    {
        len++;
    }

    /* Cast: the parser reads bytes; the ASCII text is the same bytes. */
    return (E_OK == comms_payload_parse_text((uint8_t const *) p_text, len,
                                             p_cmd));
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
 * @brief   CO-U01 .. CO-U03: command text to comms_payload_cmd_t.
 */
static void
test_parsing (void)
{
    comms_payload_cmd_t cmd;

    TEST_INIT();

    bool const b_u01 = (parses("2 60 -20 1500", &cmd)) && (2u == cmd.msg_id) &&
                       (3u == cmd.arg_count) && (60 == cmd.arg[0]) &&
                       (-20 == cmd.arg[1]) && (1500 == cmd.arg[2]) &&
                       (0 == cmd.arg[3]);

    TEST_CHECK("CO-U01", b_u01, "parses \"2 60 -20 1500\"");

    bool b_u02 = (parses(" 1\r\n", &cmd)) && (1u == cmd.msg_id) &&
                 (0u == cmd.arg_count);

    b_u02 = b_u02 && (parses("3,2", &cmd)) && (2 == cmd.arg[0]);
    b_u02 = b_u02 && (parses("0 -2147483648 2147483647", &cmd)) &&
            (INT32_SMALLEST == cmd.arg[0]) && (2147483647 == cmd.arg[1]);
    TEST_CHECK("CO-U02", b_u02,
               "accepts whitespace, commas and the full int32_t range");

    bool const b_u03 = (!parses("", &cmd)) && (!parses("stop", &cmd)) &&
                       (!parses("2 6x", &cmd)) && (!parses("-1", &cmd)) &&
                       (!parses("70000", &cmd)) &&
                       (!parses("1 2147483648", &cmd)) &&
                       (!parses("1 -21474836480", &cmd)) &&
                       (!parses("1 2 3 4 5 6", &cmd)) &&
                       (!parses("1 -", &cmd));

    TEST_CHECK("CO-U03", b_u03,
               "rejects text, bad ids, overflow and too many args");
    TEST_SUMMARY("comms parsing");
}

/**
 * @brief   CO-U04, CO-U05, CO-U10: numbers and telemetry words to text.
 */
static void
test_formatting (void)
{
    char     text[COMMS_PAYLOAD_WORDS_TEXT_MAX];
    uint64_t word = 0u;
    uint64_t words[TLM_WORDS];

    TEST_INIT();

    /* CO-U04: 64-bit numbers to decimal text. */
    (void) comms_payload_format_u64(text, 0u);

    bool b_u04 = same_text(text, "0");

    (void) comms_payload_format_u64(text, 0xFFFFFFFFFFFFFFFFull);
    b_u04 = b_u04 && (same_text(text, "18446744073709551615"));
    TEST_CHECK("CO-U04", b_u04, "formats 0 and 2^64-1");

    /* CO-U05: telemetry packing matches the dashboard. */
    word = protocol_tlm_put(word, TLM_SEQ, 42u);
    word = protocol_tlm_put(word, TLM_NAV_STATE, 4u);       /* BYPASS */
    word = protocol_tlm_put(word, TLM_NAV_REASON, 0u);
    word = protocol_tlm_put(word, TLM_FAULTS, 0x84u);       /* Line, Wi-Fi */
    word = protocol_tlm_put(word, TLM_MOTION, 1u);          /* STRAIGHT */
    word = protocol_tlm_put(word, TLM_LINE, 2u);            /* SEARCHING */
    word = protocol_tlm_put(word, TLM_SPEED_LEFT, 345u);
    word = protocol_tlm_put(word, TLM_SPEED_RIGHT, TLM_SPEED_MAX);
    word = protocol_tlm_put(word, TLM_OBSTACLE, 123u);
    word = protocol_tlm_put(word, TLM_RSSI, 55u);
    word = protocol_tlm_put(word, TLM_VERSION, TLM_LAYOUT);
    (void) comms_payload_format_u64(text, word);

    bool const b_u05 = (GOLDEN_WORD == word) &&
                       (same_text(text, GOLDEN_TEXT)) &&
                       (0x84u == protocol_tlm_get(word, TLM_FAULTS)) &&
                       (TLM_SPEED_MAX ==
                        protocol_tlm_get(word, TLM_SPEED_RIGHT));

    TEST_CHECK("CO-U05", b_u05,
               "telemetry word matches the dashboard's golden sample");

    /* CO-U10: word 1 and the two-word payload match the dashboard. */
    words[0] = word;
    words[1] = 0u;
    words[1] = protocol_tlm_put(words[1], TLM_W1_IMU_MOTION, 2u);  /* Cruise */
    words[1] = protocol_tlm_put(words[1], TLM_W1_IMU_TERRAIN, 1u); /* Climb */
    words[1] = protocol_tlm_put(words[1], TLM_W1_IMU_OK, 1u);
    words[1] = protocol_tlm_put(words[1], TLM_W1_BARCODE, 'B');
    words[1] = protocol_tlm_put(words[1], TLM_W1_BARCODE_COUNT, 5u);
    words[1] = protocol_tlm_put(words[1], TLM_W1_DISTANCE, 123456u);

    uint32_t const len = comms_payload_format_words(text, words, TLM_WORDS);
    bool const     b_u10 = (GOLDEN_W1 == words[1]) &&
                           ((sizeof(GOLDEN_PAYLOAD) - 1u) == len) &&
                           (same_text(text, GOLDEN_PAYLOAD));

    TEST_CHECK("CO-U10", b_u10,
               "word 1 and the two-word payload match the dashboard");
    TEST_SUMMARY("comms formatting");
}

/**
 * @brief   CO-U06 .. CO-U09: backoff, size limits and topic matching.
 */
static void
test_backoff_and_topics (void)
{
    net_backoff_t backoff;
    char          text[COMMS_PAYLOAD_WORDS_TEXT_MAX] = {0};

    TEST_INIT();

    /* CO-U06/07: backoff doubles up to the limit, jitter stays within
       half to all of the step, and a reset starts again from the
       minimum. */
    net_backoff_init(&backoff, BACKOFF_MIN_MS, BACKOFF_MAX_MS);

    uint32_t const wait1 = net_backoff_next(&backoff, 0u);    /* 1000: 500 */
    uint32_t const wait2 = net_backoff_next(&backoff, 0xFFFFFFFFu); /* 2000 */
    uint32_t const wait3 = net_backoff_next(&backoff, 0u);    /* 4000: 2000 */
    uint32_t const wait4 = net_backoff_next(&backoff, 1000u); /* 8000 */
    uint32_t const wait5 = net_backoff_next(&backoff, 0u);    /* 8000: 4000 */

    bool const b_u06 = (500u == wait1) && (wait2 >= 1000u) &&
                       (wait2 <= 2000u) && (2000u == wait3) &&
                       (5000u == wait4) && (4000u == wait5);

    TEST_CHECK("CO-U06", b_u06, "backoff doubles to the limit with jitter");
    net_backoff_reset(&backoff);
    TEST_CHECK("CO-U07", (500u == net_backoff_next(&backoff, 0u)),
               "backoff restarts from the minimum after a reset");

    /* CO-U08: messages over the limits are refused, never cut. */
    bool const b_u08 =
        (E_PAR == comms_publish("robocar/test", text, NET_PAYLOAD_MAX + 1u,
                                false)) &&
        (E_PAR == comms_publish("a/topic/that/is/far/longer/than/the/sixty-"
                                "three/byte/topic/limit/allows", text, 1u,
                                false));

    TEST_CHECK("CO-U08", b_u08, "an over-long payload or topic is refused");

    /* CO-U09: subscriptions match topics as a broker does. */
    bool const b_u09 =
        (net_topic_matches("robocar/a/cmd", "robocar/a/cmd")) &&
        (!net_topic_matches("robocar/a/cmd", "robocar/b/cmd")) &&
        (net_topic_matches("robocar/+/cmd", "robocar/b/cmd")) &&
        (!net_topic_matches("robocar/+/cmd", "robocar/b/log")) &&
        (net_topic_matches("robocar/#", "robocar/b/log")) &&
        (net_topic_matches("robocar/#", "robocar")) &&
        (!net_topic_matches("robocar/a", "robocar/a/cmd"));

    TEST_CHECK("CO-U09", b_u09, "topic filters with + and # match as in MQTT");
    TEST_SUMMARY("comms helpers");
}

#endif /* APP_TEST */

/*** end of file ***/
