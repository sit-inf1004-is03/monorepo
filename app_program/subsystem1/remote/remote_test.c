/**
 * @file    remote_test.c
 * @brief   On-target tests of remote.  Built only with make TEST=1 (with or
 *          without NET=lwip).  Record results in testing/subsystem1/remote/
 *          TEST_LOG.md.  Coded to BARR-C:2018.
 *
 * Commands are put straight into the remote message buffer, exactly as the
 * network bridge does for a message on robocar/id/cmd.
 *
 * No START is sent: in a test build that would drive the car.  START, STOP
 * and CLEAR reaching Navigation is checked on the car (SYS-11).
 */

#include "remote_test.h"
#include "remote.h"
#include "../comms/comms.h"
#include "../../test_report.h"

#if APP_TEST

#define SETTLE_MS               (100)   /* Time for the task to take all. */

static void deliver (char const * p_topic, char const * p_text);
static bool all_taken (void);

/**
 * @brief   RE-U01: anything arriving is handled, nothing is left behind and
 *          the task keeps going: a ping, text, an unknown number, too many
 *          arguments, and a broadcast ping.
 */
void
remote_test (void)
{
    TEST_INIT();

    deliver(COMMS_TOPIC_CMD, "0 77");
    deliver(COMMS_TOPIC_CMD, "stop");
    deliver(COMMS_TOPIC_CMD, "9");
    deliver(COMMS_TOPIC_CMD, "1 2 3 4 5 6");
    deliver(COMMS_TOPIC_CMD_ALL, "0 78");
    (void) tk_dly_tsk(SETTLE_MS);
    TEST_CHECK("RE-U01", all_taken(),
               "pings, text and unknown commands are all consumed");
    TEST_SUMMARY("remote");
}

/**
 * @brief   Put a text message into the remote buffer, as the bridge does.
 * @param   p_topic  The topic it came on.
 * @param   p_text   The payload, NUL-terminated.
 */
static void
deliver (char const * p_topic, char const * p_text)
{
    comms_msg_t msg = {0};
    uint32_t    idx = 0u;

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
    CHECK(tk_snd_mbf(remote_get_mbf(), &msg, net_bridge_msg_size(msg.len),
                     TMO_FEVR), "deliver");
}

/**
 * @brief   Is the remote buffer empty: has the task taken every message?
 * @return  true if it is.
 */
static bool
all_taken (void)
{
    T_RMBF state;

    return ((E_OK == tk_ref_mbf(remote_get_mbf(), &state)) &&
            (comms_mbf_size(REMOTE_MBF_MESSAGES) == state.frbufsz));
}

#endif /* APP_TEST */

/*** end of file ***/
