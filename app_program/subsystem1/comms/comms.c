/**
 * @file    comms.c
 * @brief   Comms: message buffers to and from the network bridge.  Coded
 *          to BARR-C:2018.
 *
 * Runs on the CALLER's task and only touches kernel objects; the network
 * calls are all in net/net_bridge.c, on the kernel's Wi-Fi service task.
 *
 * Start-up order matters: the bridge must not use a message buffer before
 * it exists.  So the buffers are created in the _init() phase,
 * subscriptions are collected as the other modules' _init()s run, and only
 * comms_start(), after every _init(), hands them to the bridge.  Until
 * then the bridge does not connect to the broker.
 */

#include "comms.h"
#include "../net/net_start.h"

#define TX_MBF_SIZE         (2048)  /* Bytes of queued outgoing messages. */
#define MBF_HEADER_BYTES    (8u)    /* Kernel's bookkeeping per message. */

static ID                g_tx_mbf = 0;
static net_bridge_sub_t  g_subs[COMMS_MAX_SUBS];
static net_bridge_t      g_bridge;
static bool              gb_started = false;
static uint32_t volatile g_tx_refused = 0u; /* Dropped before the bridge. */

/**
 * @brief   Create the TX message buffer.  Call from usermain(), before any
 *          other module's _init() that subscribes.
 * @return  E_OK, or the error from tk_cre_mbf().
 */
ER
comms_init (void)
{
    T_CMBF cmbf = {0};
    ER     result = E_OK;

    /* The buffer copies each message, so a sender's comms_msg_t can be a
       local variable.  Only the used part of the payload is stored. */
    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = TX_MBF_SIZE;
    cmbf.maxmsz = sizeof(comms_msg_t);
    g_tx_mbf    = tk_cre_mbf(&cmbf);

    if (g_tx_mbf < E_OK)
    {
        result = g_tx_mbf;
    }

    return (result);
}

/**
 * @brief   Hand the TX buffer and the subscriptions to the bridge, which
 *          then connects.  Call last, after every module's _init().
 * @return  As net_start().
 */
ER
comms_start (void)
{
    g_bridge.tx_mbf         = g_tx_mbf;
    g_bridge.p_subs         = g_subs;
    g_bridge.p_status_topic = COMMS_TOPIC_STATUS;
    g_bridge.p_log_topic    = COMMS_TOPIC_LOG;
    gb_started              = true;

    return (net_start(&g_bridge));
}

/**
 * @brief   Deliver messages on a topic to a message buffer, at least once
 *          (QoS 1), so that commands are not lost.  Only before
 *          comms_start().
 * @param   p_topic  The topic or filter; it must never change (a string
 *                   literal, such as COMMS_TOPIC_CMD).
 * @param   mbf      The receiving task's message buffer.
 * @return  E_OK; E_OBJ after comms_start(); E_LIMIT when COMMS_MAX_SUBS
 *          topics are already subscribed.
 */
ER
comms_subscribe (char const * p_topic, ID mbf)
{
    ER result = E_OK;

    if (gb_started)
    {
        result = E_OBJ;             /* The bridge already has the table. */
    }
    else if (COMMS_MAX_SUBS == g_bridge.sub_count)
    {
        result = E_LIMIT;           /* Raise COMMS_MAX_SUBS. */
    }
    else
    {
        net_bridge_sub_t * const p_sub = &g_subs[g_bridge.sub_count];

        p_sub->p_topic = p_topic;
        p_sub->rx_mbf  = mbf;
        p_sub->qos     = 1u;
        g_bridge.sub_count++;
    }

    return (result);
}

/**
 * @brief   The bufsz for a message buffer that holds at least a number of
 *          whole messages.
 * @param   messages  How many.
 * @return  The size in bytes.
 */
int32_t
comms_mbf_size (uint32_t messages)
{
    /* Cast: a few messages of a few hundred bytes fit an int32_t. */
    return ((int32_t)(messages * (sizeof(comms_msg_t) + MBF_HEADER_BYTES)));
}

/**
 * @brief   Was a message received on a topic?
 * @param   p_msg    The message.
 * @param   p_topic  The topic, NUL-terminated.
 * @return  true if the topics are the same.
 */
bool
comms_topic_is (comms_msg_t const * p_msg, char const * p_topic)
{
    uint32_t idx = 0u;

    while (('\0' != p_msg->topic[idx]) &&
           (p_msg->topic[idx] == p_topic[idx]))
    {
        idx++;
    }

    return (p_msg->topic[idx] == p_topic[idx]);
}

/**
 * @brief   Queue a message for the broker, from any task; never waits.
 * @param   p_topic   The topic, NUL-terminated, at most NET_TOPIC_MAX.
 * @param   p_data    The payload.
 * @param   len       Its length, at most NET_PAYLOAD_MAX.
 * @param   b_retain  true for the broker to keep it for new subscribers.
 * @return  E_OK if queued; E_PAR if the topic or payload is too long (it
 *          is refused, never cut); E_TMOUT if the queue is full.
 */
ER
comms_publish (char const * p_topic, void const * p_data, uint32_t len,
               bool b_retain)
{
    comms_msg_t     msg;
    uint8_t const * p_src  = p_data;
    uint32_t        idx    = 0u;
    ER              result = E_OK;

    while ((E_OK == result) && ('\0' != p_topic[idx]))
    {
        if (NET_TOPIC_MAX == idx)
        {
            result = E_PAR;
        }
        else
        {
            msg.topic[idx] = p_topic[idx];
            idx++;
        }
    }

    if (len > NET_PAYLOAD_MAX)
    {
        result = E_PAR;
    }

    if (E_OK == result)
    {
        msg.topic[idx] = '\0';

        for (uint32_t byte = 0u; byte < len; byte++)
        {
            msg.payload[byte] = p_src[byte];
        }

        msg.qos      = 0u;
        msg.b_retain = b_retain;

        /* Cast: len is at most NET_PAYLOAD_MAX, checked above. */
        msg.len = (uint16_t) len;

        /* TMO_POL: never wait.  If the queue is full, drop the message. */
        result = tk_snd_mbf(g_tx_mbf, &msg, net_bridge_msg_size(msg.len),
                            TMO_POL);

        if (result < E_OK)
        {
            g_tx_refused++;
        }
    }

    return (result);
}

/**
 * @brief   Print one line on the console and queue it on robocar/id/log.
 * @param   p_text  The line, NUL-terminated; cut at NET_PAYLOAD_MAX bytes
 *                  for the broker.
 * @return  As comms_publish().
 */
ER
comms_log (char const * p_text)
{
    uint32_t len = 0u;

    while ((len < NET_PAYLOAD_MAX) && ('\0' != p_text[len]))
    {
        len++;
    }

    /* Cast: tm_printf() takes UB text; ASCII is the same 8-bit bytes. */
    (void) tm_printf((UB const *) "%s\n", p_text);

    return (comms_publish(COMMS_TOPIC_LOG, p_text, len, false));
}

/**
 * @brief   Queue packed telemetry words (protocol.h) on robocar/id/
 *          telemetry, while online.
 * @param   p_words  The words.
 * @param   count    How many, at most TLM_WORDS.
 * @return  As comms_publish(); E_TMOUT while offline, as a sample is only
 *          worth sending while it is current.
 */
ER
comms_telemetry (uint64_t const * p_words, uint32_t count)
{
    net_bridge_info_t info;
    ER                result = E_TMOUT;

    comms_get_info(&info);

    if (info.b_online)
    {
        char           text[COMMS_PAYLOAD_WORDS_TEXT_MAX];
        uint32_t const len = comms_payload_format_words(text, p_words,
                                                        count);

        result = comms_publish(COMMS_TOPIC_TELEMETRY, text, len, false);
    }

    return (result);
}

/**
 * @brief   The connection state and counters, including messages that
 *          never reached the bridge.
 * @param   p_info  Where to put them.
 */
void
comms_get_info (net_bridge_info_t * p_info)
{
    net_bridge_get_info(p_info);
    p_info->tx_dropped += g_tx_refused;
}

/*** end of file ***/
