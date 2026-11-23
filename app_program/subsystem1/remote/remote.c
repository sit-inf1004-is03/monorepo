/**
 * @file    remote.c
 * @brief   Remote: dashboard commands to Navigation, and the Wi-Fi-loss
 *          fault.  Coded to BARR-C:2018.
 *
 * Command flow: dashboard, broker, network bridge, the remote message
 * buffer, remote_task, nav_post_remote() (IF-13), Navigation.
 *
 * Each message arrives as its topic and payload; the payload is parsed
 * here into a command number and its arguments.  nav_post_remote() never
 * waits, so a command reaches Navigation within its next step; Navigation
 * itself decides whether it applies (START only from IDLE or STOPPED,
 * CLEAR only from FAULT).
 *
 * Between commands the task watches the connection.  When an online car
 * goes offline it raises NAV_FLT_WIFI_LOSS (IF-12) once.  That bit is
 * outside Navigation's stop mask: the car carries on, degraded.  It stays
 * set, like every fault bit, until Navigation clears them all on a remote
 * CLEAR in the FAULT state.
 */

#include "remote.h"
#include "../comms/comms.h"
#include "../../navigation/navigation.h"

/* protocol.h keeps Navigation's numbers for the three remote commands. */
#if (CMD_START != NAV_REMOTE_START) || (CMD_STOP != NAV_REMOTE_STOP) || \
    (CMD_CLEAR != NAV_REMOTE_CLEAR)
# error "protocol.h CMD_* differ from navigation.h NAV_REMOTE_*"
#endif

#define LINK_CHECK_MS           (500)   /* How often the link is checked. */

static ID g_mbf = 0;

static void         remote_task (INT stacd, void * p_exinf);
static char const * command_name (uint16_t cmd_id);
static void         execute (comms_payload_cmd_t const * p_cmd,
                             bool b_broadcast);
static void         check_link (bool * p_b_was_online);
static void         handle (comms_msg_t const * p_msg);

/**
 * @brief   Create the command buffer and subscribe it to this car's and
 *          every car's command topic.
 * @return  E_OK, or the first error.
 */
ER
remote_init (void)
{
    T_CMBF cmbf = {0};
    ER     result = E_OK;

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = comms_mbf_size(REMOTE_MBF_MESSAGES);
    cmbf.maxmsz = sizeof(comms_msg_t);
    g_mbf       = tk_cre_mbf(&cmbf);

    if (g_mbf < E_OK)
    {
        result = g_mbf;
    }
    else
    {
        /* Two topics, one buffer: commands for this car, and for all. */
        result = comms_subscribe(COMMS_TOPIC_CMD, g_mbf);

        if (E_OK == result)
        {
            result = comms_subscribe(COMMS_TOPIC_CMD_ALL, g_mbf);
        }
    }

    return (result);
}

/**
 * @brief   Start remote_task.
 * @return  E_OK, or the error from creating it.
 */
ER
remote_start (void)
{
    ER       result = E_OK;
    ID const tid    = app_start_task(remote_task, REMOTE_PRI, 0u,
                                     "remote_task");

    if (tid < E_OK)
    {
        result = tid;
    }

    return (result);
}

/**
 * @brief   The command buffer, for the self-tests to put commands in as
 *          the network bridge does.
 * @return  Its ID.
 */
ID
remote_get_mbf (void)
{
    return (g_mbf);
}

/**
 * @brief   Wait for commands, and check the connection between them.
 * @param   stacd    Unused start code.
 * @param   p_exinf  Unused.
 */
static void
remote_task (INT stacd, void * p_exinf)
{
    comms_msg_t msg;
    bool        b_was_online = false;

    (void) stacd;
    (void) p_exinf;

    for (;;)
    {
        INT const received = tk_rcv_mbf(g_mbf, &msg, LINK_CHECK_MS);

        check_link(&b_was_online);

        if (received >= E_OK)
        {
            handle(&msg);
        }
        else if (E_TMOUT != received)
        {
            CHECK(received, "tk_rcv_mbf");
        }
        else
        {
            /* No command: only the connection was checked. */
        }
    }
}

/**
 * @brief   Name one of Navigation's remote commands, for the log.
 * @param   cmd_id  CMD_START, CMD_STOP or CMD_CLEAR.
 * @return  Its name, or "?".
 */
static char const *
command_name (uint16_t cmd_id)
{
    char const * p_name = "?";

    switch (cmd_id)
    {
        case CMD_START:
            p_name = "START";
        break;

        case CMD_STOP:
            p_name = "STOP";
        break;

        case CMD_CLEAR:
            p_name = "CLEAR";
        break;

        default:
            /* Keep "?". */
        break;
    }

    return (p_name);
}

/**
 * @brief   Carry out one command and log what happened.
 * @param   p_cmd        The command.
 * @param   b_broadcast  true if it was sent to every car.
 */
static void
execute (comms_payload_cmd_t const * p_cmd, bool b_broadcast)
{
    char               line[NET_PAYLOAD_MAX];
    char const * const p_from = b_broadcast ? " (broadcast)" : "";

    switch (p_cmd->msg_id)
    {
        case CMD_PING:
            /* Casts: tm_sprintf() takes UB text; ASCII is the same bytes. */
            (void) tm_sprintf((UB *) line, (UB const *) "remote: pong %d%s",
                              p_cmd->arg[0], p_from);
        break;

        case CMD_START:
        case CMD_STOP:
        case CMD_CLEAR:
        {
            ER const err = nav_post_remote(p_cmd->msg_id);

            if (E_OK == err)
            {
                /* Casts: ASCII text, as above. */
                (void) tm_sprintf((UB *) line, (UB const *) "remote: %s to"
                                  " navigation%s", command_name(p_cmd->msg_id),
                                  p_from);
            }
            else
            {
                /* Casts: ASCII text, as above. */
                (void) tm_sprintf((UB *) line, (UB const *) "remote: %s"
                                  " refused by navigation (%d)%s",
                                  command_name(p_cmd->msg_id), err, p_from);
            }
        }
        break;

        default:
            /* Casts: ASCII text, as above. */
            (void) tm_sprintf((UB *) line, (UB const *) "remote: unknown"
                              " command %d", p_cmd->msg_id);
        break;
    }

    (void) comms_log(line);
}

/**
 * @brief   Raise the Wi-Fi-loss fault once each time an online car goes
 *          offline.
 * @param   p_b_was_online  In: online at the last check.  Out: online now.
 */
static void
check_link (bool * p_b_was_online)
{
    net_bridge_info_t net;

    comms_get_info(&net);

    if ((*p_b_was_online) && (!net.b_online))
    {
        CHECK(nav_raise_fault(NAV_FLT_WIFI_LOSS), "nav_raise_fault");
        (void) comms_log("remote: connection lost, Wi-Fi-loss fault raised");
    }

    *p_b_was_online = net.b_online;
}

/**
 * @brief   Parse a received message and carry it out, or log why not.
 * @param   p_msg  The message.
 */
static void
handle (comms_msg_t const * p_msg)
{
    comms_payload_cmd_t cmd;

    if (E_OK == comms_payload_parse_msg(p_msg, &cmd))
    {
        execute(&cmd, comms_topic_is(p_msg, COMMS_TOPIC_CMD_ALL));
    }
    else
    {
        char line[NET_PAYLOAD_MAX + NET_TOPIC_MAX];

        /* Casts: ASCII text, as in execute(). */
        (void) tm_sprintf((UB *) line, (UB const *) "remote: ignored a"
                          " message on %s: not \"<id> [arg ...]\"",
                          p_msg->topic);
        (void) comms_log(line);
    }
}

/*** end of file ***/
