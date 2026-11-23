/**
 * @file    net_bridge.c
 * @brief   The network bridge: everything here runs on the kernel's Wi-Fi
 *          service task (cyw43_utk_task), the only task allowed to call
 *          lwIP, MQTT or the radio.  How it gets there without changing
 *          the kernel: net_start.c.  Coded to BARR-C:2018.
 *
 * bridge_tick() runs every TICK_MS as an lwIP timer that re-arms itself;
 * the kernel's loop runs it from sys_check_timeouts(), as it does the lwIP
 * callbacks further down.  Every message-buffer call here uses TMO_POL, so
 * a slow or stuck task can never hold up the radio or the network stack.
 *
 * bridge_tick() is a state machine over the three layers that can fail:
 *
 *     Wi-Fi join, then DHCP lease, then MQTT session, then ONLINE
 *
 *   - Before every join attempt (and once after the kernel's own join at
 *     boot) the radio scans and the console lists every network it can
 *     see, marking the one in WIFI_SSID.  Set NET_SCAN_BEFORE_JOIN to 0 to
 *     skip it.
 *   - A failed layer is retried after a wait that doubles with every
 *     failure in a row, with jitter (net_backoff.c): Wi-Fi 1 s to 60 s,
 *     MQTT 1 s to 30 s.  Success resets that layer's wait.
 *   - Losing a lower layer closes the layers above it first.
 *   - No DHCP lease within WIFI_LINK_TIMEOUT_MS re-joins the Wi-Fi, and
 *     MQTT_FAILS_BEFORE_REJOIN broker failures in a row re-join it too, in
 *     case the link itself is stale.
 *   - lwIP's MQTT keepalive closes a session whose broker went quiet.
 *   - Subscriptions are made again after every connect (lwIP always asks
 *     for a clean session, so the broker forgets them).
 *
 * Pico SDK side of the size_t split: cyw43.h is included, tk/tkernel.h is
 * not.  The two kernel calls used are declared by hand below, as the
 * kernel's own cyw43_utk.c does for tk_dly_tsk().
 *
 * NOTE: Casts between char and UB (unsigned char) only pass ASCII text to
 * the kernel's tm_printf() and tm_sprintf(); both are 8-bit bytes, so the
 * text is unchanged.  Each such cast is marked "Cast: ASCII text".
 *
 * NOTE: memory_barrier() is the only inline assembly.  Rule 1.1.c allows
 * it in a module that drives hardware, as this one drives the radio.
 */

#include "net_bridge.h"

#if TM_NET_LWIP

#include <string.h>

#include "cyw43.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/sys.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "net_backoff.h"
#include "net_topic.h"
#include <tk/errno.h>
#include <tm/tmonitor.h>

/* Exactly as declared in tk/syscall.h. */
extern ER  tk_snd_mbf (ID mbfid, CONST void * msg, INT msgsz, TMO tmout);
extern INT tk_rcv_mbf (ID mbfid, void * msg, TMO tmout);

#define TICK_MS                 (10u)    /* How often bridge_tick() runs. */
#define WIFI_LINK_TIMEOUT_MS    (30000u) /* Join + DHCP must end within. */
#define MQTT_CONNECT_TIMEOUT_MS (10000u) /* The broker must accept within. */
#define MQTT_KEEPALIVE_S        (30u)
#define MQTT_FAILS_BEFORE_REJOIN (5u)
#define RSSI_PERIOD_MS          (5000u)
#define TX_PER_TICK             (8u)     /* Queued messages sent per tick. */
#define WIFI_BACKOFF_MIN_MS     (1000u)
#define WIFI_BACKOFF_MAX_MS     (60000u)
#define MQTT_BACKOFF_MIN_MS     (1000u)
#define MQTT_BACKOFF_MAX_MS     (30000u)

/* List the visible networks before joining; 0 = don't. */
#ifndef NET_SCAN_BEFORE_JOIN
# define NET_SCAN_BEFORE_JOIN   (1)
#endif
#define SCAN_TIMEOUT_MS         (10000u) /* A scan normally takes 2-4 s. */
#define SCAN_MAX                (16u)    /* Access points listed. */
#define SSID_MAX                (32u)    /* Bytes, without the NUL. */
#define BSSID_BYTES             (6u)
#define PRINTABLE_FIRST         (0x20u)
#define PRINTABLE_END           (0x7Fu)
#define AUTH_RSN                (4u)     /* Scan result auth_mode bits. */
#define AUTH_WPA                (2u)
#define AUTH_ENCRYPTED          (1u)

#define STATUS_ONLINE           "online"
#define STATUS_OFFLINE          "offline"
#define LOG_LINE_MAX            (NET_PAYLOAD_MAX + 32u)

/* Same default and makefile override as the kernel's join at boot
   (lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c). */
#ifndef WIFI_AUTH_MODE
# define WIFI_AUTH_MODE         CYW43_AUTH_WPA2_MIXED_PSK
#endif

/** What the lwIP connection callback reported, for bridge_tick(). */
typedef enum
{
    CONN_NONE = 0,
    CONN_ACCEPTED,
    CONN_LOST
} conn_event_t;

/** One access point seen by the scan in progress. */
typedef struct
{
    uint8_t  bssid[BSSID_BYTES];
    char     ssid[SSID_MAX + 1u];   /* Printable, NUL-terminated. */
    int32_t  rssi;                  /* dBm, the strongest heard. */
    uint16_t channel;
    uint8_t  auth;                  /* AUTH_* bits. */
    bool     b_listed;
} seen_ap_t;

/* Read by any task, through net_bridge_get_info(). */
static net_bridge_info_t volatile   g_info;

/* Set once by net_bridge_arm(), before the first tick. */
static net_bridge_t const * volatile gp_bridge = NULL;

/* Everything below is used on the Wi-Fi task only. */
static net_backoff_t    g_wifi_backoff;
static net_backoff_t    g_mqtt_backoff;
static mqtt_client_t *  gp_client = NULL;
static ip_addr_t        g_broker_addr;
static net_bridge_state_t g_state = NET_WIFI_JOIN;
static uint32_t         g_deadline = 0u;    /* Timeout or retry time. */
static uint32_t         g_mqtt_fails = 0u;  /* Broker failures in a row. */
static uint32_t         g_next_rssi = 0u;
static uint32_t         g_sub_next = 0u;    /* Next subscription to make. */
static bool             gb_status_sent = false;
static char             g_log_line[LOG_LINE_MAX];

/* Set by the lwIP callbacks, acted on by bridge_tick(). */
static conn_event_t     g_conn_event = CONN_NONE;
static int32_t          g_conn_status = 0;  /* mqtt_connection_status_t. */
static bool             gb_subscribe_failed = false;

/* The networks seen by the scan in progress. */
static seen_ap_t        g_seen[SCAN_MAX];
static uint32_t         g_seen_count = 0u;
static bool             gb_seen_more = false;   /* More than SCAN_MAX. */
static net_bridge_state_t g_after_scan = NET_WIFI_JOIN;
static uint32_t         g_scan_started = 0u;
static bool             gb_scanned_once = false;

/* The message being received; copied out before the last callback ends. */
static net_bridge_msg_t g_rx_msg;
static int32_t          g_rx_sub = -1;      /* Index in p_subs; -1 none. */
static bool             gb_rx_too_long = false;

/* The message being sent, kept while lwIP's output buffer is full. */
static net_bridge_msg_t g_tx_msg;
static bool             gb_tx_pending = false;

static void         memory_barrier (void);
static bool         time_reached (uint32_t time_ms);
static void         enter (net_bridge_state_t state);
static void         bridge_log (char const * p_line);
static char const * link_text (int link);
static void         wifi_reset (void);
static char const * auth_text (uint8_t auth);
static int          on_scan_result (void * p_env,
                                    cyw43_ev_scan_result_t const * p_result);
static void         scan_then (net_bridge_state_t next);
static void         scan_report (void);
static void         wifi_retry_later (char const * p_why);
static void         wifi_join (void);
static void         wifi_wait (int link);
static void         mqtt_retry_later (char const * p_why);
static void         on_connection (mqtt_client_t * p_client, void * p_arg,
                                   mqtt_connection_status_t status);
static void         bridge_connect (void);
static void         on_subscribed (void * p_arg, err_t result);
static void         subscribe_all (void);
static void         send_queued (void);
static void         on_incoming_publish (void * p_arg, char const * p_topic,
                                         u32_t total_len);
static void         on_incoming_data (void * p_arg, u8_t const * p_data,
                                      u16_t len, u8_t flags);
static bool         bridge_setup (void);
static void         bridge_tick (void * p_arg);
static void         tick_online (void);

/**
 * @brief   Hand the bridge its message buffers and start its tick.  The
 *          only lwIP call made from outside the Wi-Fi task; net_start.c
 *          says why it is safe.
 * @param   p_bridge  What the bridge works with; it and every message
 *                    buffer in it already exist.
 */
void
net_bridge_arm (net_bridge_t const * p_bridge)
{
    gp_bridge = p_bridge;
    memory_barrier();
    sys_timeout(TICK_MS, bridge_tick, NULL);
}

/**
 * @brief   Copy the connection state and counters, from any task.
 * @param   p_info  Where to copy them.
 */
void
net_bridge_get_info (net_bridge_info_t * p_info)
{
    memory_barrier();

    /* Cast: drops volatile for the copy only.  Each field is one aligned
       word, so a reader sees every field whole, if not all from the same
       moment, which is all the callers need. */
    *p_info = *(net_bridge_info_t const *) &g_info;
}

/**
 * @brief   Full memory barrier, so the other core sees writes in order.
 */
static void
memory_barrier (void)
{
    __asm__ volatile ("dmb" ::: "memory");
}

/**
 * @brief   Has a time on lwIP's millisecond clock been reached?
 * @param   time_ms  The time, from sys_now().
 * @return  true once sys_now() is at or past it.
 */
static bool
time_reached (uint32_t time_ms)
{
    /* Cast: the difference of two wrapping 32-bit times, read as signed,
       is negative until time_ms comes, for times within 24 days. */
    return ((int32_t)(sys_now() - time_ms) >= 0);
}

/**
 * @brief   Go to a new state, and publish it for the other tasks.
 * @param   state  The new state.
 */
static void
enter (net_bridge_state_t state)
{
    g_state        = state;
    g_info.state   = state;
    g_info.b_online = (NET_ONLINE == state);
}

/**
 * @brief   Print a line on the console, and queue it for the log topic like
 *          a task's message, so that the dashboard also sees what happened
 *          while the car was offline.  Long lines are cut.
 * @param   p_line  The line, NUL-terminated.
 */
static void
bridge_log (char const * p_line)
{
    static net_bridge_msg_t msg;
    size_t                  len = strlen(p_line);

    /* Cast: ASCII text. */
    (void) tm_printf((UB const *) "%s\n", p_line);

    if (NULL != gp_bridge->p_log_topic)
    {
        if (len > NET_PAYLOAD_MAX)
        {
            len = NET_PAYLOAD_MAX;
        }

        msg.qos      = 0u;
        msg.b_retain = false;

        /* Cast: len is at most NET_PAYLOAD_MAX, which fits 16 bits. */
        msg.len = (uint16_t) len;
        (void) strncpy(msg.topic, gp_bridge->p_log_topic, NET_TOPIC_MAX);
        msg.topic[NET_TOPIC_MAX] = '\0';
        (void) memcpy(msg.payload, p_line, len);

        if (tk_snd_mbf(gp_bridge->tx_mbf, &msg,
                       net_bridge_msg_size(msg.len), TMO_POL) < E_OK)
        {
            g_info.tx_dropped++;
        }
    }
}

/**
 * @brief   Describe a link state of the radio driver.
 * @param   link  A CYW43_LINK_* value.
 * @return  A short description.
 */
static char const *
link_text (int link)
{
    char const * p_text = "unknown link state";

    switch (link)
    {
        case CYW43_LINK_DOWN:
            p_text = "link down";
        break;

        case CYW43_LINK_JOIN:
            p_text = "joining";
        break;

        case CYW43_LINK_NOIP:
            p_text = "no DHCP lease";
        break;

        case CYW43_LINK_UP:
            p_text = "up";
        break;

        case CYW43_LINK_FAIL:
            p_text = "association failed";
        break;

        case CYW43_LINK_NONET:
            p_text = "SSID not found (2.4 GHz only)";
        break;

        case CYW43_LINK_BADAUTH:
            p_text = "wrong password or security mode";
        break;

        default:
            /* Keep the default text. */
        break;
    }

    return (p_text);
}

/**
 * @brief   Drop any association or failed attempt, so the link reads DOWN.
 *
 * NOTE: Clearing wifi_join_state is what the driver itself does when the
 * access point disassociates (cyw43_ctrl.c, CYW43_EV_DISASSOC).  It is
 * needed because the kernel's Wi-Fi task re-joins on a fixed 5 s timer
 * whenever it sees a FAILED status; moving the status to DOWN cancels that
 * timer, leaving the timing of the next attempt to the backoff alone.
 */
static void
wifi_reset (void)
{
    (void) cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
    cyw43_state.wifi_join_state = 0u;
}

/**
 * @brief   Describe an access point's security.
 * @param   auth  AUTH_* bits from the scan.
 * @return  A short description.
 */
static char const *
auth_text (uint8_t auth)
{
    char const * p_text = "open";

    if (0u != (auth & AUTH_RSN))
    {
        p_text = (0u != (auth & AUTH_WPA)) ? "WPA/WPA2" : "WPA2/WPA3";
    }
    else if (0u != (auth & AUTH_WPA))
    {
        p_text = "WPA";
    }
    else if (0u != (auth & AUTH_ENCRYPTED))
    {
        p_text = "WEP";
    }
    else
    {
        /* Keep "open". */
    }

    return (p_text);
}

/**
 * @brief   Record one beacon heard by the scan.  Called from cyw43_poll()
 *          on this task; the same access point usually answers several
 *          times, and only its strongest signal is kept.
 * @param   p_env     Unused.
 * @param   p_result  The beacon, or NULL.
 * @return  0, to go on scanning.
 */
static int
on_scan_result (void * p_env, cyw43_ev_scan_result_t const * p_result)
{
    bool b_known = false;

    (void) p_env;

    for (uint32_t idx = 0u;
         (NULL != p_result) && (idx < g_seen_count) && (!b_known); idx++)
    {
        if (0 == memcmp(g_seen[idx].bssid, p_result->bssid, BSSID_BYTES))
        {
            b_known = true;

            if (p_result->rssi > g_seen[idx].rssi)
            {
                g_seen[idx].rssi = p_result->rssi;
            }
        }
    }

    if ((NULL == p_result) || b_known)
    {
        /* Nothing new to record. */
    }
    else if (SCAN_MAX == g_seen_count)
    {
        gb_seen_more = true;
    }
    else
    {
        seen_ap_t * const p_ap = &g_seen[g_seen_count];
        uint32_t          len  = 0u;

        g_seen_count++;
        (void) memcpy(p_ap->bssid, p_result->bssid, BSSID_BYTES);

        for (len = 0u; (len < p_result->ssid_len) && (len < SSID_MAX); len++)
        {
            uint8_t const byte = p_result->ssid[len];

            /* Cast: byte is printable ASCII here, so it fits a char. */
            p_ap->ssid[len] = ((byte >= PRINTABLE_FIRST) &&
                               (byte < PRINTABLE_END)) ? (char) byte : '?';
        }

        p_ap->ssid[len] = '\0';
        p_ap->rssi      = p_result->rssi;
        p_ap->channel   = p_result->channel;
        p_ap->auth      = p_result->auth_mode;
        p_ap->b_listed  = false;
    }

    return (0);
}

/**
 * @brief   Scan, then go to another state; with NET_SCAN_BEFORE_JOIN 0, go
 *          there at once.
 * @param   next  The state to go to after the scan.
 */
static void
scan_then (net_bridge_state_t next)
{
    g_after_scan    = next;
    gb_scanned_once = true;

    if (0 == NET_SCAN_BEFORE_JOIN)
    {
        enter(next);
    }
    else
    {
        cyw43_wifi_scan_options_t options = {0};

        g_seen_count = 0u;
        gb_seen_more = false;

        int const err = cyw43_wifi_scan(&cyw43_state, &options, NULL,
                                        on_scan_result);

        if (0 != err)
        {
            /* Cast: ASCII text. */
            (void) tm_printf((UB const *) "net: wifi scan did not start"
                             " (%d)\n", err);
            enter(next);
        }
        else
        {
            /* Cast: ASCII text. */
            (void) tm_printf((UB const *) "net: scanning for Wi-Fi"
                             " networks ...\n");
            g_scan_started = sys_now();
            g_deadline     = g_scan_started + SCAN_TIMEOUT_MS;
            enter(NET_WIFI_SCAN);
        }
    }
}

/**
 * @brief   Print what the scan found on the console, strongest first.
 *          Only a one-line summary goes to the log topic: a full list on
 *          every retry would fill the TX buffer while offline.
 */
static void
scan_report (void)
{
    seen_ap_t const * p_want = NULL;

    /* Cast: ASCII text. */
    (void) tm_printf((UB const *) "net: %d Wi-Fi access point(s) visible"
                     " after %u ms:\n", g_seen_count,
                     sys_now() - g_scan_started);

    for (uint32_t listed = 0u; listed < g_seen_count; listed++)
    {
        seen_ap_t * p_best = &g_seen[0];
        bool        b_any  = false;

        /* Pick the strongest access point not yet listed. */
        for (uint32_t idx = 0u; idx < g_seen_count; idx++)
        {
            if ((!g_seen[idx].b_listed) &&
                ((!b_any) || (g_seen[idx].rssi > p_best->rssi)))
            {
                p_best = &g_seen[idx];
                b_any  = true;
            }
        }

        p_best->b_listed = true;

        bool const b_ours = (0 == strcmp(p_best->ssid, WIFI_SSID));

        /* Cast: ASCII text. */
        (void) tm_printf((UB const *) "net:   %4d dBm  ch %2d  "
                         "%02x:%02x:%02x:%02x:%02x:%02x  %-9s  \"%s\"%s%s\n",
                         p_best->rssi, p_best->channel,
                         p_best->bssid[0], p_best->bssid[1],
                         p_best->bssid[2], p_best->bssid[3],
                         p_best->bssid[4], p_best->bssid[5],
                         auth_text(p_best->auth), p_best->ssid,
                         ('\0' == p_best->ssid[0]) ? " (hidden)" : "",
                         b_ours ? "  <-- WIFI_SSID" : "");

        if ((NULL == p_want) && b_ours)
        {
            p_want = p_best;            /* The strongest with our name. */
        }
    }

    if (gb_seen_more)
    {
        /* Cast: ASCII text. */
        (void) tm_printf((UB const *) "net:   (and more, not listed)\n");
    }

    if (NULL != p_want)
    {
        /* Casts: ASCII text. */
        (void) tm_sprintf((UB *) g_log_line, (UB const *) "net: wifi scan:"
                          " %d APs; \"%s\" seen at %d dBm, ch %d, %s",
                          g_seen_count, WIFI_SSID, p_want->rssi,
                          p_want->channel, auth_text(p_want->auth));
    }
    else
    {
        /* Casts: ASCII text. */
        (void) tm_sprintf((UB *) g_log_line, (UB const *) "net: wifi scan:"
                          " %d APs; \"%s\" not among them (out of range,"
                          " 5 GHz only, or another name?)", g_seen_count,
                          WIFI_SSID);
    }

    bridge_log(g_log_line);
}

/**
 * @brief   Give up on this join and wait before the next one.
 * @param   p_why  What went wrong, for the log.
 */
static void
wifi_retry_later (char const * p_why)
{
    wifi_reset();

    uint32_t const wait = net_backoff_next(&g_wifi_backoff, LWIP_RAND());

    /* Casts: ASCII text. */
    (void) tm_sprintf((UB *) g_log_line, (UB const *) "net: wifi %s; join"
                      " again in %u ms", p_why, wait);
    bridge_log(g_log_line);
    g_deadline = sys_now() + wait;
    enter(NET_WIFI_BACKOFF);
}

/**
 * @brief   Ask the radio to join WIFI_SSID.  The BSSID is not given: the
 *          chip picks any access point with this SSID.
 */
static void
wifi_join (void)
{
    g_info.wifi_joins++;

    /* Cast: ASCII text. */
    (void) tm_printf((UB const *) "net: joining \"%s\" (attempt %u)\n",
                     WIFI_SSID, g_info.wifi_joins);

    /* Casts: the driver takes the SSID and password as bytes; the ASCII
       text is the same bytes. */
    int const err = cyw43_wifi_join(&cyw43_state, strlen(WIFI_SSID),
                                    (uint8_t const *) WIFI_SSID,
                                    strlen(WIFI_PASSWORD),
                                    (uint8_t const *) WIFI_PASSWORD,
                                    WIFI_AUTH_MODE, NULL,
                                    CYW43_CHANNEL_NONE);

    if (0 != err)
    {
        wifi_retry_later("join request refused by the driver");
    }
    else
    {
        g_deadline = sys_now() + WIFI_LINK_TIMEOUT_MS;
        enter(NET_WIFI_WAIT);
    }
}

/**
 * @brief   Wait for the join and the DHCP lease to finish.
 * @param   link  The radio's link state, CYW43_LINK_*.
 */
static void
wifi_wait (int link)
{
    switch (link)
    {
        case CYW43_LINK_UP:
        {
            uint32_t const ipv4 = ip4_addr_get_u32(
                netif_ip4_addr(&cyw43_state.netif[CYW43_ITF_STA]));

            g_info.ipv4 = ipv4;
            net_backoff_reset(&g_wifi_backoff);

            /* Casts: ASCII text. */
            (void) tm_sprintf((UB *) g_log_line, (UB const *) "net: wifi up,"
                              " ip %d.%d.%d.%d", ipv4 & 0xFFu,
                              (ipv4 >> 8) & 0xFFu, (ipv4 >> 16) & 0xFFu,
                              ipv4 >> 24);
            bridge_log(g_log_line);

            if (!gb_scanned_once)
            {
                /* The kernel joined at boot without a scan: list the
                   networks once now, for the record. */
                scan_then(NET_MQTT_CONNECT);
            }
            else
            {
                enter(NET_MQTT_CONNECT);
            }
        }
        break;

        case CYW43_LINK_JOIN:
        case CYW43_LINK_NOIP:
            if (time_reached(g_deadline))
            {
                wifi_retry_later((CYW43_LINK_NOIP == link)
                                 ? "got no DHCP lease in time"
                                 : "join timed out");
            }
        break;

        default:
            wifi_retry_later(link_text(link));      /* DOWN or a failure. */
        break;
    }
}

/**
 * @brief   Close the broker session and wait before the next attempt.
 *          After MQTT_FAILS_BEFORE_REJOIN failures in a row, re-join the
 *          Wi-Fi instead, in case the link is stale.
 * @param   p_why  What went wrong, for the log.
 */
static void
mqtt_retry_later (char const * p_why)
{
    mqtt_disconnect(gp_client);
    enter(NET_MQTT_BACKOFF);    /* Not online: telemetry stops queueing. */
    g_mqtt_fails++;

    if (g_mqtt_fails >= MQTT_FAILS_BEFORE_REJOIN)
    {
        g_mqtt_fails = 0u;
        net_backoff_reset(&g_mqtt_backoff);

        /* Casts: ASCII text. */
        (void) tm_sprintf((UB *) g_log_line, (UB const *) "net: mqtt %s, %d"
                          " times in a row", p_why, MQTT_FAILS_BEFORE_REJOIN);
        bridge_log(g_log_line);
        wifi_retry_later("may be stale");
    }
    else
    {
        uint32_t const wait = net_backoff_next(&g_mqtt_backoff, LWIP_RAND());

        /* Casts: ASCII text. */
        (void) tm_sprintf((UB *) g_log_line, (UB const *) "net: mqtt %s (%d);"
                          " connect again in %u ms", p_why, g_conn_status,
                          wait);
        bridge_log(g_log_line);
        g_deadline = sys_now() + wait;
    }
}

/**
 * @brief   lwIP's connection callback: note the outcome for bridge_tick().
 * @param   p_client  Unused.
 * @param   p_arg     Unused.
 * @param   status    MQTT_CONNECT_ACCEPTED, or why it failed or closed.
 */
static void
on_connection (mqtt_client_t * p_client, void * p_arg,
               mqtt_connection_status_t status)
{
    (void) p_client;
    (void) p_arg;

    /* Cast: the status is a small enum value, which fits an int32_t. */
    g_conn_status = (int32_t) status;
    g_conn_event  = (MQTT_CONNECT_ACCEPTED == status) ? CONN_ACCEPTED
                                                      : CONN_LOST;
}

/**
 * @brief   Open a session with the broker, with "offline" as the will.
 */
static void
bridge_connect (void)
{
    struct mqtt_connect_client_info_t client_info = {0};

    client_info.client_id  = MQTT_CLIENT_ID;
    client_info.keep_alive = MQTT_KEEPALIVE_S;

    if (NULL != gp_bridge->p_status_topic)
    {
        client_info.will_topic  = gp_bridge->p_status_topic;
        client_info.will_msg    = STATUS_OFFLINE;   /* Sent if we vanish. */
        client_info.will_qos    = 1u;
        client_info.will_retain = 1u;
    }

    g_info.mqtt_connects++;

    /* Cast: ASCII text. */
    (void) tm_printf((UB const *) "net: connecting to broker "
                     MQTT_BROKER_IP ":%d (attempt %u)\n", MQTT_BROKER_PORT,
                     g_info.mqtt_connects);

    g_conn_event  = CONN_NONE;
    g_conn_status = 0;

    err_t const err = mqtt_client_connect(gp_client, &g_broker_addr,
                                          MQTT_BROKER_PORT, on_connection,
                                          NULL, &client_info);

    if (ERR_OK != err)
    {
        g_conn_status = err;
        mqtt_retry_later("connect failed");
    }
    else
    {
        g_deadline = sys_now() + MQTT_CONNECT_TIMEOUT_MS;
        enter(NET_MQTT_WAIT);
    }
}

/**
 * @brief   lwIP's subscribe callback: note a refusal for bridge_tick().
 * @param   p_arg   Unused.
 * @param   result  ERR_OK if the broker accepted the subscription.
 */
static void
on_subscribed (void * p_arg, err_t result)
{
    (void) p_arg;

    if (ERR_OK != result)
    {
        gb_subscribe_failed = true;
    }
}

/**
 * @brief   Subscribe to every topic, then say "online".  If lwIP has no
 *          room for another request (ERR_MEM), stop and carry on from the
 *          same place on the next tick.
 */
static void
subscribe_all (void)
{
    err_t err = ERR_OK;

    while ((g_sub_next < gp_bridge->sub_count) && (ERR_OK == err))
    {
        net_bridge_sub_t const * const p_sub = &gp_bridge->p_subs[g_sub_next];

        err = mqtt_subscribe(gp_client, p_sub->p_topic, p_sub->qos,
                             on_subscribed, NULL);

        if (ERR_OK == err)
        {
            g_sub_next++;
        }
    }

    if ((ERR_OK == err) && (NULL != gp_bridge->p_status_topic) &&
        (!gb_status_sent))
    {
        /* Only a full buffer is retried; any other failure leaves the
           status to the next connect. */
        if (ERR_MEM == mqtt_publish(gp_client, gp_bridge->p_status_topic,
                                    STATUS_ONLINE,
                                    sizeof(STATUS_ONLINE) - 1u, 1u, 1u,
                                    NULL, NULL))
        {
            err = ERR_MEM;
        }
        else
        {
            gb_status_sent = true;
        }
    }

    if (ERR_MEM == err)
    {
        /* lwIP is full: carry on from here on the next tick. */
    }
    else if (ERR_OK != err)
    {
        g_conn_status = err;
        mqtt_retry_later("subscribe failed");
    }
    else
    {
        g_mqtt_fails = 0u;
        net_backoff_reset(&g_mqtt_backoff);
        g_next_rssi = sys_now();
        enter(NET_ONLINE);

        /* Casts: ASCII text. */
        (void) tm_sprintf((UB *) g_log_line, (UB const *) "net: online as "
                          MQTT_CLIENT_ID ", %d topics subscribed",
                          gp_bridge->sub_count);
        bridge_log(g_log_line);
    }
}

/**
 * @brief   Send what the tasks queued on the TX message buffer, up to
 *          TX_PER_TICK messages, stopping early when lwIP is full.
 */
static void
send_queued (void)
{
    bool b_more = true;

    for (uint32_t sent = 0u; b_more && (sent < TX_PER_TICK); sent++)
    {
        if (!gb_tx_pending)
        {
            gb_tx_pending = (tk_rcv_mbf(gp_bridge->tx_mbf, &g_tx_msg,
                                        TMO_POL) >= E_OK);
        }

        if (!gb_tx_pending)
        {
            b_more = false;             /* The queue is empty. */
        }
        else
        {
            err_t err = ERR_VAL;        /* Malformed: drop it. */

            if (g_tx_msg.len <= NET_PAYLOAD_MAX)
            {
                g_tx_msg.topic[NET_TOPIC_MAX] = '\0';
                err = mqtt_publish(gp_client, g_tx_msg.topic,
                                   g_tx_msg.payload, g_tx_msg.len,
                                   (0u != g_tx_msg.qos) ? 1u : 0u,
                                   g_tx_msg.b_retain ? 1u : 0u, NULL, NULL);
            }

            if (ERR_MEM == err)
            {
                b_more = false;         /* lwIP is full: next tick. */
            }
            else
            {
                if (ERR_OK != err)
                {
                    g_info.tx_dropped++;
                }

                gb_tx_pending = false;
            }
        }
    }
}

/**
 * @brief   lwIP's first callback for an incoming message: its topic.  The
 *          pointers lwIP passes are only valid during each call, so the
 *          message is copied into g_rx_msg, and from there into the RX
 *          message buffer before the last call returns.
 * @param   p_arg      Unused.
 * @param   p_topic    The topic, NUL-terminated.
 * @param   total_len  The whole payload's length.
 */
static void
on_incoming_publish (void * p_arg, char const * p_topic, u32_t total_len)
{
    (void) p_arg;

    gb_rx_too_long = ((strlen(p_topic) > NET_TOPIC_MAX) ||
                      (total_len > NET_PAYLOAD_MAX));
    (void) strncpy(g_rx_msg.topic, p_topic, NET_TOPIC_MAX);
    g_rx_msg.topic[NET_TOPIC_MAX] = '\0';
    g_rx_msg.qos      = 0u;
    g_rx_msg.b_retain = false;
    g_rx_msg.len      = 0u;

    g_rx_sub = -1;

    for (uint32_t idx = 0u; (idx < gp_bridge->sub_count) && (g_rx_sub < 0);
         idx++)
    {
        if (net_topic_matches(gp_bridge->p_subs[idx].p_topic, p_topic))
        {
            /* Cast: idx is below sub_count, a small table size. */
            g_rx_sub = (int32_t) idx;
        }
    }
}

/**
 * @brief   lwIP's callback for part of an incoming payload.  On the last
 *          part, the message goes to the matching subscriber's buffer.
 * @param   p_arg   Unused.
 * @param   p_data  This part of the payload.
 * @param   len     Its length.
 * @param   flags   MQTT_DATA_FLAG_LAST on the last part.
 */
static void
on_incoming_data (void * p_arg, u8_t const * p_data, u16_t len, u8_t flags)
{
    uint32_t room = NET_PAYLOAD_MAX - g_rx_msg.len;
    uint32_t take = (len < room) ? len : room;

    (void) p_arg;
    (void) memcpy(&g_rx_msg.payload[g_rx_msg.len], p_data, take);

    /* Cast: the total is at most NET_PAYLOAD_MAX, which fits 16 bits. */
    g_rx_msg.len = (uint16_t)(g_rx_msg.len + take);

    if (0u == (flags & MQTT_DATA_FLAG_LAST))
    {
        /* More to come. */
    }
    else if (g_rx_sub < 0)
    {
        g_info.rx_dropped++;            /* No subscription matches. */
    }
    else if (gb_rx_too_long)
    {
        g_info.rx_dropped++;

        /* Casts: ASCII text. */
        (void) tm_sprintf((UB *) g_log_line, (UB const *) "net: dropped a"
                          " message on %s: over %d bytes", g_rx_msg.topic,
                          NET_PAYLOAD_MAX);
        bridge_log(g_log_line);
    }
    else
    {
        ER const err = tk_snd_mbf(gp_bridge->p_subs[g_rx_sub].rx_mbf,
                                  &g_rx_msg,
                                  net_bridge_msg_size(g_rx_msg.len),
                                  TMO_POL);

        if (err < E_OK)
        {
            g_info.rx_dropped++;

            /* Casts: ASCII text. */
            (void) tm_sprintf((UB *) g_log_line, (UB const *) "net: dropped"
                              " a message on %s: receiver busy (%d)",
                              g_rx_msg.topic, err);
            bridge_log(g_log_line);
        }
    }
}

/**
 * @brief   Create the MQTT client and wait for the kernel's join at boot.
 * @return  false if the client cannot be made: then the bridge stops.
 */
static bool
bridge_setup (void)
{
    bool b_ok = false;

    net_backoff_init(&g_wifi_backoff, WIFI_BACKOFF_MIN_MS,
                     WIFI_BACKOFF_MAX_MS);
    net_backoff_init(&g_mqtt_backoff, MQTT_BACKOFF_MIN_MS,
                     MQTT_BACKOFF_MAX_MS);
    gp_client = mqtt_client_new();

    if (NULL == gp_client)
    {
        /* Cast: ASCII text. */
        (void) tm_printf((UB const *) "net: out of lwIP memory for the MQTT"
                         " client\n");
    }
    else if (0 == ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&g_broker_addr)))
    {
        /* Cast: ASCII text. */
        (void) tm_printf((UB const *) "net: MQTT_BROKER_IP \""
                         MQTT_BROKER_IP "\" is not an IPv4 address"
                         " (config/wifi_secrets.h)\n");
    }
    else
    {
        IP_SET_TYPE(&g_broker_addr, IPADDR_TYPE_V4);
        mqtt_set_inpub_callback(gp_client, on_incoming_publish,
                                on_incoming_data, NULL);

        /* The kernel's Wi-Fi task already asked to join at boot: count
           that as the first attempt and wait for its outcome. */
        g_info.wifi_joins = 1u;
        g_deadline        = sys_now() + WIFI_LINK_TIMEOUT_MS;
        enter(NET_WIFI_WAIT);
        b_ok = true;
    }

    return (b_ok);
}

/**
 * @brief   One step of the bridge, every TICK_MS, as an lwIP timer.
 * @param   p_arg  Unused.
 */
static void
bridge_tick (void * p_arg)
{
    (void) p_arg;

    bool const b_ready = (NULL != gp_client) || (bridge_setup());

    if (b_ready)
    {
        sys_timeout(TICK_MS, bridge_tick, NULL);

        int const link = cyw43_tcpip_link_status(&cyw43_state,
                                                 CYW43_ITF_STA);

        /* The layers above the link only exist while it is up. */
        if ((g_state >= NET_MQTT_CONNECT) && (CYW43_LINK_UP != link))
        {
            mqtt_disconnect(gp_client);
            g_info.rssi = 0;

            /* Casts: ASCII text. */
            (void) tm_sprintf((UB *) g_log_line, (UB const *) "net: wifi"
                              " lost (%s)", link_text(link));
            bridge_log(g_log_line);
            g_deadline = sys_now() + WIFI_LINK_TIMEOUT_MS;
            enter(NET_WIFI_WAIT);
        }

        switch (g_state)
        {
            case NET_WIFI_JOIN:
                wifi_join();
            break;

            case NET_WIFI_WAIT:
                wifi_wait(link);
            break;

            case NET_WIFI_BACKOFF:
                if (link < 0)
                {
                    wifi_reset();   /* A late failure report: keep the
                                       kernel's own re-join quiet. */
                }

                if (time_reached(g_deadline))
                {
                    scan_then(NET_WIFI_JOIN);
                }
            break;

            case NET_WIFI_SCAN:
                if ((!cyw43_wifi_scan_active(&cyw43_state)) ||
                    (time_reached(g_deadline)))
                {
                    scan_report();
                    enter(g_after_scan);
                }
            break;

            case NET_MQTT_CONNECT:
                bridge_connect();
            break;

            case NET_MQTT_WAIT:
                if (CONN_ACCEPTED == g_conn_event)
                {
                    g_conn_event        = CONN_NONE;
                    gb_subscribe_failed = false;
                    g_sub_next          = 0u;
                    gb_status_sent      = false;
                    enter(NET_MQTT_SUBSCRIBE);
                    subscribe_all();
                }
                else if (CONN_LOST == g_conn_event)
                {
                    mqtt_retry_later("broker refused or closed");
                }
                else if (time_reached(g_deadline))
                {
                    mqtt_retry_later("broker did not answer");
                }
                else
                {
                    /* Still waiting. */
                }
            break;

            case NET_MQTT_SUBSCRIBE:
            case NET_ONLINE:
                tick_online();
            break;

            case NET_MQTT_BACKOFF:
                if (time_reached(g_deadline))
                {
                    enter(NET_MQTT_CONNECT);
                }
            break;

            default:
                /* No other states. */
            break;
        }
    }
    else
    {
        /* Misconfigured: the bridge stops for good. */
    }
}

/**
 * @brief   One tick while subscribing or online: watch the session, finish
 *          subscribing, send queued messages and sample the signal.
 */
static void
tick_online (void)
{
    if (CONN_LOST == g_conn_event)
    {
        mqtt_retry_later("connection lost");
    }
    else if (gb_subscribe_failed)
    {
        mqtt_retry_later("subscription refused");
    }
    else if (NET_MQTT_SUBSCRIBE == g_state)
    {
        subscribe_all();
    }
    else
    {
        send_queued();

        if (time_reached(g_next_rssi))
        {
            int32_t rssi = 0;

            g_next_rssi = sys_now() + RSSI_PERIOD_MS;

            if (0 == cyw43_wifi_get_rssi(&cyw43_state, &rssi))
            {
                g_info.rssi = rssi;
            }
        }
    }
}

#else /* !TM_NET_LWIP: built without the network. */

/**
 * @brief   Without the network there is no bridge to arm.
 * @param   p_bridge  Unused.
 */
void
net_bridge_arm (net_bridge_t const * p_bridge)
{
    (void) p_bridge;
}

/**
 * @brief   Without the network the car is never online.
 * @param   p_info  Set to all zeros.
 */
void
net_bridge_get_info (net_bridge_info_t * p_info)
{
    *p_info = (net_bridge_info_t){0};
}

#endif /* TM_NET_LWIP */

/**
 * @brief   Bytes to send to, or reserve in, a message buffer for a message:
 *          only the used part of the payload is stored.
 * @param   len  The payload's length, at most NET_PAYLOAD_MAX.
 * @return  The message size for tk_snd_mbf().
 */
int32_t
net_bridge_msg_size (uint16_t len)
{
    /* Cast: the structure is a few hundred bytes, which fits an int32_t. */
    int32_t const header = (int32_t)(sizeof(net_bridge_msg_t) -
                                     NET_PAYLOAD_MAX);

    return (header + len);
}

/**
 * @brief   Name a connection state, for printing.
 * @param   state  The state.
 * @return  Its name.
 */
char const *
net_bridge_state_name (net_bridge_state_t state)
{
    char const * p_name = "?";

    switch (state)
    {
        case NET_WIFI_JOIN:
            p_name = "wifi-join";
        break;

        case NET_WIFI_WAIT:
            p_name = "wifi-wait";
        break;

        case NET_WIFI_BACKOFF:
            p_name = "wifi-backoff";
        break;

        case NET_WIFI_SCAN:
            p_name = "wifi-scan";
        break;

        case NET_MQTT_CONNECT:
            p_name = "mqtt-connect";
        break;

        case NET_MQTT_WAIT:
            p_name = "mqtt-wait";
        break;

        case NET_MQTT_SUBSCRIBE:
            p_name = "mqtt-subscribe";
        break;

        case NET_MQTT_BACKOFF:
            p_name = "mqtt-backoff";
        break;

        case NET_ONLINE:
            p_name = "online";
        break;

        default:
            /* Keep "?". */
        break;
    }

    return (p_name);
}

/*** end of file ***/
