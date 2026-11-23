/**
 * @file    net_bridge.h
 * @brief   The network bridge: MQTT on the kernel's Wi-Fi service task,
 *          message buffers towards the application.  Coded to
 *          BARR-C:2018.
 *
 * NOT A SUBSYSTEM.  No subsystem task calls anything in
 * app_program/subsystem1/net/ except through comms/ (comms.h).
 * Everything that touches lwIP, MQTT or the radio runs on the kernel's
 * Wi-Fi service task (cyw43_utk_task); the rest of the application
 * exchanges messages with it like this:
 *
 *     task, net_bridge_msg_t, TX message buffer, bridge, broker
 *     broker, bridge, net_bridge_msg_t, RX message buffer, task
 *
 * Why it lives here and not in the kernel's mqtt_utk.c: see
 * KERNEL_ISSUES.md.  In short, the kernel is not modified, and its MQTT=1
 * build starts a heartbeat client that would clash with this one, so this
 * is built with "make NET=lwip".
 *
 * Limits, and what happens at them:
 *   - A topic longer than NET_TOPIC_MAX or a payload longer than
 *     NET_PAYLOAD_MAX is dropped and counted (rx_dropped, tx_dropped).
 *   - A full RX message buffer drops the incoming message (counted): the
 *     bridge never waits for a task.
 *   - While offline, TX messages stay in the TX buffer and are sent after
 *     reconnecting; when it is full, the sender's tk_snd_mbf() with
 *     TMO_POL fails and the sender drops the message.
 *
 * NOTE: This header uses only standard and tk/typedef.h types, so that it
 * can be included both next to tk/tkernel.h and next to the Pico SDK's
 * cyw43.h, which cannot share a file with tk/tkernel.h (both define
 * size_t).
 */

#ifndef NET_BRIDGE_H
#define NET_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/typedef.h>

#define NET_TOPIC_MAX           (63u)   /* Bytes, without the NUL. */
#define NET_PAYLOAD_MAX         (128u)  /* Bytes. */

/**
 * One message, in either direction.  In a message buffer only the used
 * part of payload[] is stored: send net_bridge_msg_size(msg.len) bytes, and
 * create each buffer with maxmsz = sizeof(net_bridge_msg_t).
 */
typedef struct
{
    uint8_t  qos;                       /* TX: 0 or 1. */
    bool     b_retain;                  /* TX: the broker keeps it. */
    uint16_t len;                       /* Payload bytes. */
    char     topic[NET_TOPIC_MAX + 1u]; /* NUL-terminated. */
    uint8_t  payload[NET_PAYLOAD_MAX];  /* Not terminated. */
} net_bridge_msg_t;

/**
 * A subscription: messages on matching topics (the + and # wildcards are
 * allowed) are copied to rx_mbf.  The first matching entry wins.
 */
typedef struct
{
    char const * p_topic;
    ID           rx_mbf;
    uint8_t      qos;                   /* 0 or 1. */
} net_bridge_sub_t;

/**
 * What the bridge is given at start-up.  p_status_topic is optional (NULL
 * = none): "online" is published there (retained) on every connect, and
 * the broker publishes "offline" as the last will when the car disappears
 * without saying so.  p_log_topic is optional (NULL = console only): the
 * bridge's own messages (link lost, retrying in ...) are queued there too,
 * so they reach the broker after a reconnect.
 */
typedef struct
{
    ID                       tx_mbf;    /* Tasks to broker. */
    net_bridge_sub_t const * p_subs;    /* Broker to tasks. */
    uint32_t                 sub_count;
    char const *             p_status_topic;
    char const *             p_log_topic;
} net_bridge_t;

/** Connection state, in the order the bridge goes through it. */
typedef enum
{
    NET_WIFI_JOIN = 0,      /* Asking the radio to join. */
    NET_WIFI_WAIT,          /* Joining, or waiting for a DHCP lease. */
    NET_WIFI_BACKOFF,       /* Wi-Fi failed; waiting to try again. */
    NET_WIFI_SCAN,          /* Listing the networks in range. */
    NET_MQTT_CONNECT,       /* Opening a session with the broker. */
    NET_MQTT_WAIT,          /* Waiting for the broker to accept it. */
    NET_MQTT_SUBSCRIBE,     /* (Re)subscribing to every topic. */
    NET_MQTT_BACKOFF,       /* Broker failed; waiting to try again. */
    NET_ONLINE              /* Connected and subscribed. */
} net_bridge_state_t;

/** Connection state and counters, readable from any task. */
typedef struct
{
    net_bridge_state_t state;
    bool               b_online;        /* state is NET_ONLINE. */
    int32_t            rssi;            /* dBm, 0 = unknown. */
    uint32_t           ipv4;            /* First octet in bits 0-7. */
    uint32_t           wifi_joins;      /* Join attempts since boot. */
    uint32_t           mqtt_connects;   /* Broker connection attempts. */
    uint32_t           rx_dropped;      /* Incoming messages not given. */
    uint32_t           tx_dropped;      /* Outgoing messages not sent. */
} net_bridge_info_t;

int32_t      net_bridge_msg_size (uint16_t len);
void         net_bridge_arm (net_bridge_t const * p_bridge);
void         net_bridge_get_info (net_bridge_info_t * p_info);
char const * net_bridge_state_name (net_bridge_state_t state);

#endif /* NET_BRIDGE_H */

/*** end of file ***/
