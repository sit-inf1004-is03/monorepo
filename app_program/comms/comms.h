/**
 * @file    comms.h
 * @brief   Comms: what subsystems use to send and receive MQTT messages.
 *          Coded to BARR-C:2018.
 *
 * No subsystem task calls lwIP or MQTT.  The network bridge (net/) runs
 * the MQTT client on the kernel's Wi-Fi service task, and tasks exchange
 * messages with it through message buffers:
 *
 *     task, comms_publish(), TX message buffer, bridge, broker
 *     broker, bridge, RX message buffer, tk_rcv_mbf(), task
 *
 * This module owns the TX message buffer, collects the subscriptions, and
 * hands both to the bridge in comms_start().  The payload format of
 * protocol.h is parsed and written by comms_payload.h.
 *
 * Receiving: a task creates its own message buffer, with bufsz from
 * comms_mbf_size() and maxmsz = sizeof(comms_msg_t), and in its _init()
 * subscribes it to topics:
 *
 *     comms_subscribe(COMMS_TOPIC_CMD, my_mbf);
 *     ...
 *     (void) tk_rcv_mbf(my_mbf, &msg, TMO_FEVR);
 *     if (E_OK == comms_payload_parse_msg(&msg, &cmd)) ...
 *
 * A message for a task whose buffer is full is dropped: the bridge never
 * waits for a task.
 *
 * Sending, from any task, never blocks:
 *
 *     comms_log("battery low");
 *     comms_telemetry(words, TLM_WORDS);
 *     comms_publish(COMMS_TOPIC_EVENT, p_data, len, false);
 *
 * Log lines and plain publishes wait in the TX buffer while offline and
 * are sent after reconnecting; telemetry is dropped while offline, as only
 * the latest value matters.  A full TX buffer drops the new message.
 */

#ifndef COMMS_H
#define COMMS_H

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"
#include "../protocol.h"
#include "../net/net_bridge.h"
#include "comms_payload.h"

#if TM_NET_LWIP
# include "wifi_secrets.h"              /* MQTT_CLIENT_ID */
#else
# define MQTT_CLIENT_ID     "offline"   /* Built without the network. */
#endif

/* Full topic names (protocol.h), as string literals. */
#define COMMS_TOPIC_PREFIX      "robocar/" MQTT_CLIENT_ID "/"
#define COMMS_TOPIC_STATUS      COMMS_TOPIC_PREFIX TOPIC_STATUS
#define COMMS_TOPIC_TELEMETRY   COMMS_TOPIC_PREFIX TOPIC_TELEMETRY
#define COMMS_TOPIC_EVENT       COMMS_TOPIC_PREFIX TOPIC_EVENT
#define COMMS_TOPIC_LOG         COMMS_TOPIC_PREFIX TOPIC_LOG
#define COMMS_TOPIC_CMD         COMMS_TOPIC_PREFIX TOPIC_CMD
#define COMMS_TOPIC_CONFIG      COMMS_TOPIC_PREFIX TOPIC_CONFIG
#define COMMS_TOPIC_CMD_ALL     "robocar/all/" TOPIC_CMD

#define COMMS_MAX_SUBS          (6u)    /* Topics that can be subscribed. */

/** A message as received: topic and payload. */
typedef net_bridge_msg_t comms_msg_t;

ER      comms_init (void);
ER      comms_start (void);
ER      comms_subscribe (char const * p_topic, ID mbf);
int32_t comms_mbf_size (uint32_t messages);
bool    comms_topic_is (comms_msg_t const * p_msg, char const * p_topic);
ER      comms_publish (char const * p_topic, void const * p_data,
                       uint32_t len, bool b_retain);
ER      comms_log (char const * p_text);
ER      comms_telemetry (uint64_t const * p_words, uint32_t count);
void    comms_get_info (net_bridge_info_t * p_info);

#endif /* COMMS_H */

/*** end of file ***/
