/**
 * @file    net_topic.h
 * @brief   MQTT topic filter matching for the network bridge.  Coded to
 *          BARR-C:2018.
 *
 * A pure function with no kernel, lwIP or radio calls, so it is built in
 * every configuration and comms_test.c can check it directly.
 */

#ifndef NET_TOPIC_H
#define NET_TOPIC_H

#include <stdbool.h>

bool net_topic_matches (char const * p_filter, char const * p_topic);

#endif /* NET_TOPIC_H */

/*** end of file ***/
