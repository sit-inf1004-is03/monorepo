/**
 * @file    lwip_mqtt.h
 * @brief   Buffer sizes for lwIP's MQTT client as this application builds
 *          it (lwip_mqtt.c).  Coded to BARR-C:2018.
 *
 * lwIP's defaults (a 256-byte output buffer, 4 requests in flight) leave
 * little room for the subscriptions, the "online" status and the log lines
 * sent right after connecting.  The bridge copes with the defaults too (it
 * retries on ERR_MEM); these sizes only make that rarer.  They change only
 * the inside of mqtt_client_t, which nothing outside lwip_mqtt.c looks
 * into, and they need no change to the kernel's lwipopts.h.
 */

#ifndef LWIP_MQTT_H
#define LWIP_MQTT_H

#define MQTT_OUTPUT_RINGBUF_SIZE    (1024)
#define MQTT_REQ_MAX_IN_FLIGHT      (8)

#endif /* LWIP_MQTT_H */

/*** end of file ***/
