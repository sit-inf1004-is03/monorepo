/*
 *	lwip_mqtt.c
 *	lwIP's MQTT client (lib/lwip/src/apps/mqtt/mqtt.c from the Pico SDK),
 *	compiled as part of the application.
 *
 *	Why here and not with "make MQTT=1": MQTT=1 also links the kernel's
 *	demo client (lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c), which the
 *	Wi-Fi task starts by itself. It connects with the same client ID and
 *	would keep knocking the bridge off the broker, and changing it means
 *	changing the kernel. So this application is built with
 *	"make NET=lwip" and brings mqtt.c in itself.
 *
 *	It also sizes the client's buffers without editing the kernel's
 *	lwipopts.h: lwIP's defaults (a 256-byte output buffer, 4 requests in
 *	flight) leave little room for the subscriptions, the "online" status
 *	and the log lines sent right after connecting. (The bridge copes with
 *	the defaults too -- it retries on ERR_MEM -- this only makes that
 *	rarer.) These sizes only change the inside of mqtt_client_t, which
 *	nothing outside this file looks into.
 */

#if TM_NET_LWIP

#if TM_MQTT
#error "Build with: make NET=lwip   (not MQTT=1: see app_program/net/lwip_mqtt.c)"
#endif

#define MQTT_OUTPUT_RINGBUF_SIZE	1024
#define MQTT_REQ_MAX_IN_FLIGHT		8

/* lwIP's include directory is on the include path; its sources are next
   to it, at $(PICO_SDK_PATH)/lib/lwip/src/apps/mqtt/mqtt.c */
#include "lwip/../../apps/mqtt/mqtt.c"

#endif /* TM_NET_LWIP */
