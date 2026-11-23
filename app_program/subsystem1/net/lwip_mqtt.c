/**
 * @file    lwip_mqtt.c
 * @brief   lwIP's MQTT client (lib/lwip/src/apps/mqtt/mqtt.c from the Pico
 *          SDK), compiled as part of the application.  Coded to
 *          BARR-C:2018, with one deviation, below.
 *
 * Why here and not with "make MQTT=1": MQTT=1 also links the kernel's demo
 * client (lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c), which the Wi-Fi
 * task starts by itself.  It connects with the same client ID and would
 * keep knocking the bridge off the broker, and changing it means changing
 * the kernel.  So this application is built with "make NET=lwip" and
 * brings mqtt.c in itself, with the buffer sizes in lwip_mqtt.h.
 *
 * DEVIATION from Rule 4.3.f (no source file includes another source
 * file): this file includes lwIP's mqtt.c.  It is third-party code that
 * the kernel's NET=lwip build does not compile, and adding it to the
 * shared makefile would also need the buffer sizes passed to that one
 * file.  Including it keeps the build files and the kernel unchanged.
 * Approval: pending the project manager (BARR-C deviation procedure).
 */

#if TM_NET_LWIP

#if TM_MQTT
#error "Build with make NET=lwip, not MQTT=1: see net/lwip_mqtt.c"
#endif

#include "lwip_mqtt.h"

/* lwIP's include directory is on the include path; its sources are next
   to it, at PICO_SDK_PATH/lib/lwip/src/apps/mqtt/mqtt.c. */
#include "lwip/../../apps/mqtt/mqtt.c"

#endif /* TM_NET_LWIP */

/*** end of file ***/
