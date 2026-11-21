/**
 * @file    net_start.h
 * @brief   Starts the network bridge on the kernel's Wi-Fi task.  Coded to
 *          BARR-C:2018.
 */

#ifndef NET_START_H
#define NET_START_H

#include <tk/typedef.h>

#include "net_bridge.h"

#define NET_START_PRI           (5)     /* Just below the Wi-Fi task's 4. */

ER net_start (net_bridge_t const * p_bridge);

#endif /* NET_START_H */

/*** end of file ***/
