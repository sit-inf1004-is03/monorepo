/**
 * @file    net_backoff.h
 * @brief   Retry timing for the network bridge: exponential backoff with
 *          jitter.  Coded to BARR-C:2018.
 *
 * Pure functions with no kernel, lwIP or radio calls, so they are built in
 * every configuration and comms_test.c can check them directly.
 */

#ifndef NET_BACKOFF_H
#define NET_BACKOFF_H

#include <stdint.h>

/** One layer's retry timing. */
typedef struct
{
    uint32_t min_ms;    /* First wait. */
    uint32_t max_ms;    /* Longest wait. */
    uint32_t step_ms;   /* The wait the next failure gets. */
} net_backoff_t;

void     net_backoff_init (net_backoff_t * p_backoff, uint32_t min_ms,
                           uint32_t max_ms);
void     net_backoff_reset (net_backoff_t * p_backoff);
uint32_t net_backoff_next (net_backoff_t * p_backoff, uint32_t random);

#endif /* NET_BACKOFF_H */

/*** end of file ***/
