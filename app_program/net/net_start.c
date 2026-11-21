/**
 * @file    net_start.c
 * @brief   Starts the network bridge (net_bridge.c) on the kernel's Wi-Fi
 *          task.  Coded to BARR-C:2018.
 *
 * Kernel side of the size_t split: tk/tkernel.h, no cyw43.h.
 *
 * Why a task of its own: lwIP is built NO_SYS=1, with no locking, so only
 * ONE task may ever call it, and that is the kernel's Wi-Fi service task
 * (lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c).  The kernel offers no
 * hook into that task's loop, but the loop does call lwIP's
 * sys_check_timeouts(), which runs any lwIP timer that is due.  So the
 * bridge runs as an lwIP timer that re-arms itself, and only the very
 * first sys_timeout() call, in net_bridge_arm(), is made from outside the
 * Wi-Fi task: from here.
 *
 * NOTE: That one call is safe because of where this task runs.  It is
 * pinned to the Wi-Fi task's core with a lower priority, so it can run
 * only while the Wi-Fi task is blocked in a kernel call: its 2 ms poll
 * delay, or a radio driver wait.  None of those happen part-way through
 * changing lwIP's timer list or memory pools, which is all sys_timeout()
 * touches.
 */

#include "../app.h"
#include "net_start.h"
#if TM_NET_LWIP
#include "cyw43_utk.h"
#endif

#if TM_NET_LWIP
#define RADIO_POLL_MS           (10)    /* How often the radio is checked. */

static net_bridge_t const * gp_start_bridge = NULL;

static void net_start_task (INT stacd, void * p_exinf);
#endif

/**
 * @brief   Start the bridge, from a task, once, after creating every
 *          message buffer named in *p_bridge.
 * @param   p_bridge  What the bridge works with.  The bridge keeps the
 *                    pointer: the structure, its table and its strings
 *                    must never change or go away.
 * @return  E_OK, or the error from creating the start-up task.
 */
ER
net_start (net_bridge_t const * p_bridge)
{
    ER result = E_OK;

#if TM_NET_LWIP
    gp_start_bridge = p_bridge;

    ID const tid = app_start_task(net_start_task, NET_START_PRI, TP_PRC1,
                                  "net_start");

    if (tid < E_OK)
    {
        result = tid;
    }
#else
    (void) p_bridge;

    /* Cast: tm_printf() takes UB text; ASCII is the same 8-bit bytes. */
    (void) tm_printf((UB const *) "net: built without the network, nothing"
                     " will be sent.  Build with: make NET=lwip\n");
#endif

    return (result);
}

#if TM_NET_LWIP
/**
 * @brief   Wait for the radio to finish starting, arm the bridge, and end.
 * @param   stacd    Unused start code.
 * @param   p_exinf  Unused.
 */
static void
net_start_task (INT stacd, void * p_exinf)
{
    T_CYW43_UTK_STATUS radio;

    (void) stacd;
    (void) p_exinf;

    /* lwIP is ready once the Wi-Fi task has finished starting up. */
    do
    {
        (void) tk_dly_tsk(RADIO_POLL_MS);
        cyw43_utk_get_status(&radio);
    } while (0u == radio.init_complete);

    if (0u != radio.ready)
    {
        net_bridge_arm(gp_start_bridge);
    }
    else
    {
        /* Casts: tm_printf() takes UB text, and ASCII is the same 8-bit
           bytes; the driver's result is an int32_t error code, which INT
           (32 bits) holds unchanged for %d. */
        (void) tm_printf((UB const *) "net: the radio failed to start (%d),"
                         " so there is no network\n",
                         (INT) radio.init_result);
    }

    tk_ext_tsk();       /* Done: the task ends and frees its stack. */
}
#endif

/*** end of file ***/
