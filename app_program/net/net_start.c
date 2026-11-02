/*
 *	net_start.c
 *	Starts the network bridge (net_bridge.c) on the kernel's Wi-Fi task.
 *
 *	Kernel side of the size_t split: tk/tkernel.h, no cyw43.h.
 *
 *	Why a task of its own
 *	---------------------
 *	lwIP is built NO_SYS=1: no locking, so only ONE task may ever call it,
 *	and that is the kernel's Wi-Fi service task (lib/libwifi/sysdepend/
 *	pico_rp2040/cyw43_utk.c). The kernel offers no hook into that task's
 *	loop, but the loop does call lwIP's sys_check_timeouts(), which runs
 *	any lwIP timer that is due. So the bridge runs as an lwIP timer that
 *	re-arms itself, and only the very first sys_timeout() call, in
 *	net_bridge_arm(), is made from outside the Wi-Fi task: from here.
 *
 *	That one call is safe because of where this task runs. It is pinned to
 *	the Wi-Fi task's core with a lower priority, so it can run only while
 *	the Wi-Fi task is blocked in a kernel call: its 2 ms poll delay, or a
 *	radio driver wait. None of those happen part-way through changing
 *	lwIP's timer list or memory pools, which is all sys_timeout() touches.
 *
 *	APIs shown: tk_ext_tsk
 */

#include "../app.h"
#include "net_bridge.h"
#if TM_NET_LWIP
#include "cyw43_utk.h"		/* cyw43_utk_get_status(): is the radio up? */
#endif

/* Just below the kernel's Wi-Fi task (priority 4) */
#define NET_PRI_START	5

#if TM_NET_LWIP
LOCAL const NET_BRIDGE	*start_bridge;

LOCAL void net_start_task(INT stacd, void *exinf)
{
	T_CYW43_UTK_STATUS	radio;

	(void)stacd; (void)exinf;

	/* lwIP is ready once the Wi-Fi task has finished starting up */
	do {
		tk_dly_tsk(10);
		cyw43_utk_get_status(&radio);
	} while (!radio.init_complete);

	if (radio.ready) {
		net_bridge_arm(start_bridge);
	} else {
		tm_printf((UB*)"net: the radio failed to start (%d), so there"
			  " is no network\n", (INT)radio.init_result);
	}

	tk_ext_tsk();		/* done: the task ends and frees its stack */
}
#endif

EXPORT ER net_start(const NET_BRIDGE *bridge)
{
#if TM_NET_LWIP
	ID	tid;

	start_bridge = bridge;
	tid = app_start_task(net_start_task, NET_PRI_START, TP_PRC1,
			     "net_start");
	return (tid < E_OK) ? tid : E_OK;
#else
	(void)bridge;
	tm_printf((UB*)"net: built without the network, nothing will be"
		  " sent.  Build with: make NET=lwip\n");
	return E_OK;
#endif
}
