/*
 *	usb_console.c
 *	USB CDC-ACM backend for the T-Monitor console (RaspberryPi Pico / Pico W)
 *
 *	Producers (tm_snd_dat, called from any context including handlers) only
 *	append to a ring buffer and return.  A service task owns TinyUSB and
 *	drains the ring to the CDC endpoint.  An SMP producer may spin briefly
 *	behind the other processor's ring update, but it never waits for the host
 *	or performs a scheduler wait.  An unplugged or unenumerated host therefore
 *	cannot stall the kernel -- output is dropped and counted instead.
 */

#include <tk/tkernel.h>
#include <tk/typedef.h>
#include <sys/sysdef.h>

#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif

#if USE_TMONITOR && TM_CONSOLE_USB_CDC

/* usb_tinyusb_glue.c -- keeps TinyUSB headers out of this translation unit. */
IMPORT int	tm_tusb_init(void);
IMPORT void	tm_tusb_service(void);
IMPORT int	tm_tusb_event_ready(void);
IMPORT int	tm_tusb_mounted(void);
IMPORT int	tm_tusb_connected(void);
IMPORT unsigned	tm_tusb_write_available(void);
IMPORT void	tm_tusb_write_char(char value);
IMPORT void	tm_tusb_write_flush(void);
IMPORT void	tm_tusb_discard_input(void);

#define USB_TASK_PRIORITY	3
#if TK_SUPPORT_SMP
#define USB_TASK_STACK		4096
#else
#define USB_TASK_STACK		2048
#endif
#define USB_POLL_MS		10
#define USB_TX_SIZE		4096U
#define USB_TX_MASK		(USB_TX_SIZE - 1U)
#define USB_TX_CHUNK		64U

/*
 * LOCAL (s5-local-test): flight recorder.  Every console byte is also kept
 * in rec_buffer, newest overwriting oldest, even with no host attached.
 * When a serial monitor opens the port (DTR rises), the recorder is sent
 * first, then live output continues.  So: run the car on battery, plug the
 * USB in WITHOUT resetting, open the monitor, and read what happened.
 */
#define USB_REC_SIZE		32768U
#define USB_REC_MASK		(USB_REC_SIZE - 1U)

LOCAL UB		rec_buffer[USB_REC_SIZE];
LOCAL volatile UW	rec_total;	/* bytes ever written (wraps) */
LOCAL UW		rep_pos;	/* replay cursor (rec_total units) */
LOCAL UW		rep_end;	/* replay stops here */
LOCAL UB		rep_active;
LOCAL UB		was_connected;

LOCAL void rep_banner(const char *text)
{
	while(*text != '\0') {
		tm_tusb_write_char(*text++);
	}
	tm_tusb_write_flush();
}

LOCAL UB		tx_buffer[USB_TX_SIZE];
LOCAL volatile UW	tx_head;
LOCAL volatile UW	tx_tail;
LOCAL volatile UB	usb_started;

#if TK_SUPPORT_SMP
LOCAL T_SPLOCK		tx_lock;
#endif

/*
 * Keep the producer ring safe in all supported configurations.  Interrupt
 * masking is sufficient on UP; SMP additionally needs a cross-core lock.
 * The interrupt-preserving form is required because T-Monitor output is
 * permitted from both task and handler context.
 */
LOCAL void tx_lock_enter(UINT *intsts)
{
#if TK_SUPPORT_SMP
	(void)ISpinLock(&tx_lock, intsts);
#else
	DI(*intsts);
#endif
}

LOCAL void tx_lock_leave(UINT intsts)
{
#if TK_SUPPORT_SMP
	(void)ISpinUnlock(&tx_lock, intsts);
#else
	EI(intsts);
#endif
}

/* Bytes discarded because the ring was full.  Exposed so a test can assert
   that a run produced no loss rather than assuming it. */
EXPORT volatile UW	tm_usb_dropped;

/*
 * Link state, for diagnostics that cannot rely on the console itself:
 *	0  service task not started
 *	1  started, not enumerated by a host
 *	2  enumerated (mounted)
 *	3  enumerated and host asserted DTR
 */
EXPORT volatile UW	tm_usb_link_state;

EXPORT volatile UW	tm_usb_init_ok;

EXPORT UW tm_usb_state(void)
{
	return tm_usb_link_state;
}

/* Bytes still queued for the host.  A producer that wants its output to
   survive must wait for this to reach zero before doing anything that could
   overrun the ring. */
EXPORT UW tm_usb_pending(void)
{
	UINT	intsts;
	UW	pending;

	tx_lock_enter(&intsts);
	pending = (tx_head - tx_tail) & USB_TX_MASK;
	tx_lock_leave(intsts);

	return pending;
}

/*
 * Ordered bring-up checks, for readout over GPIO when USB does not
 * enumerate and the console therefore cannot describe its own failure.
 * Returns the number of consecutive checks that passed.
 */
IMPORT volatile UW	tm_usb_irq_count;
IMPORT volatile W	tm_usb_def_int_ercd;
IMPORT volatile UW	tm_usb_nvic_enabled;
IMPORT UW		tm_tusb_diag_controller_en(void);
IMPORT UW		tm_tusb_diag_pullup_en(void);
IMPORT UW		tm_tusb_diag_bus_seen(void);

EXPORT INT tm_usb_diag_level(void)
{
	if(!tm_usb_init_ok)		return 0;	/* tusb_init failed      */
	if(tm_usb_def_int_ercd != 0)	return 1;	/* tk_def_int failed     */
	if(!tm_usb_nvic_enabled)	return 2;	/* NVIC not unmasked     */
	if(!tm_tusb_diag_controller_en()) return 3;	/* controller disabled   */
	if(!tm_tusb_diag_pullup_en())	return 4;	/* D+ pull-up off        */
	if(tm_usb_irq_count == 0)	return 5;	/* no USB interrupt      */
	if(!tm_tusb_diag_bus_seen())	return 6;	/* no host bus activity  */
	if(tm_usb_link_state < 2)	return 7;	/* not enumerated        */
	return 8;					/* fully up              */
}

IMPORT volatile W	tm_usb_service_tskid;

LOCAL void usb_service_task(INT stacd, void *exinf)
{
	unsigned	available;
	UB		chunk[USB_TX_CHUNK];
	UW		count;
	UINT		intsts;

	(void)stacd;
	(void)exinf;

	/* Publish the task id before the controller is initialised, so an
	   interrupt arriving during enumeration always has somewhere to go. */
	tm_usb_service_tskid = tk_get_tid();

	tm_usb_init_ok = tm_tusb_init() ? 1 : 0;
	if(!tm_usb_init_ok) {
		tm_usb_service_tskid = 0;
		tk_ext_tsk();
		return;
	}

	tm_usb_link_state = 1;

	for(;;) {
		/* Drain the whole event burst before yielding.  Returning to
		   the scheduler between events would put a tick period into
		   each stage of every control transfer. */
		do {
			tm_tusb_service();
		} while(tm_tusb_event_ready());

		tm_usb_link_state = tm_tusb_mounted()
				  ? (tm_tusb_connected() ? 3 : 2)
				  : 1;

		/* Gate on enumeration, not on DTR.  If the host is not reading,
		   write_available() falls to zero and the loop simply stops --
		   it never waits. */
		/* LOCAL: monitor just opened -> replay the recorder.  The
		   tx ring holds only a copy of recorded bytes, so it is
		   emptied; output after rep_end goes through it as usual. */
		if(tm_usb_link_state == 3 && !was_connected) {
			tx_lock_enter(&intsts);
			tx_tail = tx_head;
			rep_end = rec_total;
			rep_pos = (rep_end > USB_REC_SIZE)
				  ? (rep_end - USB_REC_SIZE) : 0U;
			tx_lock_leave(intsts);
			rep_active = 1;
			rep_banner("\r\n===== RECORDED LOG (before you "
				   "connected) =====\r\n");
		}
		was_connected = (tm_usb_link_state == 3) ? 1 : 0;
		if(!was_connected) rep_active = 0;

		if(tm_tusb_mounted() && rep_active) {
			available = tm_tusb_write_available();
			while(available > 0 && rep_active) {
				count = 0;
				tx_lock_enter(&intsts);
				/* overwritten while replaying: skip ahead */
				if((rec_total - rep_pos) > USB_REC_SIZE) {
					rep_pos = rec_total - USB_REC_SIZE;
					/* all of it overwritten: end the replay */
					if((W)(rep_pos - rep_end) > 0) {
						rep_pos = rep_end;
					}
				}
				while(count < USB_TX_CHUNK && count < available &&
				      rep_pos != rep_end) {
					chunk[count++] =
					    rec_buffer[rep_pos & USB_REC_MASK];
					rep_pos++;
				}
				tx_lock_leave(intsts);
				for(UW i = 0; i < count; i++) {
					tm_tusb_write_char((char)chunk[i]);
				}
				available -= count;
				if(rep_pos == rep_end) {
					rep_active = 0;
					tm_tusb_write_flush();
					rep_banner("\r\n===== END OF RECORDED "
						   "LOG, LIVE FROM HERE "
						   "=====\r\n");
				}
			}
			tm_tusb_write_flush();
		}

		if(tm_tusb_mounted() && !rep_active) {
			available = tm_tusb_write_available();
			while(available > 0) {
				count = 0;
				tx_lock_enter(&intsts);
				while(count < USB_TX_CHUNK && count < available &&
				      tx_tail != tx_head) {
					chunk[count++] = tx_buffer[tx_tail];
					tx_tail = (tx_tail + 1U) & USB_TX_MASK;
				}
				tx_lock_leave(intsts);

				if(count == 0) break;
				for(UW i = 0; i < count; i++) {
					tm_tusb_write_char((char)chunk[i]);
				}
				available -= count;
			}
			tm_tusb_write_flush();
		}
		tm_tusb_discard_input();

		/* Sleep until the controller interrupts, with a periodic
		   fallback so the ring still drains when the link is idle. */
		tk_slp_tsk(USB_POLL_MS);
	}
}

/*
 * Start the console service.  Called from knl_start_device(), i.e. from the
 * initial task once the kernel is fully up, so every application gets the
 * console without having to wire it in itself.
 */
EXPORT ER tm_usb_console_start(void)
{
	T_CTSK	ctsk;
	ID	tskid;
	ER	ercd;

	if(usb_started) return E_OK;

#if TK_SUPPORT_SMP
	ercd = InitSpinLock(&tx_lock);
	if(ercd < E_OK) return ercd;
#endif

	/* TinyUSB is initialised inside the task itself, so the task id is
	   published before the controller can raise its first interrupt. */
	/*
	 * Pinned to processor 1 (core 0).
	 *
	 * TinyUSB masks the USB interrupt by writing the calling core's NVIC,
	 * and the NVIC is per core.  A service task free to migrate would
	 * enable the interrupt on whichever core happened to run tusb_init()
	 * and then take its critical sections on the other one, where the mask
	 * protects nothing -- and enumeration fails, which is precisely how
	 * this resurfaced once the global scheduler was allowed to place this
	 * task.  Interrupt ownership and the task that services it must live on
	 * the same processor.
	 */
	ctsk.exinf   = NULL;
#if TK_SUPPORT_SMP
	ctsk.tskatr  = TA_HLNG | TA_RNG0 | TA_ASSPRC;
	ctsk.assprc  = TP_PRC1;
#else
	ctsk.tskatr  = TA_HLNG | TA_RNG0;
	ctsk.assprc  = 0;
#endif
	ctsk.task    = (FP)usb_service_task;
	ctsk.itskpri = USB_TASK_PRIORITY;
	ctsk.stksz   = USB_TASK_STACK;
	ctsk.bufptr  = NULL;

	tskid = tk_cre_tsk(&ctsk);
	if(tskid <= 0) return (ER)tskid;

	ercd = tk_sta_tsk(tskid, 0);
	if(ercd < E_OK) {
		(void)tk_del_tsk(tskid);
		return ercd;
	}

	usb_started = 1;
	return E_OK;
}

/*
 * Append to the ring.  This may spin on the short SMP ring lock, but never
 * waits on the host and never performs a scheduler wait.
 */
EXPORT void tm_usb_console_put(const UB *buf, INT size)
{
	UW	next;
	UINT	intsts;

	tx_lock_enter(&intsts);
	while(size-- > 0) {
		rec_buffer[rec_total & USB_REC_MASK] = *buf;	/* LOCAL */
		rec_total++;
		next = (tx_head + 1U) & USB_TX_MASK;
		if(next == tx_tail) {		/* full: drop, and count it */
			tm_usb_dropped++;
			buf++;
			continue;
		}
		tx_buffer[tx_head] = *buf++;
		tx_head = next;
	}
	tx_lock_leave(intsts);
}

#endif /* USE_TMONITOR && TM_CONSOLE_USB_CDC */
