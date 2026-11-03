/**
 * @file    faults.h
 * @brief   Team-wide fault mask and READY bits on ONE micro T-Kernel
 *          event flag (Week 6 "evtSystem", extended in Tribunal 1).
 *
 * Owner of this header: Buddy 4 (Tribunal 1, Version C, gap G1).
 *
 * Why an event flag (G1 decision)
 * -------------------------------
 * The RP2040 Cortex-M0+ has no LDREX/STREX, and this port runs tasks on BOTH
 * cores at once, so "pause task switching" helpers do not protect a plain
 * variable: the other core keeps running.  tk_set_flg() and tk_clr_flg() are
 * atomic kernel operations on every core, may be called from a task OR from
 * a handler, and wake waiting tasks immediately (needed for a prompt stop
 * on a latched fault).
 *
 * Bit map (32-bit flag pattern)
 * -----------------------------
 *   bits  0-7   latched faults, one owner per bit (Tribunal 1, 2.6 bit map)
 *   bits  8-12  READY bits, set once a subsystem has calibrated (Week 6 4.1.5)
 *   bit   16    STOP_TURN command event for Motion Control (gap G10)
 *   bits 24-27  per-consumer NOTIFY bits (gap G2, see below)
 *   bits 28-30  reserved for on-target tests only
 *
 * How to wait without spinning or stealing bits (gap G2)
 * ------------------------------------------------------
 * Fault bits are LATCHED.  Waiting on them with TWF_ORW returns at once while
 * any is set (the waiter spins), and TWF_CLR would wipe other owners' bits.
 * So faults_set() also sets every NOTIFY bit.  Each consumer waits ONLY on its
 * own NOTIFY bit through faults_wait(), which uses TWF_ORW | TWF_BITCLR: the
 * kernel then clears just that consumer's NOTIFY bit, never a fault bit.
 * The consumer reads the latched faults with faults_get().  Example for
 * Motion Control's 10 ms loop:
 *
 *     er = faults_wait(FAULTS_NOTIFY_MOTION | FAULTS_EVT_STOP_TURN,
 *                      remaining_ms, &pattern);
 *     if ((E_OK == er) && (0u != (faults_get() & FAULTS_STOP_MASK))) ...
 *
 * The flag is created with TA_WMUL so several tasks may wait on it.
 *
 * Ownership rules
 * ---------------
 *   - A subsystem sets only its own fault bits (table below).
 *   - Clearing: the owner may clear its own bit when its recovery succeeds
 *     (the IMU does this for its IMU-fault bit).  Navigation may clear any
 *     bit on a system Reset.
 *   - Nobody uses a plain |= or &= on a copy of the mask.
 */

#ifndef FAULTS_H
#define FAULTS_H

/* NOTE: no <stddef.h> here.  micro T-Kernel's tk/syslib.h defines its own
 * size_t (as long), which conflicts with <stddef.h>; NULL comes from
 * tk/typedef.h instead.
 */
#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

/* Latched fault bits (agreed in Tribunal 1, Zoom 08:34-09:15). */
#define FAULTS_MOTOR_STALL      (0x00000001u)   /* Buddy 2 */
#define FAULTS_ENCODER          (0x00000002u)   /* Buddy 2 */
#define FAULTS_LINE_LOST        (0x00000004u)   /* Buddy 3 */
#define FAULTS_IMU              (0x00000008u)   /* Buddy 4 */
/* bit 4 (0x10) free: was FAULTS_ESTOP, removed - impact is telemetry only */
#define FAULTS_NO_ECHO          (0x00000020u)   /* Buddy 5 */
#define FAULTS_SCAN_ERROR       (0x00000040u)   /* Buddy 5 */
#define FAULTS_WIFI_LOSS        (0x00000080u)   /* Buddy 1 */
#define FAULTS_ALL_MASK         (0x000000FFu)

/*
 * Faults that must stop the car.  WIFI_LOSS is excluded (Degraded mode only,
 * Tribunal 1, 2.1 Buddy 1).  IMU fault and NO_ECHO are excluded so that a
 * sensor dropout degrades the run instead of ending it.
 * TODO: PROPOSED by Buddy 4; Navigation owner to confirm (gap G10).
 */
#define FAULTS_STOP_MASK        (FAULTS_MOTOR_STALL | FAULTS_ENCODER | \
                                 FAULTS_LINE_LOST | FAULTS_SCAN_ERROR)

/* READY bits: set once after calibration, read by Navigation. */
#define FAULTS_READY_LINE       (0x00000100u)   /* Buddy 3 */
#define FAULTS_READY_IMU        (0x00000200u)   /* Buddy 4 */
#define FAULTS_READY_SCAN       (0x00000400u)   /* Buddy 5 */
#define FAULTS_READY_MOTION     (0x00000800u)   /* Buddy 2 */
#define FAULTS_READY_COMM       (0x00001000u)   /* Buddy 1 */
#define FAULTS_READY_ALL        (0x00001F00u)

/* Command event bits (edge events, cleared by the waiter). */
#define FAULTS_EVT_STOP_TURN    (0x00010000u)   /* Buddy 3/5 -> Buddy 2 */

/* Per-consumer notify bits, all set by every faults_set() call. */
#define FAULTS_NOTIFY_MOTION    (0x01000000u)
#define FAULTS_NOTIFY_NAV       (0x02000000u)
#define FAULTS_NOTIFY_COMM      (0x04000000u)
#define FAULTS_NOTIFY_TEST      (0x08000000u)
#define FAULTS_NOTIFY_ALL       (0x0F000000u)

/* Reserved for on-target stress tests (subsystem4_test.c). */
#define FAULTS_TEST_MASK        (0x70000000u)

/* Bits that faults_signal() / faults_clear_status() may touch. */
#define FAULTS_STATUS_MASK      (FAULTS_READY_ALL | FAULTS_EVT_STOP_TURN | \
                                 FAULTS_TEST_MASK)

ER       faults_init (void);
ER       faults_set (uint32_t fault_bits);
ER       faults_clear (uint32_t fault_bits);
uint32_t faults_get (void);
bool     faults_is_set (uint32_t fault_bits);
ER       faults_signal (uint32_t status_bits);
ER       faults_clear_status (uint32_t status_bits);
ER       faults_wait (uint32_t wait_bits, TMO timeout_ms,
                      uint32_t * p_pattern);
ER       faults_wait_all (uint32_t wait_bits, TMO timeout_ms);

#endif /* FAULTS_H */

/*** end of file ***/
