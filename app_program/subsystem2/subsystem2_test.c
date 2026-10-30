/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.
 */

#include "subsystem2.h"
#include "../gpio_irq.h"
#include "../test_report.h"
#include "pid.h"
#include "s2_test_b2.h"
#include "s2_test_motor.h"
#include "s2_test_motion.h"

#if APP_TEST

/* Which manual test runs after the automatic ones.  Only one at a time, so
   the motors never start while a hand is on a wheel:
     S2_MANUAL_NONE none: use this while Navigation's NAV-M01 runs
                    (NAV_MANUAL_M01 in navigation_test.c)
     S2_MANUAL_B2   encoder pulses, wheels turned by hand (S2-M01)
     S2_MANUAL_M02  motors, car on a stand, USB power (S2-M02)
     S2_MANUAL_M03  task_motion commands, car on a stand, USB power
                    (S2-M03) */
#define S2_MANUAL_NONE		(0)
#define S2_MANUAL_B2		(1)
#define S2_MANUAL_M02		(2)
#define S2_MANUAL_M03		(3)
#define S2_MANUAL_TEST		(S2_MANUAL_NONE)

#define S2_TEST_PID_STEPS	(100)	/* S2-U08: enough to saturate */
#define S2_TEST_PID_DT_SEC	(0.01f)	/* one task_motion period */

#define S2_TEST_STILL_MSEC	(100u)	/* how long S2-U04 watches the wheels */
#define S2_TEST_TIMER_MSEC	(1000u)	/* S2-U06 measures the timer over this */
#define S2_TEST_TIMER_TICKS	(1000000u)	/* expected: 1 tick per usec */
#define S2_TEST_TIMER_SLACK	(20000u)	/* +/- 2 percent */

// Stand-in handler, used only to check that GP5 is already taken.
//
static void
s2_test_irq_isr (uint32_t gpio, uint32_t events, uint32_t now_usec)
{
	(void) gpio;
	(void) events;
	(void) now_usec;
}

EXPORT void subsystem2_test(void)
{
	T_RSEM	rsem;
	T_RMPF	rmpf;
	UW	seq;
	INT	v, bad = 0;
	uint32_t	count_left  = 0u;
	uint32_t	count_right = 0u;
	uint32_t	timer_start = 0u;
	uint32_t	timer_ticks = 0u;
	pid_ctrl_t	pid;
	float		out_full    = 0.0f;
	float		out_back    = 0.0f;
	float		out_reset   = 0.0f;
	int		i;

	TEST_INIT();

	/* S2-U01: every sample value is inside the documented 0..100 range */
	for (seq = 0; seq < 40; seq++) {
		v = s2_make_value(seq);
		if (v < 0 || v > 100) bad++;
	}
	TEST_CHECK("S2-U01", bad == 0, "sample values stay within 0..100");

	/* S2-U02: flow control never over- or under-counts */
	TEST_CHECK("S2-U02",
		   tk_ref_sem(sem_slots, &rsem) == E_OK &&
		   rsem.semcnt >= 0 && rsem.semcnt <= S2_POOL_COUNT &&
		   tk_ref_mpf(mpf_samples, &rmpf) == E_OK &&
		   rmpf.frbcnt <= S2_POOL_COUNT,
		   "free slots and free blocks within pool size");

	// S2-U03: one pulse is 0.330 mm, so 1000 pulses are 330 mm and one
	// revolution (637 pulses) is the 210 mm circumference.
	//
	TEST_CHECK("S2-U03",
		   (330u == drv_encoder_pulses_to_mm(1000u)) &&
		   (210u == drv_encoder_pulses_to_mm(DRV_ENCODER_PULSES_PER_REV)),
		   "encoder pulses convert to mm");

	// S2-U04: wheels at rest (car on the bench, motors off).
	//
	count_left  = drv_encoder_get_count(DRV_ENCODER_LEFT);
	count_right = drv_encoder_get_count(DRV_ENCODER_RIGHT);
	tk_dly_tsk(S2_TEST_STILL_MSEC);
	TEST_CHECK("S2-U04",
		   (count_left == drv_encoder_get_count(DRV_ENCODER_LEFT)) &&
		   (count_right == drv_encoder_get_count(DRV_ENCODER_RIGHT)) &&
		   (0u == drv_encoder_get_speed(DRV_ENCODER_LEFT)) &&
		   (0u == drv_encoder_get_speed(DRV_ENCODER_RIGHT)),
		   "still wheels: counts steady, speed 0");

	// S2-U05: drv_encoder_init() registered GP5, so a second handler is
	// refused.
	//
	TEST_CHECK("S2-U05",
		   (E_OBJ == gpio_irq_register(DRV_ENCODER_PIN_LEFT,
					       GPIO_IRQ_EDGE_RISE,
					       s2_test_irq_isr)),
		   "GPIO dispatcher refuses a second handler on GP5");

	// S2-U06: the timer behind every encoder timestamp (and the chatter
	// filter's 2 ms) really counts at 1 MHz, measured against the kernel's
	// own 1 ms tick.
	//
	timer_start = gpio_irq_get_time_usec();
	tk_dly_tsk(S2_TEST_TIMER_MSEC);
	timer_ticks = gpio_irq_get_time_usec() - timer_start;
	tm_printf((UB *) "[TEST] S2-U06 note: %lu timer ticks in %u ms "
		  "(expect %u)\n", timer_ticks, S2_TEST_TIMER_MSEC,
		  S2_TEST_TIMER_TICKS);
	TEST_CHECK("S2-U06",
		   (timer_ticks >= (S2_TEST_TIMER_TICKS - S2_TEST_TIMER_SLACK)) &&
		   (timer_ticks <= (S2_TEST_TIMER_TICKS + S2_TEST_TIMER_SLACK)),
		   "encoder timer counts at 1 MHz (+/- 2 percent)");

	// S2-U07: P term alone: 2 x (100 - 40) = 120.
	//
	pid_init(&pid, 2.0f, 0.0f, 0.0f, -1000.0f, 1000.0f);
	out_full = pid_update(&pid, 100.0f, 40.0f, S2_TEST_PID_DT_SEC);
	TEST_CHECK("S2-U07", (out_full > 119.9f) && (out_full < 120.1f),
		   "PID proportional term");

	// S2-U08: I term alone, held at the 100 limit for 1 s, would wind up
	// to 1000 without anti-windup.  With it, one step of error -10
	// brings the output straight down from 100 to 99.
	//
	pid_init(&pid, 0.0f, 10.0f, 0.0f, -100.0f, 100.0f);
	for (i = 0; i < S2_TEST_PID_STEPS; i++) {
		out_full = pid_update(&pid, 100.0f, 0.0f, S2_TEST_PID_DT_SEC);
	}
	out_back = pid_update(&pid, 0.0f, 10.0f, S2_TEST_PID_DT_SEC);
	TEST_CHECK("S2-U08",
		   (out_full > 99.9f) && (out_full < 100.1f) &&
		   (out_back > 98.9f) && (out_back < 99.1f),
		   "PID output saturates without integral windup");

	// S2-U09: reset clears the memory but keeps the gains: one step of
	// error 60 gives 10 x 60 x 0.01 = 6, as from new.
	//
	pid_reset(&pid);
	out_reset = pid_update(&pid, 100.0f, 40.0f, S2_TEST_PID_DT_SEC);
	TEST_CHECK("S2-U09", (out_reset > 5.9f) && (out_reset < 6.1f),
		   "PID reset clears memory, keeps gains");

	// S2-U10: Motion's distance conversions: 1000 mm is 3030 pulses
	// (0.330 mm each), and 3 pulses are 990 um.
	//
	TEST_CHECK("S2-U10",
		   (3030u == drv_encoder_mm_to_pulses(1000u)) &&
		   (990u == drv_encoder_pulses_to_um(3u)),
		   "encoder mm-to-pulses and pulses-to-um");

	TEST_SUMMARY("subsystem2");

	// The manual tests move the wheels, so they start only after S2-U04
	// and run in their own task.
	//
#if (S2_MANUAL_B2 == S2_MANUAL_TEST)
	CHECK(s2_test_b2_start(), "s2_test_b2_start");
#elif (S2_MANUAL_M02 == S2_MANUAL_TEST)
	CHECK(s2_test_motor_start(), "s2_test_motor_start");
#elif (S2_MANUAL_M03 == S2_MANUAL_TEST)
	CHECK(s2_test_motion_start(), "s2_test_motion_start");
#else
	/* S2_MANUAL_NONE */
#endif
}

#endif /* APP_TEST */
