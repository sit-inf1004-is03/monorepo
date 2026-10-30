/** @file s2_test_motor.h
 *
 * @brief Subsystem 2 manual test S2-M02 (a rerun of test plan B3 with
 *        drv_motor): motor wiring, direction and speed.  Built only with
 *        make TEST=1.
 */

#ifndef S2_TEST_MOTOR_H
#define S2_TEST_MOTOR_H

#include "../app.h"

#if APP_TEST

ER s2_test_motor_start(void);

#endif /* APP_TEST */

#endif /* S2_TEST_MOTOR_H */

/*** end of file ***/

