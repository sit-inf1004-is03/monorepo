/** @file s2_test_motion.h
 *
 * @brief Subsystem 2 manual test S2-M03: task_motion's straight moves and
 *        pivots, on a stand.  Built only with make TEST=1.
 */

#ifndef S2_TEST_MOTION_H
#define S2_TEST_MOTION_H

#include "../app.h"

#if APP_TEST

ER s2_test_motion_start(void);

#endif /* APP_TEST */

#endif /* S2_TEST_MOTION_H */

/*** end of file ***/

