/** @file obstacle_scan.h
 *
 * @brief Obstacle Scan task (Subsystem 5).
 *
 * Coding standard: BARR-C:2018.
 */

#ifndef OBSTACLE_SCAN_H
#define OBSTACLE_SCAN_H

#include "../app.h"

/*!
 * @brief  Obstacle Scan task entry (started by subsystem5_start()).
 * @param  stacd    Start code (unused).
 * @param  p_exinf  Extended information (unused).
 *
 * NOTE: Signature fixed by the kernel's task entry type.
 */
void obstacle_scan_task(INT stacd, void * p_exinf);

#endif /* OBSTACLE_SCAN_H */

/*** end of file ***/
