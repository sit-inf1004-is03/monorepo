/** @file subsystem5.h
 *
 * @brief Subsystem 5 -- Obstacle Scan & Avoidance.  Student: You Rui En
 *        (2502821).
 *
 * Owns: ultrasonic sensor (TRIG GP16, ECHO GP17 via gpio_irq), scan servo
 * (GP12, PWM slice 6).  One task, obstacle_scan_task.
 *
 * Talks to Navigation through navigation.h (IF-11 obstacle / bypass plan,
 * IF-12 fault bits 5-6, IF-14 vehicle state) and to Motion through
 * motion.h (IF-09 StopTurn).  It never drives the motors itself.
 *
 * Coding standard: BARR-C:2018.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"

#define S5_PRI_MAIN     (20)    // Priority band 20-21 (TEAM.md section 4).

#endif /* SUBSYSTEM5_H */

/*** end of file ***/
