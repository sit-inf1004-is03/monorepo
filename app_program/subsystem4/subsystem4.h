/**
 * @file    subsystem4.h
 * @brief   Subsystem 4 -- IMU-Based Motion and Terrain Monitoring.
 *          Buddy 4: Andrew Chia Kai Xun Bedina (2501298).
 *
 * Public interface of subsystem 4.  Other subsystems include it as
 * "../subsystem4/subsystem4.h" and use only what is listed here.
 *
 * Hardware: MPU-9250 on I2C1, GP2 = SDA, GP3 = SCL (Grove port 2).
 * Task:     imu_task, priority S4_PRI_IMU, any core, every 10 ms.
 *
 * What the other subsystems get (details in imu_task.h and imu_pack.h):
 *   Buddy 1  imu_task_get_snapshot(), imu_task_get_aux() every publish;
 *            imu_task_read_event() drained with TMO_POL; layouts in
 *            imu_pack.h (snapshot 4 B, aux 4 B, event 2 B).
 *   Buddy 2  ESTOP (fault bit 4) in the shared fault flag, waited on with
 *            faults_wait(FAULTS_NOTIFY_MOTION, ...) (common/faults.h);
 *            calls imu_task_set_encoder_source() from subsystem2_init().
 *   Nav      READY_IMU, IMU fault (bit 3) and ESTOP in the fault flag;
 *            calls imu_task_signal_course_done() at the end of the run.
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "../common/faults.h"
#include "imu_task.h"
#include "imu_pack.h"

#define S4_PRI_IMU      (18)    /* priority band 18-19 (TEAM.md section 4) */

#endif /* SUBSYSTEM4_H */

/*** end of file ***/
