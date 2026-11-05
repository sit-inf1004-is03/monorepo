/**
 * @file    imu_pack.h
 * @brief   Bit layouts of everything the IMU publishes (minimum-bit data
 *          representation, Tribunal 1 change requirement).
 *
 * All layouts use explicit shifts and masks (bitpack.h), never C bit-fields,
 * so Buddy 1's laptop decoder can use the same table.  Signed fields are two's
 * complement of the field width.  Values are clamped before packing.
 *
 * SNAPSHOT word (32 bits, overwritten every 10 ms, one atomic store):
 *   bits  0-7   pitch          int8,  0.25 deg/LSB, -32.00..+31.75 deg
 *   bits  8-14  peak_mm        uint7, highest hump peak of the run, 0..127 mm
 *   bit   15    peak_sat       1 = a hump exceeded 127 mm
 *   bits 16-17  motion class   0 stationary, 1 accelerating, 2 cruising,
 *                              3 turning
 *   bits 18-19  terrain state  0 level, 1 climbing, 2 descending,
 *                              3 hump closed
 *   bit   20    imu_fault      copy of fault bit 3
 *   bit   21    reserved, 0    (was estop; removed, impact is event-only)
 *   bit   22    tilted         pitch or roll beyond the tilt limit
 *   bit   23    no_encoder     hump distance uses the fallback speed
 *   bits 24-31  yaw rate       int8,  4 deg/s/LSB, -512..+508 deg/s
 *
 * AUX word (32 bits, same update rule; slower-changing attitude data):
 *   bits  0-7   roll           int8,  0.25 deg/LSB, -32.00..+31.75 deg
 *   bits  8-16  heading        uint9, 0..359 deg (integrated gyro, drifts)
 *   bits 17-19  hump count     uint3, saturates at 7
 *   bits 20-31  reserved, 0
 *
 * EVENT (16 bits, one per message in the IMU event buffer):
 *   bits 13-15  type (imu_pack_event_t)
 *   bits  0-12  payload:
 *     HUMP         [6:0] peak mm, [7] saturated, [12:8] max pitch deg 0..31
 *                  (peak roll deg for a side hump, see imu_terrain.h)
 *     MOTION       [1:0] new motion class
 *     TERRAIN      [1:0] new terrain state
 *     IMPACT       [7:0] peak |a|-1g deviation, 0.1 g/LSB, 0..25.5 g
 *     RECOVERED    0
 *     IMU_FAULT    [2:0] reason (imu_pack_fault_reason_t)
 *     IMU_OK       0
 *     COURSE_DONE  [6:0] run peak mm, [7] saturated, [10:8] hump count
 */

#ifndef IMU_PACK_H
#define IMU_PACK_H

#include <stdint.h>
#include <stdbool.h>

#include "imu_math.h"

#define IMU_PACK_PITCH_Q_PER_DEG    (4.0f)
#define IMU_PACK_YAW_DPS_PER_Q      (4.0f)
#define IMU_PACK_IMPACT_Q_PER_G     (10.0f)
#define IMU_PACK_PEAK_MAX_MM        (127u)
#define IMU_PACK_HEADING_MAX_DEG    (359u)
#define IMU_PACK_HUMP_COUNT_MAX     (7u)
#define IMU_PACK_EVENT_PAYLOAD_MAX  (0x1FFFu)

/** Event type, 3 bits. */
typedef enum
{
    IMU_PACK_EVT_HUMP        = 0,
    IMU_PACK_EVT_MOTION      = 1,
    IMU_PACK_EVT_TERRAIN     = 2,
    IMU_PACK_EVT_IMPACT      = 3,
    IMU_PACK_EVT_RECOVERED   = 4,
    IMU_PACK_EVT_IMU_FAULT   = 5,
    IMU_PACK_EVT_IMU_OK      = 6,
    IMU_PACK_EVT_COURSE_DONE = 7
} imu_pack_event_t;

/** Reason carried by an IMU_FAULT event, 3 bits. */
typedef enum
{
    IMU_PACK_FAULT_NONE      = 0,
    IMU_PACK_FAULT_NO_DEVICE = 1,   /**< No answer on I2C1.              */
    IMU_PACK_FAULT_WHO_AM_I  = 2,   /**< Unknown device ID.              */
    IMU_PACK_FAULT_CONFIG    = 3,   /**< Configuration did not stick.    */
    IMU_PACK_FAULT_READ      = 4,   /**< Repeated read failures.         */
    IMU_PACK_FAULT_FROZEN    = 5,   /**< Data stopped changing.          */
    IMU_PACK_FAULT_BUS_STUCK = 6,   /**< SDA/SCL held low after recovery. */
    IMU_PACK_FAULT_FILTER    = 7    /**< Non-finite filter output.       */
} imu_pack_fault_reason_t;

/** Unpacked snapshot word. */
typedef struct
{
    int32_t  pitch_q;           /**< 0.25 deg steps, -128..127.  */
    uint32_t peak_mm;           /**< 0..127.                     */
    bool     b_peak_saturated;
    uint32_t motion_class;      /**< imu_motion_class_t value.   */
    uint32_t terrain_state;     /**< imu_terrain_state_t value.  */
    bool     b_imu_fault;
    bool     b_tilted;
    bool     b_no_encoder;
    int32_t  yaw_rate_q;        /**< 4 deg/s steps, -128..127.   */
} imu_pack_snapshot_t;

/** Unpacked aux word. */
typedef struct
{
    int32_t  roll_q;            /**< 0.25 deg steps, -128..127.  */
    uint32_t heading_deg;       /**< 0..359.                     */
    uint32_t hump_count;        /**< 0..7.                       */
} imu_pack_aux_t;

int32_t  imu_pack_quantize_angle (float32_t angle_deg);
int32_t  imu_pack_quantize_yaw_rate (float32_t rate_dps);
uint32_t imu_pack_quantize_peak (float32_t peak_mm, bool * p_b_saturated);
uint32_t imu_pack_quantize_heading (float32_t heading_deg);

uint32_t imu_pack_snapshot (imu_pack_snapshot_t const * p_snap);
void     imu_pack_unpack_snapshot (uint32_t word,
                                   imu_pack_snapshot_t * p_snap);
uint32_t imu_pack_aux (imu_pack_aux_t const * p_aux);
void     imu_pack_unpack_aux (uint32_t word, imu_pack_aux_t * p_aux);

uint16_t imu_pack_event (imu_pack_event_t type, uint32_t payload);
void     imu_pack_unpack_event (uint16_t event, imu_pack_event_t * p_type,
                                uint32_t * p_payload);
uint32_t imu_pack_hump_payload (uint32_t peak_mm, bool b_saturated,
                                uint32_t max_pitch_deg);
uint32_t imu_pack_impact_payload (float32_t peak_g);
uint32_t imu_pack_course_payload (uint32_t peak_mm, bool b_saturated,
                                  uint32_t hump_count);

#endif /* IMU_PACK_H */

/*** end of file ***/
