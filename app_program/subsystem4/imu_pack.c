/**
 * @file    imu_pack.c
 * @brief   Pack and unpack the IMU snapshot, aux and event layouts.
 *
 * The field table in imu_pack.h is the contract with Buddy 1's telemetry
 * frame and laptop decoder.  Changing a shift or width here changes that
 * contract and must be agreed with the team.
 */

#include <stddef.h>

#include "imu_pack.h"
#include "../common/bitpack.h"

/* Snapshot word. */
#define SNAP_PITCH_SHIFT        (0u)
#define SNAP_PITCH_WIDTH        (8u)
#define SNAP_PEAK_SHIFT         (8u)
#define SNAP_PEAK_WIDTH         (7u)
#define SNAP_SAT_SHIFT          (15u)
#define SNAP_MOTION_SHIFT       (16u)
#define SNAP_MOTION_WIDTH       (2u)
#define SNAP_TERRAIN_SHIFT      (18u)
#define SNAP_TERRAIN_WIDTH      (2u)
#define SNAP_FAULT_SHIFT        (20u)
/* bit 21 reserved (was SNAP_ESTOP_SHIFT; impact is event-only) */
#define SNAP_TILT_SHIFT         (22u)
#define SNAP_NOENC_SHIFT        (23u)
#define SNAP_YAW_SHIFT          (24u)
#define SNAP_YAW_WIDTH          (8u)

/* Aux word. */
#define AUX_ROLL_SHIFT          (0u)
#define AUX_ROLL_WIDTH          (8u)
#define AUX_HEADING_SHIFT       (8u)
#define AUX_HEADING_WIDTH       (9u)
#define AUX_COUNT_SHIFT         (17u)
#define AUX_COUNT_WIDTH         (3u)

/* Event. */
#define EVT_PAYLOAD_SHIFT       (0u)
#define EVT_PAYLOAD_WIDTH       (13u)
#define EVT_TYPE_SHIFT          (13u)
#define EVT_TYPE_WIDTH          (3u)

/* Event payload sub-fields. */
#define PL_PEAK_SHIFT           (0u)
#define PL_PEAK_WIDTH           (7u)
#define PL_SAT_SHIFT            (7u)
#define PL_PITCH_SHIFT          (8u)
#define PL_PITCH_WIDTH          (5u)
#define PL_PITCH_MAX            (31u)
#define PL_COUNT_SHIFT          (8u)
#define PL_COUNT_WIDTH          (3u)
#define PL_IMPACT_MAX           (255)

#define FLAG_WIDTH              (1u)
#define INT8_FIELD_MIN          (-128)
#define INT8_FIELD_MAX          (127)
#define DEG_FULL_TURN           (360)

static uint32_t flag_bit (bool b_flag);

/**
 * @brief   Angle to the 0.25 deg int8 field (pitch, roll), clamped.
 * @param   angle_deg  Angle in degrees.
 * @return  -128..127.
 */
int32_t
imu_pack_quantize_angle (float32_t angle_deg)
{
    return (bitpack_clamp_s32(imu_math_round(angle_deg *
                                             IMU_PACK_PITCH_Q_PER_DEG),
                              INT8_FIELD_MIN, INT8_FIELD_MAX));
}

/**
 * @brief   Yaw rate to the 4 deg/s int8 field, clamped.
 * @param   rate_dps  Yaw rate in deg/s.
 * @return  -128..127.
 */
int32_t
imu_pack_quantize_yaw_rate (float32_t rate_dps)
{
    return (bitpack_clamp_s32(imu_math_round(rate_dps /
                                             IMU_PACK_YAW_DPS_PER_Q),
                              INT8_FIELD_MIN, INT8_FIELD_MAX));
}

/**
 * @brief   Peak height to the 7-bit mm field plus saturation flag.
 * @param   peak_mm        Height in mm (negative counts as 0).
 * @param   p_b_saturated  Receives true if the height exceeded 127 mm.
 * @return  0..127.
 */
uint32_t
imu_pack_quantize_peak (float32_t peak_mm, bool * p_b_saturated)
{
    int32_t const rounded = imu_math_round(peak_mm);

    if (NULL != p_b_saturated)
    {
        *p_b_saturated = (rounded > (int32_t)IMU_PACK_PEAK_MAX_MM);
    }

    /* Cast: clamped to 0..127, non-negative. */
    return ((uint32_t)bitpack_clamp_s32(rounded, 0,
                                        (int32_t)IMU_PACK_PEAK_MAX_MM));
}

/**
 * @brief   Heading to the 9-bit 0..359 field.
 * @param   heading_deg  Heading in degrees (any value).
 * @return  0..359.
 */
uint32_t
imu_pack_quantize_heading (float32_t heading_deg)
{
    int32_t whole = imu_math_round(imu_math_wrap_360(heading_deg));

    if (whole >= DEG_FULL_TURN)
    {
        whole = 0;
    }

    /* Cast: whole is in 0..359 after wrapping. */
    return ((uint32_t)bitpack_clamp_s32(whole, 0,
                                        (int32_t)IMU_PACK_HEADING_MAX_DEG));
}

/**
 * @brief   Build the snapshot word.
 * @param   p_snap  Fields; each is clamped to its width.
 * @return  The packed word (0 if @p p_snap is NULL).
 */
uint32_t
imu_pack_snapshot (imu_pack_snapshot_t const * p_snap)
{
    uint32_t word = 0u;

    if (NULL != p_snap)
    {
        word = bitpack_insert(word, bitpack_encode_signed(p_snap->pitch_q,
                                                          SNAP_PITCH_WIDTH),
                              SNAP_PITCH_SHIFT, SNAP_PITCH_WIDTH);
        word = bitpack_insert(word,
                              bitpack_clamp_u32(p_snap->peak_mm,
                                                IMU_PACK_PEAK_MAX_MM),
                              SNAP_PEAK_SHIFT, SNAP_PEAK_WIDTH);
        word = bitpack_insert(word, flag_bit(p_snap->b_peak_saturated),
                              SNAP_SAT_SHIFT, FLAG_WIDTH);
        word = bitpack_insert(word,
                              bitpack_clamp_u32(p_snap->motion_class,
                                  bitpack_mask(SNAP_MOTION_WIDTH)),
                              SNAP_MOTION_SHIFT, SNAP_MOTION_WIDTH);
        word = bitpack_insert(word,
                              bitpack_clamp_u32(p_snap->terrain_state,
                                  bitpack_mask(SNAP_TERRAIN_WIDTH)),
                              SNAP_TERRAIN_SHIFT, SNAP_TERRAIN_WIDTH);
        word = bitpack_insert(word, flag_bit(p_snap->b_imu_fault),
                              SNAP_FAULT_SHIFT, FLAG_WIDTH);
        word = bitpack_insert(word, flag_bit(p_snap->b_tilted),
                              SNAP_TILT_SHIFT, FLAG_WIDTH);
        word = bitpack_insert(word, flag_bit(p_snap->b_no_encoder),
                              SNAP_NOENC_SHIFT, FLAG_WIDTH);
        word = bitpack_insert(word,
                              bitpack_encode_signed(p_snap->yaw_rate_q,
                                                    SNAP_YAW_WIDTH),
                              SNAP_YAW_SHIFT, SNAP_YAW_WIDTH);
    }

    return (word);
}

/**
 * @brief   Decode a snapshot word (used by tests and as the reference for
 *          Buddy 1's decoder).
 * @param   word    Packed word.
 * @param   p_snap  Receives the fields.
 */
void
imu_pack_unpack_snapshot (uint32_t word, imu_pack_snapshot_t * p_snap)
{
    if (NULL != p_snap)
    {
        p_snap->pitch_q = bitpack_decode_signed(
            bitpack_extract(word, SNAP_PITCH_SHIFT, SNAP_PITCH_WIDTH),
            SNAP_PITCH_WIDTH);
        p_snap->peak_mm = bitpack_extract(word, SNAP_PEAK_SHIFT,
                                          SNAP_PEAK_WIDTH);
        p_snap->b_peak_saturated =
            (0u != bitpack_extract(word, SNAP_SAT_SHIFT, FLAG_WIDTH));
        p_snap->motion_class = bitpack_extract(word, SNAP_MOTION_SHIFT,
                                               SNAP_MOTION_WIDTH);
        p_snap->terrain_state = bitpack_extract(word, SNAP_TERRAIN_SHIFT,
                                                SNAP_TERRAIN_WIDTH);
        p_snap->b_imu_fault =
            (0u != bitpack_extract(word, SNAP_FAULT_SHIFT, FLAG_WIDTH));
        p_snap->b_tilted =
            (0u != bitpack_extract(word, SNAP_TILT_SHIFT, FLAG_WIDTH));
        p_snap->b_no_encoder =
            (0u != bitpack_extract(word, SNAP_NOENC_SHIFT, FLAG_WIDTH));
        p_snap->yaw_rate_q = bitpack_decode_signed(
            bitpack_extract(word, SNAP_YAW_SHIFT, SNAP_YAW_WIDTH),
            SNAP_YAW_WIDTH);
    }
}

/**
 * @brief   Build the aux word.
 * @param   p_aux  Fields; each is clamped to its width.
 * @return  The packed word (0 if @p p_aux is NULL).
 */
uint32_t
imu_pack_aux (imu_pack_aux_t const * p_aux)
{
    uint32_t word = 0u;

    if (NULL != p_aux)
    {
        word = bitpack_insert(word, bitpack_encode_signed(p_aux->roll_q,
                                                          AUX_ROLL_WIDTH),
                              AUX_ROLL_SHIFT, AUX_ROLL_WIDTH);
        word = bitpack_insert(word,
                              bitpack_clamp_u32(p_aux->heading_deg,
                                                IMU_PACK_HEADING_MAX_DEG),
                              AUX_HEADING_SHIFT, AUX_HEADING_WIDTH);
        word = bitpack_insert(word,
                              bitpack_clamp_u32(p_aux->hump_count,
                                                IMU_PACK_HUMP_COUNT_MAX),
                              AUX_COUNT_SHIFT, AUX_COUNT_WIDTH);
    }

    return (word);
}

/**
 * @brief   Decode an aux word.
 * @param   word   Packed word.
 * @param   p_aux  Receives the fields.
 */
void
imu_pack_unpack_aux (uint32_t word, imu_pack_aux_t * p_aux)
{
    if (NULL != p_aux)
    {
        p_aux->roll_q = bitpack_decode_signed(
            bitpack_extract(word, AUX_ROLL_SHIFT, AUX_ROLL_WIDTH),
            AUX_ROLL_WIDTH);
        p_aux->heading_deg = bitpack_extract(word, AUX_HEADING_SHIFT,
                                             AUX_HEADING_WIDTH);
        p_aux->hump_count  = bitpack_extract(word, AUX_COUNT_SHIFT,
                                             AUX_COUNT_WIDTH);
    }
}

/**
 * @brief   Build a 16-bit event.
 * @param   type     Event type.
 * @param   payload  Payload, clamped to 13 bits.
 * @return  The event.
 */
uint16_t
imu_pack_event (imu_pack_event_t type, uint32_t payload)
{
    uint32_t word = 0u;

    /* Cast: the enum constants are 0..7, non-negative. */
    word = bitpack_insert(word, (uint32_t)type, EVT_TYPE_SHIFT,
                          EVT_TYPE_WIDTH);
    word = bitpack_insert(word,
                          bitpack_clamp_u32(payload,
                                            IMU_PACK_EVENT_PAYLOAD_MAX),
                          EVT_PAYLOAD_SHIFT, EVT_PAYLOAD_WIDTH);

    /* Cast: the fields occupy bits 0-15. */
    return ((uint16_t)word);
}

/**
 * @brief   Decode a 16-bit event.
 * @param   event      Packed event.
 * @param   p_type     Receives the type.
 * @param   p_payload  Receives the 13-bit payload.
 */
void
imu_pack_unpack_event (uint16_t event, imu_pack_event_t * p_type,
                       uint32_t * p_payload)
{
    if ((NULL != p_type) && (NULL != p_payload))
    {
        uint32_t const type = bitpack_extract(event, EVT_TYPE_SHIFT,
                                              EVT_TYPE_WIDTH);
        imu_pack_event_t const types[] =
        {
            IMU_PACK_EVT_HUMP, IMU_PACK_EVT_MOTION, IMU_PACK_EVT_TERRAIN,
            IMU_PACK_EVT_IMPACT, IMU_PACK_EVT_RECOVERED,
            IMU_PACK_EVT_IMU_FAULT, IMU_PACK_EVT_IMU_OK,
            IMU_PACK_EVT_COURSE_DONE
        };

        /* type is 3 bits, so it always indexes the 8-entry table. */
        *p_type    = types[type];
        *p_payload = bitpack_extract(event, EVT_PAYLOAD_SHIFT,
                                     EVT_PAYLOAD_WIDTH);
    }
}

/**
 * @brief   Payload of a HUMP event.
 * @param   peak_mm        Packed peak, 0..127.
 * @param   b_saturated    Peak exceeded 127 mm.
 * @param   max_pitch_deg  Steepest climb, 0..31 deg.
 * @return  13-bit payload.
 */
uint32_t
imu_pack_hump_payload (uint32_t peak_mm, bool b_saturated,
                       uint32_t max_pitch_deg)
{
    uint32_t word = 0u;

    word = bitpack_insert(word, bitpack_clamp_u32(peak_mm,
                                                  IMU_PACK_PEAK_MAX_MM),
                          PL_PEAK_SHIFT, PL_PEAK_WIDTH);
    word = bitpack_insert(word, flag_bit(b_saturated), PL_SAT_SHIFT,
                          FLAG_WIDTH);
    word = bitpack_insert(word, bitpack_clamp_u32(max_pitch_deg,
                                                  PL_PITCH_MAX),
                          PL_PITCH_SHIFT, PL_PITCH_WIDTH);

    return (word);
}

/**
 * @brief   Payload of an IMPACT event.
 * @param   peak_g  Peak | |a| - 1 g | deviation in g.
 * @return  0..255 in 0.1 g steps.
 */
uint32_t
imu_pack_impact_payload (float32_t peak_g)
{
    /* Cast: clamped to 0..255, non-negative. */
    return ((uint32_t)bitpack_clamp_s32(imu_math_round(peak_g *
                                            IMU_PACK_IMPACT_Q_PER_G),
                                        0, PL_IMPACT_MAX));
}

/**
 * @brief   Payload of a COURSE_DONE event.
 * @param   peak_mm      Packed run peak, 0..127.
 * @param   b_saturated  Run peak exceeded 127 mm.
 * @param   hump_count   Humps in the run (saturates at 7).
 * @return  13-bit payload.
 */
uint32_t
imu_pack_course_payload (uint32_t peak_mm, bool b_saturated,
                         uint32_t hump_count)
{
    uint32_t word = 0u;

    word = bitpack_insert(word, bitpack_clamp_u32(peak_mm,
                                                  IMU_PACK_PEAK_MAX_MM),
                          PL_PEAK_SHIFT, PL_PEAK_WIDTH);
    word = bitpack_insert(word, flag_bit(b_saturated), PL_SAT_SHIFT,
                          FLAG_WIDTH);
    word = bitpack_insert(word, bitpack_clamp_u32(hump_count,
                                                  IMU_PACK_HUMP_COUNT_MAX),
                          PL_COUNT_SHIFT, PL_COUNT_WIDTH);

    return (word);
}

/**
 * @brief   Boolean to a 1-bit field value.
 * @param   b_flag  Flag.
 * @return  1 or 0.
 */
static uint32_t
flag_bit (bool b_flag)
{
    return (b_flag ? 1u : 0u);
}

/*** end of file ***/
