/**
 * @file    imu_impact.c
 * @brief   Sudden-impact detection and timed recovery.
 */

#include <stddef.h>

#include "imu_impact.h"
#include "../common/bitpack.h"

#define ONE_G               (1.0f)

/* Layout of imu_impact_t.state_bits. */
#define ACTIVE_SHIFT        (0u)
#define ACTIVE_WIDTH        (1u)
#define SETTLE_SHIFT        (1u)
#define SETTLE_WIDTH        (6u)
#define SETTLE_MAX          (63u)

static void set_bits (imu_impact_t * p_impact, uint32_t b_active,
                      uint32_t settle_count);

/**
 * @brief   Reset to "no impact".
 * @param   p_impact  Detector state.
 */
void
imu_impact_init (imu_impact_t * p_impact)
{
    if (NULL != p_impact)
    {
        p_impact->peak_g     = 0.0f;
        p_impact->state_bits = 0u;
        p_impact->holdoff    = 0u;
    }
}

/**
 * @brief   Start the post-hump hold-off (called when a hump closes).
 * @param   p_impact  Detector state.
 */
void
imu_impact_start_holdoff (imu_impact_t * p_impact)
{
    if (NULL != p_impact)
    {
        p_impact->holdoff = IMU_IMPACT_HOLDOFF_SAMPLES;
    }
}

/**
 * @brief   Look for a shock in one sample.
 * @param   p_impact     Detector state.
 * @param   accel_mag_g  |a| in g.
 * @param   lin_fwd_g    Forward acceleration with gravity removed, in g.
 * @param   b_on_hump    true while the terrain region is DESCENDING.
 * @return  true when a NEW impact starts on this sample.
 */
bool
imu_impact_detect (imu_impact_t * p_impact, float32_t accel_mag_g,
                   float32_t lin_fwd_g, bool b_on_hump)
{
    bool b_new_impact = false;

    if ((NULL != p_impact) && !imu_impact_is_active(p_impact))
    {
        bool const      b_relaxed = b_on_hump || (p_impact->holdoff > 0u);
        float32_t const scale     = b_relaxed ? IMU_IMPACT_HUMP_SCALE : 1.0f;
        float32_t const deviation = imu_math_abs(accel_mag_g - ONE_G);

        if (p_impact->holdoff > 0u)
        {
            p_impact->holdoff--;
        }

        if ((deviation > (IMU_IMPACT_DEV_G * scale)) ||
            (imu_math_abs(lin_fwd_g) > (IMU_IMPACT_FWD_G * scale)))
        {
            p_impact->peak_g = deviation;
            set_bits(p_impact, 1u, 0u);
            b_new_impact = true;
        }
    }

    return (b_new_impact);
}

/**
 * @brief   While an impact is active, check whether the car has settled.
 * @param   p_impact     Detector state.
 * @param   accel_mag_g  |a| in g.
 * @return  true on the sample where "Recovered" fires.
 */
bool
imu_impact_check_recovered (imu_impact_t * p_impact, float32_t accel_mag_g)
{
    bool b_recovered = false;

    if ((NULL != p_impact) && imu_impact_is_active(p_impact))
    {
        float32_t const deviation = imu_math_abs(accel_mag_g - ONE_G);
        uint32_t        settle    = bitpack_extract(p_impact->state_bits,
                                                    SETTLE_SHIFT,
                                                    SETTLE_WIDTH);

        if (deviation > p_impact->peak_g)
        {
            p_impact->peak_g = deviation;
        }

        if (deviation < IMU_IMPACT_SETTLE_G)
        {
            settle = bitpack_clamp_u32(settle + 1u, SETTLE_MAX);
        }
        else
        {
            settle = 0u;
        }

        if (settle >= IMU_IMPACT_RECOVER_SAMPLES)
        {
            set_bits(p_impact, 0u, 0u);
            b_recovered = true;
        }
        else
        {
            set_bits(p_impact, 1u, settle);
        }
    }

    return (b_recovered);
}

/**
 * @brief   Whether an impact is in progress.
 * @param   p_impact  Detector state.
 * @return  true between detection and recovery.
 */
bool
imu_impact_is_active (imu_impact_t const * p_impact)
{
    return ((NULL != p_impact) &&
            (0u != bitpack_extract(p_impact->state_bits, ACTIVE_SHIFT,
                                   ACTIVE_WIDTH)));
}

/**
 * @brief   Largest |a| - 1 g deviation seen in the current/last impact.
 * @param   p_impact  Detector state.
 * @return  Deviation in g.
 */
float32_t
imu_impact_get_peak_g (imu_impact_t const * p_impact)
{
    return ((NULL != p_impact) ? p_impact->peak_g : 0.0f);
}

/**
 * @brief   Store the active flag and settle counter.
 * @param   p_impact      Detector state.
 * @param   b_active      1 while an impact is active.
 * @param   settle_count  Consecutive settled samples, 0..63.
 */
static void
set_bits (imu_impact_t * p_impact, uint32_t b_active, uint32_t settle_count)
{
    uint32_t word = 0u;

    word = bitpack_insert(word, b_active, ACTIVE_SHIFT, ACTIVE_WIDTH);
    word = bitpack_insert(word, settle_count, SETTLE_SHIFT, SETTLE_WIDTH);

    /* Cast: the fields occupy bits 0-6, so the word fits in 8 bits. */
    p_impact->state_bits = (uint8_t)word;
}

/*** end of file ***/
