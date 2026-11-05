/**
 * @file    imu_terrain.c
 * @brief   Terrain region, hump peak-height estimate and tilt detection.
 *
 * See imu_terrain.h for the states and the wheelbase height model.
 */

#include <stddef.h>

#include "imu_terrain.h"
#include "../common/bitpack.h"

/* Layout of imu_terrain_t.state_bits. */
#define STATE_SHIFT         (0u)
#define STATE_WIDTH         (2u)
#define COUNT_SHIFT         (2u)
#define COUNT_WIDTH         (3u)
#define TILT_COUNT_SHIFT    (5u)
#define TILT_COUNT_WIDTH    (3u)
#define TILTED_SHIFT        (8u)
#define TILTED_WIDTH        (1u)
#define LEVEL_COUNT_SHIFT   (9u)
#define LEVEL_COUNT_WIDTH   (7u)
#define COUNT_MAX           (7u)
#define LEVEL_COUNT_MAX     (127u)

#define MAX_PITCH_FIELD     (31)    /* 5-bit field in the hump event */
#define HUMP_COUNT_MAX      (255u)

static void     reset_model (imu_terrain_t * p_terrain);
static void     reset_side (imu_terrain_t * p_terrain);
static float32_t median_push (float32_t * p_window, uint32_t * p_head,
                              uint32_t * p_count, float32_t value);
static void     track_height (imu_terrain_t * p_terrain, float32_t pitch_deg,
                              float32_t distance_mm);
static bool     level_step (imu_terrain_t * p_terrain, float32_t pitch_deg,
                            float32_t distance_mm, bool b_moving);
static bool     climbing_step (imu_terrain_t * p_terrain,
                               float32_t pitch_deg, float32_t distance_mm);
static bool     descending_step (imu_terrain_t * p_terrain,
                                 float32_t pitch_deg, float32_t distance_mm);
static bool     side_step (imu_terrain_t * p_terrain, float32_t roll_deg,
                           float32_t distance_mm, bool b_moving);
static void     side_watch (imu_terrain_t * p_terrain, float32_t roll_deg,
                            bool b_moving);
static bool     side_measure (imu_terrain_t * p_terrain, float32_t roll_deg,
                              float32_t distance_mm);
static void     close_hump (imu_terrain_t * p_terrain, bool b_cut_short);
static void     close_side_hump (imu_terrain_t * p_terrain, bool b_cut_short);
static void     finish_hump (imu_terrain_t * p_terrain, float32_t peak_mm,
                             float32_t steepest_deg, imu_terrain_side_t side,
                             bool b_cut_short);
static void     update_tilt (imu_terrain_t * p_terrain, float32_t pitch_deg,
                             float32_t roll_deg);
static uint32_t get_field (imu_terrain_t const * p_terrain, uint32_t shift,
                           uint32_t width);
static void     set_field (imu_terrain_t * p_terrain, uint32_t value,
                           uint32_t shift, uint32_t width);
static void     set_state (imu_terrain_t * p_terrain,
                           imu_terrain_state_t state);

/**
 * @brief   Full reset, including the run statistics.
 * @param   p_terrain  Terrain state.
 */
void
imu_terrain_init (imu_terrain_t * p_terrain)
{
    if (NULL != p_terrain)
    {
        uint32_t bins = 0u;

        if (IMU_TERRAIN_WHEELBASE_MM > 0.0f)
        {
            /* Cast: the rounded value is clamped to 1..HISTORY_BINS. */
            bins = (uint32_t)bitpack_clamp_s32(
                       imu_math_round(IMU_TERRAIN_WHEELBASE_MM /
                                      IMU_TERRAIN_BIN_MM),
                       1, (int32_t)IMU_TERRAIN_HISTORY_BINS);
        }

        p_terrain->history_bins   = bins;
        p_terrain->b_rear_climbs  = (0 != IMU_TERRAIN_REAR_CLIMBS);
        p_terrain->run_peak_mm    = 0.0f;
        p_terrain->hump_count     = 0u;
        p_terrain->state_bits     = 0u;
        p_terrain->b_result_ready = false;
        p_terrain->b_side_armed   = false;
        reset_model(p_terrain);
        reset_side(p_terrain);
    }
}

/**
 * @brief   Return to LEVEL (re-entering Monitoring after an impact).
 *          Run statistics and any untaken result are kept.
 * @param   p_terrain  Terrain state.
 */
void
imu_terrain_reset_region (imu_terrain_t * p_terrain)
{
    if (NULL != p_terrain)
    {
        set_state(p_terrain, IMU_TERRAIN_LEVEL);
        set_field(p_terrain, 0u, COUNT_SHIFT, COUNT_WIDTH);
        set_field(p_terrain, 0u, LEVEL_COUNT_SHIFT, LEVEL_COUNT_WIDTH);
        reset_model(p_terrain);
        reset_side(p_terrain);
    }
}

/**
 * @brief   Run the terrain region and tilt detection for one sample.
 * @param   p_terrain    Terrain state.
 * @param   pitch_deg    Filtered pitch, nose up positive.
 * @param   roll_deg     Filtered roll.
 * @param   distance_mm  Distance travelled since the previous sample.
 * @param   b_moving     Encoders (or vibration) show the car is moving.
 * @return  true if the terrain state changed on this sample.
 */
bool
imu_terrain_update (imu_terrain_t * p_terrain, float32_t pitch_deg,
                    float32_t roll_deg, float32_t distance_mm, bool b_moving)
{
    bool b_changed = false;

    if (NULL != p_terrain)
    {
        float32_t const distance = (distance_mm > 0.0f) ? distance_mm : 0.0f;

        update_tilt(p_terrain, pitch_deg, roll_deg);

        switch (imu_terrain_get_state(p_terrain))
        {
            case IMU_TERRAIN_LEVEL:
                b_changed = level_step(p_terrain, pitch_deg, distance,
                                       b_moving);

                if (b_changed)
                {
                    /* A climb opened: it measures this ramp from here on. */
                    p_terrain->b_side_armed = false;
                    reset_side(p_terrain);
                }
                else
                {
                    b_changed = side_step(p_terrain, roll_deg, distance,
                                          b_moving);
                }
            break;

            case IMU_TERRAIN_CLIMBING:
                b_changed = climbing_step(p_terrain, pitch_deg, distance);
            break;

            case IMU_TERRAIN_DESCENDING:
                b_changed = descending_step(p_terrain, pitch_deg, distance);
            break;

            default:
                /* HUMP_CLOSED lasts one sample: "Return to Level". */
                imu_terrain_reset_region(p_terrain);
                b_changed = true;
            break;
        }
    }

    return (b_changed);
}

/**
 * @brief   Close an open hump early (impact while on a hump), keeping the
 *          peak measured so far, and return the region to LEVEL.
 * @param   p_terrain  Terrain state.
 * @return  true if a hump was open and a result is now ready.
 */
bool
imu_terrain_force_close (imu_terrain_t * p_terrain)
{
    bool b_closed = false;

    if (NULL != p_terrain)
    {
        imu_terrain_state_t const state = imu_terrain_get_state(p_terrain);

        if ((IMU_TERRAIN_CLIMBING == state) ||
            (IMU_TERRAIN_DESCENDING == state))
        {
            close_hump(p_terrain, true);
            b_closed = true;
        }
        else if (p_terrain->b_side_open)
        {
            close_side_hump(p_terrain, true);
            b_closed = true;
        }
        else
        {
            /* Nothing open. */
        }

        imu_terrain_reset_region(p_terrain);
    }

    return (b_closed);
}

/**
 * @brief   Collect the result of the last closed hump, once.
 * @param   p_terrain  Terrain state.
 * @param   p_hump     Receives the result.
 * @return  true if a result was waiting.
 */
bool
imu_terrain_take_result (imu_terrain_t * p_terrain,
                         imu_terrain_hump_t * p_hump)
{
    bool b_taken = false;

    if ((NULL != p_terrain) && (NULL != p_hump) && p_terrain->b_result_ready)
    {
        *p_hump = p_terrain->result;
        p_terrain->b_result_ready = false;
        b_taken = true;
    }

    return (b_taken);
}

/**
 * @brief   Current terrain state.
 * @param   p_terrain  Terrain state.
 * @return  The state.
 */
imu_terrain_state_t
imu_terrain_get_state (imu_terrain_t const * p_terrain)
{
    imu_terrain_state_t state = IMU_TERRAIN_LEVEL;

    if (NULL != p_terrain)
    {
        /* Casts: the enum constants are 0..3, non-negative. */
        switch (get_field(p_terrain, STATE_SHIFT, STATE_WIDTH))
        {
            case (uint32_t)IMU_TERRAIN_CLIMBING:
                state = IMU_TERRAIN_CLIMBING;
            break;

            case (uint32_t)IMU_TERRAIN_DESCENDING:
                state = IMU_TERRAIN_DESCENDING;
            break;

            case (uint32_t)IMU_TERRAIN_HUMP_CLOSED:
                state = IMU_TERRAIN_HUMP_CLOSED;
            break;

            default:
                state = IMU_TERRAIN_LEVEL;
            break;
        }
    }

    return (state);
}

/**
 * @brief   Whether the car is tilted beyond the pitch or roll limit.
 * @param   p_terrain  Terrain state.
 * @return  true while tilted (with persistence applied).
 */
bool
imu_terrain_is_tilted (imu_terrain_t const * p_terrain)
{
    return ((NULL != p_terrain) &&
            (0u != get_field(p_terrain, TILTED_SHIFT, TILTED_WIDTH)));
}

/**
 * @brief   Whether a side hump is being measured.
 * @param   p_terrain  Terrain state.
 * @return  true from the moment it opens until it closes.
 */
bool
imu_terrain_side_is_open (imu_terrain_t const * p_terrain)
{
    return ((NULL != p_terrain) && p_terrain->b_side_open);
}

/**
 * @brief   Highest hump peak of the run (FR6 "highest hump peak").
 * @param   p_terrain  Terrain state.
 * @return  Peak in mm, after the calibration gain; 0 if no hump yet.
 */
float32_t
imu_terrain_get_run_peak_mm (imu_terrain_t const * p_terrain)
{
    return ((NULL != p_terrain) ? p_terrain->run_peak_mm : 0.0f);
}

/**
 * @brief   Number of humps closed during the run (saturates at 255).
 * @param   p_terrain  Terrain state.
 * @return  Hump count.
 */
uint32_t
imu_terrain_get_hump_count (imu_terrain_t const * p_terrain)
{
    return ((NULL != p_terrain) ? p_terrain->hump_count : 0u);
}

/**
 * @brief   Clear the height model before a new hump measurement.
 * @param   p_terrain  Terrain state.
 */
static void
reset_model (imu_terrain_t * p_terrain)
{
    for (uint32_t bin = 0u; bin < IMU_TERRAIN_HISTORY_BINS; bin++)
    {
        p_terrain->history_mm[bin] = 0.0f;
    }

    p_terrain->history_head  = 0u;
    p_terrain->front_mm      = 0.0f;
    p_terrain->peak_mm       = 0.0f;
    p_terrain->max_pitch_deg = 0.0f;
    p_terrain->travel_mm     = 0.0f;
    p_terrain->bin_fill_mm   = 0.0f;
    p_terrain->level_travel_mm = 0.0f;
    p_terrain->window_head  = 0u;
    p_terrain->window_count = 0u;
}

/**
 * @brief   Clear the side hump measurement.  b_side_armed is kept: it says
 *          whether roll has been level, which this does not change.
 * @param   p_terrain  Terrain state.
 */
static void
reset_side (imu_terrain_t * p_terrain)
{
    p_terrain->side_peak_deg  = 0.0f;
    p_terrain->side_travel_mm = 0.0f;
    p_terrain->roll_head      = 0u;
    p_terrain->roll_count     = 0u;
    p_terrain->side_count     = 0u;
    p_terrain->side           = IMU_TERRAIN_SIDE_NONE;
    p_terrain->b_side_open    = false;
}

/**
 * @brief   Median of the last IMU_TERRAIN_PEAK_WINDOW samples.  A jolt
 *          shorter than half the window is dropped entirely, where a low-pass
 *          would only spread it out, and the top of a real ramp is hardly
 *          clipped.
 * @param   p_window  Ring of IMU_TERRAIN_PEAK_WINDOW samples.
 * @param   p_head    Next slot in the ring.
 * @param   p_count   Samples held so far.
 * @param   value     New sample.
 * @return  Median of the samples held so far.
 */
static float32_t
median_push (float32_t * p_window, uint32_t * p_head, uint32_t * p_count,
             float32_t value)
{
    float32_t sorted[IMU_TERRAIN_PEAK_WINDOW];

    p_window[*p_head] = value;
    *p_head = (*p_head + 1u) % IMU_TERRAIN_PEAK_WINDOW;

    if (*p_count < IMU_TERRAIN_PEAK_WINDOW)
    {
        (*p_count)++;
    }

    /* Insertion sort; the window is small and only runs during a hump. */
    for (uint32_t i = 0u; i < *p_count; i++)
    {
        float32_t const sample = p_window[i];
        uint32_t        slot   = i;

        while ((slot > 0u) && (sorted[slot - 1u] > sample))
        {
            sorted[slot] = sorted[slot - 1u];
            slot--;
        }

        sorted[slot] = sample;
    }

    return (sorted[*p_count / 2u]);
}

/**
 * @brief   Advance the height model by one sample.
 * @param   p_terrain    Terrain state.
 * @param   pitch_deg    Filtered pitch.
 * @param   distance_mm  Distance travelled (>= 0).
 */
static void
track_height (imu_terrain_t * p_terrain, float32_t pitch_deg,
              float32_t distance_mm)
{
    float32_t const sin_pitch = imu_math_sin_deg(pitch_deg);

    p_terrain->travel_mm += distance_mm;

    if (!p_terrain->b_rear_climbs)
    {
        /* Rear wheels beside the ramp: the rear stays at ground level. */
        p_terrain->front_mm = IMU_TERRAIN_WHEELBASE_MM *
            imu_math_sin_deg(median_push(p_terrain->pitch_window,
                                         &p_terrain->window_head,
                                         &p_terrain->window_count,
                                         pitch_deg));
    }
    else if (0u == p_terrain->history_bins)
    {
        /* Simple method: average height over one wheelbase. */
        p_terrain->front_mm += sin_pitch * distance_mm;
    }
    else
    {
        uint32_t pushes = 0u;

        /* Record h_front at every bin boundary crossed.  The slot after
         * the newest one then holds h_front from one wheelbase ago.
         */
        p_terrain->bin_fill_mm += distance_mm;

        while ((p_terrain->bin_fill_mm >= IMU_TERRAIN_BIN_MM) &&
               (pushes < IMU_TERRAIN_HISTORY_BINS))
        {
            p_terrain->history_mm[p_terrain->history_head] =
                p_terrain->front_mm;
            p_terrain->history_head++;

            if (p_terrain->history_head >= p_terrain->history_bins)
            {
                p_terrain->history_head = 0u;
            }

            p_terrain->bin_fill_mm -= IMU_TERRAIN_BIN_MM;
            pushes++;
        }

        if (pushes >= IMU_TERRAIN_HISTORY_BINS)
        {
            /* Implausibly long step: the whole window is up to date. */
            p_terrain->bin_fill_mm = 0.0f;
        }

        p_terrain->front_mm =
            p_terrain->history_mm[p_terrain->history_head] +
            (IMU_TERRAIN_WHEELBASE_MM * sin_pitch);
    }

    if (p_terrain->front_mm > p_terrain->peak_mm)
    {
        p_terrain->peak_mm = p_terrain->front_mm;
    }

    if (pitch_deg > p_terrain->max_pitch_deg)
    {
        p_terrain->max_pitch_deg = pitch_deg;
    }
}

/**
 * @brief   LEVEL: watch for a sustained climb while moving.
 *
 * The height model is reset whenever pitch is inside the level band and
 * runs as soon as it leaves it, so the rise before the climb threshold is
 * already measured when the hump opens.  A climb is only counted while
 * the car moves: pitch that changes on a stopped car (filter re-levelling,
 * someone lifting the car) is not a hump.
 *
 * @param   p_terrain    Terrain state.
 * @param   pitch_deg    Filtered pitch.
 * @param   distance_mm  Distance travelled.
 * @param   b_moving     Encoders (or vibration) show movement.
 * @return  true on LEVEL -> CLIMBING.
 */
static bool
level_step (imu_terrain_t * p_terrain, float32_t pitch_deg,
            float32_t distance_mm, bool b_moving)
{
    bool     b_changed = false;
    uint32_t count     = get_field(p_terrain, COUNT_SHIFT, COUNT_WIDTH);

    if (imu_math_abs(pitch_deg) < IMU_TERRAIN_LEVEL_DEG)
    {
        reset_model(p_terrain);
    }
    else
    {
        track_height(p_terrain, pitch_deg, distance_mm);
    }

    if (b_moving && (pitch_deg > IMU_TERRAIN_CLIMB_DEG))
    {
        count = bitpack_clamp_u32(count + 1u, COUNT_MAX);

        if (count >= IMU_TERRAIN_PERSIST_SAMPLES)
        {
            set_state(p_terrain, IMU_TERRAIN_CLIMBING);
            count     = 0u;
            b_changed = true;
        }
    }
    else
    {
        count = 0u;
    }

    set_field(p_terrain, count, COUNT_SHIFT, COUNT_WIDTH);

    return (b_changed);
}

/**
 * @brief   CLIMBING: measure, and watch for a sustained descent, or for
 *          the car being level again without ever tipping nose down.
 * @param   p_terrain    Terrain state.
 * @param   pitch_deg    Filtered pitch.
 * @param   distance_mm  Distance travelled.
 * @return  true on a state change.
 */
static bool
climbing_step (imu_terrain_t * p_terrain, float32_t pitch_deg,
               float32_t distance_mm)
{
    bool     b_changed   = false;
    bool     b_level_end = false;
    uint32_t count       = get_field(p_terrain, COUNT_SHIFT, COUNT_WIDTH);
    uint32_t level_count = get_field(p_terrain, LEVEL_COUNT_SHIFT,
                                     LEVEL_COUNT_WIDTH);

    track_height(p_terrain, pitch_deg, distance_mm);

    if (pitch_deg < -IMU_TERRAIN_CLIMB_DEG)
    {
        count = bitpack_clamp_u32(count + 1u, COUNT_MAX);
    }
    else
    {
        count = 0u;
    }

    /* Level again: count samples and the distance covered while level. */
    if (imu_math_abs(pitch_deg) < IMU_TERRAIN_LEVEL_DEG)
    {
        level_count = bitpack_clamp_u32(level_count + 1u, LEVEL_COUNT_MAX);
        p_terrain->level_travel_mm += distance_mm;
    }
    else
    {
        level_count = 0u;
        p_terrain->level_travel_mm = 0.0f;
    }

    if (level_count >= IMU_TERRAIN_PERSIST_SAMPLES)
    {
        /* Level for a whole wheelbase: both axles are at the same height,
         * so the hump is behind the car (or the car is on a long flat
         * top).  Without distance (no encoders, or stopped) fall back to
         * a time limit.
         */
        b_level_end = (level_count >= IMU_TERRAIN_TOP_LEVEL_SAMPLES) ||
                      ((IMU_TERRAIN_WHEELBASE_MM > 0.0f) &&
                       (p_terrain->level_travel_mm >=
                        IMU_TERRAIN_WHEELBASE_MM));
    }

    if (count >= IMU_TERRAIN_PERSIST_SAMPLES)
    {
        set_state(p_terrain, IMU_TERRAIN_DESCENDING);
        b_changed = true;
    }
    else if (b_level_end)
    {
        close_hump(p_terrain, false);
        b_changed = true;
    }
    else if (p_terrain->travel_mm > IMU_TERRAIN_MAX_HUMP_MM)
    {
        close_hump(p_terrain, true);
        b_changed = true;
    }
    else
    {
        /* Still climbing (or on a short flat hump top). */
    }

    if (b_changed)
    {
        count       = 0u;
        level_count = 0u;
        p_terrain->level_travel_mm = 0.0f;
    }

    set_field(p_terrain, count, COUNT_SHIFT, COUNT_WIDTH);
    set_field(p_terrain, level_count, LEVEL_COUNT_SHIFT, LEVEL_COUNT_WIDTH);

    return (b_changed);
}

/**
 * @brief   DESCENDING: measure, and close the hump once level again.
 * @param   p_terrain    Terrain state.
 * @param   pitch_deg    Filtered pitch.
 * @param   distance_mm  Distance travelled.
 * @return  true on a state change.
 */
static bool
descending_step (imu_terrain_t * p_terrain, float32_t pitch_deg,
                 float32_t distance_mm)
{
    bool     b_changed = false;
    uint32_t count     = get_field(p_terrain, COUNT_SHIFT, COUNT_WIDTH);

    track_height(p_terrain, pitch_deg, distance_mm);

    if (imu_math_abs(pitch_deg) < IMU_TERRAIN_LEVEL_DEG)
    {
        count = bitpack_clamp_u32(count + 1u, COUNT_MAX);
    }
    else
    {
        count = 0u;
    }

    if (count >= IMU_TERRAIN_PERSIST_SAMPLES)
    {
        close_hump(p_terrain, false);
        count     = 0u;
        b_changed = true;
    }
    else if (p_terrain->travel_mm > IMU_TERRAIN_MAX_HUMP_MM)
    {
        close_hump(p_terrain, true);
        count     = 0u;
        b_changed = true;
    }
    else
    {
        /* Still on the way down. */
    }

    set_field(p_terrain, count, COUNT_SHIFT, COUNT_WIDTH);

    return (b_changed);
}

/**
 * @brief   One LEVEL sample of the side hump measurement (see imu_terrain.h).
 * @param   p_terrain    Terrain state.
 * @param   roll_deg     Filtered roll.
 * @param   distance_mm  Distance travelled.
 * @param   b_moving     Encoders (or vibration) show movement.
 * @return  true on closing a side hump.
 */
static bool
side_step (imu_terrain_t * p_terrain, float32_t roll_deg,
           float32_t distance_mm, bool b_moving)
{
    bool b_changed = false;

    if (imu_terrain_is_tilted(p_terrain))
    {
        /* Lifted or falling: this roll says nothing about a hump. */
        p_terrain->b_side_armed = false;
        reset_side(p_terrain);
    }
    else if (p_terrain->b_side_open)
    {
        b_changed = side_measure(p_terrain, roll_deg, distance_mm);
    }
    else
    {
        side_watch(p_terrain, roll_deg, b_moving);
    }

    return (b_changed);
}

/**
 * @brief   No side hump open: watch for sustained roll to one side.  Samples
 *          outside the level band already feed the median, so the rise
 *          before the threshold is not lost.
 * @param   p_terrain  Terrain state.
 * @param   roll_deg   Filtered roll.
 * @param   b_moving   Encoders (or vibration) show movement.
 */
static void
side_watch (imu_terrain_t * p_terrain, float32_t roll_deg, bool b_moving)
{
    float32_t const size = imu_math_abs(roll_deg);

    if (size < IMU_TERRAIN_LEVEL_DEG)
    {
        p_terrain->b_side_armed = true;
        reset_side(p_terrain);
    }
    else
    {
        imu_terrain_side_t const side = (roll_deg < 0.0f) ?
            IMU_TERRAIN_SIDE_RIGHT : IMU_TERRAIN_SIDE_LEFT;
        (void)median_push(p_terrain->roll_window, &p_terrain->roll_head,
                          &p_terrain->roll_count, size);

        if (p_terrain->b_side_armed && b_moving &&
            (size > IMU_TERRAIN_SIDE_ON_DEG) &&
            ((0u == p_terrain->side_count) || (side == p_terrain->side)))
        {
            p_terrain->side = side;
            p_terrain->side_count++;

            if (p_terrain->side_count >= IMU_TERRAIN_PERSIST_SAMPLES)
            {
                p_terrain->b_side_open   = true;
                p_terrain->side_count    = 0u;
                p_terrain->side_peak_deg = IMU_TERRAIN_SIDE_ON_DEG;
            }
        }
        else
        {
            p_terrain->side_count = 0u;
        }
    }
}

/**
 * @brief   Side hump open: track the peak and watch for level roll.
 * @param   p_terrain    Terrain state.
 * @param   roll_deg     Filtered roll.
 * @param   distance_mm  Distance travelled.
 * @return  true on closing the side hump.
 */
static bool
side_measure (imu_terrain_t * p_terrain, float32_t roll_deg,
              float32_t distance_mm)
{
    bool            b_closed = false;
    float32_t const size     = imu_math_abs(roll_deg);
    float32_t const median   = median_push(p_terrain->roll_window,
                                           &p_terrain->roll_head,
                                           &p_terrain->roll_count, size);

    /* A part-full window can be mostly jolt, so wait until it is full. */
    if ((IMU_TERRAIN_PEAK_WINDOW == p_terrain->roll_count) &&
        (median > p_terrain->side_peak_deg))
    {
        p_terrain->side_peak_deg = median;
    }

    p_terrain->side_travel_mm += distance_mm;
    p_terrain->side_count = (size < IMU_TERRAIN_LEVEL_DEG) ?
                            (p_terrain->side_count + 1u) : 0u;

    if (p_terrain->side_count >= IMU_TERRAIN_SIDE_OFF_SAMPLES)
    {
        close_side_hump(p_terrain, false);
        b_closed = true;
    }
    else if (p_terrain->side_travel_mm >= IMU_TERRAIN_MAX_HUMP_MM)
    {
        close_side_hump(p_terrain, true);
        b_closed = true;
    }
    else
    {
        /* Still on the hump. */
    }

    return (b_closed);
}

/**
 * @brief   Finalise a pitch hump.
 * @param   p_terrain    Terrain state.
 * @param   b_cut_short  true if closed by the distance limit or an impact.
 */
static void
close_hump (imu_terrain_t * p_terrain, bool b_cut_short)
{
    finish_hump(p_terrain,
                p_terrain->peak_mm *
                (p_terrain->b_rear_climbs ? IMU_TERRAIN_HEIGHT_GAIN :
                                            IMU_TERRAIN_FLAT_GAIN),
                p_terrain->max_pitch_deg, IMU_TERRAIN_SIDE_NONE, b_cut_short);
}

/**
 * @brief   Finalise a side hump.  Roll must level out before the next one.
 * @param   p_terrain    Terrain state.
 * @param   b_cut_short  true if closed by the distance limit or an impact.
 */
static void
close_side_hump (imu_terrain_t * p_terrain, bool b_cut_short)
{
    float32_t const peak_deg = p_terrain->side_peak_deg;

    finish_hump(p_terrain,
                IMU_TERRAIN_TRACK_MM * imu_math_sin_deg(peak_deg) *
                IMU_TERRAIN_SIDE_GAIN,
                peak_deg, p_terrain->side, b_cut_short);
    p_terrain->b_side_armed = false;
    reset_side(p_terrain);
}

/**
 * @brief   Publish a hump result, update run statistics, enter HUMP_CLOSED.
 * @param   p_terrain     Terrain state.
 * @param   peak          Peak height in mm, after gain.
 * @param   steepest_deg  Steepest pitch, or peak roll for a side hump.
 * @param   side          Wheel on the hump, or IMU_TERRAIN_SIDE_NONE.
 * @param   b_cut_short   true if closed by the distance limit or an impact.
 */
static void
finish_hump (imu_terrain_t * p_terrain, float32_t peak,
             float32_t steepest_deg, imu_terrain_side_t side,
             bool b_cut_short)
{
    imu_terrain_hump_t * const p_result = &p_terrain->result;
    int32_t const rounded = imu_math_round(peak);

    /* Clamp then pack (bitpack.h rule 1): 0..127 plus a saturation flag. */
    p_result->peak_mm        = (peak > 0.0f) ? peak : 0.0f;
    /* Cast: clamped to 0..127, fits in uint8_t. */
    p_result->peak_mm_packed = (uint8_t)bitpack_clamp_s32(
                                   rounded, 0,
                                   (int32_t)IMU_TERRAIN_PEAK_MAX_MM);
    p_result->b_saturated    = (rounded > (int32_t)IMU_TERRAIN_PEAK_MAX_MM);
    /* Cast: clamped to 0..31, fits in uint8_t. */
    p_result->max_pitch_deg  = (uint8_t)bitpack_clamp_s32(
                                   imu_math_round(steepest_deg),
                                   0, MAX_PITCH_FIELD);
    p_result->b_cut_short    = b_cut_short;
    p_result->side           = side;

    if (p_result->peak_mm > p_terrain->run_peak_mm)
    {
        p_terrain->run_peak_mm = p_result->peak_mm;
    }

    p_terrain->hump_count = bitpack_clamp_u32(p_terrain->hump_count + 1u,
                                              HUMP_COUNT_MAX);
    p_terrain->b_result_ready = true;
    set_state(p_terrain, IMU_TERRAIN_HUMP_CLOSED);
}

/**
 * @brief   Tilt detection with the same persistence rule as the regions.
 * @param   p_terrain  Terrain state.
 * @param   pitch_deg  Filtered pitch.
 * @param   roll_deg   Filtered roll.
 */
static void
update_tilt (imu_terrain_t * p_terrain, float32_t pitch_deg,
             float32_t roll_deg)
{
    bool const b_over = (imu_math_abs(pitch_deg) >
                         IMU_TERRAIN_TILT_PITCH_DEG) ||
                        (imu_math_abs(roll_deg) > IMU_TERRAIN_TILT_ROLL_DEG);
    bool const b_tilted = imu_terrain_is_tilted(p_terrain);
    uint32_t   count    = get_field(p_terrain, TILT_COUNT_SHIFT,
                                    TILT_COUNT_WIDTH);

    if (b_over != b_tilted)
    {
        count = bitpack_clamp_u32(count + 1u, COUNT_MAX);

        if (count >= IMU_TERRAIN_PERSIST_SAMPLES)
        {
            set_field(p_terrain, b_over ? 1u : 0u, TILTED_SHIFT,
                      TILTED_WIDTH);
            count = 0u;
        }
    }
    else
    {
        count = 0u;
    }

    set_field(p_terrain, count, TILT_COUNT_SHIFT, TILT_COUNT_WIDTH);
}

/**
 * @brief   Read one field of the packed state word.
 * @param   p_terrain  Terrain state.
 * @param   shift      Field position.
 * @param   width      Field width.
 * @return  Field value.
 */
static uint32_t
get_field (imu_terrain_t const * p_terrain, uint32_t shift, uint32_t width)
{
    return (bitpack_extract(p_terrain->state_bits, shift, width));
}

/**
 * @brief   Write one field of the packed state word (private to the IMU
 *          task, so the read-modify-write cannot race).
 * @param   p_terrain  Terrain state.
 * @param   value      Field value within the field width.
 * @param   shift      Field position.
 * @param   width      Field width.
 */
static void
set_field (imu_terrain_t * p_terrain, uint32_t value, uint32_t shift,
           uint32_t width)
{
    /* Cast: the packed fields occupy bits 0-15, so the word fits in 16 bits. */
    p_terrain->state_bits = (uint16_t)bitpack_insert(p_terrain->state_bits,
                                                     value, shift, width);
}

/**
 * @brief   Store the terrain state field.
 * @param   p_terrain  Terrain state.
 * @param   state      New state.
 */
static void
set_state (imu_terrain_t * p_terrain, imu_terrain_state_t state)
{
    /* Cast: the enum constants are 0..3, non-negative. */
    set_field(p_terrain, (uint32_t)state, STATE_SHIFT, STATE_WIDTH);
}

/*** end of file ***/
