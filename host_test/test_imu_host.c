/**
 * @file    test_imu_host.c
 * @brief   PC unit tests for the pure IMU modules (no kernel, no hardware).
 *
 * Compiles the SAME source files as the firmware (imu_math, imu_filter,
 * imu_odom, imu_motion, imu_terrain, imu_impact, imu_pack, imu_monitor,
 * imu_sim, common/bitpack) with a host C compiler and feeds them simulated
 * sensor data.  Run with run_host_tests.sh; results use the same
 * "[TEST] <ID> PASS|FAIL" format as the on-target tests.
 *
 * <math.h> is used here ONLY as a reference to check imu_math.c.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../app_program/common/bitpack.h"
#include "../app_program/subsystem4/imu_math.h"
#include "../app_program/subsystem4/imu_odom.h"
#include "../app_program/subsystem4/imu_terrain.h"
#include "../app_program/subsystem4/imu_impact.h"
#include "../app_program/subsystem4/imu_pack.h"
#include "../app_program/subsystem4/imu_monitor.h"
#include "../app_program/subsystem4/imu_sim.h"
#include "../app_program/subsystem4/imu.h"

#define DT_S                (0.01f)
#define CALIB_STEPS         (130u)
#define ENCODER_WRAP_START  (0x00000FF0u)
#define NFR4_TOLERANCE      (0.20f)

static int g_pass = 0;
static int g_fail = 0;
static int g_no_gyro = 0;   /* 1: simulate the LSM303D (GY-511) */

/** Counters gathered while a scenario runs. */
typedef struct
{
    uint32_t humps;
    uint32_t impacts;
    uint32_t recoveries;
    float    last_peak_mm;
    uint32_t last_packed_mm;
    uint32_t terrain_trace;     /* one nibble per terrain state change */
    uint32_t terrain_changes;
    uint32_t motion_seen;       /* bit per motion class seen */
} scenario_t;

static void check (char const * p_id, int b_ok, char const * p_desc)
{
    if (b_ok)
    {
        g_pass++;
        printf("[TEST] %s PASS  %s\n", p_id, p_desc);
    }
    else
    {
        g_fail++;
        printf("[TEST] %s FAIL  %s\n", p_id, p_desc);
    }
}

static void record (imu_monitor_output_t const * p_out, scenario_t * p_sc)
{
    if (p_out->b_hump_closed)
    {
        p_sc->humps++;
        p_sc->last_peak_mm   = p_out->hump.peak_mm;
        p_sc->last_packed_mm = p_out->hump.peak_mm_packed;
    }

    if (p_out->b_impact)
    {
        p_sc->impacts++;
    }

    if (p_out->b_recovered)
    {
        p_sc->recoveries++;
    }

    if (p_out->b_terrain_changed && (p_sc->terrain_changes < 8u))
    {
        p_sc->terrain_trace |= ((uint32_t)p_out->terrain_state & 0xFu) <<
                               (4u * p_sc->terrain_changes);
        p_sc->terrain_changes++;
    }

    p_sc->motion_seen |= 1u << (uint32_t)p_out->motion_class;
}

/* Run n steps of the simulator through the monitor. */
static void run (imu_monitor_t * p_mon, imu_sim_t * p_sim,
                 imu_sim_profile_t const * p_prof, uint32_t steps,
                 int b_encoder, scenario_t * p_sc)
{
    for (uint32_t step = 0u; step < steps; step++)
    {
        drv_imu_sample_t raw;
        imu_odom_counts_t    counts;
        imu_monitor_output_t out;

        (void)imu_sim_step(p_sim, p_prof, DT_S, &raw);
        counts.b_has_left  = b_encoder ? true : false;
        counts.b_has_right = b_encoder ? true : false;
        counts.left_count  = imu_sim_encoder(p_sim);
        counts.right_count = imu_sim_encoder(p_sim);
        imu_monitor_step(p_mon, &raw, &counts, DT_S, &out);
        record(&out, p_sc);
    }
}

static void start (imu_monitor_t * p_mon, imu_sim_t * p_sim,
                   imu_sim_profile_t * p_prof, uint32_t enc_start)
{
    scenario_t dummy;

    memset(&dummy, 0, sizeof(dummy));
    imu_monitor_init(p_mon);
    p_mon->terrain.b_rear_climbs = true;    /* imu_sim's default car */
    imu_sim_init(p_sim, enc_start, 12345u);
    imu_sim_flat_profile(p_prof);
    p_prof->b_no_gyro = g_no_gyro ? true : false;
    run(p_mon, p_sim, p_prof, CALIB_STEPS, 1, &dummy);
}

static void test_math (void)
{
    double max_sqrt = 0.0;
    double max_atan = 0.0;
    double max_sin  = 0.0;

    for (double x = 1e-6; x < 1e6; x *= 1.37)
    {
        double const err = fabs((double)imu_math_sqrt((float)x) - sqrt(x)) / sqrt(x);
        max_sqrt = (err > max_sqrt) ? err : max_sqrt;
    }

    for (int iy = -50; iy <= 50; iy++)
    {
        for (int ix = -50; ix <= 50; ix++)
        {
            double const y = iy * 0.37;
            double const x = ix * 0.41;
            double const ref = atan2(y, x) * 57.29577951308232;
            double err = fabs((double)imu_math_atan2_deg((float)y, (float)x) - ref);

            if ((0 == iy) && (ix < 0))
            {
                err = fabs(fabs((double)imu_math_atan2_deg((float)y, (float)x)) -
                           180.0);
            }
            max_atan = (err > max_atan) ? err : max_atan;
        }
    }

    for (double a = -720.0; a <= 720.0; a += 0.7)
    {
        double const err = fabs((double)imu_math_sin_deg((float)a) -
                                sin(a * 0.017453292519943295));
        max_sin = (err > max_sin) ? err : max_sin;
    }

    printf("       sqrt rel err %.2e, atan2 err %.2e deg, sin err %.2e\n",
           max_sqrt, max_atan, max_sin);
    check("H-U01", max_sqrt < 1e-6, "imu_math_sqrt relative error < 1e-6");
    check("H-U02", max_atan < 1e-3, "imu_math_atan2_deg error < 0.001 deg");
    check("H-U03", max_sin < 1e-5, "imu_math_sin_deg error < 1e-5");
    check("H-U04", (imu_math_round(2.5f) == 3) &&
                   (imu_math_round(-2.5f) == -3) &&
                   (imu_math_round(1e20f) == INT32_MAX) &&
                   (imu_math_round(NAN) == 0),
          "imu_math_round: half away from zero, saturates, NaN -> 0");
    check("H-U05", !imu_math_is_finite(INFINITY) && !imu_math_is_finite(NAN)
                   && imu_math_is_finite(1.0f),
          "imu_math_is_finite rejects inf and NaN (Barr 5.4.b.v)");
}

static void test_bitpack (void)
{
    int b_ok = 1;

    for (uint32_t width = 2u; width < 32u; width++)
    {
        int32_t const half = (int32_t)(1u << (width - 1u));
        int32_t const values[] = { -half, -1, 0, 1, half - 1 };

        for (uint32_t i = 0u; i < 5u; i++)
        {
            uint32_t const field = bitpack_encode_signed(values[i], width);
            if ((bitpack_decode_signed(field, width) != values[i]) ||
                (field > bitpack_mask(width)))
            {
                b_ok = 0;
            }
        }

        /* Out of range values saturate instead of wrapping. */
        if ((bitpack_decode_signed(bitpack_encode_signed(half, width), width)
             != (half - 1)) ||
            (bitpack_decode_signed(bitpack_encode_signed(-half - 1, width),
                                   width) != -half))
        {
            b_ok = 0;
        }
    }

    check("H-U06", b_ok, "signed fields round-trip and saturate, widths 2-31");

    uint32_t word = 0xFFFFFFFFu;
    word = bitpack_insert(word, 0u, 8u, 7u);
    check("H-U07", (word == 0xFFFF80FFu) &&
                   (bitpack_extract(0x12345678u, 4u, 8u) == 0x67u) &&
                   (bitpack_insert(0u, 0xFFu, 0u, 3u) == 0x7u) &&
                   (bitpack_clamp_u32(200u, 127u) == 127u),
          "insert/extract/clamp touch only their own field");
}

static void test_pack (void)
{
    imu_pack_snapshot_t in;
    imu_pack_snapshot_t out;
    int b_ok = 1;

    for (int pass = 0; pass < 3; pass++)
    {
        memset(&in, 0, sizeof(in));
        in.pitch_q          = (0 == pass) ? -128 : ((1 == pass) ? 127 : -3);
        in.peak_mm          = (0 == pass) ? 0u : ((1 == pass) ? 127u : 64u);
        in.b_peak_saturated = (1 == pass);
        in.motion_class     = (uint32_t)pass + 1u;
        in.terrain_state    = 3u - (uint32_t)pass;
        in.b_imu_fault      = (1 == pass);
        in.b_tilted         = (2 == pass);
        in.b_no_encoder     = (1 == pass);
        in.yaw_rate_q       = (0 == pass) ? 127 : ((1 == pass) ? -128 : 5);

        uint32_t const word = imu_pack_snapshot(&in);
        imu_pack_unpack_snapshot(word, &out);
        printf("       golden snapshot %d: 0x%08X\n", pass, (unsigned)word);

        if (0 != memcmp(&in, &out, sizeof(in)))
        {
            b_ok = 0;
        }
    }

    check("H-U08", b_ok, "snapshot word round-trips at min/max/mid");

    in.pitch_q = 0;
    check("H-U09", (imu_pack_quantize_angle(-40.0f) == -128) &&
                   (imu_pack_quantize_angle(31.75f) == 127) &&
                   (imu_pack_quantize_angle(-0.25f) == -1) &&
                   (imu_pack_quantize_yaw_rate(600.0f) == 127),
          "angle and yaw-rate quantisers clamp instead of wrapping");

    bool b_sat = false;
    uint32_t const p1 = imu_pack_quantize_peak(126.6f, &b_sat);
    int const b_ok1 = (127u == p1) && !b_sat;
    uint32_t const p2 = imu_pack_quantize_peak(300.0f, &b_sat);
    check("H-U10", b_ok1 && (127u == p2) && b_sat &&
                   (0u == imu_pack_quantize_peak(-5.0f, &b_sat)),
          "peak: 7-bit clamp with saturation flag (G9)");

    imu_pack_aux_t aux_in = { -77, 359u, 7u };
    imu_pack_aux_t aux_out;
    imu_pack_unpack_aux(imu_pack_aux(&aux_in), &aux_out);
    check("H-U11", (aux_out.roll_q == -77) && (aux_out.heading_deg == 359u) &&
                   (aux_out.hump_count == 7u) &&
                   (imu_pack_quantize_heading(-1.0f) == 359u) &&
                   (imu_pack_quantize_heading(359.7f) == 0u),
          "aux word round-trips; heading wraps to 0..359");

    int b_evt = 1;
    for (uint32_t type = 0u; type < 8u; type++)
    {
        imu_pack_event_t got_type;
        uint32_t         got_payload;
        uint16_t const   evt = imu_pack_event((imu_pack_event_t)type,
                                              0x1FFFu - type);
        imu_pack_unpack_event(evt, &got_type, &got_payload);
        if (((uint32_t)got_type != type) || (got_payload != (0x1FFFu - type)))
        {
            b_evt = 0;
        }
    }
    uint32_t const hp = imu_pack_hump_payload(200u, true, 45u);
    check("H-U12", b_evt && (hp == ((31u << 8) | 0x80u | 127u)) &&
                   (imu_pack_impact_payload(30.0f) == 255u),
          "16-bit events round-trip; payload fields clamp");
}

static void test_odom (void)
{
    imu_odom_t        odom;
    imu_odom_counts_t counts = { 0u, 0u, true, true };

    check("H-U13", (imu_odom_delta(4u, 0u) == 4u) &&
                   (imu_odom_delta(0x00000002u, 0xFFFFFFFEu) == 4u) &&
                   (imu_odom_delta(7u, 7u) == 0u),
          "32-bit wrap-safe encoder difference (wrap at 2^32)");

    imu_odom_init(&odom);
    (void)imu_odom_update(&odom, &counts, DT_S, false);   /* prime at 0 */
    counts.left_count  = 4u;
    counts.right_count = 4u;
    float const d1 = imu_odom_update(&odom, &counts, DT_S, false);
    counts.left_count  = 1004u;     /* +1000 pulses: implausible, discarded */
    counts.right_count = 5u;        /* +1 pulse on the other wheel */
    float const d2 = imu_odom_update(&odom, &counts, DT_S, false);
    check("H-U14", (fabsf(d1 - 4.0f * IMU_ODOM_MM_PER_PULSE) < 1e-3f) &&
                   (fabsf(d2 - 0.5f * IMU_ODOM_MM_PER_PULSE) < 1e-3f) &&
                   (imu_odom_glitches(&odom) == 1u),
          "cumulative counts; implausible jump discarded and counted");

    imu_odom_counts_t none = { 0u, 0u, false, false };
    float const d3 = imu_odom_update(&odom, &none, DT_S, true);
    check("H-U15", (fabsf(d3 - IMU_ODOM_FALLBACK_MM_PER_S * DT_S) < 1e-4f) &&
                   imu_odom_is_fallback(&odom),
          "no encoder registered: fallback speed used and flagged");
}

static void test_calibration (void)
{
    imu_monitor_t     mon;
    imu_sim_t         sim;
    imu_sim_profile_t prof;
    scenario_t        sc;

    memset(&sc, 0, sizeof(sc));
    start(&mon, &sim, &prof, 0u);
    check("H-U16", IMU_MONITOR_MONITORING == imu_monitor_get_mode(&mon),
          "calibration finishes within 1.3 s on a still car");
    check("H-U17", (fabsf(mon.filter.gyro_bias_dps[0] - 0.6f) < 0.1f) &&
                   (fabsf(mon.filter.gyro_bias_dps[1] + 0.9f) < 0.1f) &&
                   (fabsf(mon.filter.gyro_bias_dps[2] - 1.3f) < 0.1f),
          "gyro bias estimated within 0.1 deg/s");

    run(&mon, &sim, &prof, 300u, 1, &sc);
    imu_filter_attitude_t const * const p_att = imu_monitor_get_attitude(&mon);
    check("H-U18", (0u == sc.humps) && (0u == sc.impacts) &&
                   (fabsf(p_att->pitch_deg) < 0.5f) &&
                   (IMU_MOTION_STATIONARY == mon.motion.state_bits % 4u) &&
                   (IMU_TERRAIN_LEVEL == imu_terrain_get_state(&mon.terrain)),
          "3 s still: stationary, level, no hump, no impact, |pitch|<0.5");

    /* Car moved during calibration: restarts, then completes. */
    imu_monitor_init(&mon);
    imu_sim_init(&sim, 0u, 7u);
    prof.yaw_rate_dps = 60.0f;
    run(&mon, &sim, &prof, 20u, 1, &sc);
    int const b_waiting = (IMU_MONITOR_CALIBRATING ==
                           imu_monitor_get_mode(&mon));
    prof.yaw_rate_dps = 0.0f;
    run(&mon, &sim, &prof, CALIB_STEPS, 1, &sc);
    check("H-U19", b_waiting &&
                   (IMU_MONITOR_MONITORING == imu_monitor_get_mode(&mon)),
          "calibration restarts while the car turns, then completes");
}

/* Drive over one hump; return the estimate, or -1 if no single hump. */
static float hump_run (float height, float length, float speed,
                       float car_wheelbase, int b_encoder,
                       scenario_t * p_sc)
{
    imu_monitor_t     mon;
    imu_sim_t         sim;
    imu_sim_profile_t prof;
    float             result = -1.0f;

    memset(p_sc, 0, sizeof(*p_sc));
    start(&mon, &sim, &prof, ENCODER_WRAP_START);
    prof.hump_height_mm = height;
    prof.hump_length_mm = length;
    prof.hump_start_mm  = 400.0f;
    prof.wheelbase_mm   = car_wheelbase;
    prof.speed_mm_s     = speed;

    uint32_t const steps = (uint32_t)(((400.0f + length + car_wheelbase +
                                        600.0f) / speed) / DT_S);
    run(&mon, &sim, &prof, steps, b_encoder, p_sc);

    if (1u == p_sc->humps)
    {
        result = p_sc->last_peak_mm;
    }

    return (result);
}

static void test_hump_accuracy (void)
{
    float const heights[] = { 15.0f, 20.0f, 30.0f, 50.0f, 80.0f, 120.0f };
    float const lengths[] = { 150.0f, 300.0f, 600.0f, 1000.0f };
    float const speeds[]  = { 100.0f, 200.0f, 300.0f };
    float worst = 0.0f;
    int   b_all_detected = 1;
    int   b_sequence     = 1;
    int   runs           = 0;

    printf("       height  length  speed   estimate  error\n");

    for (uint32_t h = 0u; h < 6u; h++)
    {
        for (uint32_t l = 0u; l < 4u; l++)
        {
            /* Steepest slope of a cosine hump is atan(pi H / L).  Skip
             * profiles steeper than 35 deg (the car could not drive them)
             * and gentler than 6 deg (below the 4 deg climb threshold once
             * averaged over the wheelbase, so not a hump by design).
             */
            float const max_slope = 3.14159f * heights[h] / lengths[l];

            for (uint32_t s = 0u; (s < 3u) && (max_slope <= 0.70f) &&
                                 (max_slope >= 0.105f); s++)
            {
                scenario_t sc;

                runs++;
                float const est = hump_run(heights[h], lengths[l], speeds[s],
                                           IMU_TERRAIN_WHEELBASE_MM, 1, &sc);
                float const err = (est - heights[h]) / heights[h];

                if (est < 0.0f)
                {
                    b_all_detected = 0;
                    printf("       %6.0f  %6.0f  %5.0f   NOT DETECTED "
                           "(humps=%u)\n", (double)heights[h],
                           (double)lengths[l], (double)speeds[s],
                           (unsigned)sc.humps);
                }
                else
                {
                    worst = (fabsf(err) > worst) ? fabsf(err) : worst;
                    if ((0u == s) || (fabsf(err) > 0.05f))
                    {
                        printf("       %6.0f  %6.0f  %5.0f   %8.1f  %+5.1f%%\n",
                               (double)heights[h], (double)lengths[l],
                               (double)speeds[s], (double)est,
                               (double)(100.0f * err));
                    }
                }

                /* Expected terrain changes, one nibble each: CLIMBING (1),
                 * DESCENDING (2), HUMP_CLOSED (3), LEVEL (0).
                 */
                if ((0x0321u != sc.terrain_trace) ||
                    (4u != sc.terrain_changes))
                {
                    b_sequence = 0;
                }
            }
        }
    }

    printf("       worst error over %d runs: %.1f%%\n", runs,
           (double)(100.0f * worst));
    check("H-I01", b_all_detected,
          "every hump 15-120 mm high, 150-1000 mm long (slope 6-35 deg), "
          "100-300 mm/s detected exactly once");
    check("H-I02", worst <= NFR4_TOLERANCE,
          "NFR4: peak estimate within +/-20% in every run");
    check("H-I03", worst <= 0.10f,
          "design margin: peak estimate within +/-10% in every run");
    check("H-I04", b_sequence,
          "terrain sequence Level->Climbing->Descending->HumpClosed->Level");

    scenario_t sc;
    float const big = hump_run(150.0f, 600.0f, 200.0f,
                               IMU_TERRAIN_WHEELBASE_MM, 1, &sc);
    check("H-I05", (big > 127.0f) && (127u == sc.last_packed_mm),
          "150 mm hump: packed value saturates at 127 mm");
}

static void test_hump_variants (void)
{
    scenario_t sc;

    /* Wheelbase set 20 mm wrong on the car. */
    float const short_wb = hump_run(50.0f, 300.0f, 200.0f,
                                    IMU_TERRAIN_WHEELBASE_MM - 20.0f, 1, &sc);
    float const long_wb  = hump_run(50.0f, 300.0f, 200.0f,
                                    IMU_TERRAIN_WHEELBASE_MM + 20.0f, 1, &sc);
    printf("       wheelbase -20 mm: %.1f mm, +20 mm: %.1f mm (true 50)\n",
           (double)short_wb, (double)long_wb);
    check("H-I06", (fabsf(short_wb - 50.0f) < 10.0f) &&
                   (fabsf(long_wb - 50.0f) < 10.0f),
          "wheelbase measured 20 mm wrong: still within NFR4");

    /* No encoder: fallback speed equals the true speed. */
    float const fb = hump_run(50.0f, 300.0f, IMU_ODOM_FALLBACK_MM_PER_S,
                              IMU_TERRAIN_WHEELBASE_MM, 0, &sc);
    printf("       no encoder (fallback speed): %.1f mm (true 50)\n",
           (double)fb);
    check("H-I07", fabsf(fb - 50.0f) <= (NFR4_TOLERANCE * 50.0f),
          "no encoder registered: fallback still within NFR4");

    /* Small hump below the detection threshold is not reported. */
    float const tiny = hump_run(3.0f, 600.0f, 200.0f,
                                IMU_TERRAIN_WHEELBASE_MM, 1, &sc);
    check("H-I08", (tiny < 0.0f) && (0u == sc.humps),
          "3 mm undulation (slope < climb threshold) is not a hump");

    /* Undrivable 68 deg hump: the gyro clips at 500 deg/s and the estimate
     * goes wrong.  The filter must re-sync and the terrain region must
     * return to LEVEL instead of locking up.
     */
    imu_monitor_t     mon;
    imu_sim_t         sim;
    imu_sim_profile_t prof;

    memset(&sc, 0, sizeof(sc));
    start(&mon, &sim, &prof, 0u);
    prof.hump_height_mm = 120.0f;
    prof.hump_length_mm = 150.0f;
    prof.hump_start_mm  = 300.0f;
    prof.speed_mm_s     = 300.0f;
    run(&mon, &sim, &prof, 600u, 1, &sc);
    printf("       steep hump: pitch after 6 s %.2f deg, humps %u\n",
           (double)imu_monitor_get_attitude(&mon)->pitch_deg,
           (unsigned)sc.humps);
    check("H-I09", (fabsf(imu_monitor_get_attitude(&mon)->pitch_deg) < 1.0f) &&
                   (IMU_TERRAIN_LEVEL == imu_terrain_get_state(&mon.terrain)) &&
                   (sc.humps >= 1u),
          "gyro clipped on an undrivable hump: filter re-syncs, back to LEVEL");
}

static void test_persistence (void)
{
    imu_terrain_t terrain;
    int b_ok = 1;

    imu_terrain_init(&terrain);
    for (uint32_t i = 0u; i < 3u; i++)
    {
        (void)imu_terrain_update(&terrain, 10.0f, 0.0f, 2.0f, true);
    }
    for (uint32_t i = 0u; i < 20u; i++)
    {
        if (imu_terrain_update(&terrain, 0.0f, 0.0f, 2.0f, true))
        {
            b_ok = 0;
        }
    }
    check("H-U20", b_ok &&
                   (IMU_TERRAIN_LEVEL == imu_terrain_get_state(&terrain)),
          "3-sample pitch spike does not open a hump (persistence = 5)");

    imu_terrain_init(&terrain);
    for (uint32_t i = 0u; i < 4u; i++)
    {
        (void)imu_terrain_update(&terrain, 0.0f, 25.0f, 0.0f, false);
    }
    int const b_not_yet = !imu_terrain_is_tilted(&terrain);
    (void)imu_terrain_update(&terrain, 0.0f, 25.0f, 0.0f, false);
    check("H-U21", b_not_yet && imu_terrain_is_tilted(&terrain),
          "25 deg roll flagged as tilt after 5 samples");

    /* Nose up, then straight back to level (the real car on a short
     * hump never tips nose down).  At 2 mm per sample the wheelbase is about
     * 38 samples of level travel.
     */
    imu_terrain_hump_t hump;
    uint32_t           closed_at = 0u;
    int                b_desc    = 0;

    imu_terrain_init(&terrain);
    for (uint32_t i = 0u; i < 20u; i++)
    {
        (void)imu_terrain_update(&terrain, 10.0f, 0.0f, 2.0f, true);
    }
    int const b_climbing = (IMU_TERRAIN_CLIMBING ==
                            imu_terrain_get_state(&terrain));
    for (uint32_t i = 1u; (i <= 200u) && (0u == closed_at); i++)
    {
        (void)imu_terrain_update(&terrain, 0.0f, 0.0f, 2.0f, true);
        b_desc = b_desc || (IMU_TERRAIN_DESCENDING ==
                            imu_terrain_get_state(&terrain));
        if (IMU_TERRAIN_HUMP_CLOSED == imu_terrain_get_state(&terrain))
        {
            closed_at = i;
        }
    }
    int const b_result = imu_terrain_take_result(&terrain, &hump);
    printf("       climb then level: closed after %u level samples, "
           "peak %.1f mm\n", (unsigned)closed_at, (double)hump.peak_mm);
    check("H-U32", b_climbing && !b_desc && b_result && !hump.b_cut_short &&
                   (closed_at >= 36u) && (closed_at <= 40u) &&
                   (1u == imu_terrain_get_hump_count(&terrain)),
          "climb then level (no nose-down): hump closes after one "
          "wheelbase of level travel");

    /* Same, but no distance (no encoders yet): closes on the 1 s limit. */
    imu_terrain_init(&terrain);
    for (uint32_t i = 0u; i < 20u; i++)
    {
        (void)imu_terrain_update(&terrain, 10.0f, 0.0f, 0.0f, true);
    }
    closed_at = 0u;
    for (uint32_t i = 1u; (i <= 200u) && (0u == closed_at); i++)
    {
        (void)imu_terrain_update(&terrain, 0.0f, 0.0f, 0.0f, true);
        if (IMU_TERRAIN_HUMP_CLOSED == imu_terrain_get_state(&terrain))
        {
            closed_at = i;
        }
    }
    check("H-U33", closed_at == IMU_TERRAIN_TOP_LEVEL_SAMPLES,
          "climb then level with no encoder distance: closes after 1 s");

    /* A short level crest (40 mm, less than the wheelbase) followed by a
     * real descent still goes through DESCENDING.
     */
    imu_terrain_init(&terrain);
    for (uint32_t i = 0u; i < 20u; i++)
    {
        (void)imu_terrain_update(&terrain, 10.0f, 0.0f, 2.0f, true);
    }
    for (uint32_t i = 0u; i < 20u; i++)
    {
        (void)imu_terrain_update(&terrain, 0.0f, 0.0f, 2.0f, true);
    }
    int const b_still_climbing = (IMU_TERRAIN_CLIMBING ==
                                  imu_terrain_get_state(&terrain));
    b_desc = 0;
    for (uint32_t i = 0u; i < 20u; i++)
    {
        (void)imu_terrain_update(&terrain, -10.0f, 0.0f, 2.0f, true);
        b_desc = b_desc || (IMU_TERRAIN_DESCENDING ==
                            imu_terrain_get_state(&terrain));
    }
    int b_closed = 0;
    for (uint32_t i = 0u; i < 10u; i++)
    {
        (void)imu_terrain_update(&terrain, 0.0f, 0.0f, 2.0f, true);
        b_closed = b_closed || (IMU_TERRAIN_HUMP_CLOSED ==
                                imu_terrain_get_state(&terrain));
    }
    check("H-U34", b_still_climbing && b_desc && b_closed &&
                   (1u == imu_terrain_get_hump_count(&terrain)),
          "short flat top then nose-down: still CLIMBING->DESCENDING->"
          "closed, one hump");
}

static void test_impact (void)
{
    imu_monitor_t     mon;
    imu_sim_t         sim;
    imu_sim_profile_t prof;
    scenario_t        sc;

    memset(&sc, 0, sizeof(sc));
    start(&mon, &sim, &prof, 0u);

    /* +3 g shock for two samples on a still car. */
    uint32_t settle_steps = 0u;
    for (uint32_t step = 0u; step < 200u; step++)
    {
        drv_imu_sample_t raw;
        imu_odom_counts_t    counts = { 0u, 0u, true, true };
        imu_monitor_output_t out;

        (void)imu_sim_step(&sim, &prof, DT_S, &raw);
        if (step < 2u)
        {
            raw.accel[2] = (int16_t)(raw.accel[2] + 12288);
        }
        imu_monitor_step(&mon, &raw, &counts, DT_S, &out);
        record(&out, &sc);
        if (out.b_impact)
        {
            check("H-U22", (0u == step) && (out.impact_peak_g > 2.5f),
                  "+3 g shock detected on the very first sample");
        }
        if (out.b_recovered && (0u == settle_steps))
        {
            settle_steps = step;
        }
    }
    printf("       recovered after %u samples\n", (unsigned)settle_steps);
    check("H-U23", (1u == sc.impacts) && (1u == sc.recoveries) &&
                   (settle_steps >= IMU_IMPACT_RECOVER_SAMPLES) &&
                   (settle_steps <= IMU_IMPACT_RECOVER_SAMPLES + 5u),
          "one impact; Recovered after ~0.5 s settled near 1 g");

    /* A 1.6 g bump while DESCENDING a hump is ignored ... */
    imu_impact_t impact;
    imu_impact_init(&impact);
    int const b_on_hump = imu_impact_detect(&impact, 2.6f, 0.0f, true);
    /* ... but the same bump on level ground is an impact. */
    int const b_level   = imu_impact_detect(&impact, 2.6f, 0.0f, false);
    imu_impact_t impact2;
    imu_impact_init(&impact2);
    int const b_hard    = imu_impact_detect(&impact2, 4.5f, 0.0f, true);
    check("H-U24", !b_on_hump && b_level && b_hard,
          "hump landing ignored while descending; real crash still caught");

    imu_impact_init(&impact);
    imu_impact_start_holdoff(&impact);
    int const b_holdoff = imu_impact_detect(&impact, 2.6f, 0.0f, false);
    check("H-U25", !b_holdoff, "hold-off after a hump closes");
}

static void test_motion (void)
{
    imu_monitor_t     mon;
    imu_sim_t         sim;
    imu_sim_profile_t prof;
    scenario_t        sc;

    memset(&sc, 0, sizeof(sc));
    start(&mon, &sim, &prof, 0u);

    prof.speed_mm_s = 200.0f;
    run(&mon, &sim, &prof, 100u, 1, &sc);
    imu_motion_class_t const cruise = imu_motion_get_class(&mon.motion);

    prof.yaw_rate_dps = 90.0f;
    run(&mon, &sim, &prof, 6u, 1, &sc);
    imu_motion_class_t const turn = imu_motion_get_class(&mon.motion);

    prof.yaw_rate_dps = 0.0f;
    prof.accel_g      = 0.2f;
    run(&mon, &sim, &prof, 30u, 1, &sc);
    imu_motion_class_t const accel = imu_motion_get_class(&mon.motion);
    imu_terrain_state_t const terrain = imu_terrain_get_state(&mon.terrain);

    prof.accel_g    = 0.0f;
    prof.speed_mm_s = 0.0f;
    run(&mon, &sim, &prof, 40u, 1, &sc);
    imu_motion_class_t const still = imu_motion_get_class(&mon.motion);

    check("H-U26", IMU_MOTION_CRUISING == cruise, "steady 200 mm/s: CRUISING");
    check("H-U27", IMU_MOTION_TURNING == turn,
          "90 deg/s yaw: TURNING within 6 samples");
    check("H-U28", (IMU_MOTION_ACCELERATING == accel) &&
                   (IMU_TERRAIN_LEVEL == terrain),
          "0.2 g forward: ACCELERATING, and not mistaken for climbing");
    check("H-U29", IMU_MOTION_STATIONARY == still,
          "stopped for 0.4 s: back to STATIONARY");

    /* Gyro drift left 5 deg of pitch error; a stop re-levels it at the
     * 2 deg/s correction limit.
     */
    mon.filter.attitude.pitch_deg = 5.0f;
    run(&mon, &sim, &prof, 300u, 1, &sc);
    check("H-U30", fabsf(imu_monitor_get_attitude(&mon)->pitch_deg) < 0.5f,
          "5 deg drift removed within 3 s of standing still");

    /* Long acceleration that a slow complementary filter would mistake
     * for a climb: 0.15 g for 1 s while cruising.
     */
    memset(&sc, 0, sizeof(sc));
    prof.speed_mm_s = 200.0f;
    prof.accel_g    = 0.15f;
    run(&mon, &sim, &prof, 100u, 1, &sc);
    check("H-U31", (0u == sc.humps) &&
                   (IMU_TERRAIN_LEVEL == imu_terrain_get_state(&mon.terrain)),
          "0.15 g for 1 s on flat ground: no hump opened");
}

/* Body-axis sample of a car rolled left side up by roll_deg, with the board
 * mounted upside down (turned 180 deg about the forward axis, as on the
 * bench car of 28 Oct 2026): Y and Z read negated.  |a| is 0.87 g, as that
 * GY-511 read; dither flips the sign of the tiny Y reading, so the raw roll
 * angle jumps between +180 and -180 deg the way real noise makes it.
 */
static void upside_down_input (float roll_deg, uint32_t step,
                               imu_filter_input_t * p_in)
{
    float const r      = roll_deg * 3.14159265f / 180.0f;
    float const dither = (0u == (step & 1u)) ? 0.002f : -0.002f;

    /* Earth's field in the level frame: forward, left, up (raw counts). */
    float const mx = 1500.0f;
    float const my = 300.0f;
    float const mz = -900.0f;

    memset(p_in, 0, sizeof(*p_in));
    p_in->accel_g[IMU_FILTER_FWD]  = 0.0f;
    p_in->accel_g[IMU_FILTER_LEFT] = -(0.87f * sinf(r)) + dither;
    p_in->accel_g[IMU_FILTER_UP]   = -(0.87f * cosf(r));
    p_in->mag[IMU_FILTER_FWD]      = mx;
    p_in->mag[IMU_FILTER_LEFT]     = -((my * cosf(r)) + (mz * sinf(r)));
    p_in->mag[IMU_FILTER_UP]       = -((-my * sinf(r)) + (mz * cosf(r)));
    p_in->b_has_gyro = false;
    p_in->b_has_mag  = true;
}

/* Board mounted upside down: roll must not wrap through +/-180 deg. */
static void test_upside_down (void)
{
    imu_filter_t       filter;
    imu_filter_input_t in;
    uint32_t           step = 0u;
    float              worst_roll = 0.0f;
    float              worst_rate = 0.0f;

    imu_filter_init(&filter);
    for (step = 0u; (step < 300u) && !imu_filter_is_calibrated(&filter);
         step++)
    {
        upside_down_input(0.0f, step, &in);
        (void)imu_filter_calibrate(&filter, &in);
    }

    /* 1 s still, lift the left side to 25 deg over 1 s, hold 1 s, lower it
     * over 1 s, then 1 s still.
     */
    for (uint32_t i = 0u; i < 500u; i++)
    {
        float roll_true = 0.0f;

        if ((i >= 100u) && (i < 200u))
        {
            roll_true = 0.25f * (float)(i - 100u);
        }
        else if ((i >= 200u) && (i < 300u))
        {
            roll_true = 25.0f;
        }
        else if ((i >= 300u) && (i < 400u))
        {
            roll_true = 25.0f - (0.25f * (float)(i - 300u));
        }

        upside_down_input(roll_true, step + i, &in);
        (void)imu_filter_update(&filter, &in, DT_S, false);

        imu_filter_attitude_t const * const p_att =
            imu_filter_get_attitude(&filter);
        float const roll_err = fabsf(p_att->roll_deg - roll_true);
        float const rate     = fabsf(p_att->yaw_rate_dps);

        worst_roll = (roll_err > worst_roll) ? roll_err : worst_roll;
        worst_rate = (rate > worst_rate) ? rate : worst_rate;
    }

    imu_filter_attitude_t const * const p_end =
        imu_filter_get_attitude(&filter);

    printf("       upside down: worst roll error %.1f deg, worst yaw rate "
           "%.1f dps, end roll %.1f deg\n", (double)worst_roll,
           (double)worst_rate, (double)p_end->roll_deg);
    check("H-A10", imu_filter_is_calibrated(&filter) &&
                   (worst_roll < 5.0f) && (worst_rate < 20.0f) &&
                   (fabsf(p_end->roll_deg) < 1.0f),
          "board upside down: 25 deg roll reads +25, no 360 deg jump, "
          "no false turn");

    /* A sensor reading 0.87 g at rest still calibrates normally. */
    imu_filter_calib_t result = IMU_FILTER_CALIB_BUSY;

    imu_filter_init(&filter);
    for (step = 0u; (step < 300u) && !imu_filter_is_calibrated(&filter);
         step++)
    {
        upside_down_input(0.0f, step, &in);
        result = imu_filter_calibrate(&filter, &in);
    }
    check("H-A11", (IMU_FILTER_CALIB_DONE == result) &&
                   (step <= (IMU_FILTER_CALIB_SAMPLES + 1u)),
          "0.87 g sensor, still: calibrated in 1 s, not forced");

    /* Pushed along the floor for 0.3 s at the start (no gyro to see it):
     * the push must not end up in the level reference.
     */
    uint32_t restarts = 0u;

    imu_filter_init(&filter);
    for (step = 0u; (step < 300u) && !imu_filter_is_calibrated(&filter);
         step++)
    {
        upside_down_input(0.0f, step, &in);
        if (step < 30u)
        {
            in.accel_g[IMU_FILTER_FWD] = 0.004f * (float)step;
        }
        result = imu_filter_calibrate(&filter, &in);
        restarts += (IMU_FILTER_CALIB_RESTART == result) ? 1u : 0u;
    }
    printf("       pushed during calibration: %u restarts, level reference "
           "pitch %.2f deg\n", (unsigned)restarts,
           (double)filter.pitch_offset_deg);
    check("H-A12", (IMU_FILTER_CALIB_DONE == result) && (restarts > 0u) &&
                   (fabsf(filter.pitch_offset_deg) < 0.5f),
          "0.87 g sensor pushed while calibrating: restarts, push not "
          "used as level");
}

/* Accelerometer + compass only (LSM303D on the GY-511): no gyroscope. */
static void test_accel_only (void)
{
    imu_monitor_t     mon;
    imu_sim_t         sim;
    imu_sim_profile_t prof;
    scenario_t        sc;

    g_no_gyro = 1;

    /* Calibration and a still car. */
    memset(&sc, 0, sizeof(sc));
    start(&mon, &sim, &prof, 0u);
    int const b_cal = (IMU_MONITOR_MONITORING == imu_monitor_get_mode(&mon));
    run(&mon, &sim, &prof, 300u, 1, &sc);
    imu_filter_attitude_t const * const p_att = imu_monitor_get_attitude(&mon);
    check("H-A01", b_cal && (fabsf(p_att->pitch_deg) < 0.5f) &&
                   (fabsf(p_att->yaw_rate_dps) < 5.0f) && (0u == sc.humps) &&
                   (IMU_MOTION_STATIONARY == imu_motion_get_class(&mon.motion)),
          "no gyro: calibrates; 3 s still: level, no turn, no hump");

    /* Hump grid, as H-I01..H-I04. */
    float const heights[] = { 15.0f, 20.0f, 30.0f, 50.0f, 80.0f, 120.0f };
    float const lengths[] = { 150.0f, 300.0f, 600.0f, 1000.0f };
    float const speeds[]  = { 100.0f, 200.0f, 300.0f };
    float worst = 0.0f;
    int   b_all_detected = 1;
    int   b_sequence     = 1;
    int   runs           = 0;

    printf("       no gyro: height  length  speed   estimate  error\n");
    for (uint32_t h = 0u; h < 6u; h++)
    {
        for (uint32_t l = 0u; l < 4u; l++)
        {
            float const max_slope = 3.14159f * heights[h] / lengths[l];

            for (uint32_t s = 0u; (s < 3u) && (max_slope <= 0.70f) &&
                                 (max_slope >= 0.105f); s++)
            {
                runs++;
                float const est = hump_run(heights[h], lengths[l], speeds[s],
                                           IMU_TERRAIN_WHEELBASE_MM, 1, &sc);
                float const err = (est - heights[h]) / heights[h];

                if (est < 0.0f)
                {
                    b_all_detected = 0;
                    printf("       %14.0f  %6.0f  %5.0f   NOT DETECTED "
                           "(humps=%u)\n", (double)heights[h],
                           (double)lengths[l], (double)speeds[s],
                           (unsigned)sc.humps);
                }
                else
                {
                    worst = (fabsf(err) > worst) ? fabsf(err) : worst;
                    if (fabsf(err) > 0.10f)
                    {
                        printf("       %14.0f  %6.0f  %5.0f   %8.1f  %+5.1f%%\n",
                               (double)heights[h], (double)lengths[l],
                               (double)speeds[s], (double)est,
                               (double)(100.0f * err));
                    }
                }

                if ((0x0321u != sc.terrain_trace) ||
                    (4u != sc.terrain_changes))
                {
                    b_sequence = 0;
                }
            }
        }
    }
    printf("       no gyro: worst error over %d runs: %.1f%%\n", runs,
           (double)(100.0f * worst));
    check("H-A02", b_all_detected,
          "no gyro: every hump of the H-I01 grid detected exactly once");
    check("H-A03", worst <= NFR4_TOLERANCE,
          "no gyro, steady speed: NFR4 peak within +/-20% in every run");
    check("H-A04", b_sequence,
          "no gyro: Level->Climbing->Descending->HumpClosed->Level");

    /* Realistic start: 0 -> 200 mm/s in 0.25 s (0.08 g), then cruise. */
    memset(&sc, 0, sizeof(sc));
    start(&mon, &sim, &prof, 0u);
    prof.accel_g = 0.08f;
    for (uint32_t step = 0u; step < 25u; step++)
    {
        prof.speed_mm_s = 8.0f * (float)(step + 1u);
        run(&mon, &sim, &prof, 1u, 1, &sc);
    }
    prof.accel_g = 0.0f;
    run(&mon, &sim, &prof, 200u, 1, &sc);
    check("H-A05", (0u == sc.humps) && (0u == sc.terrain_changes),
          "no gyro: start-up acceleration on flat ground opens no hump");

    /* Turning from the compass. */
    memset(&sc, 0, sizeof(sc));
    run(&mon, &sim, &prof, 500u, 1, &sc);
    int const b_no_false_turn = (0u == (sc.motion_seen &
                                        (1u << IMU_MOTION_TURNING)));
    prof.yaw_rate_dps = 90.0f;
    uint32_t turn_steps = 0u;
    for (uint32_t step = 0u; (step < 50u) && (0u == turn_steps); step++)
    {
        run(&mon, &sim, &prof, 1u, 1, &sc);
        if (IMU_MOTION_TURNING == imu_motion_get_class(&mon.motion))
        {
            turn_steps = step + 1u;
        }
    }
    float const heading_start = imu_monitor_get_attitude(&mon)->heading_deg;
    run(&mon, &sim, &prof, 100u, 1, &sc);
    float const turned = imu_math_wrap_360(
        imu_monitor_get_attitude(&mon)->heading_deg - heading_start);
    printf("       no gyro: TURNING after %u samples, 1 s at 90 deg/s "
           "turned %.1f deg\n", (unsigned)turn_steps, (double)turned);
    check("H-A06", b_no_false_turn,
          "no gyro: 5 s straight at 200 mm/s never classed TURNING");
    check("H-A07", (turn_steps > 0u) && (turn_steps <= 20u),
          "no gyro: 90 deg/s turn classed TURNING within 0.2 s (compass)");
    check("H-A08", fabsf(turned - 90.0f) < 10.0f,
          "no gyro: compass heading follows a 90 deg turn within 10 deg");

    /* Impact needs only the accelerometer. */
    memset(&sc, 0, sizeof(sc));
    start(&mon, &sim, &prof, 0u);
    uint32_t impact_step = 99u;
    for (uint32_t step = 0u; step < 100u; step++)
    {
        drv_imu_sample_t     raw;
        imu_odom_counts_t    counts = { 0u, 0u, true, true };
        imu_monitor_output_t out;

        (void)imu_sim_step(&sim, &prof, DT_S, &raw);
        if (step < 2u)
        {
            raw.accel[2] = (int16_t)(raw.accel[2] + 12288);
        }
        imu_monitor_step(&mon, &raw, &counts, DT_S, &out);
        record(&out, &sc);
        if (out.b_impact && (99u == impact_step))
        {
            impact_step = step;
        }
    }
    check("H-A09", (0u == impact_step) && (1u == sc.impacts) &&
                   (1u == sc.recoveries),
          "no gyro: +3 g shock detected on the first sample, then recovered");

    /* Known limit, reported but not a pass/fail: a long acceleration while
     * cruising (H-U31) cannot be told apart from a slope without a gyro.
     */
    memset(&sc, 0, sizeof(sc));
    start(&mon, &sim, &prof, 0u);
    prof.speed_mm_s = 200.0f;
    run(&mon, &sim, &prof, 100u, 1, &sc);
    prof.accel_g = 0.15f;
    run(&mon, &sim, &prof, 100u, 1, &sc);
    printf("       no gyro, known limit: 0.15 g for 1 s while cruising -> "
           "terrain %s\n",
           (IMU_TERRAIN_LEVEL == imu_terrain_get_state(&mon.terrain)) ?
           "LEVEL" : "CLIMBING (false hump; a gyro is needed for this)");

    test_upside_down();

    g_no_gyro = 0;
}

/* One simulator step through the monitor, feeding the ramp detection. */
static void ramp_step (imu_monitor_t * p_mon, imu_sim_t * p_sim,
                       imu_sim_profile_t const * p_prof)
{
    scenario_t sc;

    memset(&sc, 0, sizeof(sc));
    run(p_mon, p_sim, p_prof, 1u, 1, &sc);
    imu_filter_attitude_t const * const p_att =
        imu_monitor_get_attitude(p_mon);
    (void)imu_ramp_update(p_att->pitch_deg, p_att->roll_deg);
}

static void test_ramp (void)
{
    /* imu_task's 10 ms: on after 6 samples, off after 10. */
    imu_ramp_stop();
    int const b_start_clear = !imu_is_ok() && !imu_on_ramp();
    uint32_t on_at = 0u;
    for (uint32_t i = 1u; (i <= 20u) && (0u == on_at); i++)
    {
        if (imu_ramp_update(8.0f, 0.0f))
        {
            on_at = i;
        }
    }
    uint32_t off_at = 0u;
    for (uint32_t i = 1u; (i <= 30u) && (0u == off_at); i++)
    {
        if (imu_ramp_update(1.0f, 0.0f))
        {
            off_at = i;
        }
    }
    check("H-U35", b_start_clear && (6u == on_at) && (10u == off_at) &&
                   !imu_on_ramp(),
          "ramp on after 60 ms at 6 deg, off after 100 ms at 4 deg");

    /* A 5-sample spike is not a ramp; between 4 and 6 deg keeps the state;
     * nose down counts the same; roll counts as tilt.
     */
    imu_ramp_stop();
    for (uint32_t i = 0u; i < 5u; i++)
    {
        (void)imu_ramp_update(10.0f, 0.0f);
    }
    int const b_spike_ignored = !imu_on_ramp();
    (void)imu_ramp_update(0.0f, 0.0f);
    for (uint32_t i = 0u; i < 6u; i++)
    {
        (void)imu_ramp_update(-8.0f, 0.0f);
    }
    int const b_nose_down = imu_on_ramp();
    for (uint32_t i = 0u; i < 50u; i++)
    {
        (void)imu_ramp_update(5.0f, 0.0f);
    }
    int const b_held = imu_on_ramp();
    imu_ramp_stop();
    for (uint32_t i = 0u; i < 6u; i++)
    {
        (void)imu_ramp_update(4.5f, 4.5f);
    }
    uint32_t const tilt = imu_get_tilt_x10();
    check("H-U36", b_spike_ignored && b_nose_down && b_held &&
                   imu_on_ramp() && (tilt >= 63u) && (tilt <= 64u),
          "spike ignored; nose down, roll and the 4-6 deg band behave");

    /* Sensor lost on a ramp: never left "on a ramp". */
    imu_ramp_stop();
    check("H-U37", !imu_on_ramp() && !imu_is_ok() &&
                   (0u == imu_get_tilt_x10()),
          "sensor lost: not on a ramp, not ok, tilt 0");

    /* Whole pipeline, no gyro: a 100 mm hump over 1 m has a 17 deg slope.
     * Flat driving before it must not read as a ramp.
     */
    imu_monitor_t     mon;
    imu_sim_t         sim;
    imu_sim_profile_t prof;
    uint32_t          on_steps = 0u;
    int               b_false  = 0;

    g_no_gyro = 1;
    start(&mon, &sim, &prof, 0u);
    prof.hump_height_mm = 100.0f;
    prof.hump_length_mm = 1000.0f;
    prof.hump_start_mm  = 400.0f;
    prof.wheelbase_mm   = IMU_TERRAIN_WHEELBASE_MM;
    prof.speed_mm_s     = 200.0f;
    imu_ramp_stop();
    for (uint32_t step = 0u; step < 1200u; step++)
    {
        drv_imu_sample_t     raw;
        imu_odom_counts_t    counts;
        imu_monitor_output_t out;

        (void)imu_sim_step(&sim, &prof, DT_S, &raw);
        counts.b_has_left  = true;
        counts.b_has_right = true;
        counts.left_count  = imu_sim_encoder(&sim);
        counts.right_count = imu_sim_encoder(&sim);
        imu_monitor_step(&mon, &raw, &counts, DT_S, &out);
        imu_filter_attitude_t const * const p_att =
            imu_monitor_get_attitude(&mon);
        (void)imu_ramp_update(p_att->pitch_deg, p_att->roll_deg);
        b_false = b_false || (imu_on_ramp() && (sim.rear_pos_mm < 200.0f));
        on_steps += imu_on_ramp() ? 1u : 0u;
    }
    int const b_ended_flat = !imu_on_ramp();
    g_no_gyro = 0;
    imu_ramp_stop();
    printf("       ramp pipeline: on ramp for %u ms of a 12 s run\n",
           (unsigned)(on_steps * 10u));
    check("H-A13", !b_false && (on_steps >= 100u) && b_ended_flat,
          "no gyro: simulated slope through the monitor sets and clears "
          "imu_on_ramp(), flat driving does not");

    /* Sensor lost and back while stopped on a slope: the level must be
     * kept, or the slope reads flat and flat ground reads as a ramp.
     */
    g_no_gyro = 1;
    start(&mon, &sim, &prof, 0u);
    prof.hump_height_mm = 100.0f;
    prof.hump_length_mm = 1000.0f;
    prof.hump_start_mm  = 400.0f;
    prof.wheelbase_mm   = IMU_TERRAIN_WHEELBASE_MM;
    prof.speed_mm_s     = 200.0f;
    imu_ramp_stop();
    for (uint32_t i = 0u; (i < 1000u) && (imu_get_tilt_x10() < 100u); i++)
    {
        ramp_step(&mon, &sim, &prof);
    }
    prof.speed_mm_s = 0.0f;
    for (uint32_t i = 0u; i < 50u; i++)
    {
        ramp_step(&mon, &sim, &prof);
    }
    imu_ramp_stop();
    int const b_resumed = imu_monitor_resume(&mon) ? 1 : 0;
    for (uint32_t i = 0u; i < 30u; i++)
    {
        ramp_step(&mon, &sim, &prof);
    }
    int const      b_slope    = imu_on_ramp();
    uint32_t const tilt_slope = imu_get_tilt_x10();
    prof.speed_mm_s = 200.0f;
    for (uint32_t i = 0u; (i < 2000u) && (sim.rear_pos_mm < 1800.0f); i++)
    {
        ramp_step(&mon, &sim, &prof);
    }
    prof.speed_mm_s = 0.0f;
    for (uint32_t i = 0u; i < 300u; i++)
    {
        ramp_step(&mon, &sim, &prof);
    }
    uint32_t const tilt_flat = imu_get_tilt_x10();
    printf("       dropout on a slope: tilt %u.%u deg there, %u.%u deg on "
           "flat ground\n", (unsigned)(tilt_slope / 10u),
           (unsigned)(tilt_slope % 10u), (unsigned)(tilt_flat / 10u),
           (unsigned)(tilt_flat % 10u));
    check("H-A14", b_resumed && b_slope && (tilt_slope >= 80u) &&
                   !imu_on_ramp() && (tilt_flat <= 10u),
          "no gyro: sensor back on a slope keeps the level: ramp there, "
          "not on flat ground");

    /* Never calibrated (sensor lost at boot, or a new run): no resume. */
    imu_monitor_init(&mon);
    int const b_refused = !imu_monitor_resume(&mon);
    g_no_gyro = 0;
    imu_ramp_stop();
    check("H-A15", b_refused &&
                   (IMU_MONITOR_CALIBRATING == imu_monitor_get_mode(&mon)),
          "not calibrated yet: resume refused, calibration still runs");
}

/* The real course: only the ball climbs the ramp, the rear wheels pass
 * either side.  Measured ramp: 10 mm high, 196 mm long.  Returns the peak.
 */
static float ball_only_peak (float height, float length, float speed)
{
    imu_monitor_t     mon;
    imu_sim_t         sim;
    imu_sim_profile_t prof;
    scenario_t        sc;

    memset(&sc, 0, sizeof(sc));
    g_no_gyro = 1;
    start(&mon, &sim, &prof, 0u);
    mon.terrain.b_rear_climbs = false;
    prof.b_rear_flat    = true;
    prof.hump_height_mm = height;
    prof.hump_length_mm = length;
    prof.hump_start_mm  = 400.0f;
    prof.wheelbase_mm   = IMU_TERRAIN_WHEELBASE_MM;
    prof.speed_mm_s     = speed;
    run(&mon, &sim, &prof,
        (uint32_t)(((400.0f + length + 200.0f) / speed) * 100.0f), 1, &sc);
    g_no_gyro = 0;

    return (1u == sc.humps) ? (sc.last_peak_mm / IMU_TERRAIN_FLAT_GAIN) :
                              -1.0f;
}

static void test_rear_flat (void)
{
    float const p10a = ball_only_peak(10.0f, 196.0f, 120.0f);
    float const p10b = ball_only_peak(10.0f, 196.0f, 40.0f);
    float const p20  = ball_only_peak(20.0f, 300.0f, 120.0f);

    printf("       ball only: 10 mm ramp %.1f mm at 120 mm/s, %.1f mm at "
           "40 mm/s; 20 mm ramp %.1f mm\n", (double)p10a, (double)p10b,
           (double)p20);
    check("H-A16", (fabsf(p10a - 10.0f) <= 2.0f) &&
                   (fabsf(p10b - 10.0f) <= 2.0f) &&
                   (fabsf(p20 - 20.0f) <= 4.0f),
          "no gyro, rear wheels beside the ramp: one hump, height within "
          "20 %");

    /* A short spike must not set the peak: 8 deg, then 40 ms at 12 deg. */
    imu_terrain_t      terrain;
    imu_terrain_hump_t hump;

    imu_terrain_init(&terrain);
    for (uint32_t i = 0u; i < 100u; i++)
    {
        float const pitch = ((i >= 50u) && (i < 54u)) ? 12.0f : 8.0f;

        (void)imu_terrain_update(&terrain, pitch, 0.0f, 1.0f, true);
    }
    for (uint32_t i = 0u; i < 150u; i++)
    {
        (void)imu_terrain_update(&terrain, 0.0f, 0.0f, 1.0f, true);
    }
    int const   b_result = imu_terrain_take_result(&terrain, &hump);
    float const clean    = IMU_TERRAIN_FLAT_GAIN *
                           IMU_TERRAIN_WHEELBASE_MM * sinf(8.0f * 0.0174533f);

    printf("       spike: peak %.1f mm, 8 deg alone is %.1f mm\n",
           (double)hump.peak_mm, (double)clean);
    check("H-U38", b_result && (hump.peak_mm <= (clean * 1.12f)),
          "40 ms pitch spike adds less than 12 % to the peak");

    /* Sensor lost part way over the ramp: the hump is reported, once. */
    imu_monitor_t     mon;
    imu_sim_t         sim;
    imu_sim_profile_t prof;
    scenario_t        sc;

    memset(&sc, 0, sizeof(sc));
    g_no_gyro = 1;
    start(&mon, &sim, &prof, 0u);
    mon.terrain.b_rear_climbs = false;
    prof.b_rear_flat    = true;
    prof.hump_height_mm = 10.0f;
    prof.hump_length_mm = 196.0f;
    prof.hump_start_mm  = 400.0f;
    prof.wheelbase_mm   = IMU_TERRAIN_WHEELBASE_MM;
    prof.speed_mm_s     = 120.0f;
    for (uint32_t i = 0u; (i < 1500u) && (IMU_TERRAIN_CLIMBING !=
         imu_terrain_get_state(&mon.terrain)); i++)
    {
        run(&mon, &sim, &prof, 1u, 1, &sc);
    }
    run(&mon, &sim, &prof, 30u, 1, &sc);
    imu_terrain_hump_t lost;
    int const b_closed = imu_monitor_close_hump(&mon, &lost) ? 1 : 0;
    int const b_again  = imu_monitor_close_hump(&mon, &lost) ? 1 : 0;
    int const b_level  = (IMU_TERRAIN_LEVEL ==
                          imu_terrain_get_state(&mon.terrain));
    g_no_gyro = 0;
    check("H-A17", b_closed && !b_again && b_level && lost.b_cut_short &&
                   (lost.peak_mm > 2.0f) &&
                   (1u == imu_monitor_get_hump_count(&mon)),
          "sensor lost on a hump: reported once, cut short, peak kept");
}

int main (void)
{
    printf("[TEST] === host test run start ===\n");
    test_math();
    test_bitpack();
    test_pack();
    test_odom();
    test_calibration();
    test_persistence();
    test_impact();
    test_motion();
    test_hump_accuracy();
    test_hump_variants();
    test_accel_only();
    test_ramp();
    test_rear_flat();
    printf("[TEST] host SUMMARY %d passed, %d failed\n", g_pass, g_fail);

    return (0 == g_fail) ? 0 : 1;
}
