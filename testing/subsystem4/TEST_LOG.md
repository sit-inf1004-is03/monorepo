# Subsystem 4 — test log

**Subsystem:** IMU-Based Motion and Terrain Monitoring (Buddy 4)  ·  **Student ID:** 2501298  ·  **Code:** `app_program/subsystem4/`, `app_program/common/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

### On-target, automatic (`make TEST=1`, `subsystem4_test.c`)

Needs the IMU on GP2/GP3 (GY-511 / LSM303D as fitted, or an MPU-9250) and the car standing still on a level bench.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | Sensor found on I2C1 and calibrated | Week 6 6.3.3 (init), IF-S4-4 READY_IMU | auto | `subsystem4_test.c` | `[TEST] S4-U01 PASS` within 3 s of boot |
| S4-U02 | 10 ms period held, no overrun, worst cycle < 2 ms | Task table (10 ms), Tribunal 2.2.4 timing | auto | counts cycles over 1 s | 97–103 cycles, 0 overruns, max exec < 2000 µs |
| S4-U03 | Still car reads STATIONARY, LEVEL, \|pitch\| < 2°, no ESTOP | Figure 5 initial states | auto | reads the snapshot word | PASS |
| S4-U04 | Snapshot golden vector identical on target and host | IF-S4-1 layout (T3) | auto | packs host vector 2 | `0x054740FD` |
| S4-U05 | Fault flag: 2 tasks + 1 handler toggle own bits 20 000×, no lost/phantom bits, real faults untouched | G1, IR-3, test T1 | auto | stress test, bits 28–30 | 0 errors on every writer |
| S4-U06 | Simulated impact gives an IMPACT event quickly (telemetry only, no stop) | test T6 | auto | `imu_task_test_inject_shock()`, `imu_task_read_event()` | IMPACT event within 50 ms |
| S4-U07 | After an impact the IMU keeps running and nothing is latched, in S4 or in Navigation | Figure 5 "Recovered", IF-12 | auto | waits 1.5 s | READY_IMU set, no IMU fault, `NAV_FLT_IMU` clear, STATIONARY |
| S4-U08 | Hump algorithm on the RP2040 soft-float, 50 mm simulated hump | FR3, NFR4 | auto | `imu_sim` through a private monitor | peak within ±10 %, < 0.5 ms per sample |
| S4-U09 | 32-bit encoder wrap and clamp helpers on target | IF-06 | auto | `imu_odom_delta(0x00000002, 0xFFFFFFFE)` | 4 |

### On-target, manual

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-M01 | Mounting / sign check | `IMU_MOUNT_*` in `imu_filter.h` | manual | Set `IMU_TASK_TRACE_ENABLE` 1. Lift the nose ~10°, then the left side ~10°, then turn the car left by hand. GY-511: also spin the car a full turn and note min/max of `mag raw` per axis. | pitch +, roll +, yaw rate + (fix `IMU_MOUNT_*` if not); GY-511: `IMU_MAG_OFFSET_*` = (max + min) / 2 per axis |
| S4-M02 | End-to-end hump height (test T4) | FR3, NFR4, IR-8 | manual | Measure 3 humps with a ruler. Drive over each 5× at 2 speeds with Motion registering its encoder words. | `imu: HUMP peak` within ±20 % (set `IMU_TERRAIN_HEIGHT_GAIN` / `IMU_TERRAIN_WHEELBASE_MM` from results); packed = float ±1 mm |
| S4-M03 | Tap / collision test | Week 6 6.3.2 shock, FAULTS_ESTOP | manual | Tap the chassis lightly, then push the car into a box at cruise speed. | light tap: no impact; collision: `imu: IMPACT`, ESTOP, recovery after ~0.5 s still |
| S4-M04 | I2C fault and recovery (G5) | Week 6 6.3.3 read failure, fault bit 3 | manual | Pull SDA mid-run, wait 3 s, plug it back. | fault bit 3 within 30 ms, `imu: FAULT reason 4`, READY_IMU cleared; after re-plug `IMU_OK`, re-calibration, bit 3 cleared |
| S4-M05 | Classification with motors running | Figure 5 motion region | manual | Run the car: stand, straight, pivot, stop. | STATIONARY → ACCELERATING → CRUISING → TURNING → STATIONARY events |
| S4-M06 | Course completed | Figure 5 final state, FR6 | manual | Call `imu_task_signal_course_done()` after a run. | `COURSE_DONE` event with run peak and hump count; task idles |

### PC (host) tests, `host_test/run_host_tests.sh`

Same C sources as the firmware, compiled with a PC compiler and fed simulated sensor data.

| ID | What is tested | Requirement | Expected result |
|---|---|---|---|
| H-U01..05 | `imu_math` accuracy (sqrt, atan2, sin, round, isfinite) | no libm in the link | errors below the stated limits |
| H-U06..07 | `bitpack` signed/unsigned fields, clamp, insert/extract | minimum-bit rule, clamp-then-pack | round-trip and saturate, widths 2–31 |
| H-U08..12 | Snapshot / aux / event layouts (golden vectors) | IF-S4-1..3, T3 | every field round-trips at min/max/mid |
| H-U13..15 | Encoder wrap, glitch rejection, fallback speed | Tribunal 2.3/2.4 | 0xFFE→0x002 = 4 pulses |
| H-U16..19 | Calibration, gyro bias, restart if moving | Week 6 6.3 | bias within 0.1 °/s |
| H-U20..21 | Persistence (3-sample spike), tilt | Week 6 6.3.2 rule 1 | no false hump; tilt after 5 samples |
| H-U22..25 | Impact detection, recovery, hump suppression, hold-off | Week 6 6.3.2 rules 2–3, Tribunal 1.1 | as described |
| H-U26..31 | Motion classes; acceleration not read as climbing; drift removed at stops | Week 6 6.3.1, Tribunal 1.2 "accelerometer-only pitch" | as described |
| H-U32..34 | Climb then level closes the hump after one wheelbase, or 1 s with no distance; a short flat top then nose-down still goes through DESCENDING | car never tips nose down on the hump | as described |
| H-U35..37, H-A13 | Ramp detection (`imu.c`): on after 60 ms at 6°, off after 100 ms at 4°, spike ignored, cleared when the sensor is lost; simulated slope through the monitor | IF-S4-7 | as described |
| H-I01..05 | Hump detection and peak accuracy, 42 profiles (15–120 mm, 150–1000 mm, slopes 6–35°, 100–300 mm/s) | FR3, NFR4 | detected once each, error ≤ 10 % (NFR4: 20 %), sequence Level→Climbing→Descending→HumpClosed→Level, saturation at 127 mm |
| H-I06..09 | Wheelbase ±20 mm, no encoder, sub-threshold undulation, gyro clipping recovery | NFR4, G5-style robustness | within NFR4; no lock-up |
| H-A01 | No gyro (GY-511): calibrates; 3 s still reads level, no turn, no hump | Week 6 6.3 | as described |
| H-A02..04 | No gyro: H-I01 hump grid at steady speed | FR3, NFR4 | every hump detected once; peak within ±20 %; Level→Climbing→Descending→HumpClosed→Level |
| H-A05 | No gyro: start-up acceleration on flat ground | 400 ms settled gate | no hump opened (fails without the gate) |
| H-A06..08 | No gyro: turning from the compass | Figure 5 motion region (deviation: Tribunal 1 weak idea 3) | 5 s straight never TURNING; 90 °/s turn classed TURNING within 0.2 s; heading within 10° after a 90° turn |
| H-A09 | No gyro: +3 g shock | Week 6 6.3.2 shock | ESTOP on the first sample, then Recovered |
| H-A10 | Board mounted upside down (level roll near ±180°), 0.87 g, left side lifted 25° | S4-M01 finding, 28 Oct | roll error < 5°, no 360° jump, yaw rate < 20 °/s |
| H-A11 | 0.87 g sensor, still, calibrating | Week 6 6.3 (calibration) | calibrated in 1 s, not forced |
| H-A12 | 0.87 g sensor pushed along the floor during calibration (no gyro) | Week 6 6.3 (restart if moving) | restarts; level reference within 0.5° |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-24 | H-U01..H-U31, H-I01..H-I09 (40 checks) | not yet committed | Andrew (with AI assistance) | PASS (40/40) | `logs/2026-10-24_host_tests.txt` | PC build, gcc 16.1 -std=c99 -Wall -Wextra; worst hump error 5.7 % |
| 2026-10-24 | build | not yet committed | Andrew (with AI assistance) | PASS | `logs/2026-10-24_build.txt` | arm-none-eabi-gcc 14.2.1: 5 build variants link; subsystem4 + common compile with no warnings under -Wall -Wextra and 12 more |
| YYYY-MM-DD | S4-U01..S4-U09 | | | | | not run yet: needs the IMU hardware |
| 2026-10-28 | S4-U01..S4-U09 (first run, GY-511, trace off) | not yet committed | Andrew (with AI assistance) | FAIL (5/9) | `logs/2026-10-28_target_tests_first_run.txt` | U01, U03, U04, U05, U09 pass. U02: "max exec 30096 us" included the one-time sensor probe (start-up delays), not a sampling cycle. U06/U07: the test shock added +3 g to the up axis, but on the upside-down board that axis reads −0.86 g, so \|a\| only rose ~1.46 g (< 1.5 g limit); no ESTOP. U08: peak 48.0 mm (4 %), but worst step 833 µs > 0.5 ms |
| 2026-10-28 | S4-U01..S4-U09 (after test fixes) | not yet committed | Andrew (with AI assistance) | FAIL (8/9) | `logs/2026-10-28_target_tests_after_fixes.txt` | Fixes: max_exec_us counts sampling cycles only; test shock added in the direction gravity reads. U02 PASS (101 cycles/s, 0 overruns, worst 1839 µs, close to the 2 ms limit). U06 PASS (`IMPACT 3.4 g`, waiter woke after 7.7 ms). U07 PASS. U08 still FAIL: peak 48.0 mm, mean 278 µs, worst 898 µs at step 464. Wall-clock timings include interrupts and the other core (Wi-Fi) sharing the flash cache; cause of the spikes not yet separated |
| 2026-10-27 | bench: sensor identity | not yet committed | Andrew (with AI assistance) | INFO | `Project/imu_i2c_scan` console output | Fitted board is a GY-511: LSM303D at 0x1E, WHO_AM_I 0x49, no gyro. 24 Oct firmware gave `FAULT reason 1` (only probed 0x68/0x69). Code reworked for it |
| 2026-10-28 | H-U01..H-U31, H-I01..H-I09, H-A01..H-A09 (49 checks) | not yet committed | Andrew (with AI assistance) | PASS (49/49) | `logs/2026-10-28_host_tests.txt` | PC build, gcc 16.1; worst hump error 5.7 % with gyro, 11.2 % without (GY-511 mode, steady speed) |
| 2026-10-28 | bench: sensor on I2C1 | not yet committed | Andrew (with AI assistance) | INFO | `logs/2026-10-28_i2c_scan_no_answer.txt` | Firmware reported `FAULT reason 1`; scanner showed SDA/SCL high but no answer (wiring fault). After rewiring: LSM303D found at 0x1D, on the next boot at 0x1E (SA0 pin possibly not firmly connected) |
| 2026-10-28 | S4-M01 (signs) | not yet committed | Andrew (with AI assistance) | FAIL | `logs/2026-10-28_S4-M01_before_wrap_fix.txt` | Board mounted upside down on the car. Nose up: pitch +27° (correct). Left side up: roll printed −335° (= +25° unwrapped) with false yaw spikes to −1043 °/s and TURNING. Left turn: yaw +, heading 6°→85° (correct). Cause: roll offset ~178° not wrapped; fixed in `imu_filter.c`, test H-A10 added |
| 2026-10-28 | H-U01..H-U31, H-I01..H-I09, H-A01..H-A10 (50 checks) | not yet committed | Andrew (with AI assistance) | PASS (50/50) | `logs/2026-10-28_host_tests_wrap_fix.txt` | H-A10 failed before the fix (roll error 361°, yaw 345 °/s), passes after (1.3°, 0.7 °/s). Hump results unchanged |
| 2026-10-28 | S4-M01 (signs, retest) | not yet committed | Andrew (with AI assistance) | PASS | `logs/2026-10-28_S4-M01_after_wrap_fix.txt` | Left side up: roll +21°; right side up: roll −28°; yaw stayed within ±4 °/s while tilting. Quick left turn: +48 °/s, `motion 3` TURNING. `IMU_MOUNT_*` unchanged. Compass offsets (`IMU_MAG_OFFSET_*`) still to measure. No encoders connected, so the bench lift was read as a hump (fallback speed; expected) |
| 2026-10-28 | H-* (52 checks, adds H-A11, H-A12) | not yet committed | Andrew (with AI assistance) | PASS (52/52) | `logs/2026-10-28_host_tests_calib_fix.txt` | Calibration stillness now judged by change from the first sample, not by \|a\| = 1 g. H-A11/H-A12 failed on the old code (always forced; push leaked 1.12° into level), pass now. On the car: `calibrated (forced=0)` |
| 2026-10-28 | S4-M01 (compass offsets) | not yet committed | Andrew (with AI assistance) | PASS | `logs/2026-10-28_compass_spin_and_tap.txt` | Flat turn by hand, more than one full circle: `mag raw` X −3490..1308, Y −5067..1878 → `IMU_MAG_OFFSET_X` −1091, `_Y` −1594 (`_Z` 0: not measurable from a flat turn). Half-ranges X 2399, Y 3472, so there is also soft-iron distortion (ellipse ~1.45:1), not corrected. TURNING shown during the faster parts of the turn |
| 2026-10-28 | H-* (52 checks) | not yet committed | Andrew (with AI assistance) | PASS (52/52) | `logs/2026-10-28_host_tests_mag_offsets.txt` | `imu_sim` now adds `IMU_MAG_OFFSET_*` to its compass output (the car's own field), so H-A07/H-A08 check the offsets are removed |
| 2026-10-28 | S4-M03 (tap, first try) | not yet committed | Andrew (with AI assistance) | FAIL (hardware) | `logs/2026-10-28_compass_spin_and_tap.txt` | Firm tap: `IMPACT 4.9 g -> ESTOP` on time, but in the same instant `FAULT reason 4` (I2C reads failing): the knock disturbed the sensor's connection. Firmware recovered by itself (re-found at 0x1E after 1 s, re-calibrated, ESTOP cleared), as designed for G5. Also `IMPACT 1.1 g` when the turn was stopped sharply by hand (forward-jolt limit 1 g). Wiring must be secured before S4-M02/M03 |
| 2026-10-28 | S4-M03 (taps, retest) | not yet committed | Andrew (with AI assistance) | PARTIAL | `logs/2026-10-28_tap_retest.txt` | 5 taps on the chassis, wiring not changed: every one gave `IMPACT -> ESTOP` and `recovered` 0.5 s later, and no `FAULT` (largest 2.8 g), so the earlier fault came from the harder 4.9 g knock. Light taps also stopped the car (printed 0.5–1.4 g): they pass the forward-jolt rule (> 1 g); the printed value is only the \|a\| − 1 g part. Expected "light tap: no impact" not met; limits to be set from the collision run with motors |
| 2026-10-28 | S4-M03 (taps on the front, third run) | not yet committed | Andrew (with AI assistance) | INFO | `logs/2026-10-28_tap_light_medium_firm.txt` | Light, medium and firm tap on the front of the chassis: no `IMPACT` this time, only vibration (`motion 2`). Hand taps land on both sides of the 1 g forward limit, so they cannot set the limits. Peak sizes not visible (trace is every 250 ms) |
| 2026-10-28 | S4-M03 (car pushed to roll into a box) | not yet committed | Andrew (with AI assistance) | PARTIAL | `logs/2026-10-28_roll_into_box.txt` | 2–3 runs: one hit gave `IMPACT 1.0 g -> ESTOP` and `recovered` 0.5 s later; one run (≈12.3 s, −0.28 g in the sampled line) did not trigger. Added a debug-only `imu: peak mg dev .. fwd ..` trace line (largest jolt since the last line) to size each hit next time |
| 2026-10-28 | S4-M03 (roll into a box, with peak trace) | not yet committed | Andrew (with AI assistance) | PASS (bench) | `logs/2026-10-28_roll_into_box_peaks.txt` | Firmware with compass offsets + peak trace, `calibrated (forced=0)`. 4 pushes, harder each time. Peak forward jolt / \|a\| change: 330/84 mg and 318/133 mg (gentle, no impact); 1942/1202 mg and 1471/788 mg (firm, `IMPACT -> ESTOP`, `recovered` 0.5 s later). Still car: ≤ 40/90 mg. The 1 g forward limit sits in the gap between gentle and firm hits. Still to do with motors running: collision at cruise speed and normal driving bumps |
| 2026-11-04 | H-* (55 checks, adds H-U32..34 to this copy) | not yet committed | Andrew (with AI assistance) | PASS (55/55) | `logs/2026-11-04_host_tests_nav_wiring.txt` | On top of origin/main cf382fd, with the IF-06 encoder getter (0.330 mm/pulse) and the IF-12 Navigation calls. Host tests do not compile `imu_task.c` |
| 2026-11-04 | build | not yet committed | Andrew (with AI assistance) | PARTIAL | none | This branch: all subsystem4 objects compile with no warnings against main's `navigation.h`, but the image does not build, because origin/main fails in subsystem2/subsystem3 (stray `#endif` in `subsystem3.h`, example `task_producer.c` and `task_consumer.c`). Scratch tree of origin/integration/buddy5-bypass 4c57f7b plus this subsystem4: normal and TEST=1 link with no warnings, with that branch's `imu.c` replaced by a stub. Not run on hardware |
| YYYY-MM-DD | S4-U01..S4-U09, sensor unplug (`NAV_FLT_IMU` raised, car keeps driving, cleared on reconnect) | | | | | not run yet: needs an image that builds |
| 2026-11-04 | H-* (59 checks, adds H-U35..37, H-A13) | not yet committed | Andrew (with AI assistance) | PASS (59/59) | `logs/2026-11-04_host_tests_ramp_integration.txt` | After merging integration/buddy5-bypass 925d3ee. `imu.c` is now host tested; it takes pitch and roll from `imu_task` instead of driving the sensor itself |
| 2026-11-04 | build | not yet committed | Andrew (with AI assistance) | PASS | none | Normal and TEST=1 images build and link on buddy4-imu with no warnings. `DEVCNF_USE_IIC` back to 0, so `imu_task` is the only I2C1 user. Not run on hardware |
| YYYY-MM-DD | ramp on the car (`imu_on_ramp()` on the ramp, clear on flat, no false ramp when pulling away), with motors running at the I2C bus speed of 400 kHz | | | | | not run yet |
