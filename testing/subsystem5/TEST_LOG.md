# Subsystem 5 — test log

**Subsystem:** Obstacle Scan & Avoidance  ·  **Student ID:** 2502821  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | Echo width to cm: rounding, clamp to 400 cm | Minimum-bit data (9-bit distance) | auto | `subsystem5_test.c` | `[TEST] S5-U01 PASS` |
| S5-U02 | 9-bit profile slots in one `uint64_t`: saturate at 511, no overlap | Minimum-bit data | auto | `subsystem5_test.c` | `[TEST] S5-U02 PASS` |
| S5-U03 | Median of 3 rejects one stray reading | Scan robustness | auto | `subsystem5_test.c` | `[TEST] S5-U03 PASS` |
| S5-U04 | Side choice: bottle left → RIGHT, bottle right → LEFT, wall right → LEFT, boxed in → NONE | Direction decision, IF-11 | auto | `subsystem5_test.c` | `[TEST] S5-U04 PASS` |
| S5-U05 | Edge step and sideways distance, half-width limited to OBS_MIN/MAX_HALF_WIDTH_CM (25 cm, edge 37.5° → 18 cm; edge 7.5° → 16 cm) | Bypass distance, IF-11 | auto | `subsystem5_test.c` | `[TEST] S5-U05 PASS` |
| S5-U06 | Bypass plan (side + out/past/back, 2 + 3 × 9 = 29 bits) packs through Navigation and unpacks unchanged | Minimum-bit data, IF-11 | auto | `subsystem5_test.c` | `[TEST] S5-U06 PASS` |
| S5-U07 | Ultrasonic answers (not silent) | Sensor wiring GP16/GP17 | auto (hardware) | `subsystem5_test.c`, sensor on Grove 4 | `[TEST] S5-U07 PASS` |
| S5-U08 | Three legs: out = sideways / sin 45° + OBS_OUT_EXTRA_CM (capped), past never 0, back = out leg's sideways / sin 30° + slack | Bypass plan, IF-11 | auto | `subsystem5_test.c` | `[TEST] S5-U08 PASS` |
| S5-U09 | Pivot angle never below OBS_PIVOT_MIN_STEP, capped at MAX; encoder arc for 45° | Bypass pivots, IF-06, IF-09 | auto | `subsystem5_test.c` | `[TEST] S5-U09 PASS` |
| S5-U10 | Line search zig-zag targets +15, −30, +45 … −90°, arc for a 165° sweep | Line rejoin after bypass | auto | `subsystem5_test.c` | `[TEST] S5-U10 PASS` |
| S5-U11 | Obstacle width: front-face test, half-width with beam correction, sum (+ trim) | Demo rubric "measure obstacle width" | auto | `subsystem5_test.c` | `[TEST] S5-U11 PASS` |
| S5-M01 | Servo sweeps the coarse angles 30–150° | GP12, PWM slice 6 | manual | 1. Power car 2. Watch servo at start of scan | Servo turns to each coarse angle |
| S5-M02 | Stops for an obstacle < 15 cm (TRIGGER_CM, 2 pings) | IF-11 `nav_post_obstacle` | manual | 1. Car line-following 2. Box on the line | `[S5] OBSTACLE at N cm`, N < 15, car brakes (nav state OBSTACLE) |
| S5-M03 | Goes round the open side and rejoins the line | IF-11 plan, IF-09 StopTurn | manual | 1. Box on a straight bit of line 2. Run, repeat | `[S5] PLAN`, four StopTurns, `bypass done`, car line-following again |
| S5-M04 | Sensor unplugged raises NO_ECHO | IF-12 fault bit 5 | manual | 1. Unplug ECHO 2. Run | After 10 silent pings: fault bit 5, car stops |
| S5-M05 | Measures the obstacle's width and sends it to the dashboard | Demo rubric "measure obstacle width", EVT_OBSTACLE | manual | 1. Box of known width square to the car on a straight, nothing within ~40 cm either side 2. Run into it | `[S5] WIDTH: N cm` within ±1.5 cm of the real width, both edges seen (no "at least"); Detections panel shows it |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-20 | S5-M02 | `653b797` | Ruien | PASS | `logs/2026-11-20_m02_m03_bypass_runs.txt` | 6 of 6 triggers at 8–14 cm |
| 2026-11-20 | S5-M03 | `653b797` | Ruien | PASS | `logs/2026-11-20_m02_m03_bypass_runs.txt` | 5 of 6 rejoined the line on a straight; run F (box not straight ahead, likely a bend) lost the line: known limit |
| 2026-11-23 | S5-M05 | `891f82c` | Ruien | FAIL | `logs/2026-11-23_m05_width_runs.txt` run 1 | 10.5 cm box read "at least 24.9": beam spread + clutter on the left |
| 2026-11-23 | S5-M05 | `8c5f6bc` + beam 10 (local) | Ruien | FAIL | `logs/2026-11-23_m05_width_runs.txt` run 2 | 12.9 cm box read 10.9; used to set OBS_WIDTH_BEAM_DEG 7 (`4fcf024`) |
| 2026-11-23 | S5-M05 | `4fcf024` | Ruien | FAIL | `logs/2026-11-23_m05_width_runs.txt` runs 3–4 | 15 cm box read "at least 21.1": left edge seen to 60°, something beside the box. Retest with clear sides |
