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
| S5-U05 | Fine scan edge step and lateral distance (25 cm, 45° → 40 cm) | Bypass distance, IF-11 | auto | `subsystem5_test.c` | `[TEST] S5-U05 PASS` |
| S5-U06 | Bypass plan packs into 16 bits and unpacks unchanged | Minimum-bit data, IF-11 | auto | `subsystem5_test.c` | `[TEST] S5-U06 PASS` |
| S5-U07 | Ultrasonic answers (not silent) | Sensor wiring GP16/GP17 | auto (hardware) | `subsystem5_test.c`, sensor on Grove 4 | `[TEST] S5-U07 PASS` |
| S5-M01 | Servo sweeps 0–180° | GP12, PWM slice 6 | manual | 1. Power car 2. Watch servo at start of scan | Servo turns to each coarse angle |
| S5-M02 | Stops for obstacle ≤ 30 cm | IF-11 `nav_post_obstacle` | manual | 1. Car line-following 2. Place bottle 25 cm ahead | `[S5]` obstacle print, nav state OBSTACLE |
| S5-M03 | Goes round the clear side | IF-11 plan, IF-09 StopTurn | manual | 1. Bottle offset left 2. Repeat offset right | Pivots to the open side, rejoins the line |
| S5-M04 | Sensor unplugged raises NO_ECHO | IF-12 fault bit 5 | manual | 1. Unplug ECHO 2. Run | After 10 silent pings: fault bit 5, car stops |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
