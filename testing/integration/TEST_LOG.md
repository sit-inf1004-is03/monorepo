# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | *retired:* samples flow from S2 to S3 (≥ 5 in 2 s) (template example, removed) | IF-01, IF-02 | auto | `subsystem3_test.c` | `[TEST] INT-01 PASS` |
| INT-02 | *retired:* summary lines flow from S3 to S4 (≥ 2 in 3.5 s) (template example, removed) | IF-03 | auto | `subsystem4_test.c` | `[TEST] INT-02 PASS` |
| INT-15 | a setting from the dashboard's config topic changes the telemetry period | IF-15 | auto | `telemetry_test.c` | `[TEST] INT-15 PASS` |
| INT-16 | a telemetry sample carries Navigation, Motion and line-sensor values | IF-06, IF-09, IF-14, IF-16 | auto | `telemetry_test.c` | `[TEST] INT-16 PASS` |
| INT-17 | car and dashboard agree on the telemetry bit layout | IF-15 | auto | CO-U05 and CO-U10 on the car + `npm test` in `mqtt-web-interface` | both pass with the same golden value |
| INT-18 | *retired:* telemetry now reads `barcode_get_last()` (INT-20) instead of S3's log line. Was: a barcode read logged through `comms_log()` shows in telemetry word 1, as a new read | IF-17, IF-18 | auto | `telemetry_test.c` | `[TEST] INT-18 PASS` |
| INT-19 | telemetry word 1 holds the IMU's motion class and terrain, whether its data is live, and the distance from the encoders | IF-06, IF-S4-1, IF-S4-7 | auto | `telemetry_test.c` | `[TEST] INT-19 PASS` |
| INT-20 | telemetry word 1 holds the letter and read count that Line Following's `barcode_get_last()` reports | IF-18 | auto | `telemetry_test.c` | `[TEST] INT-20 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Interface coverage

Every interface in TEAM.md section 5 and the tests that exercise it,
including unit, manual and system tests from the other logs. "Simulated"
means the test plays the other side itself, so the two real subsystems
have not yet been tested together through that interface.

| Interface | Tests | Gap |
|---|---|---|
| IF-01 .. IF-05 | INT-01, INT-02 (*retired* with the template example) | none: interfaces retired |
| IF-06 encoder counts and speed | S2-U04, S4-U09, INT-16, INT-19 (distance), SYS-12 | |
| IF-07 Motion commands and replies | S2-M03, NAV-M01 | |
| IF-08 turn level | none | no test yet |
| IF-09 StopTurn, motion activity | S2-M03, S5-M03, NAV-M01, INT-16 (activity) | |
| IF-10 Line Following / Barcode → Navigation | NAV-U01, NAV-M01 (simulated) | no test with the real Line Following |
| IF-11 Obstacle Scan → Navigation | S5-U04 .. S5-U06, NAV-U01, NAV-U03, NAV-M01 (simulated), S5-M02, S5-M03 | |
| IF-12 fault bits | NAV-M01 (IMU degrade, e-stop, CLEAR), S4-U07, S5-M04 (no echo), SYS-13 (Wi-Fi loss) | |
| IF-13 remote commands → Navigation | NAV-U01, NAV-M01 (simulated), SYS-11 | |
| IF-14 Navigation state, faults, reason | NAV-U02, INT-16 | |
| IF-15 dashboard ↔ car | INT-15, INT-17, CO-U05, CO-U09, CO-U10, RE-U01, SYS-10, SYS-11, SYS-15 | |
| IF-16 line state, obstacle distance → telemetry | INT-16 (line state), SYS-12 | the obstacle distance is checked only by hand (SYS-12) |
| IF-17 comms send functions | CO-U08, SYS-10 | |
| IF-18 last barcode read → telemetry | INT-20, SYS-15 | INT-20 checks that word 1 copies `barcode_get_last()`; only SYS-15 reads a real barcode |
| IF-19 barcode map, telemetry → Line Following | TL-U05 | a turn taken from a map set by the dashboard is checked only on the car |
| IF-S4-1 IMU snapshot word | S4-U04, H-U08 .. H-U12 (host), INT-19, SYS-15 | |
| IF-S4-2 IMU aux word | H-U08 .. H-U12 (host) | nothing reads it yet; its layout is tested on the host only |
| IF-S4-3 IMU event stream | S4-U06, H-U08 .. H-U12 (host), TL-U03, SYS-15 | a real HUMP event becoming an EVT_HUMP message is checked only by hand (SYS-15) |
| IF-S4-4 IMU fault to Navigation | S4-U01, S4-M04, NAV-M01 (IMU degrade and recovery) | |
| IF-S4-5 encoder counts to the IMU | S4-U09, H-U13 .. H-U15 (host) | |
| IF-S4-6 course done, restart | S4-M06 | |
| IF-S4-7 ramp, tilt, IMU OK | H-U35 .. H-U37, H-A13 .. H-A15 (host), INT-19 (`imu_is_ok()`) | no test with the real Line Following |

Add a row here with every new interface, and a test ID to it when the test
is written.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
