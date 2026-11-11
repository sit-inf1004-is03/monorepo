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
| INT-18 | a barcode read logged through `comms_log()` shows in telemetry word 1, as a new read | IF-17, IF-18 | auto | `telemetry_test.c` | `[TEST] INT-18 PASS` |
| INT-19 | telemetry word 1 holds the IMU's motion class and terrain, whether its data is live, and the distance from the encoders | IF-06, IF-S4-1, IF-S4-7 | auto | `telemetry_test.c` | `[TEST] INT-19 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
