# Telemetry (Wi-Fi / MQTT) — test log

Unit tests of `app_program/telemetry/`.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| TL-U01 | a period outside 100..60000 ms is refused | CFG_TELEMETRY_PERIOD | auto | `telemetry_test.c` | `[TEST] TL-U01 PASS` |
| TL-U02 | CFG_TELEMETRY_ENABLE 0 stops sending | CFG_TELEMETRY_ENABLE | auto | `telemetry_test.c` | `[TEST] TL-U02 PASS` |
| TL-U03 | an IMU HUMP event becomes `0 <mm> <over 127> <deg>`; other IMU events are not sent | EVT_HUMP, IF-S4-3 | auto | `telemetry_test.c` | `[TEST] TL-U03 PASS` |
| TL-U04 | the barcode letter is taken only from S3's `[BC] read *X*` line (also reversed reads), not from other lines | EVT_BARCODE, IF-18 | auto | `telemetry_test.c` | `[TEST] TL-U04 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
