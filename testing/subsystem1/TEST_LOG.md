# Subsystem 1 — test log

**Subsystem:** Wi-Fi / MQTT  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

Subsystem 1's tests are kept per module, with their own ID prefixes:
`comms/TEST_LOG.md` (CO-), `remote/TEST_LOG.md` (RE-) and
`telemetry/TEST_LOG.md` (TL-). The S1- tests below belonged to the
template's example LED heartbeat, which has been removed; their IDs are
kept and not reused.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | *retired:* cyclic handler is running (template example, removed) | | auto | `subsystem1_test.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | *retired:* LED task sleeps between ticks (not busy) (template example, removed) | | auto | `subsystem1_test.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | *retired:* LED on GP19 toggles every 250 ms (template example, removed) | | manual | 1. Flash normal image. 2. Watch LED / measure with scope. | 2 Hz blink, 50 % duty |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
