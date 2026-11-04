# Remote (Wi-Fi / MQTT) — test log

Unit tests of `app_program/remote/`. START / STOP / CLEAR reaching Navigation is a system test (SYS-11): a test build must not drive the car on its own.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| RE-U01 | pings, text, unknown commands and broadcasts are all consumed; the task keeps running | IF-15 | auto | `remote_test.c` | `[TEST] RE-U01 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
