# Comms and network bridge — test log

Unit tests of `app_program/comms/` and the pure helpers of `app_program/net/`: the parts that need no network. Connecting and recovering are system tests (`system/TEST_LOG.md`).

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| CO-U01 | a command parses into id and arguments | protocol.h | auto | `comms_test.c` | `[TEST] CO-U01 PASS` |
| CO-U02 | whitespace, commas and the full 32-bit range are accepted | protocol.h | auto | `comms_test.c` | `[TEST] CO-U02 PASS` |
| CO-U03 | text, bad ids, overflow and too many arguments are rejected | protocol.h | auto | `comms_test.c` | `[TEST] CO-U03 PASS` |
| CO-U04 | 64-bit numbers format as decimal (0 and 2^64-1) | protocol.h | auto | `comms_test.c` | `[TEST] CO-U04 PASS` |
| CO-U05 | packed telemetry equals the dashboard's golden sample | IF-05 | auto | `comms_test.c`; the dashboard checks the same value in `npm test` | `[TEST] CO-U05 PASS` |
| CO-U06 | backoff doubles to its limit, with jitter in [step/2, step] | recovery | auto | `comms_test.c` | `[TEST] CO-U06 PASS` |
| CO-U07 | backoff restarts from the minimum after a reset | recovery | auto | `comms_test.c` | `[TEST] CO-U07 PASS` |
| CO-U08 | an over-long payload or topic is refused, not cut | IF-03 | auto | `comms_test.c` | `[TEST] CO-U08 PASS` |
| CO-U09 | subscription filters with `+` and `#` match like a broker | IF-01, IF-02 | auto | `comms_test.c` | `[TEST] CO-U09 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
