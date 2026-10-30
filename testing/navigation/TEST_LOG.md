# Navigation — test log

**Subsystem:** Navigation / Vehicle Controller  ·  **Owner:** Glenn  ·  **Code:** `app_program/navigation/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| NAV-U01 | Every input rejects out-of-range values | IF-10 to IF-13 | auto | `navigation_test.c` | `[TEST] NAV-U01 PASS` |
| NAV-U02 | Navigation starts IDLE with no faults | IF-14 | auto | `navigation_test.c` | `[TEST] NAV-U02 PASS` |
| NAV-U03 | Bypass plan golden vectors: min, max, mid, clamp | IF-11 | auto | `navigation_test.c` | `[TEST] NAV-U03 PASS` |
| NAV-M01 | Scripted mission on a stand: START; barcodes B, D, C at junctions; A with no junction; obstacle and bypass; STOP and START; ESTOP and CLEAR | FR2, IF-07 to IF-14 | manual | `make TEST=1` with `NAV_MANUAL_M01` = 1 (`navigation_test.c`) and `S2_MANUAL_TEST` = `S2_MANUAL_NONE` (`subsystem2_test.c`). Car on a stand, USB power only. The test task plays Wing, Ruien, Rifa and Andrew | Every `[NAV-M01]` line PASS, then `done: 29 passed, 0 failed`. Watch: B pivots right, D pivots twice, C drives on without stopping, the car slows after each barcode |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | NAV-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-30 | NAV-M01 | | Glenn | PASS | `logs/2026-10-30_nav_m01.txt` | 29 passed, 0 failed, on a stand, USB power. Longest task_motion step 555 µs (period 10000). Earlier runs that day failed with a Motion stall: the right encoder's Grove 7 ground wire was loose, not a code fault |
