# Subsystem 2 — test log

**Subsystem:** Motion control (example producer still in place)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | sample values stay within 0..100 | | auto | `subsystem2_test.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | free slots and free blocks within pool size | IF-02 | auto | `subsystem2_test.c` | `[TEST] S2-U02 PASS` |
| S2-U03 | encoder pulses convert to mm (0.330 mm/pulse, 637 per revolution) | | auto | `subsystem2_test.c` | `[TEST] S2-U03 PASS` |
| S2-U04 | still wheels: counts steady, speed 0 | IF-06 | auto | `subsystem2_test.c`, car on the bench, motors off | `[TEST] S2-U04 PASS` |
| S2-U05 | GPIO dispatcher refuses a second handler on GP5 | | auto | `subsystem2_test.c` | `[TEST] S2-U05 PASS` |
| S2-U06 | Encoder timer counts at 1 MHz (+/- 2 %) | | auto | `subsystem2_test.c`: timer ticks over a 1000 ms kernel delay; the `S2-U06 note` line prints the count | `[TEST] S2-U06 PASS` |
| S2-U07 | PID proportional term: 2 × (100 − 40) = 120 | | auto | `subsystem2_test.c` | `[TEST] S2-U07 PASS` |
| S2-U08 | PID saturates at its limit without integral windup | | auto | `subsystem2_test.c` | `[TEST] S2-U08 PASS` |
| S2-U09 | PID reset clears memory, keeps gains | | auto | `subsystem2_test.c` | `[TEST] S2-U09 PASS` |
| S2-U10 | Encoder mm-to-pulses (1000 mm = 3030) and pulses-to-µm | | auto | `subsystem2_test.c` | `[TEST] S2-U10 PASS` |
| S2-M01 | Encoder pulses per wheel revolution (test plan **B2**) | A2 | manual | `make TEST=1`, car on a stand, motors off. Follow the `[B2]` prompts: turn each wheel one revolution forwards, then backwards, slowly, from tape mark to tape mark (`s2_test_b2.c`) | The four `[B2] result` lines show about the same count on the turned wheel (the pulses per revolution), 0 on the other and 0 rejected. Use that count for `DRV_ENCODER_PULSES_PER_REV` and the per-pulse distance and angle figures |
| S2-M02 | Motor wiring, direction and speed (rerun of test plan **B3** with `drv_motor`) | | manual | `make TEST=1` with `S2_MANUAL_TEST` = `S2_MANUAL_M02` in `subsystem2_test.c`. Car on a stand, battery on Vin. Each motor runs forwards 40 %, backwards 40 %, forwards 100 %, braking between (`s2_test_motor.c`) | Each motor turns only its own wheel (no `WARNING` line); "forwards" turns both wheels the car's forward way; brake stops the wheel within the 1 s; 0 rejected at 100 %. Record the full-speed pulses/s |
| S2-M03 | `task_motion` commands: straight moves and pivots | IF-07, IF-09 | manual | `make TEST=1` with `S2_MANUAL_TEST` = `S2_MANUAL_M03`. Car on a stand, USB power only. Four commands (`s2_test_motion.c`) | Steps 1–3 `status 0` (DONE); step 4 `status 1 reason 3` (timeout). Step 1 both wheels 1000 ± 10 mm, step 3 500 ± 10 mm; pivot ends within 0.2 s of `StopTurn sent`; speed trace settles near 300 mm/s; longest step well under 10000 µs |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S2-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-27 | S2-M01 | | Glenn | PASS | console, `[B2] result` lines | L 635 / 642, R 631 / 629 (fwd / back), 0 rejected, 1 stray L pulse in step 4. Mean 634 pulses per revolution; encoder is on the motor shaft. 50 µs chatter filter |
| 2026-10-27 | S2-M01 | | Glenn | PASS | console, `[B2] result` lines | Rerun: L 638 / 639, R 640 / 638 (fwd / back), 0 rejected. Stops by eye, so +/- 1 %. Mean of both runs (8 turns): 637 pulses per revolution, used for `DRV_ENCODER_PULSES_PER_REV` |
| 2026-10-27 | S2-U03 | | Glenn | PASS | `logs/2026-10-27_b2_encoder.txt` | 637 pulses per revolution, 0.330 mm/pulse |
| 2026-10-27 | S2-U04 | | Glenn | PASS | `logs/2026-10-27_b2_encoder.txt` | |
| 2026-10-27 | S2-U05 | | Glenn | PASS | `logs/2026-10-27_b2_encoder.txt` | |
| 2026-10-27 | S2-U06 | | Glenn | PASS | `logs/2026-10-27_b2_encoder.txt` | After unplug and replug: 1009448 ticks in 1000 ms. Tick started by `drv_encoder_init()`; before that fix a cold boot gave 23663 (FAIL) |
| 2026-10-27 | S2-M01 | | Glenn | PASS | `logs/2026-10-27_b2_encoder.txt` | Logged run: L 626 / 636, R 643 / 631 (fwd / back), 0 rejected, 1 stray L pulse in step 4. Mean 634; mean of all 12 turns today 636, within 1 % of the 637 in use, so 637 kept |
| 2026-10-27 | S2-M02 | | Glenn | PASS | `logs/2026-10-27_m02_motor.txt` | On a stand, USB power only (motors at about 4.6 V). Each motor moved only its own encoder (LEFT = M1, RIGHT = M2), no WARNING. 40 %: L 1066 / 1056, R 1024 / 1030 pulses/s (fwd / back). 100 %: L 2481, R 2501 pulses/s (about 0.82 m/s), 0 rejected. Brake stops within 1 s: 42–50 pulses from 40 %, 129–141 from 100 %. Directions checked by eye on the ground with the power bank: forwards, backwards, forwards on both wheels after setting `b_reversed` on the right motor |
| 2026-10-27 | S2-M03 | | Glenn | PASS (behaviour only) | none: on the ground, power bank, no USB console | All 4 steps behaved as expected. Step 1 drove straight about 1 m (measured roughly against a laptop); step 3 reversed; step 2 pivot left about 225° by eye, ending at StopTurn (~1.6 s), which puts the wheel spacing at about 115 mm; step 4 stopped by itself after the 5 s timeout. Still to do: logged run on a stand (USB only) for distances, speed trace and loop time |
