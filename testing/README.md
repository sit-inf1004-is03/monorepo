# Testing

All test records for the project live in this folder. A test only counts as
done when it is **written down here with its result and a saved log**.

```
testing/
├── README.md              this guide
├── subsystem1/            one folder per subsystem, kept by its owner
│   ├── TEST_LOG.md        test cases + results
│   ├── logs/              saved console output, photos, scope captures
│   ├── comms/             Wi-Fi / MQTT: comms and the network bridge
│   ├── remote/            Wi-Fi / MQTT: dashboard commands to Navigation
│   └── telemetry/         Wi-Fi / MQTT: telemetry to the dashboard
├── ...
├── subsystem5/
├── navigation/            Navigation / Vehicle Controller
├── integration/           tests across two or more subsystems (shared)
└── system/                tests of the whole product against its requirements (shared)
```

| Folder | Code under test | Kept by |
|---|---|---|
| `subsystem1/` … `subsystem5/` | `app_program/subsystemN/` | the subsystem's owner |
| `navigation/` | `app_program/navigation/` | Navigation's owner |
| `subsystem1/comms/` | `app_program/subsystem1/comms/`, the pure helpers of `app_program/subsystem1/net/` | Wi-Fi / MQTT (Rifa) |
| `subsystem1/remote/` | `app_program/subsystem1/remote/` | Wi-Fi / MQTT (Rifa) |
| `subsystem1/telemetry/` | `app_program/subsystem1/telemetry/` | Wi-Fi / MQTT (Rifa) |
| `integration/`, `system/` | several modules at once | the whole team |

## Three levels of test

| Level | ID format | Tests what | Written by | Where |
|---|---|---|---|---|
| Unit | `S1-U01`, `S2-U01`, … | one subsystem on its own | the subsystem owner | `subsystemN/TEST_LOG.md` |
| Integration | `INT-01`, … | an interface between subsystems (TEAM.md section 5) | the owners on both sides | `integration/TEST_LOG.md` |
| System | `SYS-01`, … | the finished product against a requirement | whole team | `system/TEST_LOG.md` |

Modules that are not numbered subsystems use their own prefix: `NAV-`
(Navigation), `CO-` (comms), `RE-` (remote) and `TL-` (telemetry), e.g.
`CO-U05`. The letter after the prefix is `U` for a unit test and `M` for a
longer manual or scripted test, e.g. `S2-M01` or `NAV-M01`.

Test IDs never change and are never reused, even if a test is dropped.

## How to run and record a test

1. **Define it first.** Add a row to the *Test cases* table in the right
   `TEST_LOG.md`: ID, what it checks, how, and the expected result.
2. **Automate it if you can.** Add a `TEST_CHECK` with the same ID to
   `app_program/subsystemN/subsystemN_test.c` (see `app_program/test_report.h`).
   A test that needs eyes or instruments (an LED blinks, a motor turns, a
   scope shows 1 kHz) is a *manual* test. Write its steps in the table instead.
3. **Build the test image and flash it.**
   ```sh
   cd build_make
   make TEST=1 -j8            # -> mtk3pico_smp1_usb_cdc_wifi_test.uf2
   make NET=lwip TEST=1 -j8   # with Wi-Fi / MQTT -> mtk3pico_smp1_usb_cdc_wifi_lwip_test.uf2
   ```
   The test image runs every subsystem's tests at power-up and prints one
   `[TEST] <ID> PASS|FAIL` line per check. (A normal `make` leaves the tests out.)
   Both images run the same automatic tests; the comms, remote and telemetry
   tests need no network. Use the `NET=lwip` one when the car should also
   connect to the broker during the run (`MQTT.md`).
4. **Save the console output** to `logs/`, named `YYYY-MM-DD_<what>.txt`:
   ```sh
   # macOS (find the port name with: ls /dev/cu.usbmodem*)
   cat /dev/cu.usbmodem1101 | tee ../testing/subsystem1/logs/2026-10-14_unit.txt
   # Linux / WSL with usbipd
   cat /dev/ttyACM0 | tee ../testing/subsystem1/logs/2026-10-14_unit.txt
   ```
   Start the command first, then reset the Pico. Press Ctrl-C when it is done.
   On Windows with PuTTY, use *Session → Logging → All session output*.
   For manual tests, save photos or scope screenshots in `logs/` the same way
   (keep each file under a few MB). Videos go in `videos/` instead (see below).
5. **Record the result.** Add a row to the *Results* table: date, test ID,
   the commit you tested (`git rev-parse --short HEAD`), who ran it,
   PASS/FAIL, and the log file. Commit the log and the table together.

## What a test run does

`usermain()` in `app_program/app_main.c` starts every task as in a normal
build, then runs the tests one module after another in the initial task:

```
[TEST] === test run start ===
comms, remote, telemetry      quick, never drive the car
subsystem1 ... subsystem3
subsystem4                    telemetry leaves the IMU's events alone meanwhile
subsystem5
navigation                    quick checks, then starts NAV-M01 in its own task
[TEST] === test run end ===
```

Each module prints its own `[TEST] <ID> PASS|FAIL` lines and a summary. A
failing check only prints `FAIL`; the run carries on, so one module's failures
never stop another module's tests from running. If `=== test run end ===`
never appears, the board hung or crashed: rebuild with `CRASH_REPORT=1`
(`README.md` section 3) to see why.

**The motors may turn during a test run. Keep the car on a stand.**

- NAV-M01 (`navigation_test.c`, `NAV_MANUAL_M01` = 1) starts 10 s after
  `=== test run end ===` and drives a scripted mission.
- A Subsystem 2 manual test runs if `S2_MANUAL_TEST` in `subsystem2_test.c` is
  not `S2_MANUAL_NONE`.
- Line Following drives the car by itself if `LINE_SELF_DRIVE` in
  `subsystem3/line_barcode.h` is set to 1 (it is 0 now). It prints its
  readings while `LINE_DEBUG` is 1, and its barcode passes while
  `BARCODE_DEBUG` is 1 (it is now). Those prints can bury the `[TEST]`
  lines; filter the saved log with
  `grep '\[TEST\]' logs/<file>.txt`.

## Retiring a test

When the code a test checks is removed, keep the row and start its *What is
tested* cell with `*retired:*` and the reason. Remove the `TEST_CHECK` from
the code. Old results stay as they are, and the ID is never given to a new
test.

## Videos of tests and demos

A manual test (an LED, a motor, a sensor reacting) is best shown with a short
video. Record 15–60 s showing the test ID and the hardware, share it as an
**unlisted YouTube** link (the only way videos are shared), list it in
**`videos/README.md`**, and put the link in the *Log / evidence* column of
the test's result row. `videos/README.md` explains how.

## Rules

- **Never edit or delete an old result.** If a test fails, record the FAIL,
  fix the code, re-run, and add a new PASS row. The history is the evidence.
- **Always record the commit.** A result is meaningless without the exact
  code it was run on. Commit your work *before* testing.
- **A FAIL needs a note** saying what went wrong and, later, which commit fixed it.
- Before merging a change into `main`, re-run your subsystem's unit tests.
  Before a demo or hand-in, run every test and record the full set.
