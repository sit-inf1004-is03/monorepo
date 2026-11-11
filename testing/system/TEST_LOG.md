# System — test log

Tests of the complete product against its requirements, run on the final hardware set-up. Run the full set before every demo and before hand-in.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | e.g. device reports a reading over WiFi every 10 s | REQ-? | manual | 1. Power on. 2. Watch the dashboard for 2 minutes. | a reading appears every 10 s ± 1 s |
| SYS-02 | | | manual | 1. … 2. … | |
| SYS-10 | car connects and reports | Wi-Fi + DHCP + MQTT | manual | 1. `npm start` in `mqtt-web-interface`. 2. Power the car (built with `make NET=lwip`). | console lists the visible networks, then `net: online`; the dashboard shows the car online with a sample every 1 s, vehicle state IDLE |
| SYS-11 | remote START / STOP / CLEAR | IF-13, IF-15 | manual | Car on the track. From the dashboard: Start, then Stop, then (after making a fault) Clear faults | state goes LINE_FOLLOW, then STOPPED with reason "remote stop", then from FAULT to IDLE; each logged as `remote: … to navigation` |
| SYS-12 | telemetry follows the car | IF-06, IF-14, IF-16 | manual | Push the car along the line by hand, then lift it off the line, then put a hand 20 cm in front of the sensor | wheel speeds rise and fall; line goes lost / searching; obstacle reads about 20 cm (once S5 is merged) |
| SYS-13 | Wi-Fi loss is a fault, not a stop | IF-12 | manual | While following the line, switch the access point off for 1 min, then on | the car keeps following; after reconnecting the faults show "Wi-Fi loss" and the log has `remote: connection lost …`; the bit stays set (Navigation clears faults only on CLEAR in the FAULT state) |
| SYS-14 | broker restart | MQTT recovery | manual | Stop `npm start` for 60 s, start it again | console shows `connect again in` with growing waits up to 30 s; the car comes back online and answers a Ping |
| SYS-15 | humps, barcodes, IMU states and distance reach the dashboard | IF-15, IF-18, IF-S4-1, IF-S4-3 | manual | Car on the track with a hump and barcodes A–D, built with `make NET=lwip`. Start it from the dashboard and let it run the course | IMU motion goes accelerating, then cruising, and turning at junctions; IMU terrain goes climbing, descending, level over the hump; each hump and barcode appears once under Detections, in order; Tallest hump shows the highest; Latest barcode shows the last letter; distance grows by about the length driven |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | SYS-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
