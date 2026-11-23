# Wi-Fi / MQTT

The car reports its state to a dashboard and takes START / STOP / CLEAR from
it, over MQTT on Wi-Fi. Owner: Rifa. The dashboard and the broker are in the
separate `mqtt-web-interface` project.

```
 car state (Navigation, Motion, encoders,
 line, ultrasonic, IMU, barcode) --> telemetry_task --> TX message buffer --+
                                 remote_task  ----> (log lines)  -------+
                                                                        v
                                    bridge (net/, on the kernel's WiFi task) <--> broker <--> dashboard
                                                                        |
 Navigation <-- nav_post_remote() -- remote_task <-- RX message buffer <-+  (cmd topics)
 (IF-13)                             telemetry_task <-- RX message buffer <-+  (config topic)
```

| Folder | What it is |
|---|---|
| `app_program/subsystem1/net/` | **Not a subsystem.** The network bridge: Wi-Fi, DHCP and the MQTT session, all on the kernel's WiFi task, which is the only task allowed to call lwIP or MQTT. Recovers each layer with exponential backoff. |
| `app_program/subsystem1/comms/` | What tasks use to reach the bridge: send with `comms_publish()` / `comms_log()`, receive by `comms_subscribe(topic, my_mbf)` in your `_init()`. Never blocks. |
| `app_program/subsystem1/remote/` | Commands from the dashboard to Navigation (`nav_post_remote()`, IF-13), and the Wi-Fi-loss fault bit when the connection drops (IF-12). |
| `app_program/subsystem1/telemetry/` | Reads every subsystem's state and publishes it as two 64-bit words; sends humps and barcodes as events when they happen. |
| `app_program/subsystem1/protocol.h` | Topics, command numbers and the telemetry bit layout. The dashboard has a copy. |

No kernel file is changed. `KERNEL_ISSUES.md` explains how this follows the
course's MQTT guidance and why it is built with `NET=lwip` rather than
`MQTT=1`.

## Build and run

1. Start the broker and dashboard on a computer on the same 2.4 GHz WPA2
   network as the car (a phone hotspot works well):
   ```sh
   cd mqtt-web-interface && npm install && npm start
   ```
   It prints this computer's IPv4 addresses. Allow TCP port 1883 through its
   firewall.
2. Copy `config/wifi_secrets.example.h` to `config/wifi_secrets.h` (never
   committed) and set `WIFI_SSID`, `WIFI_PASSWORD`, `MQTT_BROKER_IP` (from
   step 1), `MQTT_BROKER_PORT` (1883) and `MQTT_CLIENT_ID` (unique per car,
   letters, digits, `-` and `_`, e.g. `"robocar-01"`).
3. Build and flash:
   ```sh
   cd build_make
   make NET=lwip -j8      # -> mtk3pico_smp1_usb_cdc_wifi_lwip.uf2
   ```
   **Not** `MQTT=1`: that also starts the kernel's heartbeat demo client,
   which would fight this one for the broker. A plain `make` still builds,
   with the network left out.
4. On the USB console the car lists the Wi-Fi networks it can see, joins,
   and prints `net: online as robocar-01, 3 topics subscribed`. It appears
   on the dashboard at <http://localhost:8080/>.

If it does not connect, the console says which layer failed (SSID not found,
wrong password, no DHCP lease, broker did not answer) and keeps retrying with
growing waits. `make NET=lwip TEST=1` adds the self-tests (see "Testing"
below).

## Topics

| Topic | Direction | Payload |
|---|---|---|
| `robocar/<id>/status` | car → | `online` / `offline` (retained; `offline` is the last will) |
| `robocar/<id>/telemetry` | car → | two unsigned 64-bit integers in decimal, one space apart (layout below), every 500 ms by default; the first one about 300 ms after each (re)connection |
| `robocar/<id>/event` | car → | a hump or a barcode, once, when it happens (below) |
| `robocar/<id>/log` | car → | one line of text |
| `robocar/<id>/cmd`, `robocar/all/cmd` | → car | a command, below |
| `robocar/<id>/config` | → car | a setting, below |

Commands, settings and events are decimal integers separated by spaces,
the identifier first.

| Command | Payload | Effect |
|---|---|---|
| PING | `0 [token]` | the car logs `remote: pong <token>` |
| START | `1` | Navigation: IDLE or STOPPED → follow the line |
| STOP | `2` | Navigation: brake and wait |
| CLEAR | `3` | Navigation: FAULT → clear the faults, back to IDLE |

| Setting | Payload | Effect |
|---|---|---|
| Telemetry period | `1 <ms>` | 100 to 60000 ms between samples; 500 ms until set. Kept until the car restarts |
| Telemetry on/off | `2 <0 or 1>` | stop or resume telemetry |
| Barcode meaning | `3 <letter> <dir>` | what a barcode character means at the next T: the character's ASCII code (`0`-`9`, `A`-`Z`), then 0 left, 1 right, 2 straight, 3 U-turn or 4 ignore. Defaults: A left, B right, C straight, D U-turn, every other one ignored |
| Clear barcodes | `4` | ignore every barcode character |
| Show barcodes | `5` | only report the map |

Each barcode setting answers on the log with the whole map, e.g.
`barcodes: A=L B=R Z=U`. The map is lost when the car restarts; the
dashboard sends it again.

| Event | Payload | Meaning |
|---|---|---|
| HUMP | `0 <mm> <over> <tilt>` | a hump closed: its height in mm (127 with `over` = 1 if higher), and the largest pitch in degrees, or roll if only one rear wheel rode over it. From the IMU's event stream (IF-S4-3) |
| BARCODE | `1 <letter>` | a barcode was read; the letter as its ASCII code (`65` = A). From `barcode_get_last()` (IF-18) |

Events are queued while offline, like log lines, and are not retained: a
dashboard sees those sent while it is open. The dashboard keeps the tallest
hump itself. The last barcode is also in every telemetry sample.

## Telemetry layout (version 2)

The payload is word 0 and word 1 in decimal, one space apart, e.g.
`11213681551648818218 8090835030`. Bits from 0 upwards. Every field is
unsigned.

### Word 0

Unchanged from layout 1, except that the version field reads 2. A reader
that takes only the first number reads it as before.

| Bits | Field | Meaning | Source |
|---|---|---|---|
| 0–7 | sample | counter, wraps; a gap means samples were lost | telemetry |
| 8–10 | vehicle state | 0 IDLE, 1 LINE_FOLLOW, 2 JUNCTION, 3 OBSTACLE, 4 BYPASS, 5 STOPPED, 6 FAULT | `nav_get_state()`, IF-14 |
| 11–13 | stop reason | 0 none, 1 fault bit, 2 motion fault, 3 line not found, 4 remote stop | `nav_get_reason()`, IF-14 |
| 14–21 | faults | the team's fault bits: 0 motor stall, 1 encoder, 2 line lost, 3 IMU, 4 e-stop, 5 no echo, 6 scan error, 7 Wi-Fi loss | `nav_get_faults()`, IF-14 |
| 22–23 | motion | 0 idle, 1 straight, 2 pivot | `motion_get_activity()`, IF-09 |
| 24–25 | line | 0 on, 1 lost, 2 searching, 3 not reported | `line_get_state()`, S3 (IF-16) |
| 26–35 | left wheel | mm/s, 1023 = 1023 or more | `drv_encoder_get_speed()`, IF-06 |
| 36–45 | right wheel | mm/s, as above | `drv_encoder_get_speed()`, IF-06 |
| 46–54 | obstacle | cm ahead, 511 = no echo or no sensor | `ultrasonic_last_reading()`, S5 (IF-16) |
| 55–61 | Wi-Fi signal | −dBm, 0 = unknown | the bridge |
| 62–63 | layout version | 2 | `TLM_LAYOUT` |

### Word 1

Added in layout 2, because word 0 was full.

| Bits | Field | Meaning | Source |
|---|---|---|---|
| 0–1 | IMU motion | 0 stationary, 1 accelerating, 2 cruising, 3 turning | `imu_task_get_snapshot()`, IF-S4-1 |
| 2–3 | IMU terrain | 0 level, 1 climbing, 2 descending, 3 hump just closed | `imu_task_get_snapshot()`, IF-S4-1 |
| 4 | IMU data | 1 = the two fields above are live; 0 = no IMU data, ignore them | `imu_is_ok()`, IF-S4-7 |
| 5–11 | last barcode | letter as ASCII, 0 = none since power-on | `barcode_get_last()`, IF-18 |
| 12–15 | barcodes read | count, wraps at 16: a change is a new read, even of the same letter | as above |
| 16–35 | distance | cm since power-on, mean of both wheels (pivots count too), 1048575 = that or more | `drv_encoder_get_count()`, IF-06 |
| 36–63 | spare | 0 | |

To add a field, use word 1's spare bits; change `protocol.h` and
`protocol.ts` together, and update the golden samples in `comms_test.c`
(CO-U05, CO-U10) and `protocol.test.ts`. Both sides check the same value, so
a mismatch fails a test instead of showing wrong numbers. When word 1 is
full, add a word 2 the same way and bump `TLM_LAYOUT`.

The line state and the obstacle distance come from `line_get_state()` (S3)
and `ultrasonic_last_reading()` (S5), IF-16, called through their headers.
Both subsystems are in the build, so the line field never reads 3 ("not
reported") from this firmware; the value stays defined for the dashboard.

### Where the IMU and barcode data come from

Telemetry only reads what the subsystems offer. One getter was added to
Line Following for it (`barcode_get_last()`); nothing else in a subsystem
changed.

- **IMU states**: the snapshot word S4 publishes for the comms owner
  (IF-S4-1), unpacked with S4's own `imu_pack_unpack_snapshot()`.
- **Humps**: S4's event stream (IF-S4-3). The telemetry task drains it every
  100 ms (the stream holds 32 events), sends each HUMP as an event and drops
  the rest, as motion and terrain are already in the snapshot. Nothing else
  may drain it, or humps go missing.
- **Barcodes**: `barcode_get_last()` (IF-18, `subsystem3/line_barcode.h`)
  returns one 32-bit word that the barcode task updates with a single store
  on every successful read: the letter in bits 0–7 and a count of reads in
  bits 8–15. The telemetry task checks it every 100 ms and sends an event
  when the count changes, so a second read of the same letter is still a
  new event. Barcode passes end after 1 s without an edge, so two reads
  never fall within one 100 ms poll.
- **Distance**: the cumulative encoder counts (IF-06), which never reset.

### Changes outside the telemetry code

- Line Following (S3): `barcode.c` keeps the last read and its count in one
  word and offers `barcode_get_last()` (IF-18). It reads nothing new and
  changes no behaviour: the word is written next to the existing log line
  in `finish_pass()`. To be reviewed by S3's owner.
- `comms`: `comms_telemetry()` takes an array of words instead of one word,
  and `comms_payload_format_words()` builds the payload text.
- `telemetry_task` now also wakes every 100 ms to send events, not only at
  each sample. Events are sent even while telemetry is switched off
  (`CFG_TELEMETRY_ENABLE 0`).
- `app_main.c` (test builds only): telemetry stops draining the IMU's
  events while `subsystem4_test()` runs, because S4-U06 waits for an IMPACT
  event on the same stream.
- `TEAM.md`: IF-15 and IF-17 updated, IF-18 added. Test logs: CO-U10,
  TL-U03, INT-19, INT-20, SYS-15 added. TL-U04 and INT-18 tested an
  earlier stopgap that read the barcode from S3's log line; they are
  retired. The interface references of
  CO-U05, CO-U08 and CO-U09 pointed at the retired IF-01 .. IF-05 and now
  name IF-15 and IF-17.

## Coding standard

All C files of this part (`app_program/subsystem1/net/`, `comms/`, `remote/`,
`telemetry/` and `protocol.h`) follow the Barr Group Embedded C Coding
Standard, BARR-C:2018 (`barr_c_coding_standard_2018.pdf`). In short:
4-space indents and no tabs, braces on their own lines, lines of at most
80 characters, Doxygen comments on every file and function, `stdint.h` and
`bool` types, `static` for everything private, one header per source file,
`g_`, `p_` and `b_` name prefixes, constants on the left of `==`, every cast
commented, and one exit point per function.

How the rules meet the kernel and the libraries:

- **Kernel types.** `ER`, `ID`, `TMO`, `T_CMBF`, `SYSTIM` and `INT` (the
  task start code, and what `tk_rcv_mbf()` returns) appear only where a
  kernel call needs them, as in Subsystem 4. Everything else uses
  `stdint.h` types.
- **Text casts.** The kernel's `tm_printf()` and `tm_sprintf()` take `UB`
  (unsigned char) text, so every call casts ASCII `char` text to `UB`, with
  a comment. `tm_sprintf()` has no `va_list` form, so a wrapper cannot hide
  this.
- **Shared test macros.** `TEST_INIT()`, `TEST_CHECK()`, `TEST_SUMMARY()`
  (`test_report.h`) and `CHECK()` (`app.h`) belong to the template and are
  used as they are.
- **Inline assembly.** `memory_barrier()` in `net_bridge.c` is the only
  `asm`. Rule 1.1.c allows it in a module that drives hardware, as this one
  drives the radio.

Deviation, per the standard's deviation procedure:

| Rule | Where | Why | Approved by |
|---|---|---|---|
| 4.3.f: no source file includes another source file | `net/lwip_mqtt.c` includes lwIP's `mqtt.c` | Third-party code that the kernel's `NET=lwip` build does not compile. Adding it to the shared makefile would also need its buffer sizes passed to that one file; including it keeps the build files and the kernel unchanged | *pending: the project manager* |

`test_report.h` and `app.h` are shared template files and are not covered
here; nor is the dashboard, which is TypeScript.

### Abbreviations

| Abbreviation | Meaning |
|---|---|
| ASCII | the 7-bit character set used for all text |
| BSSID | the MAC address of one Wi-Fi access point |
| CFG | a setting sent to the car (`CFG_*`) |
| CMD | a command sent to the car (`CMD_*`) |
| DHCP | how the car gets its IP address from the network |
| EVT | an event sent by the car (`EVT_*`) |
| IMU | inertial measurement unit (Subsystem 4) |
| lwIP | the TCP/IP stack in the Pico SDK |
| mbf | a micro T-Kernel message buffer |
| MQTT | the publish/subscribe protocol between the car and the dashboard |
| QoS | MQTT quality of service: 0 at most once, 1 at least once |
| RSSI | received signal strength, in dBm |
| RX, TX | receive, transmit |
| SSID | the name of a Wi-Fi network |
| TLM | telemetry (`TLM_*` fields) |

## Using it from a subsystem

```c
#include "../comms/comms.h"

/* In your _init(): create a buffer with bufsz = comms_mbf_size(n) and
   maxmsz = sizeof(comms_msg_t), then subscribe it.  Add the topic to
   comms.h as a COMMS_TOPIC_* string, like COMMS_TOPIC_CMD. */
(void) comms_subscribe(COMMS_TOPIC_CMD, my_mbf);

/* In your task. */
comms_msg_t         msg;
comms_payload_cmd_t cmd;

(void) tk_rcv_mbf(my_mbf, &msg, TMO_FEVR);

if (E_OK == comms_payload_parse_msg(&msg, &cmd))
{
    /* cmd.id, cmd.arg_count, cmd.arg[0..3] */
}

(void) comms_log("battery low");    /* robocar/<id>/log, also printed. */
```

Limits: topics up to 63 bytes, payloads up to 128. A full receive buffer
drops the new message (the bridge never waits for a task); a full send queue
returns `E_TMOUT`. Log lines queued while offline are sent after reconnecting.

## Testing

| Log | Tests | What they cover |
|---|---|---|
| `testing/subsystem1/comms/TEST_LOG.md` | CO-U01 .. CO-U10 | command parsing, number formatting, the telemetry golden samples (both words), backoff, size limits, topic matching |
| `testing/subsystem1/remote/TEST_LOG.md` | RE-U01 | the remote task consumes every kind of message and keeps running |
| `testing/subsystem1/telemetry/TEST_LOG.md` | TL-U01 .. TL-U03, TL-U05 | telemetry settings, the hump event message, the barcode map (TL-U04 is retired) |
| `testing/integration/TEST_LOG.md` | INT-15 .. INT-17, INT-19, INT-20 | settings reach telemetry, both words hold the other subsystems' values (IMU, distance, barcode), the car and dashboard agree on the bit layout (INT-18 is retired) |
| `testing/system/TEST_LOG.md` | SYS-10 .. SYS-15 | connecting, remote commands, telemetry on the moving car, Wi-Fi loss, broker restart, humps and barcodes on the dashboard |

The automatic tests need no network and run in any `make TEST=1` image,
first in the test run, so their results are at the top of the console. The
system tests need the car, the broker and the dashboard (set-up in
`testing/system/TEST_LOG.md`). The dashboard has its own tests: `npm test`
in `mqtt-web-interface`, which checks the same golden samples as CO-U05 and
CO-U10.
