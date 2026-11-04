# Wi-Fi / MQTT

The car reports its state to a dashboard and takes START / STOP / CLEAR from
it, over MQTT on Wi-Fi. Owner: Rifa. The dashboard and the broker are in the
separate `mqtt-web-interface` project.

```
 car state (Navigation, Motion,
 encoders, line, ultrasonic) --> telemetry_task --> TX message buffer --+
                                 remote_task  ----> (log lines)  -------+
                                                                        v
                                    bridge (net/, on the kernel's WiFi task) <--> broker <--> dashboard
                                                                        |
 Navigation <-- nav_post_remote() -- remote_task <-- RX message buffer <-+  (cmd topics)
 (IF-13)                             telemetry_task <-- RX message buffer <-+  (config topic)
```

| Folder | What it is |
|---|---|
| `app_program/net/` | **Not a subsystem.** The network bridge: Wi-Fi, DHCP and the MQTT session, all on the kernel's WiFi task, which is the only task allowed to call lwIP or MQTT. Recovers each layer with exponential backoff. |
| `app_program/comms/` | What tasks use to reach the bridge: send with `comms_publish()` / `comms_log()`, receive by `comms_subscribe(topic, my_mbf)` in your `_init()`. Never blocks. |
| `app_program/remote/` | Commands from the dashboard to Navigation (`nav_post_remote()`, IF-13), and the Wi-Fi-loss fault bit when the connection drops (IF-12). |
| `app_program/telemetry/` | Reads every subsystem's state and publishes it as one 64-bit word. |
| `app_program/protocol.h` | Topics, command numbers and the telemetry bit layout. The dashboard has a copy. |

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
growing waits. `make NET=lwip TEST=1` adds the self-tests (`testing/comms/`,
`testing/remote/`, `testing/telemetry/`).

## Topics

| Topic | Direction | Payload |
|---|---|---|
| `robocar/<id>/status` | car → | `online` / `offline` (retained; `offline` is the last will) |
| `robocar/<id>/telemetry` | car → | one unsigned 64-bit integer in decimal (layout below), every 1 s by default |
| `robocar/<id>/log` | car → | one line of text |
| `robocar/<id>/cmd`, `robocar/all/cmd` | → car | a command, below |
| `robocar/<id>/config` | → car | a setting, below |

Commands and settings are decimal integers separated by spaces, the
identifier first.

| Command | Payload | Effect |
|---|---|---|
| PING | `0 [token]` | the car logs `remote: pong <token>` |
| START | `1` | Navigation: IDLE or STOPPED → follow the line |
| STOP | `2` | Navigation: brake and wait |
| CLEAR | `3` | Navigation: FAULT → clear the faults, back to IDLE |

| Setting | Payload | Effect |
|---|---|---|
| Telemetry period | `1 <ms>` | 100 to 60000 ms between samples |
| Telemetry on/off | `2 <0 or 1>` | stop or resume telemetry |

## Telemetry layout (version 1)

Bits from 0 upwards. Every field is unsigned.

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
| 62–63 | layout version | 1 | `TLM_LAYOUT` |

To add a field, take bits from a field that has spare range or bump
`TLM_LAYOUT`, change `protocol.h` and `protocol.ts` together, and update the
golden sample in `comms_test.c` (CO-U05) and `protocol.test.ts`. Both sides
check the same value, so a mismatch fails a test instead of showing wrong
numbers.

The line state and the obstacle distance come from functions not yet listed
as interfaces (IF-16, proposed). They are linked weakly: until S3 / S5 are in
the build, or if they rename the function, telemetry reports "not reported"
instead of failing to build.

## Using it from a subsystem

```c
#include "../comms/comms.h"

/* in your _init(): your own buffer gets robocar/<id>/my-topic */
comms_subscribe(COMMS_DEVICE_TOPIC("my-topic"), my_mbf);   /* maxmsz = sizeof(COMMS_MSG) */

/* in your task */
COMMS_MSG msg;  COMMS_CMD cmd;
tk_rcv_mbf(my_mbf, &msg, TMO_FEVR);
if (comms_parse_cmd(&msg, &cmd) == E_OK) { /* cmd.id, cmd.arg[0..3] */ }

comms_log("battery low");              /* robocar/<id>/log, also printed */
```

Limits: topics up to 63 bytes, payloads up to 128. A full receive buffer
drops the new message (the bridge never waits for a task); a full send queue
returns `E_TMOUT`. Log lines queued while offline are sent after reconnecting.
