# Kernel issues, and how the MQTT bridge follows the course guidance

The MQTT bridge in `app_program/net/` was written **without changing any kernel
file** (`kernel/`, `lib/`, `include/`, `build_make/`, `config/`, `device/`, `etc/`).
This file covers two things:

- **Section A** goes through the course guidance on MQTT point by point, saying
  where each point is met and the two places this project departs from it, and why.
- **Section B** lists each shortcoming of the kernel's Wi-Fi/network support
  that got in the way, how the application works around it, and what a kernel
  fix could be. If the kernel is fixed, the matching workaround can be removed.

All paths are relative to this folder. "Wi-Fi task" means `cyw43_utk_task()` in
`lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c`.

---

## A. The course guidance

The guidance describes this architecture, and the project follows it:

```
  subsystem --publish request--> TX message buffer --> bridge --> MQTT broker
  MQTT broker --> bridge --received message--> RX message buffer --> subsystem
```

| Guidance | Where it is met |
|---|---|
| Subsystems do not call lwIP or MQTT | Subsystems use only `app_program/comms/comms.h` (message buffers). All lwIP, MQTT and radio calls are in `app_program/net/`, which is not a subsystem. |
| Only code in the Wi-Fi service context calls lwIP/MQTT | `net_bridge.c` runs only as an lwIP timer and lwIP callbacks, both run by the Wi-Fi task's `sys_check_timeouts()` (see B1 for how it gets there). |
| Subsystem puts publish requests in a TX message buffer; the bridge publishes them | `comms_publish()` / `comms_log()` / `comms_telemetry()` → TX message buffer → `send_queued()` in `net_bridge.c`. |
| The bridge puts received messages in an RX message buffer; the subsystem takes them | `on_incoming_data()` in `net_bridge.c` → each subscriber's RX message buffer (`mbf_remote`, `mbf_telemetry`) → `tk_rcv_mbf()` in the task. |
| MQTT processing must not block the Wi-Fi task; poll the buffers | Every message-buffer call in `net_bridge.c` uses `TMO_POL`. |
| Copy topic and payload into the RX buffer before the callback returns | Both are copied into `rx_msg` as they arrive, and `rx_msg` into the RX buffer inside the last `on_incoming_data()` call. |
| Fixed size limits and a policy for full buffers / oversized messages | `NET_TOPIC_MAX` (63) and `NET_PAYLOAD_MAX` (128) in `net_bridge.h`. Oversized: refused when sending (`E_PAR`), dropped and counted when received. Full RX buffer: message dropped and counted (`rx_dropped`), bridge never waits. Full TX buffer: the sender gets `E_TMOUT` and the message is dropped (`tx_dropped`). Offline: TX messages wait in the buffer; telemetry is not queued. |
| Restore subscriptions on every reconnect | The `NET_MQTT_SUBSCRIBE` state, entered after every accepted connect, subscribes to the whole table again. |
| Handle initialisation order: no buffer used before it exists | Buffers are created in the `_init()` phase; `comms_start()` runs after every `_init()` and only then hands the buffers to the bridge. The bridge is not started before that. |
| Configure the broker in `config/wifi_secrets.h` | Yes: `MQTT_BROKER_IP`, `MQTT_BROKER_PORT`, `MQTT_CLIENT_ID`. See `MQTT.md`. |

**Departure 1: the bridge is not in `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`.**
The guidance puts it there, but that file is part of the kernel tree, which this
project does not modify. The stock `mqtt_utk.c` is a heartbeat demo with no
message buffers (B3). So the bridge is in `app_program/net/`, kept apart from
the subsystem folders, and it runs in the Wi-Fi service context as the guidance
requires.

**Departure 2: build with `NET=lwip`, not `MQTT=1`.** `MQTT=1` links the
stock `mqtt_utk.c`, which the Wi-Fi task starts by itself and which would fight
the bridge for the same client ID (B3). `NET=lwip` gives the same stack
(`WIFI=cyw43` is the default), and the bridge compiles lwIP's `mqtt.c` itself.

If the course later allows changing `mqtt_utk.c`, `net_bridge.c` can move into
it almost unchanged. B1 and B3 then go away, and the build becomes `MQTT=1`.

---

## B. Kernel shortcomings and workarounds

### 1. No way to run application code on the Wi-Fi task

**Problem.** lwIP is built `NO_SYS=1` (no locking), so only the Wi-Fi task may
call lwIP. The task's loop calls `cyw43_poll()`, `sys_check_timeouts()` and, in
an `MQTT=1` build, the kernel's own `mqtt_utk_start()`. There is no hook that
lets an application add its own code to that loop.

**Workaround.** The bridge runs as an lwIP timer that re-arms itself
(`bridge_tick()` in `app_program/net/net_bridge.c`); `sys_check_timeouts()`
then runs it on the Wi-Fi task every 10 ms. Only the first `sys_timeout()` call is
made from outside the Wi-Fi task, by a short-lived task (`net_start_task()` in
`app_program/net/net_start.c`) pinned to the Wi-Fi task's core at a lower
priority (5 vs 4). That task can only run while the Wi-Fi task is blocked in a
kernel call, never part-way through changing lwIP's timer list or memory pools.

**Kernel fix.** Have the loop call a documented, weak hook after
`sys_check_timeouts()`, or let `mqtt_utk.c` be the bridge (section A).

### 2. Wi-Fi recovery is incomplete and fixed-interval

**Problem.** The Wi-Fi task joins once at boot and re-joins every 5 s, but only
while the link status is *negative* (`FAIL`, `NONET`, `BADAUTH`):

- When the access point goes away after a successful join, the driver reports
  `CYW43_LINK_DOWN` (0), which is not negative, so **the car never re-joins**.
- A join that stays in `JOIN` or `NOIP` (no DHCP lease) is never retried.
- The 5 s interval is fixed, with no backoff.

**Workaround.** The bridge runs its own Wi-Fi state machine. It re-joins after a
`DOWN`, after a failure, or after 30 s without a DHCP lease, with exponential
backoff (1 s doubling to 60 s, with jitter). After a failure, `wifi_reset()` in
`net_bridge.c` clears `cyw43_state.wifi_join_state`, the same thing the driver
does on a disassociation. That stops the kernel's fixed 5 s re-join from firing
in between: the kernel sees `DOWN` and cancels its timer. The kernel still
prints its own "link status" lines alongside the bridge's.

**Kernel fix.** Leave join policy to the application, or add `DOWN`-after-`UP`
and a DHCP timeout to the kernel's retry, with backoff.

### 3. `MQTT=1` links a demo client that conflicts with any real one

**Problem.** `make MQTT=1` builds lwIP's `mqtt.c` together with
`lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`, which the Wi-Fi task starts
automatically. It connects with `MQTT_CLIENT_ID`, publishes a heartbeat to
`MQTT_TOPIC`, and retries on a fixed 3 s timer. It has no message buffers and no
way to switch it off. A second client with the same ID would make the broker
keep dropping one connection in favour of the other.

**Workaround.** Build with `make NET=lwip`, not `MQTT=1`; the application
compiles lwIP's `mqtt.c` itself (`app_program/net/lwip_mqtt.c`). Building with
`MQTT=1` stops with an `#error` explaining this.

**Kernel fix.** Make `mqtt_utk.c` the message-buffer bridge described in the
guidance, or build it only with a separate opt-in (e.g. `MQTT_DEMO=1`).

### 4. lwIP MQTT buffers are small, and only settable in the kernel

**Problem.** `lwipopts.h` leaves `MQTT_OUTPUT_RINGBUF_SIZE` (256 bytes) and
`MQTT_REQ_MAX_IN_FLIGHT` (4) at lwIP's defaults. That is tight for the
subscriptions, the "online" status and the queued log lines sent right after
connecting.

**Workaround.** The bridge retries on `ERR_MEM` (subscriptions and publishes
carry on next tick), so the defaults work. `app_program/net/lwip_mqtt.c` also
raises both values before including `mqtt.c`, so retries are rarer. That only
changes the inside of `mqtt_client_t`, which no other file looks into.

**Kernel fix.** Set them in `lib/liblwip/sysdepend/pico_rp2040/lwipopts.h`.

### 5. A failed radio start-up cannot be recovered

**Problem.** If the radio does not come up, or the driver refuses the join
request at boot, the Wi-Fi task stops in `for(;;) tk_dly_tsk(1000);` and never
calls lwIP again. No application code can recover from that.

**Workaround.** None possible from the application. `net_start_task()` prints
`net: the radio failed to start (<code>), so there is no network`. Power-cycle
the board. A hardware watchdog that resets the board if the bridge never comes
online would cover this.

**Kernel fix.** Treat a refused join like any other failed join (retry), and
retry a failed `cyw43_init()` after a delay.

### 6. Smaller things

- **The network is off by default.** `NET` defaults to `none`, so the bridge
  only runs when built with `make NET=lwip`. A plain `make` still builds and
  runs, and prints that the network is not built in.
- **`tm_sprintf()` has no 64-bit conversion and no buffer size.** The
  telemetry word is formatted by `comms_format_u64()`
  (`app_program/comms/comms_payload.c`); log lines are kept short by hand.
- **The Wi-Fi security mode is private to the kernel.** `WIFI_AUTH_MODE` is
  defined in `cyw43_utk.c`; `net_bridge.c` uses the same default and the same
  makefile override (`-DWIFI_AUTH_MODE=...`). If one changes, change the other.
- **`cyw43.h` and `tk/tkernel.h` cannot be included in the same file** (both
  define `size_t`). `net_bridge.c` includes `cyw43.h` and declares the two
  message-buffer calls it needs by hand, as the kernel's own `cyw43_utk.c`
  does for `tk_dly_tsk()`. `net_start.c` is on the kernel side of that split.
- **The kernel's own network scan can only be switched on in the makefile.**
  `cyw43_utk.c` has a `WIFI_DIAGNOSTIC_SCAN` option, but it is set through
  `CFLAGS`, which the makefile assigns itself. The bridge does its own scan
  before every join instead (`scan_then()` in `net_bridge.c`), using the same
  driver call, and marks the network named in `WIFI_SSID`.
- **`README.md` and code comments refer to `docs/RELEASE.md`**, which is not in
  this repository.
