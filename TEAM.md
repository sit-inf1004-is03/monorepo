# Team and subsystems

The tables are pre-filled with the **example** in `app_program/` (marked
*example*). Replace those rows with your own as you replace the example code.

Fill this in at the start of the project and keep it up to date. It is the
single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | *example:* LED heartbeat on GP19 | | | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | *example:* Producer: makes a sample every 300 ms | | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | *example:* Consumer: collects samples into statistics | | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU-based motion and terrain monitoring (GY-511 / LSM303D on I2C1) | 2501298 | | `app_program/subsystem4/`, `app_program/common/` | `testing/subsystem4/` |
| 5 | Obstacle Scan & Avoidance: ultrasonic + scan servo, bypass plan | 2502821 | | `app_program/subsystem5/` | `testing/subsystem5/` |

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | | (taken whenever the team builds with `CRASH_REPORT=1`) |
| GP1 | | (taken whenever the team builds with `CRASH_REPORT=1`) |
| GP2 | S4 | I2C1 SDA, IMU (Grove 2) |
| GP3 | S4 | I2C1 SCL, IMU (Grove 2) |
| GP4 | | |
| GP5 | S2 | left wheel encoder (input, pull-up, rising edge via `gpio_irq`) |
| GP6 | | |
| GP7 | S2 | right wheel encoder (input, pull-up, rising edge via `gpio_irq`) |
| GP8 | S2 | motor M1A (PWM 4A, `drv_motor`) |
| GP9 | S2 | motor M1B (PWM 4B, `drv_motor`) |
| GP10 | S2 | motor M2A (PWM 5A, `drv_motor`) |
| GP11 | S2 | motor M2B (PWM 5B, `drv_motor`) |
| GP12 | S5 | scan servo signal (PWM 6A, 50 Hz, `drv_servo`) |
| GP13 | | |
| GP14 | | |
| GP15 | | |
| GP16 | S5 | ultrasonic TRIG (output, Grove port 4, `drv_ultrasonic`) |
| GP17 | S5 | ultrasonic ECHO (input, both edges via `gpio_irq`, Grove port 4) |
| GP18 | | |
| GP19 | *example:* S1 | external LED (LED + 330 Ω to GND) |
| GP20 | | |
| GP21 | | |
| GP22 | | |
| GP26 | | (ADC0 capable) |
| GP27 | | (ADC1 capable) |
| GP28 | | (ADC2 capable) |

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "environment sensing" subsystem contains the I2C
sensor code, and owns I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART1 | | | |
| I2C0 | | | |
| I2C1 | S4 | `imu_task` | GP2 (SDA), GP3 (SCL) |
| SPI0 | | | |
| SPI1 | | | |
| ADC | | | GP26–GP28 |
| PWM slices 4, 5 | S2 | `task_motion`; `s2_test_motor_task` in `TEST=1` builds with `S2_MANUAL_M02` (task_motion leaves the motors alone while idle) | GP8–GP11. The port leaves the whole PWM block in reset (`USE_PTMR` is 0); `drv_motor_init()` releases it and never resets it again, so other slices (servo, slice 6) can rely on that |
| PWM slice 6 | S5 | `obstacle_scan_task` (`servo_init()` in `subsystem5_init()`) | GP12 (scan servo). `servo_init()` also releases the PWM reset, harmless after `drv_motor_init()` |
| GPIO interrupt (IO_IRQ_BANK0) | *shared* (`app_program/gpio_irq.c`) | interrupt on the core that runs usermain | one handler per pin, registered from `_init()`; S2: GP5, GP7; S5: GP17 |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

What students *can* do:

- **Read the connection state** from any task, which is always safe:
  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```
- **Send or receive data over the network** through the Wi-Fi / MQTT
  bridge (`app_program/net/`, owner: Rifa), which runs inside the service
  task without changing any kernel file. Tasks only touch message buffers:
  `comms_publish()` / `comms_log()` queue a message for the broker, and
  `comms_subscribe(topic, my_mbf)` in your `_init()` delivers a topic's
  messages to your own message buffer (`app_program/comms/comms.h`).
  Record each new topic as an interface in section 5. It needs the
  `NET=lwip` build, **not** `MQTT=1` (`MQTT.md`, `KERNEL_ISSUES.md`).

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4). Each subsystem gets its own band by default;
change it only by team agreement. Write down every task you create.

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 1 | 12–13 | *example:* led_task – 12 – any |
| 2 | 14–15 | task_motion – 14 – core 1 (away from Wi-Fi's priority 4 task on core 0); one of s2_test_b2_task, s2_test_motor_task, s2_test_motion_task – 15 – core 0 (`TEST=1` builds only) |
| 3 | 16–17 | line_task – 16 – core 0 (barcode_task – 16, not started yet) |
| 4 | 18–19 | imu_task – 18 – any |
| 5 | 20–21 | obstacle_scan_task – 20 – any |
| Navigation | 10–11 (*proposed*) | task_navigation – 10 – core 1 (with task_motion); nav_test_m01_task – 11 – core 0 (`TEST=1` builds only) |
| Wi-Fi / MQTT (Rifa) | 22–23 (*proposed*); 5 at start-up only | remote_task – 22 – any; telemetry_task – 23 – any; net_start – 5 – core 0 (runs once at start-up, then exits; must sit just below WiFi's 4, on its core). The bridge itself runs inside the kernel's WiFi task (4) |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | S2 → S3 | mailbox (owned by S3) | `mbx_samples` | `SAMPLE_MSG` in `subsystem2.h` | *retired*: the template example was removed |
| IF-02 | S3 → S2 | semaphore + fixed memory pool (owned by S2) | `sem_slots`, `mpf_samples` | receiver returns the block, then 1 slot | *retired*: the template example was removed |
| IF-03 | S3 → S4 | message buffer + event flag (owned by S4) | `mbf_summary`, `flg_events` | text line ≤ `S4_SUMMARY_MAX` bytes, then `FLG_BATCH` | *retired*: the template example was removed |
| IF-04 | S4 → S3 | shared data under a mutex (owned by S3) | `app_stats`, `mtx_stats` | `APP_STATS`, read only while locked | *retired*: the template example was removed |
| IF-05 | S4 → S1 | task control | `tid_led` | `tk_sus_tsk` / `tk_rsm_tsk` / `tk_chg_pri` | *retired*: the template example was removed |
| IF-06 | S2 → any (IMU, telemetry) | shared data, read only (owned by S2) | `drv_encoder_get_count()`, `drv_encoder_get_speed()` (mm/s) in `drv_encoder.h` | 32-bit pulses per wheel since boot, never reset, 637 per wheel revolution (0.330 mm/pulse); subtract snapshots as `UW` | *proposed* |
| IF-07 | Navigation → S2, S2 → Navigation | 2 message buffers + event flag (owned by S2), one command at a time | `motion_send()`, `motion_wait_reply()`, `motion_brake()`, `motion_set_speed_cap()` in `motion.h` | `motion_cmd_t`: straight (direction, speed mm/s, distance mm) or pivot (direction). One `motion_reply_t` each: DONE, BRAKED (ended by `motion_brake()`), or FAULT + reason. Speed cap slows a straight (junction approach). The brake command replaces Tribunal 1's "no stop command". Layout open (G10); moves to `include/msg_formats.h` once agreed | *proposed* |
| IF-08 | S3 (Line Following) → S2 | shared data, one writer | `motion_set_turn_level()` in `motion.h` | turn level −7..+7, every 20 ms; width open (M3) | *proposed* |
| IF-09 | S3 (Line Following), S5 (Obstacle Scan) → S2 | event flag (owned by S2 until G1/G2) | `motion_stop_turn()`, `motion_get_activity()` in `motion.h` | StopTurn: ends the running pivot. Senders watch `motion_get_activity()` for `MOTION_ACTIVITY_PIVOT` to know a pivot has started. Moves to bit 8 of the team event flag once agreed | *proposed* |
| IF-10 | S3 (Line Following, Barcode) → Navigation | event flag + one-writer words (owned by Navigation) | `nav_post_barcode()`, `nav_post_junction()`, `nav_set_line_state()` in `navigation/navigation.h` | barcode A–D (2 bits), junction reached, line state on / lost / searching (2 bits) | *proposed* |
| IF-11 | S5 (Obstacle Scan) → Navigation | event flag + one-writer word (owned by Navigation) | `nav_post_obstacle()`, `nav_post_bypass_plan()` | obstacle close (Navigation brakes); bypass plan: side 2 + distance 9 (cm) + angle 4 (0) + 1 reserved; bit order to agree with Ruien | *proposed* |
| IF-12 | any → Navigation | event flag (owned by Navigation until G1/G2) | `nav_raise_fault()`, `nav_clear_fault()` | the agreed fault bits 0–7, each raised only by its owner. IMU (bit 3) and Wi-Fi loss (bit 7) only degrade: the car keeps running, and the owner may clear its bit on recovery. Every other bit stops the car until a remote CLEAR (rest of G10 open) | IMU degrade: Glenn + Andrew; rest *proposed* |
| IF-13 | Wi-Fi / MQTT → Navigation | event flag + one-writer word | `nav_post_remote()` | START, STOP, CLEAR | *proposed* |
| IF-14 | Navigation → any (telemetry, Obstacle Scan) | shared data, read only | `nav_get_state()`, `nav_get_faults()`, `nav_get_reason()` | vehicle state (3 bits): IDLE, LINE_FOLLOW, JUNCTION, OBSTACLE (Obstacle Scan's cue to scan), BYPASS, STOPPED, FAULT | *proposed* |
| IF-S4-1 | S4 → S1 | 32-bit word, one atomic store | `imu_task_get_snapshot()` | pitch, highest hump, motion, terrain, faults, yaw rate (`imu_pack.h`) | proposed by S4 |
| IF-S4-2 | S4 → S1 | 32-bit word | `imu_task_get_aux()` | roll, heading, hump count | proposed by S4 |
| IF-S4-3 | S4 → S1 | message buffer (owned by S4), read with `TMO_POL` | `imu_task_read_event()` | 16-bit events: HUMP, MOTION, TERRAIN, IMPACT, IMU_FAULT, IMU_OK, COURSE_DONE | proposed by S4 |
| IF-S4-4 | S4 → Nav | IF-12 | `nav_raise_fault(NAV_FLT_IMU)`, `nav_clear_fault(NAV_FLT_IMU)` | IMU fault (bit 3), degrade only: raised once per outage, cleared when the sensor answers again. READY_IMU stays in `common/faults.h`, read only by S4 so far | IMU degrade: Glenn + Andrew |
| IF-S4-5 | S2 → S4 | IF-06 | `drv_encoder_get_count()` | 32-bit cumulative count per wheel, 0.330 mm/pulse (replaces the earlier pointer proposal) | uses IF-06 as shipped; width still open (G4) |
| IF-S4-6 | Nav → S4 | function call | `imu_task_signal_course_done()` / `imu_task_restart()` | end / start of a run | proposed by S4 |
| IF-15 | dashboard ↔ car (Wi-Fi / MQTT) | MQTT topics `robocar/<id>/…`, via the bridge's message buffers | `comms_subscribe()`, `comms_publish()` in `comms/comms.h`; `mbf_remote`, `mbf_telemetry` | topics, commands (PING, START, STOP, CLEAR) and the 64-bit telemetry layout in `app_program/protocol.h`; the dashboard's copy is `mqtt-web-interface/src/shared/protocol.ts` | *proposed* |
| IF-16 | S3 (Line Following), S5 (Obstacle Scan) → telemetry | shared data, read only | `line_get_state()` (S3), `ultrasonic_last_reading()` (S5) | line state (2 bits); nearest obstacle in cm (9 bits) or no echo. Linked weakly, so telemetry reports "not reported" until the function exists | *proposed* |
| IF-17 | any → Wi-Fi / MQTT | message buffer (owned by comms) | `comms_log()`, `comms_telemetry()`, `comms_publish()` | one line of text / a packed telemetry word / any payload ≤ 128 bytes; never blocks, dropped when the queue is full | *proposed* |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The numbers below are the example's usage.

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | 1 | 1 | 1 | 1 | 1 |
| Semaphore | 16 | | 1 | | | |
| Event flag | 16 | | | | 1 | |
| Mailbox | 12 | | | 1 | | |
| Mutex | 12 | | | 1 | | |
| Message buffer | 12 | | | | 1 | |
| Fixed memory pool | 12 | | 1 | | | |
| Variable memory pool | 12 | | | | | |
| Cyclic handler | 12 | 1 | | | 1 | |
| Alarm handler | 12 | | | | | |

(The kernel's USB and WiFi service tasks also use a few of these.)
