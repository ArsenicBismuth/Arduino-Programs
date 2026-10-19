# Air Mouse

A Bluetooth HID air mouse for a Seeed XIAO nRF54L15 / Sense, built on the
[`lolren/nrf54-arduino-core`](https://github.com/lolren/nrf54-arduino-core) board package.

`Air_Mouse.ino` reads the board's built-in ST **LSM6DS3TR-C**, turns the gyro into roll-compensated
pointer motion, and reports it as a standard BLE mouse. The onboard button is a left click.

## Hardware

| Signal | XIAO pin | nRF pin | Notes |
| --- | --- | --- | --- |
| IMU SDA | D12 | P0.04 | back-pad bus, exposed as `Wire1` |
| IMU SCL | D11 | P0.03 | back-pad bus, exposed as `Wire1` |
| IMU power gate | D20 / `IMU_MIC_EN` | P0.01 | shared IMU + microphone rail, must be HIGH |
| Left click | `PIN_BUTTON` | P0.00 | onboard button, active low |

- The IMU is only populated on **Sense** boards. On a plain (non-Sense) XIAO the sketch reports
  `imu=not-found` and never starts BLE.
- The IMU is on the back-pad I2C pair (D12/D11), **not** the D4/D5 header pair, so the sketch uses
  `Wire1`. Address is `0x6A` (`0x6B` if SA0 is pulled high).

## Build and upload

Board: **XIAO nRF54L15 / Sense**. FQBN for `arduino-cli`:

```
nrf54l15clean:nrf54l15clean:xiao_nrf54l15
```

BLE must be enabled, which is the default (`BLE Support: Enabled`). Other menu options can stay at
their defaults.

The serial log is **off by default** (`kSerialLogEnabled`), so the shipped sketch is silent and
nothing in the pointer path depends on a serial port. Every log excerpt below assumes the switch has
been turned back on:

```cpp
static const bool kSerialLogEnabled = true;  // false as shipped
```

## Startup sequence

With the log on, open the Serial Monitor at **115200** baud. On boot the sketch:

1. powers the IMU rail and verifies `WHO_AM_I`,
2. configures the sensor,
3. **measures gyro bias** — keep the board still for about a second:

```
Hold the board still, measuring gyro bias (300 samples)...
bias_dps=0.31,-0.12,0.44 spread_dps=1.20 read_err=0
```

`read_err` is how many I2C reads failed while calibrating. It should be `0` here: this
happens before the radio starts, so a non-zero value means the bus is marginal on its own.

4. starts advertising:

```
ble=advertising as "Air Mouse" (pair from the host, HID needs an encrypted link)
```

Then pair from the host: **Settings → Bluetooth → "Air Mouse"**. The console logs
`ble=connected, pairing`, and after pairing, motion and clicks start working.

The connection interval is published at **7.5 ms** (units 6/6), through the standard GAP
characteristic `0x2A04` that hosts read during service discovery. **On Android that is only a
request**, and a phone is entitled to overrule it — 30 ms is Android's own default band.

The range is deliberately tight, because a central takes the *top* of whatever range it is offered:
this sketch used to ask for 7.5–15 ms and settled at exactly 15 ms. It now asks for 7.5 ms alone, and
re-asks it a few times after encryption in case the first request landed during pairing. It never
offers a slower interval: a host that refuses 7.5 ms is left at its own choice (often 30 ms) rather
than talked down to 15 ms; reconnecting is the way out:

```
hid=asking for conn interval 7.5-7.5 ms (attempt 1)
hid=fast-conn-interval in use ci_ms=7.5
```

The periodic line carries `ci_ms=` and `tx_s=`, showing the interval the host settled on
and the pointer update rate it produces — see [Latency](#latency). A host that has already paired may
hold the old value cached, so if `ci_ms=` stays high, forget the device in the host's Bluetooth
settings and pair again.

If motion does nothing while clicks work (or vice versa), check the link is actually encrypted —
HID reports are dropped on an unencrypted link, which is why pairing is mandatory rather than
optional.

## Serial output

A debug view of the pipeline, at `kPrintIntervalMs`, with `kSerialLogEnabled = true`:

```
 roll_deg= -83.6 ptr_dps= 25.7 33.8 ptr_deg= -4.1 57.6 read_err= 1245/134200 ci_ms= 7.5 tx_s= 52
 win= 257 try= 55 pace= 16 align= 98 lat= 0 miss= 0 loop_hz= 413
```

That capture predates the pacing margin: `try` above `tx_s` is the queue being filled, and `pace` now
parks near `1000 / tx_s + 2` instead.

| Field | Meaning |
| --- | --- |
| `roll_deg` | roll about the aiming axis, from gravity — absolute and drift-free |
| `accel_mg` / `plane_g` / `gyro` | raw gravity, the gravity landing in the assumed plane, and raw rates. Only printed with `kPrintAxisDiag` on: they are the axis-mapping toolkit, see [Axis mapping](#axis-mapping--verified-on-hardware) |
| `ptr_dps` | roll-compensated pointer rates, (horizontal, vertical), deg/s, after the deadzone — so `0.0` at rest is the confirmation that nothing is creeping |
| `ptr_deg` | those rates integrated — pointer movement so far, wrapped to ±180° |
| `read_err` | failed I2C transactions / attempted. Hidden until it is non-zero |
| `ci_ms` | negotiated BLE connection interval — the report period before the radio's own limits are applied. Shown once connected |
| `tx_s` | reports the radio *accepted* per second: the pointer's real update rate. It tracks `(1000 / ci_ms) / 2.5`, **not** `1000 / ci_ms` — see [What actually sets the report rate](#what-actually-sets-the-report-rate). Measured 26–27/s at a 15 ms interval and 52/s at 7.5 ms, one report per 2.5 events at both |
| `win` | how often the send window was available. A link-health number: roughly the loop passes that landed inside the window before an event |
| `try` | reports handed to the radio per second. Once the pacing has parked it sits level with `tx_s`; more means the radio's queue was full, i.e. the sketch offered faster than the link drains |
| `pace` | the hand-over period in ms the pacing settled on. `try` is about `1000 / pace`, and `tx_s` is the rate the radio grants — the pacing aims just under it |
| `align` | share of accepted reports that went out in the aligned window rather than on the rate fallback. `100` means the loop placed every report; a low number means the fallback carried the rate |
| `lat` | slave latency in force. Only the host sets it and the sketch asks for 0, so a non-zero value is the host's doing and means it anchors less often than every event |
| `miss` | consecutive connection events the link layer failed to receive. Persistent non-zero is the device side dropping them, not the host |
| `loop_hz` | main loop passes per second — the ceiling on both the report rate and the sample rate. Below the link's own rate `(1000 / ci_ms) / 2.5` the loop, not the link, is limiting |

The sensor runs at 416 Hz, but the *effective* sample rate is one sample per main loop pass, so
`loop_hz=` is the number that matters. A sample that is not read before the next one lands is simply
overwritten in the sensor's output registers. `dt` is measured per sample, so the integration stays
correct at any rate — a slower loop costs resolution, not angle.

The line is ~150 bytes, which is ~13 ms of wire time at 115200 baud, ten times a second, and
`loop_hz` has been measured at 413 with it on — below the sensor's own 416 Hz. Whether that is the
loop blocking on the port or a TX ring buffer filling does not need to be argued about: the switch
exists so it can be measured. `printState()` is also gated on it, because the last thing that
function does is two Bluefruit link queries and each of those services the whole stack.

## I2C reliability

A 12-byte burst is roughly 110 bit times. On a real bus a few of those transactions die — a NACK
from a marginal edge, coupling from the radio, a slave that lost a bit. That is normal, and the
sketch is built so it stays normal instead of escalating:

- **Every read is retried** (`kImuReadAttempts`, 3). Failures only count if every attempt fails.
- **Slow failures are not retried.** If a transaction already burned milliseconds, it hit the
  driver's timeout; stacking more timeouts just stalls the sample loop (`kImuRetryBudgetUs`).
- **A dropped sample is free.** `dt` is measured, not assumed, so the next good sample integrates
  over its own interval. Nothing is torn down.
- **Only a sustained run of failures does anything** (`kImuConsecFaultLimit`, 32 in a row). That
  triggers bus recovery, then a check that the part still has its configuration. The gyro bias is
  **never** re-measured — that would mean asking you to hold the board still again mid-session.
- **Bus recovery is real recovery**: if the slave is holding SDA low, no transaction can even
  start, so the pins are driven by hand until it lets go and only then handed back.

Watch `read_err` in the log:

| Symptom | Likely cause |
| --- | --- |
| `read_err=0` before pairing, stays 0 | clean |
| `read_err` climbs, `stage=1` | the register-address write was NACKed — bus/electrical |
| `read_err` climbs, `stage=2` | the data phase failed — bus/electrical |
| `read_err` well above 0 and `imu=bus recovery` appears | marginal bus — but **do not** slow the clock until you have checked the rate at both speeds, see below |

```cpp
static const uint32_t kImuI2cClockHz = 400000;  // measured: slower is worse, not better
```

The obvious fix for a marginal bus is to slow it down and give the lines time to settle. **That is
measurably backwards on this hardware.** Same wiring, same firmware, connected, only the constant
changed:

| Clock | Dead transactions |
| --- | --- |
| 400 kHz | 0.53 % (4 of 756) |
| 100 kHz | 3.47 % (23 of 662) |

6.6× worse for 4× the clock period. That ratio is the whole diagnosis: the failures are **not** edges
failing to settle, they are the radio's 2.4 GHz burst landing *during* a transaction. The error rate
therefore tracks the length of the exposure window — a 13-byte burst is ~0.3 ms at 400 kHz and ~1.2 ms
at 100 kHz, and it is exposed for all of it. Slower is worse because it is exposed for longer.

The log says which case you have without any further measurement: `read_err` was **0 while unconnected**
and non-zero while connected, at both clocks. Interference. A bus with genuinely bad edges would fail
regardless of the radio.

The same argument says the error rate should track the *report rate*, since a faster pointer means the
radio is on more of the time. It does: the same bus at the same 400 kHz measured 0.53 % while
reporting at 26/s, and 0.93 % (1245 of 134200) once the pacing had taken it to 52/s.

So a slower clock is only the right answer for a bus that fails *because of* edge rate, and the test is
whether the rate improves when you slow down. Here it got worse. And it costs twice: the slower clock
also eats loop time, which is the last thing a marginal bus needs.

400 kHz is the part's specified ceiling, so there is no headroom above it to trade for a shorter window.
If `read_err` must come down, the lever is physical — shorter or shielded wiring.

One caveat that matters for what `read_err` means: the retry path makes a *failed* transaction free, but
I2C has no checksum, so a burst corrupted *without* NACKing is accepted and lands in the output as a bad
sample. That is why the error free rate is worth caring about beyond tidiness, and why 3.47 % is a much
worse trade than it looks.

## Gyro zero and creep

The bias is measured once at startup and then re-zeroed continuously, because the sensor's zero-rate
output drifts as the part warms up — a few degrees per second is normal for this part and it is the
difference between a cursor that sits still and one that slides across the screen.

The subtle part is deciding when the device is *still*. The rule this sketch used to use compared the
gyro against a magnitude threshold, and that deadlocks:

- a residual bias is a gyro reading with no rotation behind it;
- if the residual grows past the threshold, the stillness test says "moving", so the tracker stays
  off, so the residual is never removed, so the test keeps saying "moving".

Measured on hardware: the startup calibration stored `bias_dps= -0.11, -2.57, 0.01`, and some time
later the roll-compensation math put the residual corrected rate at about `(-0.21, -2.46) dps`. That
is larger than the old `kStillThresholdDps` of 1.5, so the tracker had switched off permanently and
the pointer crept at ~2.4 °/s — 38 HID counts every second, which is plainly visible.

So stillness is now judged from **gravity**, which a bias cannot fool: a gyro can report a rate with
nothing rotating, but world-up only moves if the device really rotates. The test is the angle between
the current accelerometer direction and a slow low-pass of it (`kStillMinDot`, ~0.6°), plus a check
that the magnitude is within `kStillAccelTolG` of 1 g so that translation does not pass as stillness.
Because the reference lags while the device turns and catches up when it stops, the tracker re-engages
automatically at whatever orientation the device is left in.

The trade-off that remains is unavoidable: a gyro cannot distinguish a small bias from a genuinely
slow rotation, so rotation slower than roughly a degree per second is partly absorbed. Making it
purely a bias issue costs convergence time; the current `kBiasTrackingAlpha` settles the startup
residual in a few seconds.

The tracker removes the *mean* residual, which is what creep is. It does not remove the instantaneous
noise around it, and the old magnitude gate hid that by accident: the same 1.5 °/s threshold also acted
as a deadband. `kHidDeadzoneDps` puts a deliberate one back, and it is set *above* the measured
±0.5 °/s of rest noise rather than at it, because a threshold at the noise level still passes the
noise peaks. The cost of that choice is explicit: rotation below 3 °/s produces nothing, and faster
rotation is reduced by exactly 3 °/s. It stays *soft* rather than truncating, so the response is
continuous at the edge — a deliberate 10 °/s sweep comes out as 7, not as a 3 °/s jump — and the first
knob to lower if slow aiming feels heavy. `0.0f` disables it.

## HID transport

The rules that shape the code, all of them from the core's own HID example:

- **Reports require an encrypted link.** `mouseReport()` returns `false` until the link is paired, so
  the sketch is a no-op until then.
- **Sampling and reporting are decoupled.** Motion accumulates in degrees at 416 Hz and is converted
  to counts only when a report is actually sent. Reporting at the IMU rate would just queue
  notifications and add latency.
- **Reports are aligned to the connection event.** The report is handed over just before the next event
  so it leaves in that event instead of waiting an extra interval — see [Latency](#latency).
- **The hand-over rate is paced to just under what the radio drains** (`hidMaintainPace`). The cadence
  belongs to the link, not to a timer: the radio only lets a notification out every 2–3 connection
  events, so offering one per event does not buy reports, it fills the radio's notification queue — see
  [What actually sets the report rate](#what-actually-sets-the-report-rate) for why that is and what
  measures it. That queue is the real problem — it is 8 reports deep, it drains at the radio's rate, so
  everything sitting in it arrives *after* your hand has stopped, which is a cursor that keeps going.
  Matching the rate is not enough, because a queue that is exactly full never empties: the pacing parks
  a margin *slower* than the link (`kHidPaceMarginMs`), so the queue drains and the motion stays in the
  sketch, where a stop is a stop. The rate is not derived from the interval, because it is not a clean
  function of it: measured 26–27 reports/s at a 15 ms interval (2.6 events each) and 52/s at 7.5 ms
  (2.6 each). It is measured from the refusals instead, which are the direct signal: a refusal means the
  queue was full, so in that second the radio accepted exactly what it could drain, and `1000 / tx_s` is
  the link's own period. With nothing refused there is nothing to measure, so the parked period is held
  and eased down 1 ms every `kHidPaceHoldMs` — the only way to find a link that got faster, at the cost
  of one second of refusals when the guess was wrong.
- **Deltas are signed 8-bit**, so each report is clamped to ±127 counts. Only what was really sent is
  subtracted from the accumulator, so the sub-count remainder carries over and slow motion is not
  quantised away. If a report fails, the motion stays pending for the next slot.
- **The backlog is capped** (`kHidPendingCapDeg`). If the link stalls, the cursor must not keep
  sliding after your hand has stopped; past the cap, motion is dropped.
- **Counts are rounded, not truncated.** Truncation makes a slow rotation build a whole count before
  anything is sent, which halves the update rate exactly where fine positioning happens. The
  fractional part carries either way, so the long-run average is unchanged.
- **Reports are only sent when there is something to say** (motion, a button edge, or an optional
  keepalive), never on a fixed drumbeat of zero-motion packets.

One consequence worth knowing: HID counts are integers, so a very slow steady rotation still produces
small occasional steps rather than perfectly smooth motion. That is inherent to the protocol. What the
gain does is set how much hand motion one count is worth, so a higher gain reaches that step size
sooner — see the derivation under [Configuration](#configuration).

## Latency

Pointer lag is the sum of a few independent delays and, at the end of the chain, one that is simply
not ours to remove:

| Source | Before | Now |
| --- | --- | --- |
| IMU sample period | 2.4 ms (416 Hz) | one sample per loop pass — 2.4 ms once `loop_hz` ≥ 416 |
| Gyro low-pass (`kGyroFilterTauMs`) | ~8 ms | ~3 ms |
| Report cadence | 30 ms fixed | paced to what the radio drains, just under it — ~20 ms, i.e. ~50 reports/s |
| Connection interval | 30–50 ms | the host decides; 7.5 ms here, 30 ms is Android's default |
| **Sketch-side total** | **~100 ms** | **~20–35 ms** |

Printing is *not* on that list, but it is not free either — see [Serial output](#serial-output). It is
off by default precisely so it cannot become part of the pointer's timing.

The rules that produce the "Now" column:

- **Hand the report over just before the connection event.** A report queued on a free-running timer
  has an arbitrary phase against the radio's: if it misses its event by a hair it waits a whole
  interval for the next one, and at 30 ms that is up to 30 ms of latency with nothing to do with the
  link. The radio publishes its own anchor through
  `Bluefruit.rawRadio().getConnectionTimingSnapshot()`, so the sketch sends within `kHidSendLeadUs` of
  the next anchor instead.
- **The lead is 5 ms, not 1.** The radio pre-rolls: it starts spinning 2.2 ms before the anchor
  (`kBleConnRxStartLeadUs`) and owns the loop from then on, so the window has to clear that plus the
  main loop's own granularity. Miss the window and the report waits another interval, which is why the
  lead is deliberately generous.
- **One report per hand-over period, not more.** Sending faster than the radio accepts does not get
  motion to the host any sooner: the extra notifications stack in the driver's notification queue
  behind each other, and that queue *is* the latency. The period comes from the pacing above, which
  measures what the radio takes instead of assuming it follows the event rate. The window itself is
  spaced on the radio's own microsecond clock rather than on the event counter, because only the clock
  is certain to advance.
- **A rate fallback covers a window the loop cannot reach.** Once a full period plus `kHidSendSlackMs`
  has gone by with nothing sent, the report goes out regardless of the window. That is the rate limit
  rather than the phase, and it matters specifically when the main loop is slower than the connection
  interval: then the window can only be hit by luck, and without the fallback the report rate would
  collapse to whatever the luck catches. `align=` reports the share of sends that used the window;
  `100` means the loop places every report, and a low number means the fallback is carrying the rate.
- **The loop rate is the other limit.** `loop_hz=` is the real ceiling: a report cannot be composed
  more often than the loop runs, and the IMU cannot be sampled more often either. If `loop_hz` is below
  the link's own rate, `(1000 / ci_ms) / 2.5`, the loop, not the link, is setting the update rate.
- **The loop is worth watching, because the cost was not where it looked.** Connecting used to halve it,
  724 → 357 Hz, and nothing in `hidTask` is expensive enough to explain that. The cause is a level down:
  every Bluefruit connection or security query begins with `serviceBluefruitBeforePollingState()`, which
  runs `manager().idleService()` and dispatches deferred callbacks — the same work the core already does
  once per pass in `main()`'s `nrf54l15_clean_idle_service()` and `yield()`. Asking three or four times a
  pass, at 350 passes a second, was pure duplication. Both states change only on connect, disconnect,
  pairing or a parameter update, so they are now polled and cached (`kLinkPollMs`, `kConnPollMs`).
- **The connection interval is the host's decision.** This is the one delay that cannot be designed
  away. The interval is published at 7.5 ms through GAP characteristic `0x2A04`, and the core's HAL
  queues its own update request too, but both are only *requests* — the central decides, and it takes
  the *top* of an offered range. Android's defaults are `CONNECTION_PRIORITY_BALANCED` at 30–50 ms,
  which is what a GATT client gets, and `CONNECTION_PRIORITY_HIGH` at 11.25–15 ms, which is only
  applied when a phone *app* asks for it. A HID peripheral cannot, which is why a BLE mouse does not
  feel like a wired one, on any firmware.
- **The driver asks once**, clearing its pending flag on any response, rejection included, so the sketch
  re-asks 7.5 ms (`kConnAttempts`, `kConnRetryGapMs`) in case the first request was rejected during
  pairing. Only 7.5 ms is treated as settled (`kConnFastThresholdMs`); a host that refuses it stays at
  its own interval, and no slower interval is ever offered. `ci_ms=` above 8 means the request did not
  stick, and a reconnect is the remedy.
- **Pairing is not starved by a short interval.** The core's HID example advertises 30–50 ms so LE Secure
  Connections has CPU between events, but the software P-256 keypair generation runs in the deferred
  background service, outside the connection event. This sketch therefore keeps 7.5 ms through pairing
  instead of backing off on a timer.
- **Trust but verify.** `ci_ms=` is the interval the host actually settled on and `tx_s=` the update rate
  it produces; `tx_s=` should track `(1000 / ci_ms) / 2.5`, because the radio only lets a notification
  out every 2–3 events. `align=` and `loop_hz=` then say which side is limiting — a low `align=` with
  `loop_hz` below that rate means the main loop, not the link.
- **Every millisecond of filter tau is a millisecond of lag**, which is why the gyro filter is short. It
  is not free: lowering it further trades smoothness for response.

### What actually sets the report rate

Four things are worth separating here, because only one of them is Android's:

1. **Android picks the connection interval.** This is the only part the phone decides — the sketch can
   ask (7.5 ms here; 30 ms is Android's default for a GATT client). Events per second is `1000 / ci_ms`.
2. **A peripheral only transmits inside an event the central starts**, one packet at a time. So
   events/s is the ceiling on reports/s — if every event could carry one.
3. **In this core the notification path is one packet deep, and an encrypted link cannot share an
   event.** Both read from the HAL, not inferred:
   - A notification is built into a single pending PDU slot (`connectionPendingTxValid_`), and the TX
     site says how that slot behaves: *"Replace the previous PDU only after the peer's NESN
     acknowledges it. Until then the Link Layer requires the exact same SN, ciphertext and CCM counter
     to be retransmitted."* One notification in flight at a time; the rest wait in the queue.
   - The same-event followup turnaround — the mechanism that would let a second packet share an event —
     is gated on the link **not** being encrypted (`allowPlainFollowupTurnaround`,
     `!connectionEncSessionValid_`, `nrf54l15_hal_ble_peripheral_event_tx.inc`). HID requires an
     encrypted link, so that path is always off for this sketch.

4. **What the whole chain costs is measured, not derived.** From arrival timestamps on the phone
   (`adb shell getevent -lt` — the serial counters give the rate but not its shape), the gaps between
   reports are **2 or 3 connection events**, averaging 2.5:

| Interval | Events/s | Reports/s | Events per report |
| --- | --- | --- | --- |
| 15 ms | 66.7 | 26.9 | 2.48 |
| 7.5 ms | 133 | 52 | 2.56 |

To be straight about which is which: the 2.5 is measured, twice, at two different intervals. The code
above establishes a one-packet-deep path, which explains why the rate cannot exceed one report per
event — it does **not** explain why the measured figure is 2.5 rather than 1. That residual is not
accounted for line by line anywhere here, and it is the reason the sketch measures the rate rather
than computing it.

That is why the sketch cannot simply be told `1000 / ci_ms`: on this path the honest number is
`(1000 / ci_ms) / 2.5`, and why the pacing watches refusals instead — see
[HID transport](#hid-transport). It is also why the rate is a property of the *link* rather than a
firmware constant: a core that keeps the encrypted followup turnaround, or a link that is not
encrypted, need not pay it, and nothing in this sketch can tell which case it is looking at other than
by measuring.

Anything queued beyond the one packet in flight lands in the notification queue (depth 8,
`kCustomGattNotificationQueueDepth`) and drains at that rate, one packet per turn. Eight reports at
~19 ms each is ~150 ms of motion the hand has already made, so a full queue is a cursor that keeps
going after you stop: harmless while the pacing holds it nearly empty, and exactly the lag you feel
when it does not. That is not hypothetical — one capture at 7.5 ms showed `try= 64` against `tx_s= 53`,
the queue being filled 11 times a second, which is what the pacing margin exists to stop.

What is left after this is not in the sketch: one connection interval of transmission delay is
inherent to BLE, and the host's own input pipeline (Android's `InputReader` → `InputDispatcher` →
compositor) adds a frame or two on top. A gyro air pointer will not feel like a wired mouse.

## Configuration

| Constant | Default | Effect |
| --- | --- | --- |
| `kConnIntervalMin` / `Max` | 6 / 6 | Connection interval published to the host, in 1.25 ms units → 7.5 ms. This dominates latency and update rate; raise it if a host will not pair |
| `kConnAttempts` / `kConnRetryGapMs` | 8 / 1000 | Times 7.5 ms is re-asked after encryption, and the gap between asks |
| `kConnFastThresholdMs` | 8 | Intervals at or above this are not treated as settled, so the 7.5 ms request keeps being made |
| `kHidSendLeadUs` | 5000 | How long before the next connection event the report is handed over. Must exceed the radio's 2.2 ms pre-roll plus the loop's ~1 ms granularity, but stay well under the interval |
| `kHidSendSlackMs` | 5 | How long past the paced period the send window is abandoned for the rate fallback |
| `kHidPaceWindowMs` | 1000 | Window over which the pacing counts attempts against accepted reports |
| `kHidPaceMarginMs` | 2 | Added to the measured drain period, so the queue empties and stays empty |
| `kHidPaceProbePct` | 95 | Only before a drain has been measured: the period is scaled to this percent of itself to look for headroom |
| `kHidPaceHoldMs` | 5000 | Once parked, how long before the floor is eased down 1 ms to re-probe the link |
| `kHidPaceMinRate` | 8 | Below this many reports/s a window is too quiet to pace from |
| `kHidCountsPerDeg` | 16.0 | Pointer gain, bounded by the report format — see below |
| `kHidDeadzoneDps` | 3.0 | Per-axis soft deadzone on the pointer rate, set above the measured rest noise. `0.0f` passes the raw gyro |
| `kHidPendingCapDeg` | 16 | Ceiling on unreported motion, kept near one report's worth |
| `kHidReportFloorMs` / `CeilMs` | 5 / 50 | Clamp on the paced report period |
| `kHidIdleReportEnabled` / `Ms` | false / 100 | Optional zero-motion keepalive if the host sleeps the link |
| `kButtonDebounceMs` | 20 | Button debounce window |
| `kSerialLogEnabled` | false | Master switch for every print, including the periodic line — see [Serial output](#serial-output) |
| `kImuI2cClockHz` | 400000 | IMU bus clock, at the part's ceiling. Slower is measurably *worse* here — see [I2C reliability](#i2c-reliability) |
| `kLinkPollMs` / `kConnPollMs` | 50 / 100 | How often the cached link state and the connection interval are re-read. Both are polled rather than checked per pass, because each query services the whole stack |

`kHidCountsPerDeg` is bounded from above by the report format, not by feel. One report carries at most
±127 counts (signed 8-bit), so the gain has to satisfy

```
gain <= 127 / (max_hand_rate_dps * report_period_s)
```

for the fastest motion you actually use. A quick wrist flick is around 500 deg/s and the paced period
is ~20 ms, giving `gain <= 127 / (500 * 0.020) = 12.7`. At the shipped 16.0 the ceiling is 127 / 0.020
= 6.4 kcounts/s, or **~400 deg/s** — comfortably above ordinary aiming, but a 570 deg/s flick does clip,
and the clipped part is not lost: it drains over the following reports, which is a cursor that arrives
slightly late rather than short. Lower the gain for a higher ceiling (`12` covers ~530 deg/s), and
raise it only if your motion is normally slower and you want the extra counts per degree.

Sensor configuration:

| Setting | Value | Where |
| --- | --- | --- |
| Accel range | ±4 g → 0.122 mg/LSB | `kCtrl1XlValue = 0x68` + `kAccelMgPerLsb` |
| Gyro range | ±500 dps → 17.50 mdps/LSB | `kCtrl2GValue = 0x64` + `kGyroMdpsPerLsb` |
| ODR | 416 Hz | the `0110` ODR nibble in both control registers |

Ranges are deliberately wider than ±2 g / 245 dps: a fast wrist flick clips the narrow settings, which
feels like the cursor sticking at full speed. If you change a full scale, change the matching
sensitivity constant too — they are independent numbers in the code.

Gotcha: the LSM6DS3 family encodes `FS_XL` as `00`=±2 g, `01`=±16 g, `10`=±4 g, `11`=±8 g. It is
**not** in ascending order.

### Motion tuning

| Constant | Default | Effect |
| --- | --- | --- |
| `kGyroFilterTauMs` | 3 ms | LPF on the rate signal (~53 Hz). Lower = snappier, noisier |
| `kGravityFilterTauMs` | 50 ms | LPF on the roll reference (~3 Hz). Higher = steadier roll, more lag |
| `kBiasTrackingAlpha` | 0.005 | Continuous re-zero while still. Set to 0 to disable |
| `kStillMinDot` / `kStillAccelTolG` | 0.999945 / 0.08 | How still the gravity direction and magnitude must be before re-zeroing |
| `kSerialLogEnabled` | false | Master switch for **all** serial output. Off, the sketch is silent and the periodic line's link queries do not run — see [Serial output](#serial-output) |
| `kPrintIntervalMs` | 100 | Line rate, when the log is on |
| `kPrintAxisDiag` | false | Adds `accel_mg`, `plane_g` and `gyro` to the line — for identifying `kPointAxis`, see [Axis mapping](#axis-mapping--verified-on-hardware) |

### Axis mapping — verified on hardware

The sensor axes are fixed by how the chip sits on the board, so the sketch exposes the mapping rather
than guessing. Measured on this board, lying flat and aimed at a standing phone:

| Reading | Value |
| --- | --- |
| `accel_mg` at rest | `(-84, -111, 1037)` — gravity is on **Z**, so Z is the board *normal*, not the aiming axis |
| twist about the aiming axis, aim held fixed | ~65° of rotation about **Y**, X swinging 0 → −1 g and Z 1 g → 0.4 g |
| therefore | `kPointAxis = kPointAxisY` |

Both pointer signs are `-1.0f`, verified on hardware once the axes were corrected.

**Redoing this for a different mounting.** Set `kSerialLogEnabled = true` and `kPrintAxisDiag = true`,
then read two things: hold still and find the `accel_mg` axis near ±1000 (that axis is vertical, so it
cannot be the aiming axis), and perform each gesture while watching `gyro` — the component that
responds is the axis the gesture turns about. `plane_g` should read ≈ 1.00 once the aiming axis is
right; near 0 means it is still wrong.

**Why it was wrong for so long, and why it did not look like an axis bug.** With Z as the aiming axis, a
device lying flat is *pointing at the floor* as far as the sketch is concerned — the singular pose.
Gravity fell outside the assumed plane and `plane_g` sat at `0.13–0.14`, just under
`kMinGravityInPlaneG`, so the roll reference flickered in and out of tracking instead of failing
cleanly. The pointer axes degraded to an arbitrary fixed pair of body axes: yawing (about Z) went to the
discarded channel and did nothing, while twisting (about Y) landed on an in-plane axis and appeared to
work. The report *"rolling works much better than yawing"* was itself the proof — in a correct mapping
that is impossible, because rotation about the aiming axis is discarded by construction.

Everything downstream of the axis choice is orientation-independent: roll comes from gravity in the
plane perpendicular to the aiming axis.

## Why there is no sensor fusion

For a **relative** pointer (which is what Android's HID mouse path consumes), fusion is not needed.
A mouse reports deltas, so there is no absolute frame to drift out of — you re-center by moving your
hand, exactly like lifting a physical mouse.

Fusion would only be needed for absolute pointing ("where on the screen am I aimed"), and this part
has **no magnetometer**, so yaw has no absolute reference regardless. A quaternion filter would
smooth roll/pitch but could not fix yaw.

What actually mattered, and is implemented here, is: bias calibration, wide enough ranges, a fast
enough ODR, and roll compensation so the pointer axes stay aligned with the screen when the hand
rolls.

## Known limitations

- `ptr_deg` is relative and drifts — inherent, and harmless for a relative pointer.
- Roll compensation degrades while the hand is accelerating hard (the accelerometer measures hand
  acceleration as well as gravity) and is held stale when the device points steeply up or down,
  where gravity stops defining a roll.
- The embedded FIFO and the IMU interrupt pin (`IMU_INT`, D24) are unused.
- A single HID report carries at most ±127 counts per axis, because the report fields are signed
  8-bit. With the paced ~20 ms period and the shipped gain that is a ~400 deg/s ceiling: the fastest
  flicks saturate and drain over the following reports, arriving a little late rather than short.
  Raising it means a 16-bit report, which is a change to the core's HID descriptor, not to this sketch.
- One connection interval of transmission delay is unavoidable, and the host's input pipeline adds
  more. `ci_ms=` in the log is the part of it this sketch can see. What the host settles on is its own
  decision: Android's default for a GATT client is `CONNECTION_PRIORITY_BALANCED` (30–50 ms), and the
  peripheral's published and requested values are only ever requests. In practice the request here was
  accepted and the link came up at 7.5 ms, but the host grants or refuses that, so another phone or
  Android version may well answer 30.
- A gyro cannot separate a small bias from a very slow rotation. Motion below roughly 1 °/s is therefore
  partly absorbed by the bias tracker and the deadzone. That is the sensor's precision floor, not a
  tuning failure: getting below it needs a different sensor, not a different constant.

## Next steps

1. Add a pointer-acceleration curve, per report rather than per sample, so slow movement stays precise
   while fast sweeps cover more ground without a bigger fixed gain. The per-sample soft deadzone is
   already in place (`kHidDeadzoneDps`).
2. Move the sampling path to the sensor FIFO so the effective sample rate no longer depends on how
   long the loop takes.
3. Widen the HID report to 16-bit axes, which means a custom descriptor in the core's HID layer. That
   removes the ±127-count ceiling and with it the last of the flick clipping — at that point the gain
   can be set for feel alone.
