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

## Startup sequence

Open the Serial Monitor at **115200** baud. On boot the sketch:

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

The connection interval is published at 7.5–15 ms from the start, through the standard GAP
characteristic `0x2A04` that hosts read during service discovery. **On Android that is only a
request**, and a phone is entitled to overrule it — 30 ms is Android's own default band. The sketch
therefore reports what it actually got, and if the host overruled it, walks a retry ladder towards the
band Android itself reserves for high priority:

```
hid=asking for conn interval 6.0-15.0 ms (attempt 1)
hid=asking for conn interval 11.3-15.0 ms (attempt 3)
hid=asking for conn interval 15.0-15.0 ms (attempt 5)
hid=fast-conn-interval in use ci_ms=15.0
```

If pairing itself has not finished within 4 s, it stops asking and falls back to a slow interval so
the host has room to complete the exchange:

```
hid=pairing-slow, backing off conn interval
```

Either way the periodic line carries `ci_ms=` and `tx_s=`, showing the interval the host settled on
and the pointer update rate it produces — see [Latency](#latency). A host that has already paired may
hold the old value cached, so if `ci_ms=` stays high, forget the device in the host's Bluetooth
settings and pair again.

If motion does nothing while clicks work (or vice versa), check the link is actually encrypted —
HID reports are dropped on an unencrypted link, which is why pairing is mandatory rather than
optional.

## Serial output

A debug view of the pipeline, at `kPrintIntervalMs`:

```
 roll_deg= -94.6 ptr_dps= 0.0 0.0 ptr_deg= -3.4 -3.2 ci_ms= 15.0 tx_s= 7 align= 100
 loop_hz= 728
```

| Field | Meaning |
| --- | --- |
| `roll_deg` | roll about the aiming axis, from gravity — absolute and drift-free |
| `accel_mg` / `plane_g` / `gyro` | raw gravity, the gravity landing in the assumed plane, and raw rates. Only printed with `kPrintAxisDiag` on: they are the axis-mapping toolkit, see [Axis mapping](#axis-mapping--verified-on-hardware) |
| `ptr_dps` | roll-compensated pointer rates, (horizontal, vertical), deg/s, after the deadzone — so `0.0` at rest is the confirmation that nothing is creeping |
| `ptr_deg` | those rates integrated — pointer movement so far, wrapped to ±180° |
| `read_err` | failed I2C transactions / attempted. Hidden until it is non-zero |
| `ci_ms` | negotiated BLE connection interval — the floor on pointer latency and the report rate. Shown once connected |
| `tx_s` | reports actually sent per second, averaged over a second. Should track `1000 / ci_ms` |
| `align` | share of those reports that went out in the aligned window rather than on the rate fallback. `100` means the loop places every report |
| `loop_hz` | main loop passes per second — the ceiling on both the report rate and the sample rate. Below `1000 / ci_ms` the loop, not the link, is limiting |

The sensor runs at 416 Hz, but the *effective* sample rate is one sample per main loop pass, so
`loop_hz=` is the number that matters. A sample that is not read before the next one lands is simply
overwritten in the sensor's output registers. `dt` is measured per sample, so the integration stays
correct at any rate — a slower loop costs resolution, not angle.

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
as a deadband. `kHidDeadzoneDps` puts a deliberate one back, sized to the measured ±0.5 °/s of rest
noise. It is *soft* — below the threshold the axis is held at zero, above it the rate is reduced by
exactly the threshold rather than truncated at it, so the response stays continuous at the edge and the
cost is a constant rate offset instead of a lost band. A deliberate 1 °/s sweep loses 60 % of itself;
that is still milder than the old 1.5 °/s hard gate, and it is the first knob to lower if fine
positioning feels heavy. `0.0f` disables it.

## HID transport

The rules that shape the code, all of them from the core's own HID example:

- **Reports require an encrypted link.** `mouseReport()` returns `false` until the link is paired, so
  the sketch is a no-op until then.
- **Sampling and reporting are decoupled.** Motion accumulates in degrees at 416 Hz and is converted
  to counts only when a report is actually sent. Reporting at the IMU rate would just queue
  notifications and add latency.
- **Reports are aligned to the connection event.** The cadence belongs to the link, not to a timer:
  the report is handed over just before the next event so it leaves in that event instead of waiting an
  extra interval — see [Latency](#latency).
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
| Report throttle | 30 ms fixed | aligned to the connection event, ≤ 5 ms |
| Connection interval | 30–50 ms | the host decides; 15 ms here, 30 ms is Android's default |
| **Sketch-side total** | **~100 ms** | **~10–35 ms** |

Printing is *not* on that list. `HardwareSerial::write()` copies into a TX ring buffer and only spins
when the ring is full, so a 10 Hz debug line of ~70 bytes drains easily at 115200 baud and costs the
pointer nothing.

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
- **One report per connection event, not more.** Sending faster than the link does not get motion to
  the host any sooner: the extra notifications stack in the driver's notification queue behind each
  other, and that queue *is* the latency. The window is spaced on the radio's own microsecond clock
  rather than on the event counter, because only the clock is certain to advance.
- **A rate fallback covers a window the loop cannot reach.** Once a full interval has gone by with
  nothing sent, the report goes out regardless of the window. That is the rate limit rather than the
  phase, and it matters specifically when the main loop is slower than the connection interval: then
  the window can only be hit by luck, and without the fallback the report rate would collapse to
  whatever the luck catches. `align=` reports the share of sends that used the window; `100` means the
  loop places every report, and a low number means the fallback is carrying the rate.
- **The loop rate is the other limit.** `loop_hz=` is the real ceiling: a report cannot be composed
  more often than the loop runs, and the IMU cannot be sampled more often either. If `loop_hz` is below
  `1000 / ci_ms`, the loop, not the link, is setting the update rate.
- **The loop is worth watching, because the cost was not where it looked.** Connecting used to halve it,
  724 → 357 Hz, and nothing in `hidTask` is expensive enough to explain that. The cause is a level down:
  every Bluefruit connection or security query begins with `serviceBluefruitBeforePollingState()`, which
  runs `manager().idleService()` and dispatches deferred callbacks — the same work the core already does
  once per pass in `main()`'s `nrf54l15_clean_idle_service()` and `yield()`. Asking three or four times a
  pass, at 350 passes a second, was pure duplication. Both states change only on connect, disconnect,
  pairing or a parameter update, so they are now polled and cached (`kLinkPollMs`, `kConnPollMs`).
- **The connection interval is the host's decision.** This is the one delay that cannot be designed
  away. The interval is published at 7.5–15 ms through GAP characteristic `0x2A04`, and the core's HAL
  queues its own update request too, but both are only *requests* — the central decides. Android's
  defaults are `CONNECTION_PRIORITY_BALANCED` at 30–50 ms, which is what a GATT client gets, and
  `CONNECTION_PRIORITY_HIGH` at 11.25–15 ms, which is only applied when a phone *app* asks for it. A HID
  peripheral cannot, which is why a BLE mouse does not feel like a wired one, on any firmware.
- **The driver asks once**, clearing its pending flag on any response, rejection included, so the sketch
  walks a retry ladder (`kConnEscalation`) from 7.5–15 ms towards Android's `HIGH` band in case a host
  only accepts a range it already supports.
- **Pairing is not starved by a short interval.** The core's HID example advertises 30–50 ms so LE Secure
  Connections has CPU between events, but the software P-256 keypair generation runs in the deferred
  background service, outside the connection event. A 4 s grace period still watches for a host that
  stalls and backs the interval off (`kPairingGraceMs`, `hid=pairing-slow`).
- **Trust but verify.** `ci_ms=` is the interval the host actually settled on and `tx_s=` the update rate
  it produces; `tx_s=` should track `1000 / ci_ms`. `align=` and `loop_hz=` then say which side is
  limiting — a low `align=` with `loop_hz` below `1000 / ci_ms` means the main loop, not the link.
- **Every millisecond of filter tau is a millisecond of lag**, which is why the gyro filter is short. It
  is not free: lowering it further trades smoothness for response.

What is left after this is not in the sketch: one connection interval of transmission delay is
inherent to BLE, and the host's own input pipeline (Android's `InputReader` → `InputDispatcher` →
compositor) adds a frame or two on top. A gyro air pointer will not feel like a wired mouse.

## Configuration

| Constant | Default | Effect |
| --- | --- | --- |
| `kConnIntervalMin` / `Max` | 6 / 12 | Connection interval published to the host, in 1.25 ms units → 7.5–15 ms. This dominates latency and update rate; raise it if a host will not pair |
| `kPairingGraceMs` | 4000 | Give up on fast if pairing has not finished by then |
| `kConnEscalation` | 6–12, 9–12, 12–12 | Retry ladder for the interval, tried in order. Ends inside Android's `CONNECTION_PRIORITY_HIGH` band (11.25–15 ms) |
| `kConnAttemptsPerStep` / `GapMs` | 2 / 1000 | Attempts per ladder step, and the gap between them |
| `kHidSendLeadUs` | 5000 | How long before the next connection event the report is handed over. Must exceed the radio's 2.2 ms pre-roll plus the loop's ~1 ms granularity, but stay well under the interval |
| `kHidCountsPerDeg` | 16.0 | Pointer gain, bounded by the report format — see below |
| `kHidDeadzoneDps` | 0.6 | Per-axis soft deadzone on the pointer rate, sized to the measured rest noise. `0.0f` passes the raw gyro |
| `kHidPendingCapDeg` | 16 | Ceiling on unreported motion, kept near one report's worth |
| `kHidReportFloorMs` / `CeilMs` | 5 / 50 | Clamp on the derived report period |
| `kHidIdleReportEnabled` / `Ms` | false / 100 | Optional zero-motion keepalive if the host sleeps the link |
| `kButtonDebounceMs` | 20 | Button debounce window |
| `kImuI2cClockHz` | 400000 | IMU bus clock, at the part's ceiling. Slower is measurably *worse* here — see [I2C reliability](#i2c-reliability) |
| `kLinkPollMs` / `kConnPollMs` | 50 / 100 | How often the cached link state and the connection interval are re-read. Both are polled rather than checked per pass, because each query services the whole stack |

`kHidCountsPerDeg` is bounded from above by the report format, not by feel. One report carries at most
±127 counts (signed 8-bit), so the gain has to satisfy

```
gain <= 127 / (max_hand_rate_dps * report_period_s)
```

for the fastest motion you actually use. A quick wrist flick is around 500 deg/s and the report period
is at best ~15 ms, giving `gain <= 127 / (500 * 0.015) = 16.9`. Above that the report saturates and
the leftover motion has to drain over later reports, which is exactly the lag this sketch exists to
remove. So raise it only if your motion is normally slower than that, and lower it if the cursor
overshoots.

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
| `kPrintIntervalMs` | 100 | Display rate only. Printing is ring-buffered, so this is not a latency knob |
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

**Redoing this for a different mounting.** Set `kPrintAxisDiag = true` and read two things: hold still
and find the `accel_mg` axis near ±1000 (that axis is vertical, so it cannot be the aiming axis), then
perform each gesture while watching `gyro` — the component that responds is the axis the gesture turns
about. `plane_g` should read ≈ 1.00 once the aiming axis is right; near 0 means it is still wrong.

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
- One connection interval of transmission delay is unavoidable, and the host's input pipeline adds
  more. `ci_ms=` in the log is the part of it this sketch can see. What the host settles on is its own
  decision: Android's default for a GATT client is `CONNECTION_PRIORITY_BALANCED` (30–50 ms), and the
  peripheral's published and requested values are only ever requests. In practice the request here was
  accepted and the link came up at 15 ms, but the host grants or refuses that, so another phone or
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
