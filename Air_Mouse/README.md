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

While idle-rate logging is on, the sketch prints a debug view:

```
 roll_deg= -0.4 ptr_dps= -0.3 0.1 ptr_deg= -1.2 0.6
```

| Field | Meaning |
| --- | --- |
| `roll_deg` | roll about the pointing axis, from gravity — absolute and drift-free |
| `ptr_dps` | roll-compensated pointer rates, (horizontal, vertical), deg/s |
| `ptr_deg` | those rates integrated — pointer movement so far, wrapped to ±180° |
| `read_err` | failed I2C transactions / attempted. Hidden until it is non-zero |
| `ci_ms` | negotiated BLE connection interval — the floor on pointer latency and the report rate. Shown once connected |
| `tx_s` | reports actually sent per second, averaged over a second. Should track `1000 / ci_ms` |
| `align` | share of those reports that went out in the aligned window rather than on the rate fallback. `100` means the loop places every report |
| `loop_hz` | main loop passes per second — the ceiling on both the report rate and the sample rate |
| `i2c_us` | average cost of one IMU transaction |
| `wait_us` | average cost of the idle wait while the sensor has nothing ready |
| `imu_task_us` | average cost of the whole `imuTask()` half of the loop (status poll, sample read, motion update) |
| `hid_task_us` | average cost of the whole `hidTask()` half |
| `ev_hz` | connection events per second as counted by the loop. Should equal `1000 / ci_ms`; much higher means `millis()` is running fast |

`accel_mg` and `gyro_raw_dps` are also implemented in `printState()` but are currently commented
out; they are the fields to re-enable when working out `kPointAxis`. Printed at 10 Hz, which is a
display rate only.

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
| `read_err` well above 0 and `imu=bus recovery` appears | lower `kImuI2cClockHz` to `100000` first |

```cpp
static const uint32_t kImuI2cClockHz = 400000;  // try 100000 if read_err climbs
```

400 kHz is the sensor's ceiling. A slower clock gives the lines time to settle, which is the first
thing to try for a marginal bus — and, unlike everything else here, it costs nothing.

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
| Connection interval | 30–50 ms | the host decides; 30 ms on Android |
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
  rather than on the event counter, because only the clock is certain to advance; `ev_hz=` reports the
  counter separately, so a disagreement between the two would be visible.
- **A rate fallback covers a window the loop cannot reach.** Once a full interval has gone by with
  nothing sent, the report goes out regardless of the window. That is the rate limit rather than the
  phase, and it matters specifically when the main loop is slower than the connection interval: then
  the window can only be hit by luck, and without the fallback the report rate would collapse to
  whatever the luck catches. `align=` reports the share of sends that used the window; `100` means the
  loop places every report, and a low number means the fallback is carrying the rate.
- **The loop rate is the other limit.** `loop_hz=` is the real ceiling: a report cannot be composed
  more often than the loop runs, and the IMU cannot be sampled more often either. If `loop_hz` is below
  `1000 / ci_ms`, the loop, not the link, is setting the update rate.
- **The connection interval is the host's decision.** This is the one delay that cannot be designed
  away. The interval is published at 7.5–15 ms through the standard GAP characteristic `0x2A04`
  (Peripheral Preferred Connection Parameters), and the core's HAL also queues its own update request
  when a host connects outside that range, but both are only *requests* — the central decides, and a
  peripheral cannot force it.

  Android's own defaults, from its stack documentation:

  | Priority | Interval |
  | --- | --- |
  | `CONNECTION_PRIORITY_BALANCED` | 30–50 ms |
  | `CONNECTION_PRIORITY_HIGH` | 11.25–15 ms |

  `BALANCED` is what a GATT client gets by default, which is where `ci_ms= 30.0` comes from. `HIGH` is
  only applied when a phone *app* calls `requestConnectionPriority(CONNECTION_PRIORITY_HIGH)`; a HID
  peripheral has no way to ask for it. That is why a BLE mouse on Android does not feel like a wired
  one, on any firmware.
- **The driver asks once.** The core's HAL sends the update request and clears its pending flag on any
  response, rejection included, and never retries. So the sketch walks a retry ladder
  (`kConnEscalation`) that starts at 7.5–15 ms and ends squarely inside Android's `HIGH` band, in case
  a host only accepts a range it already supports.
- **Pairing is not starved by a short interval.** The core's HID example advertises 30–50 ms so that
  LE Secure Connections has CPU between connection events, but that reasoning does not hold here: the
  software P-256 keypair generation runs in the deferred background service
  (`BleRadio::ensureSecureConnectionsLocalKeypair`), outside the connection event. A 4 s grace period
  still watches for a host that stalls, and backs the interval off if one does
  (`kPairingGraceMs`, `hid=pairing-slow`).
- **Trust but verify.** The periodic line reports `ci_ms=` (the interval the host actually settled on)
  and `tx_s=` (the pointer update rate it produces). `tx_s=` should track `1000 / ci_ms`; if it is well
  below that, something is dropping reports. `ci_ms= 30.0` with `tx_s= 33` means the host overruled the
  request, and that is the floor this design can reach. `align=` and `loop_hz=` then say which side is
  limiting: a low `align=` with `loop_hz` below `1000 / ci_ms` means the main loop, not the link.
- **Every millisecond of filter tau is a millisecond of lag** added to the whole chain, which is why
  the gyro filter is short. It is not free: lowering it further trades smoothness for response.

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
| `kHidPendingCapDeg` | 16 | Ceiling on unreported motion, kept near one report's worth |
| `kHidReportFloorMs` / `CeilMs` | 5 / 50 | Clamp on the derived report period |
| `kHidIdleReportEnabled` / `Ms` | false / 100 | Optional zero-motion keepalive if the host sleeps the link |
| `kButtonDebounceMs` | 20 | Button debounce window |

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

### Axis mapping — tune this on hardware

The sensor axes are fixed by how the chip sits on the board, so the sketch exposes the mapping
instead of guessing:

- `kPointAxis` — which body axis points at the screen. **To find it:** re-enable `accel_mg`, point the
  device at the screen in your normal grip, and pick the axis reading near 0 mg, because gravity is
  perpendicular to it.
- `kPointerHorizontalSign` / `kPointerVerticalSign` — flip if the pointer moves the wrong way. As
  verified on the current board: horizontal is `-1.0f` (it was inverted), vertical is `1.0f`.

Everything downstream of the axis choice is orientation-independent: roll comes from gravity in the
plane perpendicular to the pointing axis, so it works whatever the mounting.

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
  more. `ci_ms=` in the log is the part of it this sketch can see. On Android the interval is 30 ms
  because that is the platform's `CONNECTION_PRIORITY_BALANCED` band; the peripheral's published and
  requested values are only ever requests.
- `kPointAxis` is still unverified on hardware; the pointer signs were verified on the current board.

## Next steps

1. Verify `kPointAxis` on real hardware (re-enable `accel_mg`).
2. Add a pointer-acceleration curve and a deadzone, applied per report rather than per sample.
3. Move the sampling path to the sensor FIFO so the effective sample rate no longer depends on how
   long the loop takes.
