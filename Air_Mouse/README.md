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

`accel_mg` and `gyro_raw_dps` are also implemented in `printState()` but are currently commented
out; they are the fields to re-enable when working out `kPointAxis`. Printed at 20 Hz, which is a
display rate only — the IMU is still read at its full 416 Hz.

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

## HID transport

The rules that shape the code, all of them from the core's own HID example:

- **Reports require an encrypted link.** `mouseReport()` returns `false` until the link is paired, so
  the sketch is a no-op until then.
- **Sampling and reporting are decoupled.** Motion accumulates in degrees at 416 Hz and is converted
  to counts only when a report is actually sent (`kHidReportIntervalMs`, 30 ms). Reporting at the IMU
  rate would just queue notifications and add latency.
- **Deltas are signed 8-bit**, so each report is clamped to ±127 counts. Only what was really sent is
  subtracted from the accumulator, so the sub-count remainder carries over and slow motion is not
  quantised away. If a report fails, the motion stays pending for the next slot.
- **The backlog is capped** (`kHidPendingCapDeg`, 45°). If the link stalls, the cursor must not keep
  sliding after your hand has stopped; past the cap, motion is dropped.
- **Reports are only sent when there is something to say** (motion, a button edge, or an optional
  keepalive), never on a fixed drumbeat of zero-motion packets.

One consequence worth knowing: HID counts are integers, so very slow steady rotation produces small
occasional steps rather than perfectly smooth motion. That is inherent to the protocol, not a bug in
the accumulator.

## Configuration

| Constant | Default | Effect |
| --- | --- | --- |
| `kHidCountsPerDeg` | 6.0 | Pointer gain. Hosts apply their own acceleration on top |
| `kHidReportIntervalMs` | 30 | Report cadence; keep ≥ the connection interval |
| `kConnIntervalMin` / `Max` | 24 / 40 | 30–50 ms. `8`/`16` = 10–20 ms feels snappier if the host pairs at that rate |
| `kHidPendingCapDeg` | 45 | Ceiling on unreported motion |
| `kHidIdleReportEnabled` / `Ms` | false / 100 | Optional zero-motion keepalive if the host sleeps the link |
| `kButtonDebounceMs` | 20 | Button debounce window |

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
| `kGyroFilterTauMs` | 8 ms | LPF on the rate signal (~20 Hz). Lower = snappier, noisier |
| `kGravityFilterTauMs` | 50 ms | LPF on the roll reference (~3 Hz). Higher = steadier roll, more lag |
| `kBiasTrackingAlpha` | 0.02 | Slow re-zero while still. Set to 0 to disable |
| `kStillThresholdDps` | 1.5 | How quiet "still" must be before re-zeroing |
| `kPrintIntervalMs` | 50 | Display rate only |

### Axis mapping — tune this on hardware

The sensor axes are fixed by how the chip sits on the board, so the sketch exposes the mapping
instead of guessing:

- `kPointAxis` — which body axis points at the screen. **To find it:** re-enable `accel_mg`, point the
  device at the screen in your normal grip, and pick the axis reading near 0 mg, because gravity is
  perpendicular to it.
- `kPointerHorizontalSign` / `kPointerVerticalSign` — flip if the pointer moves the wrong way.

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
- `kPointAxis` and the two sign constants are still unverified on hardware.

## Next steps

1. Verify `kPointAxis` and the pointer signs on real hardware.
2. Add a pointer-acceleration curve and a deadzone, applied per report rather than per sample.
3. Move the sampling path to the sensor FIFO so the effective sample rate no longer depends on how
   long the loop takes.
