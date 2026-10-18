# Air Mouse

Motion input for a Seeed XIAO nRF54L15 / Sense, built on the
[`lolren/nrf54-arduino-core`](https://github.com/lolren/nrf54-arduino-core) board package.

`Air_Mouse.ino` reads the board's built-in ST **LSM6DS3TR-C** and turns the gyro into
roll-compensated "pointer rates" — the raw material a HID mouse report needs.

## Hardware

| Signal | XIAO pin | nRF pin | Notes |
| --- | --- | --- | --- |
| IMU SDA | D12 | P0.04 | back-pad bus, exposed as `Wire1` |
| IMU SCL | D11 | P0.03 | back-pad bus, exposed as `Wire1` |
| IMU power gate | D20 / `IMU_MIC_EN` | P0.01 | shared IMU + microphone rail, must be HIGH |

Notes:

- The IMU is only populated on **Sense** boards. On a plain (non-Sense) XIAO the sketch reports
  `imu=not-found`.
- The IMU is on the back-pad I2C pair (D12/D11), **not** the D4/D5 header pair, so the sketch uses
  `Wire1`. Address is `0x6A` (`0x6B` if SA0 is pulled high).

## Build and upload

Board: **XIAO nRF54L15 / Sense**. FQBN for `arduino-cli`:

```
nrf54l15clean:nrf54l15clean:xiao_nrf54l15
```

Default menu options (BLE on, VPR on, ceramic antenna, Serial via the USB bridge) work as-is.

## Startup sequence

Open the Serial Monitor at **115200** baud. On boot the sketch:

1. powers the IMU rail and verifies `WHO_AM_I`,
2. configures the sensor,
3. **measures gyro bias** — keep the board still for about a second:

```
Hold the board still, measuring gyro bias (300 samples)...
bias_dps=0.31,-0.12,0.44 spread_dps=1.20
```

The spread line is a still-detection check; if the board was moving, calibration retries (up to 3
times) rather than baking a bad offset into the pointer.

## Output

```
accel_mg=-2.0,8.0,1004.0 gyro_raw_dps=0.1,-0.3,0.0 roll_deg=-0.4 ptr_dps=-0.3,0.1 ptr_deg=-1.2,0.6
```

| Field | Meaning |
| --- | --- |
| `accel_mg` | instantaneous acceleration, body X/Y/Z, milli-g |
| `gyro_raw_dps` | instantaneous bias-corrected rotation rate, body X/Y/Z, deg/s |
| `roll_deg` | roll about the pointing axis, from gravity — absolute and drift-free |
| `ptr_dps` | roll-compensated pointer rates, (horizontal, vertical), deg/s |
| `ptr_deg` | those rates integrated — pointer movement so far, wrapped to ±180° |

Printed at 20 Hz, which is a display rate only; the IMU is still read at its full 416 Hz.

## Sensor configuration

| Setting | Value | Where |
| --- | --- | --- |
| Accel range | ±4 g → 0.122 mg/LSB | `kCtrl1XlValue = 0x68` + `kAccelMgPerLsb` |
| Gyro range | ±500 dps → 17.50 mdps/LSB | `kCtrl2GValue = 0x64` + `kGyroMdpsPerLsb` |
| ODR | 416 Hz | the `0110` ODR nibble in both control registers |

Ranges are deliberately wider than ±2 g / 245 dps: a fast wrist flick clips the narrow settings, which
feels like the cursor sticking at full speed. If you change a full scale you must change the matching
sensitivity constant — the two are independent numbers in the code.

Gotcha: the LSM6DS3 family encodes `FS_XL` as `00`=±2 g, `01`=±16 g, `10`=±4 g, `11`=±8 g. It is
**not** in ascending order.

## Axis mapping — tune this on hardware

The sensor axes are fixed by how the chip sits on the board, so the sketch exposes the mapping
instead of guessing:

- `kPointAxis` — which body axis points at the screen. **To find it:** point the device at the screen
  in your normal grip and watch `accel_mg`; the pointing axis is the one reading near 0 mg, because
  gravity is perpendicular to it.
- `kPointerHorizontalSign` / `kPointerVerticalSign` — flip if the pointer moves the wrong way.

Everything downstream of the axis choice is orientation-independent: roll is derived from gravity in
the plane perpendicular to the pointing axis, so it works whatever the mounting.

## Tuning knobs

| Constant | Default | Effect |
| --- | --- | --- |
| `kGyroFilterTauMs` | 8 ms | LPF on the rate signal (~20 Hz). Lower = snappier, noisier |
| `kGravityFilterTauMs` | 50 ms | LPF on the roll reference (~3 Hz). Higher = steadier roll, more lag |
| `kBiasTrackingAlpha` | 0.02 | slow re-zero while still. Set to 0 to disable |
| `kStillThresholdDps` | 1.5 | how quiet "still" must be before re-zeroing |
| `kPrintIntervalMs` | 50 | display rate only |

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

## Next steps

1. Send `ptr_dps` as BLE HID mouse deltas (`Bluefruit52Lib` ships with the core), accumulating
   between reports and letting each HID report carry the motion for its connection interval.
2. Add a sensitivity/pointer-acceleration curve, and a deadzone below which no report is sent.
3. Move the sampling path to the FIFO so the rate is independent of the Serial load.
