# Air Mouse

Motion input for a Seeed XIAO nRF54L15 / Sense, built on the
[`lolren/nrf54-arduino-core`](https://github.com/lolren/nrf54-arduino-core) board package.

## Current step: onboard IMU bring-up

`Air_Mouse.ino` reads the board's built-in ST **LSM6DS3TR-C** (6-axis accelerometer + gyroscope)
and streams the data over Serial.

### Hardware

| Signal | XIAO pin | nRF pin | Notes |
| --- | --- | --- | --- |
| IMU SDA | D12 | P0.04 | back-pad bus, exposed as `Wire1` |
| IMU SCL | D11 | P0.03 | back-pad bus, exposed as `Wire1` |
| IMU power gate | D20 / `IMU_MIC_EN` | P0.01 | shared IMU + microphone rail, must be HIGH |

Notes:

- The IMU is only populated on **Sense** boards. On a plain (non-Sense) XIAO the sketch will report
  `imu=not-found`.
- The IMU is on the back-pad I2C pair (D12/D11), **not** the D4/D5 header pair, so the sketch uses
  `Wire1`. I2C address is `0x6A` (`0x6B` if SA0 is pulled high).

### Build and upload

Board: **XIAO nRF54L15 / Sense**

For `arduino-cli`, the FQBN is:

```
nrf54l15clean:nrf54l15clean:xiao_nrf54l15
```

Default menu options (BLE on, VPR on, antenna ceramic, Serial via the USB bridge) work as-is.

### Expected output

Open the Serial Monitor at **115200** baud:

```
imu=ready addr=0x6A who=0x6A range=+/-2g,245dps odr=104Hz
accel_mg=-3.0,2.0,998.0 gyro_dps=0.1,-0.2,0.0 rpy_deg=0.0,0.1,0.2
```

- `accel_mg` — acceleration on X, Y, Z in **milli-g** (at rest, one axis reads about ±1000 mg).
- `gyro_dps` — rotation rate on X, Y, Z in **degrees per second**.
- `rpy_deg` — roll/pitch/yaw integrated from the gyro, in degrees, relative to power-up.

Samples are printed at 50 Hz to keep the serial link comfortable, while the IMU itself runs at 104 Hz.

### Configuration

The registers and full-scale sensitivities live at the top of the sketch:

| Setting | Value | Where |
| --- | --- | --- |
| Accel range | ±2 g → 0.061 mg/LSB | `kAccelMgPerLsb` + `imuWrite(kRegCtrl1Xl, 0x40)` |
| Gyro range | 245 dps → 8.75 mdps/LSB | `kGyroMdpsPerLsb` + `imuWrite(kRegCtrl2G, 0x40)` |
| Sample rate | 104 Hz | the `0x40` ODR bits in both control registers |

If you change a full scale, update the matching `...PerLsb` constant too.

## Known limitations

- `rpy_deg` is pure gyro dead reckoning: it drifts and is not an absolute attitude. Combining the
  accelerometer (or a filter) is the next step.
- Only the gyro/accel are read; the LSM6DS3TR-C's embedded FIFO and interrupt pin (`IMU_INT` / D24)
  are unused.

## Next steps toward an actual air mouse

1. Add gyro bias calibration at startup (sample while still, average, subtract).
2. Fuse accel + gyro for a stable attitude, or use the sensor's own filtering.
3. Map motion to pointer deltas and report them over BLE HID (`Bluefruit52Lib` is bundled with the core).
