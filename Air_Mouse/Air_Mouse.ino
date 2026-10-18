/*
 * Air_Mouse - Step 3: BLE HID mouse
 *
 * Board : Seeed XIAO nRF54L15 / Sense
 * Core  : lolren/nrf54-arduino-core  (board "XIAO nRF54L15 / Sense", variant xiao_nrf54l15)
 * IMU   : ST LSM6DS3TR-C on the back-pad bus, exposed as Wire1 (SDA = D12, SCL = D11), addr 0x6A
 *
 * Step 3 adds BLE HID: the pointer rates now drive a real Bluetooth mouse that
 * Android sees as an ordinary input device. Press the onboard button for a left
 * click. The IMU pipeline is untouched, the HID layer only consumes its output.
 *
 * The transport rules that matter, and that the code below follows:
 *   - Reports are only accepted once the link is encrypted, so the device must
 *     pair first (Just Works, NoInputNoOutput - a mouse has no keypad).
 *   - Motion is accumulated in degrees at the full 416 Hz and converted to
 *     counts only when a report is actually sent. Sending at the IMU rate would
 *     just queue notifications and add latency.
 *   - HID relative X/Y are signed 8-bit, so a delta is clamped to +/-127.
 *     Only what was really sent is subtracted from the accumulator, so the
 *     sub-count remainder carries over and slow movement stays smooth.
 *   - If a report fails to go out, the motion behind it stays pending.
 *
 * I2C resilience, which is what the sensor side is built around:
 *   - A 12-byte burst is ~110 bit times, so a marginal bus drops the odd
 *     transaction. That is a bus property, not a bug, and it must not be
 *     treated as "the sensor is gone".
 *   - Reads are retried, and a failure that already cost milliseconds (the
 *     driver timeout) is not retried at all, because stacking more timeouts
 *     only stalls the sample loop.
 *   - A dropped sample is free: dt is measured, so the next good sample
 *     integrates over its own interval.
 *   - Only a sustained run of failures triggers anything, and even then it is
 *     bus recovery plus a configuration re-check. The gyro bias is never
 *     re-measured, because re-measuring it would mean asking the user to hold
 *     the board still again in the middle of a session.
 *
 * Step 2 added:
 *   - wider full scales and a faster ODR for pointer work: +/-4 g, +/-500 dps, 416 Hz.
 *     A quick wrist flick clips +/-2 g / 245 dps, which feels like the cursor
 *     "sticking" at full speed.
 *   - gyro bias calibration at startup, so a still hand does not make the
 *     cursor creep. The dominant error source in an air mouse is not noise,
 *     it is bias.
 *   - a slow bias tracker that keeps re-zeroing while the device is held still.
 *   - roll compensation: the accelerometer's gravity vector gives the roll of
 *     the device about its pointing axis, which is used to rotate the gyro
 *     rates into screen-aligned horizontal/vertical pointer rates.
 *
 * What this deliberately does NOT do:
 *   - No magnetometer exists on this part, so there is no absolute yaw. The
 *     integrated pointer angles are relative and drift; that is fine, because a
 *     mouse is a relative device.
 *   - No quaternion/Kalman fusion. It is not needed for a relative pointer, and
 *     it would not fix yaw drift anyway.
 *
 * Serial output at 115200 is a debug view of the pipeline:
 *   accel_mg      instantaneous accel, body X,Y,Z in milli-g
 *   gyro_raw_dps  instantaneous bias-corrected gyro, body X,Y,Z in deg/s
 *   roll_deg      roll about the pointing axis, from gravity (drift free)
 *   ptr_dps       roll-compensated pointer rates, (horizontal, vertical) in deg/s
 *   ptr_deg       those rates integrated, wrapped to +/-180 deg (the pointer movement)
 *   read_err      failed I2C transactions / attempted, shown only once non-zero
 *
 * Before running: hold the board still, it measures the gyro bias first.
 * Then pair with "Air Mouse" from the host's Bluetooth settings.
 *
 * Requires the "BLE Support: Enabled" board menu option (the default).
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <bluefruit.h>

// ---------------------------------------------------------------------------
// Board wiring
// ---------------------------------------------------------------------------

// Onboard Sense IMU bus (SDA = D12, SCL = D11).
static TwoWire& imuBus = Wire1;

// LSM6DS3TR-C register map (only the registers this sketch touches).
static const uint8_t kRegWhoAmI  = 0x0F;  // WHO_AM_I, expected 0x6A
static const uint8_t kRegCtrl1Xl = 0x10;  // accel ODR + full scale
static const uint8_t kRegCtrl2G  = 0x11;  // gyro ODR + full scale
static const uint8_t kRegCtrl3C  = 0x12;  // block data update + auto increment
static const uint8_t kRegStatus  = 0x1E;  // data-ready flags
static const uint8_t kRegOutXlG  = 0x22;  // OUTX_L_G .. OUTZ_H_XL (12 bytes)

static const uint8_t kExpectedWhoAmI = 0x6A;
static const uint8_t kImuAddresses[] = {0x6A, 0x6B};

// ---------------------------------------------------------------------------
// Sensor configuration
// ---------------------------------------------------------------------------

// Both control registers pack ODR in bits 7:4 and the full scale in bits 3:2.
//
// Watch out: the LSM6DS3 family encodes FS_XL as 00=+/-2 g, 01=+/-16 g,
// 10=+/-4 g, 11=+/-8 g -- it is deliberately NOT in ascending order.
//   CTRL1_XL = 0x68 -> ODR 0110 = 416 Hz, FS 10 = +/-4 g
//   CTRL2_G  = 0x64 -> ODR 0110 = 416 Hz, FS 01 = +/-500 dps
static const uint8_t kCtrl1XlValue = 0x68;
static const uint8_t kCtrl2GValue  = 0x64;

// Sensitivities that match the two values above. If you change a range,
// change the matching constant.
static const float kAccelMgPerLsb   = 0.122f;   // +/-4 g
static const float kGyroMdpsPerLsb  = 17.50f;   // +/-500 dps
static const float kAccelGPerLsb    = kAccelMgPerLsb / 1000.0f;

// Data-ready bits in the status register.
static const uint8_t kGyroDataReady  = 0x02;
static const uint8_t kAccelDataReady = 0x01;

// ---------------------------------------------------------------------------
// I2C reliability
// ---------------------------------------------------------------------------

// The sensor's I2C ceiling. This bus is short, but the failure mode of a
// marginal one (edge rates, coupling from the BLE transmitter) is a single
// NACK that kills a whole multi-byte burst. If the log shows read errors, drop
// this to 100000 before suspecting anything else in the code.
static const uint32_t kImuI2cClockHz = 400000;

// A burst of 12 bytes is ~110 bit times, so even a tiny per-bit error rate
// shows up as an occasional dead transaction. That is a property of the bus,
// not a bug, and it must not be treated as "the sensor is gone": retry before
// believing it. Only quick failures are worth retrying -- a transaction that
// already burned milliseconds hit the driver timeout, and stacking more of
// those just stalls the sample loop.
static const uint8_t  kImuReadAttempts  = 3;
static const uint32_t kImuRetryGapUs    = 50;
static const uint32_t kImuRetryBudgetUs = 3000;

// Consecutive dead transactions before the bus is reset, and the number of
// failed resets before the part is treated as genuinely lost and looked for
// again. The gyro bias is never re-measured: it describes the part, and
// re-measuring it needs the board held still, which is not something to demand
// from the user mid-session.
static const uint16_t kImuConsecFaultLimit = 32;
static const uint8_t  kImuRecoverLimit     = 8;

// ---------------------------------------------------------------------------
// Axis mapping - the part you verify on hardware
// ---------------------------------------------------------------------------

// Which body axis points at the screen when the device is held naturally.
// To find it: point the device at the screen and look at accel_mg. The pointing
// axis is the one reading close to 0 mg (gravity is perpendicular to it).
enum PointAxis : uint8_t { kPointAxisX, kPointAxisY, kPointAxisZ };
static const PointAxis kPointAxis = kPointAxisZ;

// Flip these if the pointer moves the wrong way. Start here, then tune on hardware.
static const float kPointerHorizontalSign = 1.0f;
static const float kPointerVerticalSign   = 1.0f;

// The roll reference is only meaningful while gravity projects into the plane
// around the pointing axis. Pointing steeply up or down makes it ill-defined,
// so the last good value is held below this magnitude.
static const float kMinGravityInPlaneG = 0.15f;

// ---------------------------------------------------------------------------
// Tuning
// ---------------------------------------------------------------------------

// Bias calibration at startup.
static const uint16_t kCalibrationSamples    = 300;   // ~0.7 s at 416 Hz
static const float    kCalibrationMaxSpreadDps = 4.0f; // peak-to-peak, rejects a moving hand
static const uint8_t  kCalibrationMaxAttempts = 3;
static const uint32_t kCalibrationTimeoutMs   = 2000;

// Continuous re-zero while held still. Prevents slow creep between flicks.
// Set kBiasTrackingAlpha to 0 to disable.
static const float    kStillThresholdDps = 1.5f;
static const uint16_t kStillSamples      = 150;   // ~0.36 s at 416 Hz
static const float    kBiasTrackingAlpha = 0.02f;

// Low-pass filters. The gyro branch is fast (it carries the motion), the
// accel/gravity branch is slow (it only needs to track roll, and it is
// corrupted by hand acceleration). Every millisecond of tau here is a
// millisecond of pointer lag added to the whole chain, so the gyro filter is
// kept short: at 416 Hz, 4 ms is still ~20 samples of averaging.
static const float kGyroFilterTauMs    = 4.0f;   // ~40 Hz cutoff
static const float kGravityFilterTauMs = 50.0f;  // ~3 Hz cutoff

// Display rate. This is not the sampling rate: the IMU is still read at its
// full 416 Hz, this only throttles what gets printed. Keep it slow -- on this
// core Serial writes block until the DMA transmit finishes, so every printed
// line is time the pointer cannot report in.
static const uint32_t kPrintIntervalMs = 100;  // 10 Hz

// ---------------------------------------------------------------------------
// HID transport
// ---------------------------------------------------------------------------

static const char kDeviceName[] = "Air Mouse";

// Pointer gain: HID counts per degree of rotation. Sensible starting point;
// hosts apply their own pointer acceleration on top of this.
static const float kHidCountsPerDeg = 6.0f;

// Report cadence is derived from the link rather than fixed: one report per
// connection event. Sending faster does not reduce latency, it only stacks
// notifications behind each other, and that queue *is* the latency. These
// bounds only guard the arithmetic.
static const uint32_t kHidReportFallbackMs = 15;  // before the interval is known
static const uint32_t kHidReportFloorMs    = 5;
static const uint32_t kHidReportCeilMs     = 50;

// Connection interval, in 1.25 ms units. Two phases, because pairing and
// pointing want opposite things:
//   pairing - LE Secure Connections runs a software P-256 exchange and needs
//             CPU between connection events. The core's own HID example uses
//             30-50 ms here for exactly that reason, so start there.
//   running - once the link is encrypted there is nothing left to carry but
//             pointer data, and the interval is most of the end-to-end
//             latency. 7.5-15 ms is the band the core's NUS transport asks
//             for, so it is known to work with real centrals on this stack.
static const uint16_t kConnIntervalPairMin = 24;  // 30 ms
static const uint16_t kConnIntervalPairMax = 40;  // 50 ms
static const uint16_t kConnIntervalRunMin  = 6;   // 7.5 ms
static const uint16_t kConnIntervalRunMax  = 12;  // 15 ms

// Cap on unreported motion. If the link stalls for a second, an uncapped
// accumulator would keep the cursor moving after the hand has stopped. Past
// this the backlog is dropped instead.
static const float kHidPendingCapDeg = 45.0f;

// Onboard button, active low, as left click.
static const uint8_t  kButtonPin        = PIN_BUTTON;
static const uint32_t kButtonDebounceMs = 20;

// Some hosts like a periodic zero-motion report to stay awake. The core's HID
// example sends one every 100 ms. Off by default; enable it if the pointer
// stops responding after the device has been idle.
static const bool     kHidIdleReportEnabled = false;
static const uint32_t kHidIdleReportMs      = 100;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static uint8_t g_imuAddress = 0;  // 0 while the IMU has not been found
static uint8_t g_whoAmI     = 0;

static float g_gyroBiasDps[3] = {0.0f, 0.0f, 0.0f};
static bool  g_biasValid      = false;

// I2C health. These exist so the serial log can say whether the sensor is
// babbling or gone, which are very different problems.
static uint32_t g_imuReads        = 0;  // transactions attempted
static uint32_t g_imuReadErrors   = 0;  // transactions that failed every retry
static uint16_t g_imuConsecErrors = 0;  // consecutive failures, cleared by a success
static uint8_t  g_imuRecoveries   = 0;  // failed bus resets since the last clean reply

static uint32_t g_lastSampleUs = 0;
static uint32_t g_lastPrintMs  = 0;
static bool     g_filtersPrimed = false;

// Filter state.
static float g_gyroDps[3]      = {0.0f, 0.0f, 0.0f};  // bias-corrected, filtered, body axes
static float g_planeAccelG[2]  = {0.0f, 1.0f};        // filtered gravity in the pointing plane
static float g_accelMg[3]      = {0.0f, 0.0f, 0.0f};  // instantaneous, body axes

// Outputs.
static float g_rollRad         = 0.0f;          // roll in radians, drives the rotation
static float g_rollDeg         = 0.0f;
static float g_ptrRateDps[2]   = {0.0f, 0.0f};  // {horizontal, vertical}
static float g_ptrDeg[2]       = {0.0f, 0.0f};  // integrated, wrapped

static uint16_t g_stillCount = 0;

// ---------------------------------------------------------------------------
// HID state
// ---------------------------------------------------------------------------

// BLE peripheral objects. BLEDis and BLEBas are carried because the core's own
// HID example advertises HIDS alongside them and notes that strict Android
// hosts compare against that exact shape.
BLEDis       bledis;
BLEBas       blebas;
BLEHidAdafruit blehid;

// Pointer movement accumulated but not yet reported, in degrees. This is the
// real accumulator; g_ptrDeg above is only a wrapped display copy.
static float g_pendingDeg[2] = {0.0f, 0.0f};

static uint32_t g_hidLastReportMs = 0;
static uint32_t g_hidLastKeepaliveMs = 0;
static uint8_t  g_hidLastButtons  = 0;

// The fast connection interval is requested once per connection, after the
// link is encrypted. Requesting it repeatedly would queue an update request
// on every pass.
static bool g_hidRunParamsRequested = false;

// Debounced button state (see hidPollButton).
static bool     g_buttonRaw      = false;
static bool     g_buttonStable   = false;
static uint32_t g_buttonChangeMs = 0;

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

// Spelled out rather than using M_PI, which newlib only exposes under
// feature-test macros.
static const float kPi        = 3.14159265358979f;
static const float kRadToDeg  = 180.0f / kPi;
static const float kDegToRad  = kPi / 180.0f;

static float wrap180(float deg) {
  while (deg > 180.0f) {
    deg -= 360.0f;
  }
  while (deg < -180.0f) {
    deg += 360.0f;
  }
  return deg;
}

// First-order low-pass coefficient for the given time constant and step.
static float alphaFor(float tauMs, float dtSec) {
  if (tauMs <= 0.0f) {
    return 1.0f;  // filter disabled
  }
  const float tau = tauMs / 1000.0f;
  float alpha = 1.0f - expf(-dtSec / tau);
  if (alpha < 0.0f) {
    alpha = 0.0f;
  }
  if (alpha > 1.0f) {
    alpha = 1.0f;
  }
  return alpha;
}

// ---------------------------------------------------------------------------
// I2C helpers
// ---------------------------------------------------------------------------

// One raw read transaction. Its two phases fail for different reasons and the
// distinction is the whole diagnosis, so they are reported separately:
//   stage 1 - the register-address write (endTransmission(false))
//   stage 2 - the data read (requestFrom)
//   stage 3 - the driver returned fewer bytes than it promised
static bool imuReadOnce(uint8_t reg, uint8_t* buffer, size_t length,
                        uint8_t* stageOut, uint8_t* codeOut) {
  imuBus.beginTransmission(g_imuAddress);
  imuBus.write(reg);

  const uint8_t writeCode = imuBus.endTransmission(false);  // repeated start
  if (writeCode != 0) {
    if (stageOut != nullptr) *stageOut = 1;
    if (codeOut != nullptr) *codeOut = writeCode;
    return false;
  }

  const uint8_t received =
      imuBus.requestFrom(static_cast<int>(g_imuAddress), static_cast<int>(length));
  if (received != static_cast<uint8_t>(length)) {
    if (stageOut != nullptr) *stageOut = 2;
    if (codeOut != nullptr) *codeOut = received;
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    if (imuBus.available() <= 0) {
      if (stageOut != nullptr) *stageOut = 3;
      if (codeOut != nullptr) *codeOut = 0;
      return false;
    }
    buffer[i] = static_cast<uint8_t>(imuBus.read());
  }
  return true;
}

// Unwedge the bus. A slave interrupted mid-byte holds SDA low, and until it
// sees clocks again no transaction can even start, so no amount of retrying
// helps. Drive SCL by hand until it lets go, then hand the pins back.
static void imuBusRecover() {
  imuBus.end();

  pinMode(PIN_WIRE1_SDA, INPUT_PULLUP);
  pinMode(PIN_WIRE1_SCL, INPUT_PULLUP);
  delayMicroseconds(10);

  if (digitalRead(PIN_WIRE1_SDA) == LOW) {
    pinMode(PIN_WIRE1_SCL, OUTPUT);
    for (uint8_t i = 0; i < 9; ++i) {
      digitalWrite(PIN_WIRE1_SCL, LOW);
      delayMicroseconds(5);
      digitalWrite(PIN_WIRE1_SCL, HIGH);
      delayMicroseconds(5);
    }

    // A STOP, so the next START is unambiguous: SDA rises while SCL is high.
    pinMode(PIN_WIRE1_SDA, OUTPUT);
    digitalWrite(PIN_WIRE1_SDA, LOW);
    delayMicroseconds(5);
    digitalWrite(PIN_WIRE1_SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(PIN_WIRE1_SDA, HIGH);
    delayMicroseconds(5);
  }

  pinMode(PIN_WIRE1_SCL, INPUT);
  pinMode(PIN_WIRE1_SDA, INPUT);

  imuBus.begin();
  imuBus.setClock(kImuI2cClockHz);
  delay(1);  // let the re-enabled peripheral settle before trusting a reply
}

static bool imuWrite(uint8_t reg, uint8_t value) {
  imuBus.beginTransmission(g_imuAddress);
  imuBus.write(reg);
  imuBus.write(value);
  return imuBus.endTransmission() == 0;
}

// Retrying wrapper around imuReadOnce. A dropped sample costs the pointer
// nothing -- dt is measured, so the next good sample integrates over its own
// interval -- so the priority here is that a transient failure stays
// transient, not that it never happens.
static bool imuRead(uint8_t reg, uint8_t* buffer, size_t length) {
  if (buffer == nullptr || length == 0) {
    return false;
  }

  ++g_imuReads;

  uint8_t stage = 0;
  uint8_t code  = 0;
  const uint32_t startUs = micros();

  for (uint8_t attempt = 0; attempt < kImuReadAttempts; ++attempt) {
    if (imuReadOnce(reg, buffer, length, &stage, &code)) {
      g_imuConsecErrors = 0;
      return true;
    }
    if (micros() - startUs >= kImuRetryBudgetUs) {
      break;  // slow failure: it was a timeout, more waits will not help
    }
    delayMicroseconds(kImuRetryGapUs);
  }

  ++g_imuReadErrors;
  if (g_imuConsecErrors < 0xFFFF) {
    ++g_imuConsecErrors;
  }

  // Report the first failure and then every 200th, so a flaky bus is visible
  // without burying the motion readout under it.
  if (g_imuReadErrors == 1 || (g_imuReadErrors % 200) == 0) {
    Serial.print("imu=read-error stage=");
    Serial.print(stage);
    Serial.print(" code=");
    Serial.print(code);
    Serial.print(" consec=");
    Serial.print(g_imuConsecErrors);
    Serial.print(" total=");
    Serial.print(g_imuReadErrors);
    Serial.print('/');
    Serial.println(g_imuReads);
  }
  return false;
}

static bool imuReadU8(uint8_t reg, uint8_t* value) {
  return imuRead(reg, value, 1);
}

static int16_t readLeI16(const uint8_t* data) {
  return static_cast<int16_t>((static_cast<uint16_t>(data[1]) << 8) |
                              static_cast<uint16_t>(data[0]));
}

// Reading the output registers also clears the data-ready flags, so this
// naturally paces the loop at the configured ODR.
static bool imuDataReady() {
  uint8_t status = 0;
  if (!imuReadU8(kRegStatus, &status)) {
    return false;
  }
  return (status & (kAccelDataReady | kGyroDataReady)) != 0;
}

// Burst read: gyro X,Y,Z then accel X,Y,Z, both from the same sample instant.
static bool imuReadSample(int16_t gyroRaw[3], int16_t accelRaw[3]) {
  uint8_t raw[12] = {0};
  if (!imuRead(kRegOutXlG, raw, sizeof(raw))) {
    return false;
  }

  gyroRaw[0]  = readLeI16(&raw[0]);
  gyroRaw[1]  = readLeI16(&raw[2]);
  gyroRaw[2]  = readLeI16(&raw[4]);
  accelRaw[0] = readLeI16(&raw[6]);
  accelRaw[1] = readLeI16(&raw[8]);
  accelRaw[2] = readLeI16(&raw[10]);
  return true;
}

// ---------------------------------------------------------------------------
// IMU setup
// ---------------------------------------------------------------------------

static void imuPowerRailOn() {
  pinMode(IMU_MIC_EN, OUTPUT);
  digitalWrite(IMU_MIC_EN, HIGH);
  delay(20);  // let the rail come up before the first I2C transaction
}

static bool imuFind() {
  for (size_t i = 0; i < sizeof(kImuAddresses) / sizeof(kImuAddresses[0]); ++i) {
    g_imuAddress = kImuAddresses[i];

    uint8_t who = 0;
    if (imuReadU8(kRegWhoAmI, &who) && who == kExpectedWhoAmI) {
      g_whoAmI = who;
      return true;
    }

    if (who != 0 && who != 0xFF) {
      Serial.print("Device at 0x");
      Serial.print(g_imuAddress, HEX);
      Serial.print(" reports WHO_AM_I=0x");
      Serial.print(who, HEX);
      Serial.println(" (expected 0x6A)");
    }

    g_imuAddress = 0;
  }
  return false;
}

static bool imuConfigure() {
  // BDU=1 keeps output registers coherent during multi-byte reads,
  // IF_INC=1 auto-increments the register address for burst reads.
  if (!imuWrite(kRegCtrl3C, 0x44)) {
    return false;
  }
  if (!imuWrite(kRegCtrl1Xl, kCtrl1XlValue)) {
    return false;
  }
  return imuWrite(kRegCtrl2G, kCtrl2GValue);
}

// Recovery, and only recovery. Called once the failures stop looking like
// glitches. Nothing here re-measures the bias or requires the board to be held
// still; a bad -- or missing -- sensor should degrade, not demand attention.
static void imuServiceFaults() {
  if (g_imuConsecErrors < kImuConsecFaultLimit) {
    return;
  }
  g_imuConsecErrors = 0;

  Serial.println("imu=bus recovery");
  imuBusRecover();

  // A drooping rail can reset the part, so verify the configuration rather
  // than assume the glitch was pure bus. The measured bias survives either
  // way, because it describes the part and not the session.
  uint8_t who   = 0;
  uint8_t stage = 0;
  uint8_t code  = 0;
  if (imuReadOnce(kRegWhoAmI, &who, 1, &stage, &code) && who == kExpectedWhoAmI) {
    g_imuRecoveries = 0;
    return;
  }

  Serial.println("imu=reconfiguring after reset");
  if (imuConfigure()) {
    g_imuRecoveries = 0;
    return;
  }

  // Repeated failed resets: stop pretending this is a transient and go back to
  // discovery. imuTask re-runs imuFind/imuConfigure and keeps the old bias.
  if (++g_imuRecoveries >= kImuRecoverLimit) {
    g_imuRecoveries = 0;
    g_imuAddress    = 0;
  }
}

// ---------------------------------------------------------------------------
// Gyro bias calibration
// ---------------------------------------------------------------------------

// A still gyro is not zero. Left uncorrected, that offset integrates into a
// cursor that crawls across the screen. Average a few hundred samples while
// nothing is moving and subtract the result.
static bool calibrateGyroBias() {
  Serial.print("Hold the board still, measuring gyro bias (");
  Serial.print(kCalibrationSamples);
  Serial.println(" samples)...");

  // Count the reads that died while calibrating. This is the failure rate
  // *before* the radio starts, which is what separates "the BLE transmitter is
  // coupling into the bus" from "this bus was always marginal".
  const uint32_t errorsBefore = g_imuReadErrors;

  for (uint8_t attempt = 1; attempt <= kCalibrationMaxAttempts; ++attempt) {
    double sumDps[3] = {0.0, 0.0, 0.0};
    int16_t minRaw[3] = {32767, 32767, 32767};
    int16_t maxRaw[3] = {-32768, -32768, -32768};
    uint16_t collected = 0;

    const uint32_t startMs = millis();

    while (collected < kCalibrationSamples) {
      if (millis() - startMs > kCalibrationTimeoutMs) {
        break;  // the IMU stopped answering, use what we have
      }
      if (!imuDataReady()) {
        delay(1);
        continue;
      }

      int16_t gyroRaw[3];
      int16_t accelRaw[3];
      if (!imuReadSample(gyroRaw, accelRaw)) {
        delay(5);
        continue;
      }

      for (uint8_t i = 0; i < 3; ++i) {
        sumDps[i] += static_cast<double>(gyroRaw[i]) * kGyroMdpsPerLsb / 1000.0;
        minRaw[i] = (gyroRaw[i] < minRaw[i]) ? gyroRaw[i] : minRaw[i];
        maxRaw[i] = (gyroRaw[i] > maxRaw[i]) ? gyroRaw[i] : maxRaw[i];
      }
      ++collected;
      delay(1);
    }

    if (collected < kCalibrationSamples / 4) {
      Serial.print("calibration failed: IMU stopped responding (read_err=");
      Serial.print(g_imuReadErrors - errorsBefore);
      Serial.println(')');
      return false;
    }

    // Peak-to-peak spread tells us whether the hand was actually still.
    float worstSpreadDps = 0.0f;
    for (uint8_t i = 0; i < 3; ++i) {
      const float spread = static_cast<float>(maxRaw[i] - minRaw[i]) * kGyroMdpsPerLsb / 1000.0f;
      if (spread > worstSpreadDps) {
        worstSpreadDps = spread;
      }
    }

    if (worstSpreadDps <= kCalibrationMaxSpreadDps) {
      for (uint8_t i = 0; i < 3; ++i) {
        g_gyroBiasDps[i] = static_cast<float>(sumDps[i] / collected);
      }
      g_biasValid = true;

      Serial.print("bias_dps=");
      Serial.print(g_gyroBiasDps[0], 2);
      Serial.print(',');
      Serial.print(g_gyroBiasDps[1], 2);
      Serial.print(',');
      Serial.print(g_gyroBiasDps[2], 2);
      Serial.print(" spread_dps=");
      Serial.print(worstSpreadDps, 2);
      Serial.print(" read_err=");
      Serial.println(g_imuReadErrors - errorsBefore);
      return true;
    }

    Serial.print("calibration attempt ");
    Serial.print(attempt);
    Serial.print(": motion detected (spread ");
    Serial.print(worstSpreadDps, 1);
    Serial.println(" dps), retry");
  }

  Serial.println("calibration failed: could not hold still, keeping last bias");
  return false;
}

// ---------------------------------------------------------------------------
// Sample processing
// ---------------------------------------------------------------------------

// Pull the two rate axes and the two gravity axes that lie in the plane
// perpendicular to the pointing axis. The pair is ordered so that
// (A, B, point) is right handed, which is what makes the signs below stable.
static void extractPlane(const float vec3[3], float out2[2]) {
  switch (kPointAxis) {
    case kPointAxisX:  // plane (Y, Z)
      out2[0] = vec3[1];
      out2[1] = vec3[2];
      break;
    case kPointAxisY:  // plane (Z, X)
      out2[0] = vec3[2];
      out2[1] = vec3[0];
      break;
    default:           // plane (X, Y)
      out2[0] = vec3[0];
      out2[1] = vec3[1];
      break;
  }
}

static void updateMotion(const int16_t gyroRaw[3], const int16_t accelRaw[3], float dtSec) {
  // 1. Bias-correct the raw rates.
  float corrected[3];
  for (uint8_t i = 0; i < 3; ++i) {
    corrected[i] = static_cast<float>(gyroRaw[i]) * kGyroMdpsPerLsb / 1000.0f - g_gyroBiasDps[i];
    g_accelMg[i] = static_cast<float>(accelRaw[i]) * kAccelMgPerLsb;
  }

  // 2. Low-pass the rates. The bias tracker below uses this filtered value:
  //    while the device is still, it settles on the residual bias, which is
  //    exactly the quantity we want to keep re-zeroing.
  const float gyroAlpha = alphaFor(kGyroFilterTauMs, dtSec);
  if (!g_filtersPrimed) {
    for (uint8_t i = 0; i < 3; ++i) {
      g_gyroDps[i] = corrected[i];
    }
    g_filtersPrimed = true;
  } else {
    for (uint8_t i = 0; i < 3; ++i) {
      g_gyroDps[i] += gyroAlpha * (corrected[i] - g_gyroDps[i]);
    }
  }

  // 3. Track the bias while the device is demonstrably still. This is what
  //    keeps a long-idle cursor from creeping as the sensor warms up.
  bool still = true;
  for (uint8_t i = 0; i < 3; ++i) {
    if (fabsf(g_gyroDps[i]) > kStillThresholdDps) {
      still = false;
    }
  }
  if (still) {
    if (g_stillCount < kStillSamples) {
      ++g_stillCount;
    }
    if (g_stillCount >= kStillSamples) {
      for (uint8_t i = 0; i < 3; ++i) {
        g_gyroBiasDps[i] += g_gyroDps[i] * kBiasTrackingAlpha;
      }
    }
  } else {
    g_stillCount = 0;
  }

  // 4. Gravity in the pointing plane, low-passed hard. Accelerometer output
  //    points along world-up when the device is not accelerating, so this is
  //    an absolute (drift-free) roll reference.
  float planeAccelRaw[2];
  float accelG[3];
  for (uint8_t i = 0; i < 3; ++i) {
    accelG[i] = static_cast<float>(accelRaw[i]) * kAccelGPerLsb;
  }
  extractPlane(accelG, planeAccelRaw);

  const float gravityAlpha = alphaFor(kGravityFilterTauMs, dtSec);
  g_planeAccelG[0] += gravityAlpha * (planeAccelRaw[0] - g_planeAccelG[0]);
  g_planeAccelG[1] += gravityAlpha * (planeAccelRaw[1] - g_planeAccelG[1]);

  const float planeMag = sqrtf(g_planeAccelG[0] * g_planeAccelG[0] +
                               g_planeAccelG[1] * g_planeAccelG[1]);
  if (planeMag >= kMinGravityInPlaneG) {
    // psi is the angle of world-up inside the plane, i.e. the roll of the
    // device about its pointing axis. g_rollDeg is reported relative to the
    // pose where the plane's second axis points straight up.
    g_rollRad = atan2f(g_planeAccelG[1], g_planeAccelG[0]);
    g_rollDeg = wrap180(g_rollRad * kRadToDeg - 90.0f);
  }
  // Otherwise the device is pointing too steeply up or down for gravity to
  // define a roll; keep the last good value, which is the best guess there is.

  // 5. Roll-compensate. Rotating the in-plane rate pair by -psi expresses the
  //    motion in screen-aligned axes: the component along world-up is the
  //    horizontal pointer rate, the perpendicular one is the vertical rate.
  float planeGyro[2];
  extractPlane(g_gyroDps, planeGyro);

  const float c = cosf(g_rollRad);
  const float s = sinf(g_rollRad);
  const float horizontal = planeGyro[0] * c + planeGyro[1] * s;
  const float vertical   = -planeGyro[0] * s + planeGyro[1] * c;

  g_ptrRateDps[0] = kPointerHorizontalSign * horizontal;
  g_ptrRateDps[1] = kPointerVerticalSign * vertical;

  // 6. Integrate into pointer movement.
  //    g_pendingDeg is the real accumulator that the HID layer drains; it is
  //    capped so a stalled link cannot leave a backlog of motion that keeps the
  //    cursor sliding after the hand has stopped.
  //    g_ptrDeg is a wrapped copy for the serial readout only, and must not be
  //    used for reporting: wrapping it would discard real rotation.
  for (uint8_t i = 0; i < 2; ++i) {
    g_pendingDeg[i] += g_ptrRateDps[i] * dtSec;
    if (g_pendingDeg[i] > kHidPendingCapDeg) {
      g_pendingDeg[i] = kHidPendingCapDeg;
    } else if (g_pendingDeg[i] < -kHidPendingCapDeg) {
      g_pendingDeg[i] = -kHidPendingCapDeg;
    }
    g_ptrDeg[i] = wrap180(g_ptrDeg[i] + g_ptrRateDps[i] * dtSec);
  }
}

// Defined with the HID transport below.
static uint16_t hidConnIntervalUnits();

static void printState() {
  // Serial.print("accel_mg=");
  // Serial.print(g_accelMg[0], 1);
  // Serial.print(',');
  // Serial.print(g_accelMg[1], 1);
  // Serial.print(',');
  // Serial.print(g_accelMg[2], 1);

  // Serial.print(" gyro_raw_dps=");
  // Serial.print(g_gyroDps[0], 1);
  // Serial.print(',');
  // Serial.print(g_gyroDps[1], 1);
  // Serial.print(',');
  // Serial.print(g_gyroDps[2], 1);

  Serial.print(" roll_deg= ");
  Serial.print(g_rollDeg, 1);

  Serial.print(" ptr_dps= ");
  Serial.print(g_ptrRateDps[0], 1);
  Serial.print(' ');
  Serial.print(g_ptrRateDps[1], 1);

  Serial.print(" ptr_deg= ");
  Serial.print(g_ptrDeg[0], 1);
  Serial.print(' ');
  Serial.print(g_ptrDeg[1], 1);

  // Only shown once the bus has actually misbehaved: 0/4823 means the sensor
  // is talking cleanly, a rising count means it is not.
  if (g_imuReadErrors != 0) {
    Serial.print(" read_err= ");
    Serial.print(g_imuReadErrors);
    Serial.print('/');
    Serial.print(g_imuReads);
  }

  // The connection interval is the floor on pointer latency, so it is worth
  // watching: 15 ms means the fast request was accepted, 30-50 ms means the
  // host ignored it. Shown only once a connection exists.
  if (g_hidRunParamsRequested) {
    Serial.print(" ci_ms= ");
    Serial.print(static_cast<float>(hidConnIntervalUnits()) * 1.25f, 1);
  }

  Serial.println();
}

// ---------------------------------------------------------------------------
// HID transport
// ---------------------------------------------------------------------------

static void hidConnectCallback(uint16_t connHandle) {
  (void)connHandle;
  Serial.println("ble=connected, pairing");
  // A fresh connection starts at the pairing interval again.
  g_hidRunParamsRequested = false;
  Bluefruit.Security.requestPairing();
}

static void hidDisconnectCallback(uint16_t connHandle, uint8_t reason) {
  (void)connHandle;
  Serial.print("ble=disconnected reason=0x");
  Serial.println(reason, HEX);
  g_hidRunParamsRequested = false;
}

static void hidStartAdvertising() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  // HID hosts filter on the Appearance field, and it belongs in the GAP
  // database as well, which setAppearance above already did.
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_MOUSE);
  Bluefruit.Advertising.addService(blehid, blebas);
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // 20 ms fast, 152.5 ms slow
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

static void hidBegin() {
  Bluefruit.begin();
  Bluefruit.setName(kDeviceName);
  Bluefruit.setAppearance(BLE_APPEARANCE_HID_MOUSE);
  Bluefruit.setTxPower(4);

  // A mouse has no display and no keypad, so advertise NoInputNoOutput. That
  // makes the host pick LE Secure Connections Just Works instead of asking for
  // a passkey this device could never show or type.
  Bluefruit.Security.setIOCaps(false, false, false);

  // Start conservative so secure pairing has CPU between connection events.
  // hidTask() drops this to kConnIntervalRun* once the link is encrypted.
  Bluefruit.Periph.setConnInterval(kConnIntervalPairMin, kConnIntervalPairMax);
  Bluefruit.Periph.setConnectCallback(hidConnectCallback);
  Bluefruit.Periph.setDisconnectCallback(hidDisconnectCallback);

  bledis.setManufacturer("Air Mouse");
  bledis.setModel("XIAO nRF54L15");
  bledis.begin();
  blebas.begin();
  blebas.write(100);

  // Compact Zephyr-style HID report, which is the shape Android and Sony HID
  // hosts compare against. Note this drops the wheel and pan fields.
  blehid.setZephyrCompatibleMouse(true);
  blehid.begin();

  pinMode(kButtonPin, INPUT_PULLUP);

  hidStartAdvertising();
  Serial.print("ble=advertising as \"");
  Serial.print(kDeviceName);
  Serial.println("\" (pair from the host, HID needs an encrypted link)");
}

// HID only accepts reports on an encrypted link. Before that, mouseReport()
// silently returns false, so this is also the "is it worth trying" check.
static bool hidLinkReady() {
  return Bluefruit.connected() && Bluefruit.Security.isEncrypted(0) &&
         blehid.mouseNotifyEnabled();
}

// Degrees to HID counts, truncated toward zero and clamped to the signed 8-bit
// range the report uses.
static int8_t hidCountsFor(float deg) {
  const float counts = deg * kHidCountsPerDeg;
  int whole = static_cast<int>(counts);
  if (whole > 127) {
    whole = 127;
  } else if (whole < -127) {
    whole = -127;
  }
  return static_cast<int8_t>(whole);
}

// Non-blocking debounce: PIN_BUTTON is active low with the internal pull-up.
static void hidPollButton(uint32_t nowMs) {
  const bool raw = (digitalRead(kButtonPin) == LOW);
  if (raw != g_buttonRaw) {
    g_buttonRaw = raw;
    g_buttonChangeMs = nowMs;
    return;
  }
  if (nowMs - g_buttonChangeMs < kButtonDebounceMs) {
    return;
  }
  if (g_buttonStable != g_buttonRaw) {
    g_buttonStable = g_buttonRaw;
    Serial.println(g_buttonStable ? "button=down" : "button=up");
  }
}

// The interval the link actually settled on, in 1.25 ms units. 0 when unknown.
static uint16_t hidConnIntervalUnits() {
  const BLEConnection* conn = Bluefruit.Connection(0);
  return (conn != nullptr) ? conn->getConnectionInterval() : 0U;
}

// One report per connection event. Sending faster does not get motion to the
// host any sooner -- the extra notifications just queue behind each other, and
// that queue is the latency people feel. So the cadence follows the link.
// Cached, because reading it goes through a HAL critical section and the
// interval only moves when the host accepts a parameter update.
static uint32_t g_hidReportPeriodMs = kHidReportFallbackMs;

static void hidRefreshReportPeriod() {
  const uint16_t units = hidConnIntervalUnits();
  if (units == 0U) {
    g_hidReportPeriodMs = kHidReportFallbackMs;
    return;
  }
  // units * 1.25 ms, rounded up: reporting a hair late is harmless, reporting a
  // hair early means every report waits a whole extra event.
  uint32_t ms = (static_cast<uint32_t>(units) * 125U + 99U) / 100U;
  if (ms < kHidReportFloorMs) {
    ms = kHidReportFloorMs;
  } else if (ms > kHidReportCeilMs) {
    ms = kHidReportCeilMs;
  }
  g_hidReportPeriodMs = ms;
}

// Ask for the fast interval, once per connection, and only after the link is
// encrypted: pairing is what needed the slow one. setConnInterval() pushes a
// connection parameter update request while connected, so calling it on every
// pass would queue a request each time.
static void hidEnsureRunParams() {
  if (g_hidRunParamsRequested) {
    return;
  }
  g_hidRunParamsRequested = true;
  Bluefruit.Periph.setConnInterval(kConnIntervalRunMin, kConnIntervalRunMax);
  Serial.println("hid=requested-fast-conn-interval");
}

static void hidTask(uint32_t nowMs) {
  hidPollButton(nowMs);

  if (!hidLinkReady()) {
    // Drop the backlog rather than firing it off in a burst the moment the
    // link comes back, which would fling the cursor across the screen.
    g_pendingDeg[0] = 0.0f;
    g_pendingDeg[1] = 0.0f;
    g_hidLastButtons = 0;
    g_hidLastReportMs = nowMs;
    g_hidLastKeepaliveMs = nowMs;
    return;
  }

  hidEnsureRunParams();

  // Pace to the negotiated interval (see hidRefreshReportPeriod). A click
  // therefore goes out on the next slot, and a send that fails waits for the
  // next slot instead of spinning.
  if ((nowMs - g_hidLastReportMs) < g_hidReportPeriodMs) {
    return;
  }
  g_hidLastReportMs = nowMs;
  hidRefreshReportPeriod();

  const uint8_t buttons = g_buttonStable ? MOUSE_BUTTON_LEFT : 0;
  const bool buttonChanged = (buttons != g_hidLastButtons);
  const int8_t dx = hidCountsFor(g_pendingDeg[0]);
  const int8_t dy = hidCountsFor(g_pendingDeg[1]);
  const bool keepaliveDue =
      kHidIdleReportEnabled && ((nowMs - g_hidLastKeepaliveMs) >= kHidIdleReportMs);

  if (dx == 0 && dy == 0 && !buttonChanged && !keepaliveDue) {
    return;  // nothing to say, and no keepalive wanted yet
  }

  if (blehid.mouseReport(buttons, dx, dy, 0, 0)) {
    // Subtract only what was actually sent. The remainder stays pending, which
    // is what keeps slow movement smooth instead of quantised.
    g_pendingDeg[0] -= static_cast<float>(dx) / kHidCountsPerDeg;
    g_pendingDeg[1] -= static_cast<float>(dy) / kHidCountsPerDeg;
    g_hidLastButtons = buttons;
    g_hidLastKeepaliveMs = nowMs;
  }
  // On failure the motion simply stays pending and goes out on the next slot.
}

// ---------------------------------------------------------------------------
// IMU task
// ---------------------------------------------------------------------------

static void announceSensor() {
  Serial.print("imu=ready addr=0x");
  Serial.print(g_imuAddress, HEX);
  Serial.print(" who=0x");
  Serial.print(g_whoAmI, HEX);
  Serial.print(" range=+/-4g,500dps odr=416Hz point_axis=");
  Serial.println(static_cast<char>('X' + kPointAxis));
}

static void imuTask() {
  // Recover from a missing or wedged sensor without needing a reset.
  if (g_imuAddress == 0) {
    if (imuFind() && imuConfigure()) {
      announceSensor();
      if (!g_biasValid) {
        calibrateGyroBias();
      }
      g_lastSampleUs = micros();
      g_lastPrintMs  = millis();
      g_filtersPrimed = false;
    } else {
      g_imuAddress = 0;
      // Rate-limited: a sensor that is truly absent should say so once in a
      // while rather than every second forever.
      static uint8_t notFoundTries = 0;
      if ((notFoundTries++ % 5) == 0) {
        Serial.println("imu=not-found, retrying");
      }
      delay(1000);
      return;
    }
  }

  if (!imuDataReady()) {
    imuServiceFaults();
    delay(1);
    return;
  }

  int16_t gyroRaw[3];
  int16_t accelRaw[3];
  if (!imuReadSample(gyroRaw, accelRaw)) {
    // A dropped sample costs nothing: dt is measured, so the next good sample
    // integrates over its own interval. Tearing the sensor down here is what
    // turned a routine bus glitch into an endless re-init-and-recalibrate loop.
    imuServiceFaults();
    delay(1);
    return;
  }

  // dt is measured, not assumed, so the integration stays correct even when
  // the loop is delayed by a long serial write.
  const uint32_t nowUs = micros();
  uint32_t deltaUs = nowUs - g_lastSampleUs;
  g_lastSampleUs = nowUs;
  if (deltaUs == 0 || deltaUs > 100000UL) {
    deltaUs = 2400UL;  // fall back to the 416 Hz sample period
  }
  const float dtSec = static_cast<float>(deltaUs) / 1000000.0f;

  updateMotion(gyroRaw, accelRaw, dtSec);

  const uint32_t nowMs = millis();
  if (nowMs - g_lastPrintMs >= kPrintIntervalMs) {
    g_lastPrintMs = nowMs;
    printState();
  }
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("Air_Mouse - step 3: BLE HID mouse");
  Serial.println("bus=Wire1 (SDA=D12, SCL=D11)  target=LSM6DS3TR-C @ 0x6A/0x6B");

  imuPowerRailOn();
  imuBus.begin();
  imuBus.setClock(kImuI2cClockHz);

  if (!imuFind()) {
    Serial.println("imu=not-found (Sense board? rail powered? D12/D11 connected?)");
    return;
  }

  if (!imuConfigure()) {
    Serial.println("imu=config-failed");
    g_imuAddress = 0;
    return;
  }

  announceSensor();
  calibrateGyroBias();

  g_lastSampleUs = micros();
  g_lastPrintMs  = millis();

  // BLE starts last so the calibration instructions are on screen before the
  // device becomes connectable.
  hidBegin();
  g_hidLastReportMs = millis();
}

void loop() {
  // Both tasks run from one loop. The IMU side never blocks on the radio, and
  // the HID side is a no-op until a host pairs.
  imuTask();
  hidTask(millis());
}
