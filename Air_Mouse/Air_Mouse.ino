/*
 * Air_Mouse - Step 2: gyro-driven pointer rates
 *
 * Board : Seeed XIAO nRF54L15 / Sense
 * Core  : lolren/nrf54-arduino-core  (board "XIAO nRF54L15 / Sense", variant xiao_nrf54l15)
 * IMU   : ST LSM6DS3TR-C on the back-pad bus, exposed as Wire1 (SDA = D12, SCL = D11), addr 0x6A
 *
 * Step 2 adds, on top of step 1:
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
 *
 * Before running: hold the board still, it measures the gyro bias first.
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

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
// corrupted by hand acceleration).
static const float kGyroFilterTauMs    = 8.0f;   // ~20 Hz cutoff
static const float kGravityFilterTauMs = 50.0f;  // ~3 Hz cutoff

// Display rate. This is not the sampling rate: the IMU is still read at its
// full 416 Hz, this only throttles what gets printed.
static const uint32_t kPrintIntervalMs = 50;  // 20 Hz

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static uint8_t g_imuAddress = 0;  // 0 while the IMU has not been found
static uint8_t g_whoAmI     = 0;

static float g_gyroBiasDps[3] = {0.0f, 0.0f, 0.0f};
static bool  g_biasValid      = false;

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

static bool imuWrite(uint8_t reg, uint8_t value) {
  imuBus.beginTransmission(g_imuAddress);
  imuBus.write(reg);
  imuBus.write(value);
  return imuBus.endTransmission() == 0;
}

static bool imuRead(uint8_t reg, uint8_t* buffer, size_t length) {
  if (buffer == nullptr || length == 0) {
    return false;
  }

  imuBus.beginTransmission(g_imuAddress);
  imuBus.write(reg);
  if (imuBus.endTransmission(false) != 0) {  // repeated start, keep the bus
    return false;
  }

  if (imuBus.requestFrom(static_cast<int>(g_imuAddress), static_cast<int>(length)) !=
      static_cast<int>(length)) {
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    if (imuBus.available() <= 0) {
      return false;
    }
    buffer[i] = static_cast<uint8_t>(imuBus.read());
  }
  return true;
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
      Serial.println("calibration failed: IMU stopped responding");
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
      Serial.println(worstSpreadDps, 2);
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

  // 6. Integrate into pointer movement. This is the quantity a HID report
  //    would consume; a real mouse sends the delta since the last report and
  //    resets, here it is wrapped purely to keep the readout bounded.
  g_ptrDeg[0] = wrap180(g_ptrDeg[0] + g_ptrRateDps[0] * dtSec);
  g_ptrDeg[1] = wrap180(g_ptrDeg[1] + g_ptrRateDps[1] * dtSec);
}

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
  Serial.println(g_ptrDeg[1], 1);
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------

static void announceSensor() {
  Serial.print("imu=ready addr=0x");
  Serial.print(g_imuAddress, HEX);
  Serial.print(" who=0x");
  Serial.print(g_whoAmI, HEX);
  Serial.print(" range=+/-4g,500dps odr=416Hz point_axis=");
  Serial.println(static_cast<char>('X' + kPointAxis));
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("Air_Mouse - step 2: gyro pointer rates");
  Serial.println("bus=Wire1 (SDA=D12, SCL=D11)  target=LSM6DS3TR-C @ 0x6A/0x6B");

  imuPowerRailOn();
  imuBus.begin();
  imuBus.setClock(400000);

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
}

void loop() {
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
      delay(1000);
      return;
    }
  }

  if (!imuDataReady()) {
    delay(1);
    return;
  }

  int16_t gyroRaw[3];
  int16_t accelRaw[3];
  if (!imuReadSample(gyroRaw, accelRaw)) {
    Serial.println("imu=burst-read-failed");
    g_imuAddress = 0;
    g_biasValid = false;
    delay(250);
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
