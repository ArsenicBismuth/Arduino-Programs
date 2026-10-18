/*
 * Air_Mouse - Step 1: read the onboard IMU
 *
 * Board : Seeed XIAO nRF54L15 / Sense
 * Core  : lolren/nrf54-arduino-core  (board "XIAO nRF54L15 / Sense", variant xiao_nrf54l15)
 * IMU   : ST LSM6DS3TR-C, 6-axis accel + gyro, I2C address 0x6A (0x6B if SA0 is tied high)
 *
 * Hardware notes (Sense board only - the IMU footprint is unpopulated on the plain board):
 *   - The IMU sits on the back-pad bus, not on the D4/D5 header pair:
 *       SDA -> D12 / P0.04
 *       SCL -> D11 / P0.03
 *     That bus is exposed by the core as Wire1.
 *   - IMU_MIC_EN (D20 / P0.01) gates the shared IMU + microphone power rail.
 *     It must be driven HIGH before the IMU will answer on I2C.
 *
 * What this does:
 *   - powers the sensor rail
 *   - verifies WHO_AM_I
 *   - configures 104 Hz accel (+/-2 g) and 104 Hz gyro (245 dps)
 *   - streams accel (mg), gyro (deg/s) and dead-reckoned roll/pitch/yaw (deg)
 *
 * Open the Serial Monitor at 115200 baud, then move or rotate the board.
 */

#include <Arduino.h>
#include <Wire.h>

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

// Sensitivities for the full scales configured below.
static const float kAccelMgPerLsb   = 0.061f;  // +/-2 g   -> 0.061 mg/LSB
static const float kGyroMdpsPerLsb  = 8.75f;   // 245 dps  -> 8.75 mdps/LSB

// Data-ready bits in the status register.
static const uint8_t kGyroDataReady = 0x02;
static const uint8_t kAccelDataReady = 0x01;

// Cap the printing rate so the serial link never becomes the bottleneck
// (the IMU itself produces new samples at 104 Hz).
static const uint32_t kPrintIntervalMs = 20;  // 50 Hz

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static uint8_t  g_imuAddress  = 0;      // 0 while the IMU has not been found
static uint8_t  g_whoAmI      = 0;
static uint32_t g_lastSampleUs = 0;
static uint32_t g_lastPrintMs  = 0;

// Integrated gyro angles. Pure dead reckoning: they are relative to power-up
// and will slowly drift. Good enough to see motion, not a stable attitude.
static float g_rollDeg  = 0.0f;
static float g_pitchDeg = 0.0f;
static float g_yawDeg   = 0.0f;

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

// ---------------------------------------------------------------------------
// IMU setup
// ---------------------------------------------------------------------------

static void imuPowerRailOn() {
  pinMode(IMU_MIC_EN, OUTPUT);
  digitalWrite(IMU_MIC_EN, HIGH);
  delay(20);  // let the rail come up before the first I2C transaction
}

// Probe both possible addresses and verify WHO_AM_I.
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
  // Accel: 104 Hz, +/-2 g.
  if (!imuWrite(kRegCtrl1Xl, 0x40)) {
    return false;
  }
  // Gyro: 104 Hz, 245 dps.
  return imuWrite(kRegCtrl2G, 0x40);
}

static bool imuBegin() {
  if (!imuFind()) {
    return false;
  }
  if (!imuConfigure()) {
    return false;
  }

  g_lastSampleUs = micros();
  g_lastPrintMs  = millis();
  g_rollDeg = g_pitchDeg = g_yawDeg = 0.0f;
  return true;
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("Air_Mouse - onboard IMU bring-up");
  Serial.println("bus=Wire1 (SDA=D12, SCL=D11)  target=LSM6DS3TR-C @ 0x6A/0x6B");

  imuPowerRailOn();
  imuBus.begin();
  imuBus.setClock(400000);  // the IMU supports 400 kHz fast mode

  if (imuBegin()) {
    Serial.print("imu=ready addr=0x");
    Serial.print(g_imuAddress, HEX);
    Serial.print(" who=0x");
    Serial.print(g_whoAmI, HEX);
    Serial.println(" range=+/-2g,245dps odr=104Hz");
  } else {
    Serial.println("imu=not-found (Sense board? rail powered? D12/D11 connected?)");
  }
}

void loop() {
  // Keep retrying if the IMU was not there yet, so hot-plugging or a late
  // power rail settles without a reset.
  if (g_imuAddress == 0) {
    if (imuBegin()) {
      Serial.println("imu=ready (recovered)");
    } else {
      delay(1000);
      return;
    }
  }

  uint8_t status = 0;
  if (!imuReadU8(kRegStatus, &status)) {
    Serial.println("imu=status-read-failed");
    g_imuAddress = 0;
    delay(250);
    return;
  }

  // Wait for a fresh sample instead of hammering the bus.
  if ((status & (kAccelDataReady | kGyroDataReady)) == 0) {
    delay(2);
    return;
  }

  // One burst covers gyro X..Z then accel X..Z, so both come from the
  // same sample instant.
  uint8_t raw[12] = {0};
  if (!imuRead(kRegOutXlG, raw, sizeof(raw))) {
    Serial.println("imu=burst-read-failed");
    g_imuAddress = 0;
    delay(250);
    return;
  }

  const int16_t gx = readLeI16(&raw[0]);
  const int16_t gy = readLeI16(&raw[2]);
  const int16_t gz = readLeI16(&raw[4]);
  const int16_t ax = readLeI16(&raw[6]);
  const int16_t ay = readLeI16(&raw[8]);
  const int16_t az = readLeI16(&raw[10]);

  const float axMg = ax * kAccelMgPerLsb;
  const float ayMg = ay * kAccelMgPerLsb;
  const float azMg = az * kAccelMgPerLsb;

  const float gxDps = gx * kGyroMdpsPerLsb / 1000.0f;
  const float gyDps = gy * kGyroMdpsPerLsb / 1000.0f;
  const float gzDps = gz * kGyroMdpsPerLsb / 1000.0f;

  // Integrate the rates into angles. dt is clamped so the first sample after
  // power-up (or after a stalled loop) cannot inject a huge jump.
  const uint32_t nowUs = micros();
  uint32_t deltaUs = nowUs - g_lastSampleUs;
  g_lastSampleUs = nowUs;
  if (deltaUs == 0 || deltaUs > 100000UL) {
    deltaUs = 10000UL;  // fall back to ~100 Hz
  }
  const float dt = static_cast<float>(deltaUs) / 1000000.0f;

  g_rollDeg  += gxDps * dt;
  g_pitchDeg += gyDps * dt;
  g_yawDeg   += gzDps * dt;

  // Keep the angles bounded so long runs stay readable.
  while (g_yawDeg > 180.0f)   g_yawDeg -= 360.0f;
  while (g_yawDeg < -180.0f)  g_yawDeg += 360.0f;
  while (g_pitchDeg > 180.0f) g_pitchDeg -= 360.0f;
  while (g_pitchDeg < -180.0f) g_pitchDeg += 360.0f;
  while (g_rollDeg > 180.0f)  g_rollDeg -= 360.0f;
  while (g_rollDeg < -180.0f) g_rollDeg += 360.0f;

  const uint32_t nowMs = millis();
  if (nowMs - g_lastPrintMs < kPrintIntervalMs) {
    return;
  }
  g_lastPrintMs = nowMs;

  Serial.print("accel_mg=");
  Serial.print(axMg, 1);
  Serial.print(',');
  Serial.print(ayMg, 1);
  Serial.print(',');
  Serial.print(azMg, 1);
  Serial.print(" gyro_dps=");
  Serial.print(gxDps, 1);
  Serial.print(',');
  Serial.print(gyDps, 1);
  Serial.print(',');
  Serial.print(gzDps, 1);
  Serial.print(" rpy_deg=");
  Serial.print(g_rollDeg, 1);
  Serial.print(',');
  Serial.print(g_pitchDeg, 1);
  Serial.print(',');
  Serial.println(g_yawDeg, 1);
}
