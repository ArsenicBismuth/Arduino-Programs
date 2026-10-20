/*
 * Air_Mouse - BLE HID air mouse
 *
 * Board : Seeed XIAO nRF54L15 / Sense
 * Core  : lolren/nrf54-arduino-core   (variant xiao_nrf54l15)
 * IMU   : ST LSM6DS3TR-C, back-pad bus, Wire1 (SDA = D12, SCL = D11), addr 0x6A
 *
 * Gyro about world-vertical drives horizontal motion, about the screen-horizontal
 * axis drives vertical motion, and about the aiming axis is discarded; roll
 * compensation keeps the pointer axes on screen when the device is banked.
 * Onboard button = left click.
 *
 * Hold the board still at power-up: it measures the gyro bias first, then pair with
 * "Air Mouse" from the host's Bluetooth settings. Wiring, diagnostics, tuning and
 * the reasoning behind all of it: README.md.
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <bluefruit.h>

// For Bluefruit.rawRadio().getConnectionTimingSnapshot(), which is how the
// report gets aligned with the connection event. bluefruit.h only forward
// declares the radio, so the definition has to be pulled in explicitly.
#include <nrf54l15_hal.h>

// ---------------------------------------------------------------------------
// Serial log
// ---------------------------------------------------------------------------

// Master switch for every print in this sketch. Off by default, so the pointer path
// touches no serial port and the periodic line's link queries do not run. Set true to
// bring the whole log back. Cost and reasoning: README, "Serial output".
static const bool kSerialLogEnabled = true;

// Prints are routed through this rather than the port itself, so every call site still
// reads Serial.print(...) and the compiler removes the lot when the switch is off.
// Defined before the #define below, so these bodies do see the real Serial object.
class LogSerial {
 public:
  void begin(unsigned long baud) {
    if (kSerialLogEnabled) {
      Serial.begin(baud);
    }
  } 
  void println() {
    if (kSerialLogEnabled) {
      Serial.println();
    }
  }
  template <typename A>
  void print(A a) {
    if (kSerialLogEnabled) {
      Serial.print(a);
    }
  }
  template <typename A>
  void println(A a) {
    if (kSerialLogEnabled) {
      Serial.println(a);
    }
  }
  template <typename A, typename B>
  void print(A a, B b) {
    if (kSerialLogEnabled) {
      Serial.print(a, b);
    }
  }
  template <typename A, typename B>
  void println(A a, B b) {
    if (kSerialLogEnabled) {
      Serial.println(a, b);
    }
  }
};

static LogSerial g_logSerial;

// kSerialLogEnabled is a compile-time constant, so these inline away when off.
#define Serial g_logSerial

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

// Both registers pack ODR in bits 7:4 and full scale in bits 3:2. Careful: FS_XL is
// NOT ascending -- 00=+/-2 g, 01=+/-16 g, 10=+/-4 g, 11=+/-8 g.
//   CTRL1_XL = 0x68 -> ODR 0110 = 416 Hz, FS 10 = +/-4 g
//   CTRL2_G  = 0x64 -> ODR 0110 = 416 Hz, FS 01 = +/-500 dps
static const uint8_t kCtrl1XlValue = 0x68;
static const uint8_t kCtrl2GValue  = 0x64;

// Sensitivities that match the two values above. Change the matching constant if
// you change a range.
static const float kAccelMgPerLsb   = 0.122f;   // +/-4 g
static const float kGyroMdpsPerLsb  = 17.50f;   // +/-500 dps
static const float kAccelGPerLsb    = kAccelMgPerLsb / 1000.0f;

// Data-ready bits in the status register.
static const uint8_t kGyroDataReady  = 0x02;
static const uint8_t kAccelDataReady = 0x01;

// ---------------------------------------------------------------------------
// I2C reliability
// ---------------------------------------------------------------------------

// At the part's specified ceiling. Slower is measurably worse here -- 0.53 % dead
// transactions at 400 kHz against 3.47 % at 100 kHz -- because the failures are the
// radio landing during the transaction and a slow burst is exposed for longer.
// README, "I2C reliability".
static const uint32_t kImuI2cClockHz = 400000;

// A 12-byte burst is ~110 bit times, so a marginal bus drops the odd transaction --
// a bus property, not "the sensor is gone". Retry, but only quick failures: one that
// already burned milliseconds hit the driver timeout, and stacking those stalls the loop.
static const uint8_t  kImuReadAttempts  = 3;
static const uint32_t kImuRetryGapUs    = 50;
static const uint32_t kImuRetryBudgetUs = 3000;

// Consecutive dead transactions before the bus is reset, and failed resets before
// the part is treated as lost and looked for again. The gyro bias is never
// re-measured: it describes the part, and that would need the board held still.
static const uint16_t kImuConsecFaultLimit = 32;
static const uint8_t  kImuRecoverLimit     = 8;

// ---------------------------------------------------------------------------
// Axis mapping - measured on this board
// ---------------------------------------------------------------------------

// Which body axis aims at the screen when the device is held naturally. Measured,
// not assumed: board flat and aimed at a standing phone reads accel_mg= (-84, -111,
// 1037), so gravity is on Z, the board normal, and the aiming axis must read near
// 0 mg. plane_g is the check -- ~1.00 in any normal grip, near zero means the wrong
// axis or a pose aimed at the ceiling. Method: README, "Axis mapping".
enum PointAxis : uint8_t { kPointAxisX, kPointAxisY, kPointAxisZ };
static const PointAxis kPointAxis = kPointAxisY;

// Verified against kPointAxisY. Both came out inverted: the report's X/Y are
// relative to the screen, while these rates are rotation of the device.
static const float kPointerHorizontalSign = -1.0f;
static const float kPointerVerticalSign   = -1.0f;

// The roll reference is only meaningful while gravity projects into the plane
// around the aiming axis. Pointing steeply up or down makes it ill-defined, so
// the last good value is held below this magnitude.
static const float kMinGravityInPlaneG = 0.15f;

// ---------------------------------------------------------------------------
// Tuning
// ---------------------------------------------------------------------------

// Bias calibration at startup.
static const uint16_t kCalibrationSamples    = 300;   // ~0.7 s at 416 Hz
static const float    kCalibrationMaxSpreadDps = 4.0f; // peak-to-peak, rejects a moving hand
static const uint8_t  kCalibrationMaxAttempts = 3;
static const uint32_t kCalibrationTimeoutMs   = 2000;

// Continuous re-zero while held still, to stop slow creep between flicks. Stillness
// is judged from gravity, never from the gyro: a residual bias is a gyro reading with
// no rotation behind it, so a gyro-magnitude gate switches the tracker off exactly
// when the residual is worth removing. README, "Gyro zero and creep".
static const float    kStillDirectionTauMs = 500.0f;
static const float    kStillMinDot         = 0.999945f;  // ~0.6 deg of lag
static const float    kStillAccelTolG      = 0.08f;      // |a| within 8% of 1 g
static const uint16_t kStillSamples      = 150;   // ~0.43 s of stillness first
static const float    kBiasTrackingAlpha = 0.005f;

// The gyro branch carries the motion, so it is fast; the gravity branch only
// tracks roll and is corrupted by hand acceleration, so it is slow.
static const float kGyroFilterTauMs    = 3.0f;   // ~53 Hz cutoff
static const float kGravityFilterTauMs = 50.0f;  // ~3 Hz cutoff

// Display rate only, not the sample rate. The line is the sketch's largest optional
// cost; kSerialLogEnabled explains it and switches it off.
static const uint32_t kPrintIntervalMs = 100;  // 10 Hz

// Adds accel_mg, plane_g and gyro to the periodic line, for working out or re-checking
// kPointAxis. Method: README, "Axis mapping".
static const bool kPrintAxisDiag = false;

// ---------------------------------------------------------------------------
// HID transport
// ---------------------------------------------------------------------------

static const char kDeviceName[] = "Air Mouse";

// Base pointer sensitivity, in HID counts per degree. Hosts add their own acceleration
// on top, so this is the device's scale, not the feel. Never lowered to duck the 8-bit
// report ceiling -- that only made the pointer slow (8.0f felt bad). See README,
// "Configuration" for that arithmetic.
static const float kHidCountsPerDeg = 16.0f;

// Cadence: one hand-over per attempt period (see hidMaintainPace), which follows
// what the radio actually accepts rather than the event rate. These bounds guard
// the arithmetic; the ceiling has to clear the slowest link worth following.
static const uint32_t kHidReportFallbackMs = 15;  // before the interval is known
static const uint32_t kHidReportFloorMs    = 5;
static const uint32_t kHidReportCeilMs     = 50;

// Attempt-rate pacing. The radio takes a notification only every 2-3 events, so
// offering one per event buys nothing and fills its 8-deep queue, which then drains
// after the hand has stopped. A refusal means that queue was full, so 1000/tx_s is the
// link's own period; the pacing parks a margin slower than that and stays there.
// README, "HID transport".
static const uint32_t kHidPaceWindowMs = 1000;  // measurement window
static const uint32_t kHidPaceMarginMs = 2;     // added to the measured drain period
static const uint32_t kHidPaceProbePct = 95;    // no drain measured yet: probe faster
static const uint32_t kHidPaceHoldMs   = 5000;  // then re-probe this slowly
static const uint32_t kHidPaceMinRate  = 8;     // below this reports/s, not a measurement

// Diagnostic override: 0 = normal adaptation. Non-zero forces this attempt
// period so the link ceiling can be measured without waiting for the pace
// floor to settle.
static const uint32_t kHidPaceForceMs = 0;

// How long before the next connection event the report is handed to the radio.
// Must clear the loop's own ~1 ms granularity plus the radio's 2.2 ms pre-roll;
// miss the window and the report waits a whole interval.
static const uint32_t kHidSendLeadUs = 5000;

// Slack on the rate fallback, so the link is never queued two deep.
static const uint32_t kHidSendSlackMs = 5;

// Connection interval, in 1.25 ms units. The central decides, and it takes the TOP of
// whatever range is offered -- measured: 6-12 (7.5-15 ms) settles at exactly 15 ms --
// so the floor has to be asked for alone. This is the report rate, and the report rate
// is measured, not derived: one report per 2-3 connection events, not one per event.
// README, "What actually sets the report rate".
static const uint16_t kConnIntervalMin = 6;   // 7.5 ms
static const uint16_t kConnIntervalMax = 6;   // 7.5 ms -- no room to pick the top

// The HAL asks once and clears its pending flag on any response, rejection included,
// so a badly placed request is final. Re-ask the same 7.5 ms a few times, in case the
// first one lands during pairing, but never offer a slower interval: a host that
// refuses 7.5 ms leaves the link where it is, to be reconnected rather than lived
// with. Android's own default, 30 ms, is exactly the outcome this avoids.
static const uint8_t  kConnAttempts        = 8;
static const uint32_t kConnRetryGapMs      = 1000;
static const uint32_t kConnFastThresholdMs = 8;   // only 7.5 ms counts as settled

// Every Bluefruit connection or security query starts by running the stack service
// routine, a few hundred microseconds on a live link. Per pass at 350 passes/s that
// was most of the loop. The core still services the stack once per pass in main(),
// so polling these instead starves nothing.
static const uint32_t kConnPollMs = 100;  // connection interval / retry gap
static const uint32_t kLinkPollMs = 50;   // link ready (connected, encrypted, subscribed)

// Cap on unreported motion, near one report's worth, so a stalled link cannot
// leave the cursor sliding after the hand has stopped.
static const float kHidPendingCapDeg = 16.0f;

// Soft deadzone, per axis, before integration: a gyro cannot tell a slow rotation from a
// small offset, so without it the cursor shimmers at rest. Below the threshold the axis is
// zero; above it the rate is reduced by exactly the threshold, which stays continuous at
// the cost of a constant offset. Sized well above the +/-0.5 dps of rest noise; 0.0f disables.
static const float kHidDeadzoneDps = 3.0f;

// Onboard button, active low, as left click.
static const uint8_t  kButtonPin        = PIN_BUTTON;
static const uint32_t kButtonDebounceMs = 20;

// Some hosts like a periodic zero-motion report to stay awake.
static const bool     kHidIdleReportEnabled = false;
static const uint32_t kHidIdleReportMs      = 100;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static uint8_t g_imuAddress = 0;  // 0 while the IMU has not been found
static uint8_t g_whoAmI     = 0;

static float g_gyroBiasDps[3] = {0.0f, 0.0f, 0.0f};
static bool  g_biasValid      = false;

// I2C health: says whether the sensor is babbling or gone, which are different
// problems.
static uint32_t g_imuReads        = 0;  // transactions attempted
static uint32_t g_imuReadErrors   = 0;  // transactions that failed every retry
static uint16_t g_imuConsecErrors = 0;  // consecutive failures, cleared by a success
static uint8_t  g_imuRecoveries   = 0;  // failed bus resets since the last clean reply

static uint32_t g_lastSampleUs = 0;
static uint32_t g_lastPrintMs  = 0;
static bool     g_filtersPrimed = false;

// Filter state.
static float g_gyroDps[3]      = {0.0f, 0.0f, 0.0f};  // bias-corrected, filtered, body axes
static float g_planeAccelG[2]  = {0.0f, 1.0f};        // filtered gravity in the aiming plane
static float g_accelMg[3]      = {0.0f, 0.0f, 0.0f};  // instantaneous, body axes

// Outputs.
static float g_rollRad         = 0.0f;          // roll in radians, drives the rotation
static float g_rollDeg         = 0.0f;
static float g_ptrRateDps[2]   = {0.0f, 0.0f};  // {horizontal, vertical}
static float g_ptrDeg[2]       = {0.0f, 0.0f};  // integrated, wrapped

static uint16_t g_stillCount = 0;
// Reference direction of world-up for the stillness test.
static float g_stillDirG[3] = {0.0f, 0.0f, 0.0f};
static bool  g_stillDirValid = false;

// ---------------------------------------------------------------------------
// HID state
// ---------------------------------------------------------------------------

// BLEDis and BLEBas are carried because the core's own HID example advertises
// HIDS alongside them, and strict Android hosts compare against that shape.
BLEDis       bledis;
BLEBas       blebas;
BLEHidAdafruit blehid;

// Pointer movement accumulated but not yet reported, in degrees. The real
// accumulator; g_ptrDeg above is only a wrapped display copy.
static float g_pendingDeg[2] = {0.0f, 0.0f};

static uint32_t g_hidLastReportMs = 0;
static uint32_t g_hidLastKeepaliveMs = 0;
static uint8_t  g_hidLastButtons  = 0;

// Window spacing is measured on the radio's own microsecond clock rather than on
// the event counter, because only the clock is certain to advance.
static uint32_t g_hidLastWindowUs  = 0;

// Reporting statistics, for the periodic line: the effective rate decides whether
// the pointer feels smooth, so it is worth reading back.
static uint32_t g_hidReportCount   = 0;  // successful reports, ever
static uint32_t g_hidFallbackCount = 0;  // of those, sent on the rate fallback
static uint32_t g_hidRatePerSec    = 0;

// How often the send gate was satisfiable at all, and how often a send followed. tx_s
// alone cannot separate the three ways a report goes missing: no event offered (win
// low), the gate refusing one, or the radio declining it (try high, tx low).
static uint32_t g_hidWindowCount   = 0;  // times windowOpen came out true
static uint32_t g_hidAttemptCount  = 0;  // times mouseReport was called
static uint32_t g_hidWindowPerSec  = 0;
static uint32_t g_hidAttemptPerSec = 0;

// Pacing state (see hidMaintainPace). The period stays 0 until the first second of
// movement is measured, and 0 means "no pacing", which is the free-running state
// the first measurement is taken in.
static uint32_t g_hidPacePeriodMs    = 0;  // derived attempt period, 0 = unpaced
static uint32_t g_hidPaceFloorMs     = 0;  // drain + margin, 0 = not measured yet
static uint32_t g_hidPaceHoldMs      = 0;  // when the floor was last set or eased
static uint32_t g_hidPacePollMs      = 0;
static uint32_t g_hidPaceWindowMs    = 0;  // start of the current measurement window
static uint32_t g_hidPaceBase        = 0;  // report count at that start
static uint32_t g_hidPaceAttemptBase = 0;
static uint16_t g_hidPaceIntervalUnits = 0;  // interval the period was measured on

// Averaged over one shared one-second window. loop_hz below the link's own rate means
// the loop, not the link, is setting the report rate and the sample rate.
static uint32_t g_loopCount    = 0;
static uint32_t g_diagWindowMs = 0;
static uint32_t g_diagLoopBase = 0;
static uint32_t g_diagReportBase = 0;
static uint32_t g_diagFallbackBase = 0;
static uint32_t g_diagWindowBase = 0;
static uint32_t g_diagAttemptBase = 0;
static uint32_t g_diagLoopHz   = 0;
static uint32_t g_diagAlignedPct = 0;

// Connection-interval negotiation state, all reset per connection.
static uint32_t g_hidConnectedMs   = 0;      // when the link came up
static uint32_t g_hidConnRetryMs   = 0;      // last time we asked for a fast one
static uint8_t  g_hidConnRetries   = 0;
static bool     g_hidConnFastSeen  = false;  // logged that the host complied

// Cached link state, refreshed on a timer rather than per pass (see kLinkPollMs).
// Kept fresh by name, because the expense being avoided is invisible at the call
// site and the obvious "just check it" edit is exactly the one that regresses it.
static bool     g_linkReady  = false;
static uint32_t g_linkPollMs = 0;
static uint32_t g_hidConnPollMs = 0;

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

// Retrying wrapper around imuReadOnce. A dropped sample costs nothing because
// dt is measured, so the priority is that a transient failure stays transient.
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
// perpendicular to the aiming axis. The pair is ordered so that
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
  // 1. Bias-correct the raw rates, and scale the accelerometer once for both the
  //    roll estimate and the stillness test below.
  float corrected[3];
  float accelG[3];
  for (uint8_t i = 0; i < 3; ++i) {
    corrected[i] = static_cast<float>(gyroRaw[i]) * kGyroMdpsPerLsb / 1000.0f - g_gyroBiasDps[i];
    g_accelMg[i] = static_cast<float>(accelRaw[i]) * kAccelMgPerLsb;
    accelG[i]    = static_cast<float>(accelRaw[i]) * kAccelGPerLsb;
  }

  // 2. Low-pass the rates. The bias tracker reads this filtered value, since
  //    that is what settles on the residual bias while the device is still.
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

  // 3. Is the device still? From gravity, never from the gyro: gating on the
  //    gyro's own magnitude deadlocks, because a residual larger than the gate
  //    holds the tracker off permanently, and that residual is the creep it
  //    exists to remove.
  const float accelMag = sqrtf(accelG[0] * accelG[0] + accelG[1] * accelG[1] +
                               accelG[2] * accelG[2]);
  bool still = false;
  if (accelMag > 0.5f) {
    const float invMag = 1.0f / accelMag;
    const float dirG[3] = {accelG[0] * invMag, accelG[1] * invMag, accelG[2] * invMag};

    if (!g_stillDirValid) {
      // No reference yet: adopt this direction and let the settle count run.
      for (uint8_t i = 0; i < 3; ++i) {
        g_stillDirG[i] = dirG[i];
      }
      g_stillDirValid = true;
    } else {
      const float dot = dirG[0] * g_stillDirG[0] + dirG[1] * g_stillDirG[1] +
                        dirG[2] * g_stillDirG[2];
      still = (dot >= kStillMinDot) && (fabsf(accelMag - 1.0f) <= kStillAccelTolG);

      const float dirAlpha = alphaFor(kStillDirectionTauMs, dtSec);
      for (uint8_t i = 0; i < 3; ++i) {
        g_stillDirG[i] += dirAlpha * (dirG[i] - g_stillDirG[i]);
      }
      // Keep the reference a unit vector; it averages different directions
      // while the device turns and would otherwise shrink.
      const float dirMag = sqrtf(g_stillDirG[0] * g_stillDirG[0] +
                                 g_stillDirG[1] * g_stillDirG[1] +
                                 g_stillDirG[2] * g_stillDirG[2]);
      if (dirMag > 0.5f) {
        for (uint8_t i = 0; i < 3; ++i) {
          g_stillDirG[i] /= dirMag;
        }
      }
    }
  }

  // Re-zero while demonstrably still: stops a long-idle cursor creeping as the
  // part warms up.
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

  // 4. Gravity in the aiming plane, low-passed hard. At rest the accelerometer
  //    points along world-up, so this is an absolute, drift-free roll reference.
  float planeAccelRaw[2];
  extractPlane(accelG, planeAccelRaw);

  const float gravityAlpha = alphaFor(kGravityFilterTauMs, dtSec);
  g_planeAccelG[0] += gravityAlpha * (planeAccelRaw[0] - g_planeAccelG[0]);
  g_planeAccelG[1] += gravityAlpha * (planeAccelRaw[1] - g_planeAccelG[1]);

  const float planeMag = sqrtf(g_planeAccelG[0] * g_planeAccelG[0] +
                               g_planeAccelG[1] * g_planeAccelG[1]);
  if (planeMag >= kMinGravityInPlaneG) {
    // psi is the angle of world-up inside the plane, i.e. the device's roll.
    g_rollRad = atan2f(g_planeAccelG[1], g_planeAccelG[0]);
    g_rollDeg = wrap180(g_rollRad * kRadToDeg - 90.0f);
  }
  // Pointed too steeply up or down for gravity to define a roll: hold the last
  // good value.

  // 5. Roll-compensate: rotating the in-plane rate pair by -psi puts it on
  //    screen axes, the component along world-up being the horizontal rate.
  float planeGyro[2];
  extractPlane(g_gyroDps, planeGyro);

  const float c = cosf(g_rollRad);
  const float s = sinf(g_rollRad);
  const float horizontal = planeGyro[0] * c + planeGyro[1] * s;
  const float vertical   = -planeGyro[0] * s + planeGyro[1] * c;

  g_ptrRateDps[0] = kPointerHorizontalSign * horizontal;
  g_ptrRateDps[1] = kPointerVerticalSign * vertical;

  // 5b. Soft deadzone, per axis. Subtracting the threshold rather than truncating
  //     at it keeps the response continuous; see kHidDeadzoneDps.
  if (kHidDeadzoneDps > 0.0f) {
    for (uint8_t i = 0; i < 2; ++i) {
      const float rate = g_ptrRateDps[i];
      const float mag  = fabsf(rate);
      g_ptrRateDps[i]  = (mag <= kHidDeadzoneDps)
                             ? 0.0f
                             : rate * ((mag - kHidDeadzoneDps) / mag);
    }
  }

  // 6. Integrate. g_pendingDeg is the real accumulator the HID layer drains, and
  //    is capped so a stalled link cannot leave a backlog. g_ptrDeg is a wrapped
  //    display copy only -- never report from it, wrapping would lose rotation.
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
  if (kPrintAxisDiag) {
    Serial.print("accel_mg= ");
    Serial.print(g_accelMg[0], 0);
    Serial.print(',');
    Serial.print(g_accelMg[1], 0);
    Serial.print(',');
    Serial.print(g_accelMg[2], 0);

    Serial.print(" plane_g= ");
    Serial.print(sqrtf(g_planeAccelG[0] * g_planeAccelG[0] +
                       g_planeAccelG[1] * g_planeAccelG[1]),
                 2);

    Serial.print(" gyro= ");
    Serial.print(g_gyroDps[0], 1);
    Serial.print(',');
    Serial.print(g_gyroDps[1], 1);
    Serial.print(',');
    Serial.print(g_gyroDps[2], 1);
    Serial.print(' ');
  }

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

  // Everything below is measured over one shared one-second window.
  const uint32_t rateNowMs = millis();
  const uint32_t rateElapsedMs = rateNowMs - g_diagWindowMs;
  if (rateElapsedMs >= 1000U) {
    g_hidRatePerSec =
        (g_hidReportCount - g_diagReportBase) * 1000U / rateElapsedMs;
    const uint32_t fallbacks = g_hidFallbackCount - g_diagFallbackBase;
    const uint32_t reports = g_hidReportCount - g_diagReportBase;
    g_diagAlignedPct = (reports != 0U)
                           ? ((reports - fallbacks) * 100U / reports)
                           : 100U;
    g_diagLoopHz = (g_loopCount - g_diagLoopBase) * 1000U / rateElapsedMs;
    g_hidWindowPerSec =
        (g_hidWindowCount - g_diagWindowBase) * 1000U / rateElapsedMs;
    g_hidAttemptPerSec =
        (g_hidAttemptCount - g_diagAttemptBase) * 1000U / rateElapsedMs;

    g_diagReportBase    = g_hidReportCount;
    g_diagFallbackBase  = g_hidFallbackCount;
    g_diagWindowBase    = g_hidWindowCount;
    g_diagAttemptBase   = g_hidAttemptCount;
    g_diagLoopBase      = g_loopCount;
    g_diagWindowMs = rateNowMs;
  }

  if (g_hidConnectedMs != 0U) {
    Serial.print(" ci_ms= ");
    Serial.print(static_cast<float>(hidConnIntervalUnits()) * 1.25f, 1);
    Serial.print(" tx_s= ");
    Serial.print(g_hidRatePerSec);
    // win: how often the window was open, against 1000/ci_ms, the events the link
    // offers. try: reports handed over. tx_s: reports the radio took. pace: the
    // hand-over period in ms, so try ~= 1000/pace when paced. tx_s < try means the
    // radio's queue was full, and what is sitting in it is cursor movement to come;
    // the pacing aims to keep the two equal.
    Serial.print(" win= ");
    Serial.print(g_hidWindowPerSec);
    Serial.print(" try= ");
    Serial.print(g_hidAttemptPerSec);
    Serial.print(" pace= ");
    Serial.print(g_hidPacePeriodMs);
    // Share of reports that went out in the aligned window rather than on the
    // rate fallback. 100 means the loop places every report.
    Serial.print(" align= ");
    Serial.print(g_diagAlignedPct);

    // ci_ms sets the ceiling, tx_s is what actually left: a gap means the radio is being
    // handed fewer events than the interval implies.   lat: the host's slave latency (the
    // sketch asks for zero), so it may anchor only every (lat + 1) events.   miss: events
    // the LL failed to receive, i.e. the device dropping them.
    // Pointer dps is tx_s * 127 / kHidCountsPerDeg, and neither raises what the radio takes.
    xiao_nrf54l15::BleConnectionInfo linkInfo;
    if (Bluefruit.rawRadio().getConnectionInfo(&linkInfo)) {
      Serial.print(" lat= ");
      Serial.print(linkInfo.latency);
    }
    xiao_nrf54l15::BleConnectionTimingSnapshot linkSnap;
    if (Bluefruit.rawRadio().getConnectionTimingSnapshot(&linkSnap)) {
      Serial.print(" miss= ");
      Serial.print(linkSnap.missedEventCount);
    }
  }

  Serial.print(" loop_hz= ");
  Serial.print(g_diagLoopHz);

  Serial.println();
}

// ---------------------------------------------------------------------------
// HID transport
// ---------------------------------------------------------------------------

static void hidConnectCallback(uint16_t connHandle) {
  (void)connHandle;
  Serial.println("ble=connected, pairing");
  g_hidConnectedMs  = millis();
  g_hidConnRetries  = 0;
  g_hidConnRetryMs  = 0;
  g_hidConnFastSeen = false;
  g_hidLastWindowUs = 0;
  // Force the pacing to relearn: the ceiling belongs to the link, not the sketch.
  // Left alone it would carry the previous connection's rate into this one.
  g_hidPaceIntervalUnits = 0;
  g_diagWindowMs = millis();
  g_diagLoopBase = g_loopCount;
  g_diagReportBase = g_hidReportCount;
  g_diagFallbackBase = g_hidFallbackCount;
  g_diagWindowBase = g_hidWindowCount;
  g_diagAttemptBase = g_hidAttemptCount;
  g_hidRatePerSec = 0;
  // Ask for a fast interval on the next pass, not up to a poll interval later.
  g_hidConnPollMs = millis() - kConnPollMs;
  g_linkPollMs    = millis() - kLinkPollMs;
  Bluefruit.Security.requestPairing();
}

static void hidDisconnectCallback(uint16_t connHandle, uint8_t reason) {
  (void)connHandle;
  Serial.print("ble=disconnected reason=0x");
  Serial.println(reason, HEX);
  g_hidConnectedMs = 0;
  g_hidConnRetryMs = 0;
  g_hidLastWindowUs = 0;
  // Drop the cache now rather than at the next poll: the link is gone and
  // mouseReport() on it would just fail. Any pending motion is kept, capped, and
  // drains when the link comes back.
  g_linkReady     = false;
  g_linkPollMs    = millis() - kLinkPollMs;
  g_hidConnPollMs = millis() - kConnPollMs;
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

  // Advertised, because that is what the host actually honours (see the
  // kConnInterval* comment). Advertised before begin(), so it applies to the
  // first connection rather than needing a mid-session update.
  Bluefruit.Periph.setConnInterval(kConnIntervalMin, kConnIntervalMax);
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

// Refreshes the cached link answer. Every query below starts by running the stack
// service routine, so this is far more expensive than it looks -- see kLinkPollMs.
// A stale "not ready" only delays the first report by one poll interval.
static void hidPollLink(uint32_t nowMs) {
  if ((nowMs - g_linkPollMs) < kLinkPollMs) {
    return;
  }
  g_linkPollMs = nowMs;
  g_linkReady  = Bluefruit.connected() && Bluefruit.Security.isEncrypted(0) &&
                 blehid.mouseNotifyEnabled();
}

// Reports need an encrypted link; before that mouseReport() returns false.
static bool hidLinkReady() {
  return g_linkReady;
}

// Degrees to HID counts, clamped to the signed 8-bit report field. Rounded rather
// than truncated: truncation would make slow rotation build a whole count before
// anything is sent, exactly where fine positioning happens.
static int8_t hidCountsFor(float deg) {
  long whole = lroundf(deg * kHidCountsPerDeg);
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

// One report per connection event, so the cadence follows the link. Cached: it
// moves only when the host accepts a parameter update.
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

// The hand-over period actually in force: the link's own event period, or the
// paced period once one has been measured, whichever is longer. Never shorter than
// the event period, which would just queue reports the link cannot carry.
static uint32_t hidAttemptPeriodMs() {
  return (g_hidPacePeriodMs > g_hidReportPeriodMs) ? g_hidPacePeriodMs
                                                   : g_hidReportPeriodMs;
}

// Tracks the hand-over rate the radio actually accepts and derives the period from
// it. Polled rather than per pass: it reads the connection interval, and that query
// is not free (see kLinkPollMs).
static void hidMaintainPace(uint32_t nowMs) {
  if (kHidPaceForceMs != 0U) {
    // Bypass the adaptive floor. The actual hand-over period is still
    // max(kHidPaceForceMs, g_hidReportPeriodMs), i.e. 8 ms at a 7.5 ms
    // connection interval.
    g_hidPacePeriodMs = kHidPaceForceMs;
    return;
  }
  if (!hidLinkReady()) {
    // Pairing and notify-enable are not link-rate samples. Restart the window
    // when the link is ready so the first measurement reflects the ceiling,
    // not the pairing transient (which would park the pace too slowly).
    g_hidPaceIntervalUnits = 0;
    g_hidPacePeriodMs      = 0;
    g_hidPaceFloorMs       = 0;
    g_hidPaceWindowMs      = nowMs;
    g_hidPaceBase          = g_hidReportCount;
    g_hidPaceAttemptBase   = g_hidAttemptCount;
    return;
  }
  if ((nowMs - g_hidPacePollMs) < kConnPollMs) {
    return;
  }
  g_hidPacePollMs = nowMs;

  const uint16_t units = hidConnIntervalUnits();
  if (units == 0U) {
    return;  // no link, nothing to measure
  }
  if (units != g_hidPaceIntervalUnits) {
    // A new link, or the host moved the interval. Whatever period was in force was
    // set against the old rate, so start from scratch instead of creeping to it.
    g_hidPaceIntervalUnits = units;
    g_hidPacePeriodMs      = 0;
    g_hidPaceFloorMs       = 0;
    g_hidPaceWindowMs      = nowMs;
    g_hidPaceBase          = g_hidReportCount;
    g_hidPaceAttemptBase   = g_hidAttemptCount;
    return;
  }

  const uint32_t elapsedMs = nowMs - g_hidPaceWindowMs;
  if (elapsedMs < kHidPaceWindowMs) {
    return;
  }
  const uint32_t reports  = g_hidReportCount - g_hidPaceBase;
  const uint32_t attempts = g_hidAttemptCount - g_hidPaceAttemptBase;
  g_hidPaceBase        = g_hidReportCount;
  g_hidPaceAttemptBase = g_hidAttemptCount;
  g_hidPaceWindowMs    = nowMs;

  // A second with almost nothing handed over says nothing about the link: the hand
  // was still. Leave the period where it is rather than adapting to silence.
  if (attempts < kHidPaceMinRate) {
    return;
  }

  // The period in force. 0 means unpaced, which is the link's own event period.
  uint32_t periodMs = (g_hidPacePeriodMs != 0U) ? g_hidPacePeriodMs
                                                : hidAttemptPeriodMs();

  if (reports == 0U) {
    // Nothing got through at all while reports were on offer. Treat the link as
    // unusable rather than dividing by it, and back right off.
    periodMs = kHidReportCeilMs;
  } else if (reports < attempts) {
    // Refusals mean the queue was full, so what left is exactly what the link drains.
    // Park a margin slower: the queue then empties and stays empty, instead of holding
    // a report per attempt period of motion the hand has already finished.
    periodMs = (1000U / reports) + kHidPaceMarginMs;
    g_hidPaceFloorMs = periodMs;
    g_hidPaceHoldMs  = nowMs;
  } else if (g_hidPaceFloorMs == 0U) {
    // Nothing refused, and no drain measured yet to park on: probe faster.
    periodMs = (periodMs * kHidPaceProbePct) / 100U;
  } else {
    // Nothing refused, so the link kept up with the period in force. Hold it, and ease
    // the floor down now and then: a probe is the only way to find a link that got
    // faster, and it costs one second of refusals when the guess was wrong.
    if ((nowMs - g_hidPaceHoldMs) >= kHidPaceHoldMs) {
      if (g_hidPaceFloorMs > kHidReportFloorMs) {
        --g_hidPaceFloorMs;
      }
      g_hidPaceHoldMs = nowMs;
    }
    periodMs = g_hidPaceFloorMs;
  }

  if (periodMs < kHidReportFloorMs) {
    periodMs = kHidReportFloorMs;
  } else if (periodMs > kHidReportCeilMs) {
    periodMs = kHidReportCeilMs;
  }
  g_hidPacePeriodMs = periodMs;
}

// Connection-interval upkeep. Once encrypted it re-asks for 7.5 ms; before that
// it does nothing, because an update request would only be rejected. Polled on
// kConnPollMs -- see there for why not per pass.
static void hidMaintainConnInterval(uint32_t nowMs) {
  if ((nowMs - g_hidConnPollMs) < kConnPollMs) {
    return;
  }
  g_hidConnPollMs = nowMs;

  if (!Bluefruit.connected()) {
    return;
  }

  const uint32_t ciMs =
      (static_cast<uint32_t>(hidConnIntervalUnits()) * 125U) / 100U;

  if (!Bluefruit.Security.isEncrypted(0)) {
    return;
  }

  if (ciMs != 0U && ciMs <= kConnFastThresholdMs) {
    if (!g_hidConnFastSeen) {
      g_hidConnFastSeen = true;
      Serial.print("hid=fast-conn-interval in use ci_ms=");
      Serial.println(static_cast<float>(hidConnIntervalUnits()) * 1.25f, 1);
    }
    return;
  }

  if (g_hidConnRetries >= kConnAttempts) {
    return;
  }
  if (g_hidConnRetries != 0U && (nowMs - g_hidConnRetryMs) < kConnRetryGapMs) {
    return;
  }

  g_hidConnRetryMs = nowMs;
  ++g_hidConnRetries;
  Bluefruit.Periph.setConnInterval(kConnIntervalMin, kConnIntervalMax);
  Serial.print("hid=asking for conn interval ");
  Serial.print(static_cast<float>(kConnIntervalMin) * 1.25f, 1);
  Serial.print('-');
  Serial.print(static_cast<float>(kConnIntervalMax) * 1.25f, 1);
  Serial.print(" ms (attempt ");
  Serial.print(g_hidConnRetries);
  Serial.println(')');
}

static void hidTask(uint32_t nowMs) {
  hidPollButton(nowMs);
  hidPollLink(nowMs);
  hidMaintainConnInterval(nowMs);
  hidMaintainPace(nowMs);

  if (!hidLinkReady()) {
    // Drop the backlog rather than firing it off in a burst the moment the
    // link comes back, which would fling the cursor across the screen.
    g_pendingDeg[0] = 0.0f;
    g_pendingDeg[1] = 0.0f;
    g_hidLastButtons = 0;
    g_hidLastReportMs = nowMs;
    g_hidLastKeepaliveMs = nowMs;
    g_hidLastWindowUs = 0;
    return;
  }

  // Read the radio's anchor before deciding anything: it paces the report.
  bool windowOpen = false;
  xiao_nrf54l15::BleConnectionTimingSnapshot timing;
  if (Bluefruit.rawRadio().getConnectionTimingSnapshot(&timing)) {
    // Hand over just before the next event so it leaves in that event: a placement aid
    // now, not the rate limit. A timeUntilNextEventUs of 0 means the anchor has gone by,
    // so this event cannot be caught and there is no window to aim at.
    const uint32_t windowSpacingUs = (timing.intervalUs > kHidSendLeadUs)
                                         ? (timing.intervalUs - kHidSendLeadUs)
                                         : timing.intervalUs;
    windowOpen = timing.connected && timing.timeUntilNextEventUs != 0U &&
                 timing.timeUntilNextEventUs <= kHidSendLeadUs &&
                 (g_hidLastWindowUs == 0U ||
                  (uint32_t)(timing.nowUs - g_hidLastWindowUs) >= windowSpacingUs);
  }

  const uint8_t buttons = g_buttonStable ? MOUSE_BUTTON_LEFT : 0;
  const bool buttonChanged = (buttons != g_hidLastButtons);
  if (windowOpen) {
    ++g_hidWindowCount;
  }
  const int8_t dx = hidCountsFor(g_pendingDeg[0]);
  const int8_t dy = hidCountsFor(g_pendingDeg[1]);
  const bool keepaliveDue =
      kHidIdleReportEnabled && ((nowMs - g_hidLastKeepaliveMs) >= kHidIdleReportMs);

  if (dx == 0 && dy == 0 && !buttonChanged && !keepaliveDue) {
    // Deliberately no g_hidLastReportMs refresh here. The period now means "since
    // the last hand-over", so letting it run out while the hand is still makes the
    // first report of the next movement leave at once instead of a period late.
    return;  // nothing to say, and no keepalive wanted yet
  }

  // Two routes, both capped at one hand-over per attempt period so the radio's queue
  // cannot build up behind us. The window places the report for the event; the fallback
  // covers a loop that keeps missing it.
  const uint32_t attemptPeriodMs = hidAttemptPeriodMs();
  const uint32_t sinceReportMs   = nowMs - g_hidLastReportMs;
  const bool sendNow =
      (windowOpen && sinceReportMs >= attemptPeriodMs) ||
      (sinceReportMs >= (attemptPeriodMs + kHidSendSlackMs));
  if (!sendNow) {
    return;
  }

  g_hidLastReportMs = nowMs;
  hidRefreshReportPeriod();
  ++g_hidAttemptCount;

  if (blehid.mouseReport(buttons, dx, dy, 0, 0)) {
    // Subtract only what was actually sent. The remainder stays pending, which
    // is what keeps slow movement smooth instead of quantised.
    g_pendingDeg[0] -= static_cast<float>(dx) / kHidCountsPerDeg;
    g_pendingDeg[1] -= static_cast<float>(dy) / kHidCountsPerDeg;
    g_hidLastButtons = buttons;
    g_hidLastKeepaliveMs = nowMs;
    ++g_hidReportCount;
    if (windowOpen) {
      g_hidLastWindowUs = timing.nowUs;
    } else {
      ++g_hidFallbackCount;
    }
  }
  // On failure the motion simply stays pending and goes out on the next event.
}

// ---------------------------------------------------------------------------
// IMU task
// ---------------------------------------------------------------------------

static void announceSensor() {
  Serial.print("imu=ready addr=0x");
  Serial.print(g_imuAddress, HEX);
  Serial.print(" who=0x");
  Serial.print(g_whoAmI, HEX);
  Serial.print(" range=+/-4g,500dps odr=416Hz aiming_axis=");
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
    delay(1);  // let the sensor fill its output registers
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
  // Gated on the switch, not just the interval: printState ends with two stack-servicing
  // link queries, and there is no point paying for those ten times a second when nobody
  // is reading the line.
  if (kSerialLogEnabled && (nowMs - g_lastPrintMs >= kPrintIntervalMs)) {
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
  Serial.println("Air_Mouse - BLE HID air mouse");
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
  ++g_loopCount;
  imuTask();
  hidTask(millis());
}
