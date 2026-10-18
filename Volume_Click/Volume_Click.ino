// Volume_Click - the smallest possible BLE HID sanity check.
//
// Button pressed -> HID consumer "Volume Increment" -> button held for as long
// as you hold the physical button -> released.
//
// Holding the button is a deliberate experiment. If the host honours the press,
// a two-second hold steps the volume immediately and keeps going. If the first
// step still arrives ~460 ms in and then repeats on its own, the host is
// ignoring the press and what you are hearing is its auto-repeat.
//
// There is no IMU, no I2C, no accumulator, no report scheduling, no deadzone and
// no windows here. Nothing but a debounced edge and one notify() call. That is
// the point: Air_Mouse has ~700 ms of latency somewhere and too many moving
// parts to tell where. If this sketch changes the volume instantly, the BLE HID
// transport is fine and the problem is in how Air_Mouse produces mouse reports.
// If this sketch is also ~700 ms late, the problem is the transport or the host,
// and no amount of pointer tuning will touch it.
//
// Serial lines on each edge:
//
//   click=down t=12345 us=9 ok=1 ci=15.0
//
//   t   = millis() when the debounced edge was seen
//   us  = time the notify() call itself took, on this device
//   ok  = notify() accepted it (0 means the link was not ready / not encrypted)
//   ci  = connection interval the host actually settled on, ms
//
// "us" is the only number that measures this side. Everything after it is the
// link and the phone.

#include <bluefruit.h>
#include "nrf54l15_hal.h"

static const char* kDeviceName = "Volume Click";

// Onboard button, active low with the internal pull-up.
static const uint8_t kButtonPin = PIN_BUTTON;

// HID consumer page 0x0C, usage Volume Increment (0x00E9).
//
// If a host ignores this, the fallback is the keyboard page: key code 0x80 is
// Volume Up, sendable with blehid.keyPress()/keyRelease(). Consumer control is
// the correct usage though, and Android maps it to KEYCODE_VOLUME_UP.
static const uint16_t kUsageVolumeUp = 0x00E9;

static const uint32_t kDebounceMs = 15;
static const uint32_t kMinHoldMs  = 30;  // keeps a fast tap a valid press

// Connection interval to ask for, in 1.25 ms units.
//
// This is the experiment. The 460 ms delay measured on the Samsung is ~31
// connection events at 15 ms, which is either a fixed number of events buffered
// somewhere in the host or a fixed host timer. Those two have opposite fixes, and
// changing this constant tells them apart:
//
//   value here       | latency scales with it | latency stays ~460 ms
//   -----------------|------------------------|--------------------------
//   (6, 6)  = 7.5 ms | ~230 ms                | ~460 ms, and the host has a
//   (6, 12) = 15 ms  | ~460 ms  <- currently  | timer we cannot reach from here
//   (24, 24)= 30 ms  | ~920 ms                |
//
// So: try (6, 6) and (24, 24) and note the latency for each. If it scales, the
// shorter interval is both the cure and the proof.
static const uint16_t kConnIntervalMin = 6;   // 7.5 ms
static const uint16_t kConnIntervalMax = 12;  // 15 ms

BLEDis         bledis;
BLEBas         blebas;
BLEHidAdafruit blehid;

static bool     g_raw            = false;  // last reading
static bool     g_stable         = false;  // debounced state
static uint32_t g_rawChangeMs    = 0;
static bool     g_releasePending = false;
static uint32_t g_releaseAtMs    = 0;
static uint32_t g_pressAtMs      = 0;
static uint32_t g_pressCount     = 0;

// LED pulse on every edge. This is the point of the test you cannot get from a
// serial log: the LED is on this device and the volume slider is on the phone,
// so watching both at once tells you which side the delay is on without any
// clock comparison. Longer than the HID hold so it is actually visible.
static const uint32_t kLedPulseMs = 120;
static uint32_t g_ledOffMs        = 0;

static float connectionIntervalMs() {
  const BLEConnection* conn = Bluefruit.Connection(0);
  return (conn != nullptr)
             ? (static_cast<float>(conn->getConnectionInterval()) * 1.25f)
             : 0.0f;
}

// Link health, and the only remaining number that can tell the two sides apart.
//
// ev_s counts how many *distinct* connection events this peripheral actually
// observes per second: the radio's event counter is sampled often and counted
// only when it changes. ev_delta_s sums the counter's steps instead.
//
//   ev_s == ev_delta_s  -> the peripheral participates in every event, so the
//                          link is delivering at the advertised rate and any
//                          delay is above the radio.
//   ev_s << ev_delta_s  -> the peripheral only takes part every N events and the
//                          counter jumps N at a time. Then the link itself is
//                          the delay and no HID-side change will fix it.
//
// That distinction is invisible from a single "events per second" figure, which
// is what the old diagnostic reported and why it could not settle this.
static uint16_t g_lastCounter    = 0;
static bool     g_counterValid   = false;
static uint32_t g_evChanges      = 0;
static uint32_t g_evDelta        = 0;
static uint32_t g_windowMs       = 0;
static uint32_t g_evPerSec       = 0;
static uint32_t g_evDeltaPerSec  = 0;

static void monitorLink(uint32_t nowMs) {
  static uint32_t lastSampleMs = 0;
  if ((nowMs - lastSampleMs) < 2U) {
    return;  // sampling faster than the interval buys nothing
  }
  lastSampleMs = nowMs;

  xiao_nrf54l15::BleConnectionTimingSnapshot snapshot;
  if (Bluefruit.rawRadio().getConnectionTimingSnapshot(&snapshot)) {
    if (g_counterValid && snapshot.nextEventCounter != g_lastCounter) {
      ++g_evChanges;
      g_evDelta += static_cast<uint16_t>(snapshot.nextEventCounter - g_lastCounter);
    }
    g_lastCounter  = snapshot.nextEventCounter;
    g_counterValid = true;
  }

  if ((nowMs - g_windowMs) < 1000U) {
    return;
  }
  g_evPerSec      = g_evChanges;
  g_evDeltaPerSec = g_evDelta;
  g_evChanges     = 0;
  g_evDelta       = 0;
  g_windowMs      = nowMs;

  if (!Bluefruit.connected()) {
    return;
  }
  const BLEConnection* conn = Bluefruit.Connection(0);
  Serial.print("link ev_s=");
  Serial.print(g_evPerSec);
  Serial.print(" ev_delta_s=");
  Serial.print(g_evDeltaPerSec);
  Serial.print(" ci_ms=");
  Serial.print(connectionIntervalMs(), 1);
  Serial.print(" lat=");
  Serial.print((conn != nullptr) ? conn->getSlaveLatency() : 0U);
  Serial.println();
}

static void reportEdge(const char* what, uint32_t nowMs, uint32_t callUs, bool ok) {
  Serial.print("click=");
  Serial.print(what);
  Serial.print(" n=");
  Serial.print(g_pressCount);
  Serial.print(" t=");
  Serial.print(nowMs);
  Serial.print(" us=");
  Serial.print(callUs);
  Serial.print(" ok=");
  Serial.print(ok ? 1 : 0);
  Serial.print(" ci=");
  Serial.print(connectionIntervalMs(), 1);
  Serial.print(" lat=");
  const BLEConnection* conn = Bluefruit.Connection(0);
  Serial.print((conn != nullptr) ? conn->getSlaveLatency() : 0);
  Serial.println();
}

static void onConnect(uint16_t connHandle) {
  (void)connHandle;
  Serial.println("ble=connected, pairing (HID needs an encrypted link)");
  Bluefruit.Security.requestPairing();
}

static void onDisconnect(uint16_t connHandle, uint8_t reason) {
  (void)connHandle;
  Serial.print("ble=disconnected reason=0x");
  Serial.println(reason, HEX);
}

static void startAdvertising() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_MOUSE);
  Bluefruit.Advertising.addService(blehid, blebas);
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // 20 ms fast, 152.5 ms slow
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("Volume_Click - minimal BLE HID check (button = volume up)");

  pinMode(kButtonPin, INPUT_PULLUP);

  // Active low on this board.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Bluefruit.begin();
  Bluefruit.setName(kDeviceName);
  Bluefruit.setAppearance(BLE_APPEARANCE_HID_MOUSE);
  Bluefruit.setTxPower(4);

  // No display, no keypad, so Just Works pairing instead of a passkey this
  // device could never show or type.
  Bluefruit.Security.setIOCaps(false, false, false);

  // Ask for the same interval Air_Mouse runs at, so the comparison is fair.
  // No retry ladder: whatever the host grants is what this test measures.
  Bluefruit.Periph.setConnInterval(kConnIntervalMin, kConnIntervalMax);  // 7.5-15 ms
  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);

  bledis.setManufacturer("Volume Click");
  bledis.setModel("XIAO nRF54L15");
  bledis.begin();
  blebas.begin();
  blebas.write(100);

  blehid.begin();

  startAdvertising();
  Serial.println("ble=advertising (pair from the host, then press the button)");
}

void loop() {
  const uint32_t nowMs = millis();
  monitorLink(nowMs);

  const bool raw = (digitalRead(kButtonPin) == LOW);

  if (raw != g_raw) {
    // Edge seen; restart the debounce window and say nothing yet.
    g_raw = raw;
    g_rawChangeMs = nowMs;
  } else if (g_raw != g_stable && (nowMs - g_rawChangeMs) >= kDebounceMs) {
    g_stable = g_raw;

    if (g_stable) {
      ++g_pressCount;

      // Flash the LED first, before touching the radio, so its latency is the
      // button path alone and not contaminated by connect/notify work.
      digitalWrite(LED_BUILTIN, LOW);
      g_ledOffMs = nowMs + kLedPulseMs;

      const uint32_t startUs = micros();
      const bool ok = blehid.consumerKeyPress(kUsageVolumeUp);
      reportEdge("down", nowMs, micros() - startUs, ok);

      g_pressAtMs      = nowMs;
      g_releaseAtMs    = nowMs + kMinHoldMs;
      g_releasePending = true;
    }
  }

  if (g_ledOffMs != 0U && (int32_t)(nowMs - g_ledOffMs) >= 0) {
    g_ledOffMs = 0U;
    digitalWrite(LED_BUILTIN, HIGH);
  }

  // The usage stays pressed for as long as the button is physically held (with a
  // floor so a quick tap still registers). That is the point of this version: if
  // the host honours the press, a two-second hold changes the volume immediately
  // and keeps stepping. If the first step still arrives ~460 ms in and then
  // repeats, the host is ignoring the press and you are hearing its auto-repeat.
  if (g_releasePending && !g_stable && (int32_t)(nowMs - g_releaseAtMs) >= 0) {
    g_releasePending = false;
    const uint32_t startUs = micros();
    const bool ok = blehid.consumerKeyRelease();
    reportEdge("up", nowMs, micros() - startUs, ok);
    Serial.print("hold_ms=");
    Serial.println(nowMs - g_pressAtMs);
  }
}
