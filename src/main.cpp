#include <Arduino.h>
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include "Logger.h"

// ============================================================
// ⚙️ Hardware setup
// ============================================================
#define PIN_RX 16
#define PIN_TX 17
#define LED_PIN 2   // ✅ ESP32 DevKitC v4 onboard LED

HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;
Preferences prefs;

// ============================================================
// 📦 RC Data Structures
// ============================================================
struct analogValue {
  uint16_t raw = 1500;
  uint16_t max = 1500;
  uint16_t min = 1500;
  uint16_t val = 500;   // scaled 0..1000 (neutral = 500)
};

struct switch3Position {
  uint16_t raw = 1500;
  bool up = false;
  bool mid = false;
  bool down = false;
};

struct switch2Position {
  uint16_t raw = 1500;
  bool up = false;
  bool down = false;
};

struct RCInput_t {
  uint16_t ch[16];
  analogValue roll, pitch, throttle, yaw, s1;
  switch2Position sa, sd, se;
  switch3Position sb, sc;
  uint16_t LinkQuality = 0;
  bool isCalibrated = false;
  bool isArmed = false;
};

RCInput_t RCInput;

// ============================================================
// 🔧 Utilities & helpers
// ============================================================
static inline int clampi(int v, int lo, int hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

// Integer linear map (inclusive) with clamping
static inline int imap(int x, int inMin, int inMax, int outMin, int outMax) {
  x = clampi(x, inMin, inMax);
  long num = (long)(x - inMin) * (outMax - outMin);
  long den = (long)(inMax - inMin);
  return (int)(outMin + (den ? (num / den) : 0));
}

uint16_t assignMax(uint16_t a, uint16_t b) { return (b > a) ? b : a; }
uint16_t assignMin(uint16_t a, uint16_t b) { return (b < a) ? b : a; }

// Centered axes (roll/pitch/yaw): map [min..mid]→[0..500], [mid..max]→[500..1000] with deadzone
uint16_t scaleCentered(uint16_t raw, uint16_t maxUs, uint16_t minUs, uint16_t deadzoneUs = 20) {
  if (maxUs <= minUs) return 500;                  // not calibrated → neutral
  uint16_t mid = (uint16_t)((minUs + maxUs) / 2);
  raw = (uint16_t)clampi((int)raw, (int)minUs, (int)maxUs);

  // Flat deadzone around mid returns exactly 500
  if ((raw >= mid - deadzoneUs) && (raw <= mid + deadzoneUs)) return 500;

  if (raw < mid - deadzoneUs) {
    // Left side: 0..500
    return (uint16_t)imap((int)raw, (int)minUs, (int)(mid - deadzoneUs), 0, 500);
  } else {
    // Right side: 500..1000
    return (uint16_t)imap((int)raw, (int)(mid + deadzoneUs), (int)maxUs, 500, 1000);
  }
}

// Linear axes (throttle, S1): map [min..max] → [0..1000]
uint16_t scaleLinear(uint16_t raw, uint16_t maxUs, uint16_t minUs) {
  if (maxUs <= minUs) return 0;                    // not calibrated
  return (uint16_t)imap((int)raw, (int)minUs, (int)maxUs, 0, 1000);
}

int getLinkQuality(AlfredoCRSF& inst) {
  const crsfLinkStatistics_t* stat_ptr = inst.getLinkStatistics();
  return stat_ptr ? (int)stat_ptr->uplink_Link_quality : 0;
}

// ============================================================
// 🧩 Decode switch helpers (with string return)
// ============================================================

// Decode 2-position switch
String decodeSwitch2(switch2Position &sw, uint16_t raw, const char* name) {
  String state = "UNKNOWN";
  if (raw > 1800) {
    sw.up = true; sw.down = false; state = "UP";
  } else if (raw < 1200) {
    sw.up = false; sw.down = true; state = "DOWN";
  } else {
    sw.up = sw.down = false; state = "MID";
  }
  if (sw.raw != raw && raw > 0)   // suppress noise when raw == 0 (no frame)
    LOGD("SW", "%s → %s (%u)", name, state.c_str(), raw);
  sw.raw = raw;
  return state;
}

// Decode 3-position switch
String decodeSwitch3(switch3Position &sw, uint16_t raw, const char* name) {
  String state = "UNKNOWN";
  if (raw > 1800) {
    sw.up = true; sw.mid = false; sw.down = false; state = "UP";
  } else if (raw >= 1400 && raw <= 1600) {
    sw.up = false; sw.mid = true; sw.down = false; state = "MID";
  } else if (raw < 1200) {
    sw.up = false; sw.mid = false; sw.down = true; state = "DOWN";
  } else {
    sw.up = sw.mid = sw.down = false; state = "UNKNOWN";
  }
  if (sw.raw != raw && raw > 0)
    LOGD("SW", "%s → %s (%u)", name, state.c_str(), raw);
  sw.raw = raw;
  return state;
}

// ============================================================
// 💾 Persistent calibration (NVS)
// ============================================================
void saveCalibration();
void loadCalibration();

// ============================================================
// 🧠 Function Prototypes
// ============================================================
void updateRCInputs();
void debugPrintChannels();
void updateLED();

// ============================================================
// ⏱️ Auto-save calibration variables
// ============================================================
unsigned long lastCalibChange = 0;
const unsigned long CALIB_SAVE_DELAY = 3000;
bool calibSaved = false;

// ============================================================
// 🚀 Setup
// ============================================================
void setup() {
  Serial.begin(115200);
#if defined(DEBUG)
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 1500)) {}
  LOGI("BOOT", "Serial ready. Starting CRSF...");
#endif

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  crsf.begin(crsfSerial);
  LOGI("CRSF", "UART1 @ %d baud (RX=%d TX=%d)", (int)CRSF_BAUDRATE, PIN_RX, PIN_TX);

  loadCalibration();

  if (RCInput.isCalibrated) {
    LOGI("LED", "Calibration found → Blink 3x");
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_PIN, HIGH); delay(150);
      digitalWrite(LED_PIN, LOW); delay(150);
    }
  }
}

// ============================================================
// 🔁 Main loop
// ============================================================
void loop() {
  crsf.update();

  if (crsf.isLinkUp()) {
    updateRCInputs();

    if (RCInput.sa.up) {
      if (!RCInput.isArmed) {
        RCInput.isArmed = true;
        LOGI("RC", "SA UP → system armed");
        if (!RCInput.isCalibrated) saveCalibration();
      }
      debugPrintChannels();
    } 
    else if (RCInput.sa.down) {
      if (RCInput.isArmed)
        LOGI("RC", "SA DOWN → system disarmed, entering calibration mode");
      RCInput.isArmed = false;

      bool changed = false;
      auto updateMinMax = [&](analogValue& a, uint16_t raw) {
        uint16_t oldMin = a.min, oldMax = a.max;
        a.max = assignMax(a.max, raw);
        a.min = assignMin(a.min, raw);
        if (a.min != oldMin || a.max != oldMax) changed = true;
      };

      updateMinMax(RCInput.roll, RCInput.roll.raw);
      updateMinMax(RCInput.pitch, RCInput.pitch.raw);
      updateMinMax(RCInput.throttle, RCInput.throttle.raw);
      updateMinMax(RCInput.yaw, RCInput.yaw.raw);
      updateMinMax(RCInput.s1, RCInput.s1.raw);

      LOGI("CAL", "ROLL[min:%4u max:%4u] PITCH[min:%4u max:%4u] YAW[min:%4u max:%4u]",
           RCInput.roll.min, RCInput.roll.max,
           RCInput.pitch.min, RCInput.pitch.max,
           RCInput.yaw.min, RCInput.yaw.max);
      LOGI("CAL", "THR[min:%4u max:%4u] S1[min:%4u max:%4u]",
           RCInput.throttle.min, RCInput.throttle.max,
           RCInput.s1.min, RCInput.s1.max);

      if (changed) {
        lastCalibChange = millis();
        calibSaved = false;
      }

      if (!calibSaved && millis() - lastCalibChange > CALIB_SAVE_DELAY) {
        LOGI("CAL", "No changes → auto-saving calibration");
        saveCalibration();
        calibSaved = true;
      }
    }
  } 
  else {
    if (RCInput.isArmed) {
      LOGW("FAILSAFE", "Receiver lost → disarmed");
      RCInput.isArmed = false;
    }
    else{
      LOGI("CRSF", "Receiver disconnected");
    }
    RCInput.throttle.val = 0;
  }

  updateLED();
  delay(50);
}

// ============================================================
// 🧩 Helper Functions
// ============================================================
void updateRCInputs() {
  for (int i = 0; i < 16; i++) RCInput.ch[i] = crsf.getChannel(i + 1);

  RCInput.roll.raw     = RCInput.ch[0];
  RCInput.pitch.raw    = RCInput.ch[1];
  RCInput.throttle.raw = RCInput.ch[2];
  RCInput.yaw.raw      = RCInput.ch[3];
  RCInput.s1.raw       = RCInput.ch[9];

  // mapping yang kamu set: SD=CH5, SB=CH6, SC=CH7, SE=CH8, SA=CH4? (lihat bawah)
  decodeSwitch2(RCInput.sa, RCInput.ch[4], "SA");
  decodeSwitch3(RCInput.sb, RCInput.ch[6], "SB");
  decodeSwitch3(RCInput.sc, RCInput.ch[7], "SC");
  decodeSwitch2(RCInput.sd, RCInput.ch[5], "SD");
  decodeSwitch2(RCInput.se, RCInput.ch[8], "SE");

  RCInput.roll.val     = scaleCentered(RCInput.roll.raw,     RCInput.roll.max,     RCInput.roll.min);
  RCInput.pitch.val    = scaleCentered(RCInput.pitch.raw,    RCInput.pitch.max,    RCInput.pitch.min);
  RCInput.yaw.val      = scaleCentered(RCInput.yaw.raw,      RCInput.yaw.max,      RCInput.yaw.min);
  RCInput.throttle.val = scaleLinear  (RCInput.throttle.raw, RCInput.throttle.max, RCInput.throttle.min);
  RCInput.s1.val       = scaleLinear  (RCInput.s1.raw,       RCInput.s1.max,       RCInput.s1.min);
  RCInput.LinkQuality  = getLinkQuality(crsf);
}

void updateLED() {
  static bool ledState = false;
  static unsigned long lastBlink = 0;
  static unsigned long lastHeartbeat = 0;

  if (RCInput.isArmed) {
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  if (crsf.isLinkUp() && RCInput.sa.down) {
    if (millis() - lastBlink > 500) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastBlink = millis();
    }
    return;
  }

  if (crsf.isLinkUp()) {
    unsigned long now = millis();
    if (now - lastHeartbeat < 100) digitalWrite(LED_PIN, HIGH);
    else if (now - lastHeartbeat < 500) digitalWrite(LED_PIN, LOW);
    else if (now - lastHeartbeat >= 5000) lastHeartbeat = now;
    return;
  }

  digitalWrite(LED_PIN, LOW);
}

void saveCalibration() {
  prefs.begin("rc-cal", false);
  prefs.putUShort("roll_min", RCInput.roll.min);
  prefs.putUShort("roll_max", RCInput.roll.max);
  prefs.putUShort("pitch_min", RCInput.pitch.min);
  prefs.putUShort("pitch_max", RCInput.pitch.max);
  prefs.putUShort("thrt_min", RCInput.throttle.min);
  prefs.putUShort("thrt_max", RCInput.throttle.max);
  prefs.putUShort("yaw_min", RCInput.yaw.min);
  prefs.putUShort("yaw_max", RCInput.yaw.max);
  prefs.putUShort("s1_min", RCInput.s1.min);
  prefs.putUShort("s1_max", RCInput.s1.max);
  prefs.putBool("isCal", true);
  prefs.end();
  RCInput.isCalibrated = true;
  LOGI("NVS", "Calibration saved");
}

void loadCalibration() {
  prefs.begin("rc-cal", true);
  RCInput.roll.min     = prefs.getUShort("roll_min", 1500);
  RCInput.roll.max     = prefs.getUShort("roll_max", 1500);
  RCInput.pitch.min    = prefs.getUShort("pitch_min", 1500);
  RCInput.pitch.max    = prefs.getUShort("pitch_max", 1500);
  RCInput.throttle.min = prefs.getUShort("thrt_min", 1500);
  RCInput.throttle.max = prefs.getUShort("thrt_max", 1500);
  RCInput.yaw.min      = prefs.getUShort("yaw_min", 1500);
  RCInput.yaw.max      = prefs.getUShort("yaw_max", 1500);
  RCInput.s1.min       = prefs.getUShort("s1_min", 1500);
  RCInput.s1.max       = prefs.getUShort("s1_max", 1500);
  RCInput.isCalibrated = prefs.getBool("isCal", false);
  prefs.end();

  if (RCInput.isCalibrated)
    LOGI("NVS", "Calibration loaded");
  else
    LOGW("NVS", "No calibration found; please calibrate");
}

void debugPrintChannels() {
#if defined(DEBUG)
  LOGD("CRSF",
       "R:%4u P:%4u T:%4u Y:%4u | SA(U:%d D:%d) SB(U:%d M:%d D:%d) SC(U:%d M:%d D:%d) SD(U:%d D:%d) SE(U:%d D:%d) | S1:%4u",
       RCInput.roll.val, RCInput.pitch.val, RCInput.throttle.val, RCInput.yaw.val,
       RCInput.sa.up, RCInput.sa.down,
       RCInput.sb.up, RCInput.sb.mid, RCInput.sb.down,
       RCInput.sc.up, RCInput.sc.mid, RCInput.sc.down,
       RCInput.sd.up, RCInput.sd.down,
       RCInput.se.up, RCInput.se.down,
       RCInput.s1.val);
#endif
}
