#include <Arduino.h>
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

#include "Logger.h"
#include "PocketMaster.h"
#include "DualTB6612.h"
#include "PFServo.h"

#if __has_include("secrets.h")
  // secrets.h exists only on the local computer and is ignored by Git,
  // preventing the Wi-Fi SSID and password from entering the public repository.
  #include "secrets.h"
#else
  // The project can still compile without credentials. RC control remains
  // available, while Wi-Fi and WebSerial stay disabled.
  #define WIFI_SSID ""
  #define WIFI_PASSWORD ""
#endif

// ============================================================
// Hardware setup
// ============================================================
constexpr int PIN_RX = 21;
constexpr int PIN_TX = 20;

constexpr int PIN_SDA = 8;
constexpr int PIN_SCL = 9;

constexpr int PIN_STBY = -1;

constexpr int PIN_DIR1_XL1 = 1;
constexpr int PIN_DIR2_XL1 = 0;
constexpr int PIN_PWM_XL1  = 7;

constexpr int PIN_DIR1_XL2 = 2;
constexpr int PIN_DIR2_XL2 = 3;
constexpr int PIN_PWM_XL2  = 6;

constexpr int PIN_DIR1_L = 5;
constexpr int PIN_DIR2_L = 4;

constexpr int PIN_SERVO = 10;

constexpr int CH_PWM_XL1 = 0;
constexpr int CH_PWM_XL2 = 1;
constexpr int CH_SERVO   = 2;

DualTB6612::ChannelPins chA{PIN_DIR1_XL1, PIN_DIR2_XL1, PIN_PWM_XL1, CH_PWM_XL1};
DualTB6612::ChannelPins chB{PIN_DIR1_XL2, PIN_DIR2_XL2, PIN_PWM_XL2, CH_PWM_XL2};

// The motors use 20 kHz PWM with 10-bit resolution (duty 0..1023).
// ZeroMode::Brake actively brakes the H-bridge whenever the command is zero.
DualTB6612 motors(PIN_STBY, chA, chB, 20000, 10, ZeroMode::Brake);

// The LEGO PF servo uses LEDC channel 2. PFServo defaults to 50 Hz,
// 16-bit resolution, and a pulse-width range of 1000..2000 microseconds.
PFServo steering(PIN_SERVO, CH_SERVO);

// CRSF uses UART1, independently from USB Serial, so debug output cannot
// interfere with the ExpressLRS receiver link running at 420000 baud.
HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;
PocketMaster PocketRadio(crsf);

// AsyncWebServer serves the WebSerial page on the standard HTTP port (80).
AsyncWebServer server(80);

// The web server starts only once, after the ESP32 first obtains an IP address.
bool webSerialStarted = false;
bool wifiEnabled = false;
wl_status_t previousWiFiStatus = WL_IDLE_STATUS;
unsigned long lastWiFiAttemptMs = 0;

// This state machine replaces the previous delay-based failsafe sequence.
// Using millis() keeps CRSF reception and Wi-Fi servicing responsive.
enum class FailsafeStage : uint8_t {
  Idle,
  Braking,
  Coasting
};

FailsafeStage failsafeStage = FailsafeStage::Idle;
unsigned long failsafeStageStartedMs = 0;

void debugPrintChannels();
void startWiFi();
void serviceWiFi();
void resetFailsafe();
void serviceFailsafe();

// Every LOGx() macro eventually calls this function.
//
// USB Serial receives the original line, including ANSI escape sequences, so
// log levels remain color-coded in a compatible terminal such as PlatformIO.
//
// WebSerial receives a clean copy with ANSI sequences removed. Its browser UI
// does not need to interpret terminal color-control bytes.
void writeLogLine(const char* line) {
  Serial.print(line);

  if (!webSerialStarted) return;

  char webLine[256];
  size_t writeIndex = 0;
  bool insideAnsiSequence = false;

  for (size_t readIndex = 0;
       line[readIndex] != '\0' && writeIndex < sizeof(webLine) - 1;
       ++readIndex) {
    const char current = line[readIndex];

    // ANSI colors begin with ESC (ASCII 27), followed by characters such as
    // "[32m". Skip everything from ESC through the terminating letter 'm'.
    if (!insideAnsiSequence && current == '\x1b') {
      insideAnsiSequence = true;
      continue;
    }

    if (insideAnsiSequence) {
      if (current == 'm') {
        insideAnsiSequence = false;
      }
      continue;
    }

    webLine[writeIndex++] = current;
  }

  webLine[writeIndex] = '\0';
  WebSerial.print(webLine);
}

void startWiFi() {
  // An empty SSID means include/secrets.h has not been created yet.
  wifiEnabled = WIFI_SSID[0] != '\0';

  if (!wifiEnabled) {
    LOGW("WIFI", "Disabled: copy secrets.example.h to include/secrets.h and fill credentials");
    return;
  }

  WiFi.mode(WIFI_STA);
  // Station mode joins the existing IndiHome router instead of creating
  // a separate ESP32 access point.
  WiFi.setAutoReconnect(true);
  // Avoid repeatedly writing Wi-Fi configuration to internal flash.
  WiFi.persistent(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWiFiAttemptMs = millis();

  LOGI("WIFI", "Connecting to %s...", WIFI_SSID);
}

void serviceWiFi() {
  // This function is called every loop and never blocks while waiting for Wi-Fi.
  if (!wifiEnabled) return;

  const wl_status_t status = WiFi.status();

  if (status == WL_CONNECTED) {
    if (previousWiFiStatus != WL_CONNECTED) {
      LOGI("WIFI", "Connected. IP: %s", WiFi.localIP().toString().c_str());

      if (!webSerialStarted) {
        // WebSerial registers its /webserial route with AsyncWebServer.
        WebSerial.begin(&server);
        server.begin();
        webSerialStarted = true;
        LOGI("WEB", "Open http://%s/webserial", WiFi.localIP().toString().c_str());
      }
    }
  } else {
    if (previousWiFiStatus == WL_CONNECTED) {
      LOGW("WIFI", "Connection lost; retrying in background");
    }

    if (millis() - lastWiFiAttemptMs >= 10000UL) {
      // Limit reconnection attempts to once every 10 seconds.
      lastWiFiAttemptMs = millis();
      WiFi.reconnect();
    }
  }

  previousWiFiStatus = status;

  if (webSerialStarted) {
    // Perform WebSocket client cleanup and periodically flush buffered logs.
    WebSerial.loop();
  }
}

void resetFailsafe() {
  // Called as soon as the CRSF link becomes available again.
  failsafeStage = FailsafeStage::Idle;
}

void serviceFailsafe() {
  const unsigned long now = millis();

  switch (failsafeStage) {
    case FailsafeStage::Idle:
      // First stage: disarm and actively brake both motors immediately.
      if (PocketRadio.isArmed) {
        LOGW("FAILSAFE", "Receiver lost -> disarmed");
        PocketRadio.isArmed = false;
      } else {
        LOGI("CRSF", "Receiver disconnected");
      }

      motors.setSpeedA(0);
      motors.setSpeedB(0);
      failsafeStage = FailsafeStage::Braking;
      failsafeStageStartedMs = now;
      break;

    case FailsafeStage::Braking:
      // After 800 ms of braking, release both motors into coast mode.
      if (now - failsafeStageStartedMs >= 800UL) {
        motors.coastA();
        motors.coastB();
        failsafeStage = FailsafeStage::Coasting;
        failsafeStageStartedMs = now;
      }
      break;

    case FailsafeStage::Coasting:
      // Remain in coast for 1000 ms, then restart the monitoring cycle.
      if (now - failsafeStageStartedMs >= 1000UL) {
        failsafeStage = FailsafeStage::Idle;
      }
      break;
  }
}

void setup() {
  // USB Serial provides local debug output at 115200 baud.
  Serial.begin(115200);

  const unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 1500UL)) {
    delay(10);
  }

  LOGI("BOOT", "Serial ready. Starting CRSF...");

  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  crsf.begin(crsfSerial);
  LOGI("CRSF", "UART1 @ %d baud (RX=%d TX=%d)",
       (int)CRSF_BAUDRATE, PIN_RX, PIN_TX);

  // "rc-cal" is the NVS namespace used to store receiver calibration values.
  PocketRadio.begin("rc-cal");
  // Small movements within +/-20 microseconds of center are treated as neutral.
  PocketRadio.setDeadzoneUs(20);
  PocketRadio.loadCalibration();

  if (PocketRadio.isCallibrated) {
    LOGI("LED", "Calibration found -> Blink 3x");
  }

  steering.begin();
  // setInput(0) centers the servo because PFServo accepts -1000..+1000.
  steering.setInput(0);

  motors.begin();
  motors.setZeroMode(ZeroMode::Brake);
  // Safety: remain braked at boot until CRSF is linked and SA is armed.

  startWiFi();
}

void loop() {
  // Call this as frequently as possible to process CRSF frames and link status.
  PocketRadio.update();
  // Service Wi-Fi and WebSerial without pausing the RC control path.
  serviceWiFi();

  if (PocketRadio.isLinkUp()) {
    resetFailsafe();

    if (PocketRadio.val(SA) == 1) {
      // SA in the upper position arms the system. Only then do the motors and
      // steering follow commands from the transmitter.
      if (!PocketRadio.isArmed) {
        PocketRadio.isArmed = true;
        LOGI("RC", "SA UP -> system armed");

        if (!PocketRadio.isCallibrated) {
          PocketRadio.loadCalibration();
        }
      }

      debugPrintChannels();
      // PocketMaster has already normalized THROTTLE to 0..1000.
      motors.setSpeedA(PocketRadio.val(THROTTLE));
      motors.setSpeedB(PocketRadio.val(THROTTLE));

      // PocketMaster represents ROLL as 0..1000 with 500 at center, whereas
      // PFServo expects -1000..+1000 with zero at center.
      const int steeringInput = ((int)PocketRadio.val(ROLL) - 500) * 2;
      steering.setInput(steeringInput);
    } else if (PocketRadio.val(SA) == 3) {
      // SA in the lower position disarms the system and enters calibration,
      // during which channel minimum and maximum values are collected.
      if (PocketRadio.isArmed) {
        LOGI("RC", "SA DOWN -> system disarmed, entering calibration mode");
      }

      PocketRadio.isArmed = false;
      PocketRadio.startCalibration();

      if (PocketRadio.isCalibSaved()) {
        LOGI("CAL",
             "ROLL[max:%4u min:%4u] PITCH[max:%4u min:%4u] YAW[max:%4u min:%4u] THR[max:%4u min:%4u] S1[max:%4u min:%4u]",
             PocketRadio.max(ROLL), PocketRadio.min(ROLL),
             PocketRadio.max(PITCH), PocketRadio.min(PITCH),
             PocketRadio.max(YAW), PocketRadio.min(YAW),
             PocketRadio.max(THROTTLE), PocketRadio.min(THROTTLE),
             PocketRadio.max(S1), PocketRadio.min(S1));
      }
    }
  } else {
    // No long delay is used here; the failsafe runs as a non-blocking state machine.
    serviceFailsafe();
  }

  delay(10);
}

void debugPrintChannels() {
#if defined(DEBUG)
  static unsigned long lastPrintMs = 0;
  const unsigned long now = millis();

  // Control still updates roughly every 10 ms, but telemetry output is limited
  // to 10 Hz to avoid flooding the WebSerial buffer and consuming excess CPU.
  if (now - lastPrintMs < 100UL) return;
  lastPrintMs = now;

  LOGD("CRSF",
       "R:%4u P:%4u T:%4u Y:%4u | SA:%d SB:%d SC:%d SD:%d SE:%d | S1:%4u | LQ:%3u",
       PocketRadio.val(ROLL),
       PocketRadio.val(PITCH),
       PocketRadio.val(THROTTLE),
       PocketRadio.val(YAW),
       PocketRadio.val(SA),
       PocketRadio.val(SB),
       PocketRadio.val(SC),
       PocketRadio.val(SD),
       PocketRadio.val(SE),
       PocketRadio.val(S1),
       PocketRadio.linkQuality());
#endif
}
