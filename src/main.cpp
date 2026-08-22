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
  #include "secrets.h"
#else
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

DualTB6612 motors(PIN_STBY, chA, chB, 20000, 10, ZeroMode::Brake);
PFServo steering(PIN_SERVO, CH_SERVO);
HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;
PocketMaster PocketRadio(crsf);

AsyncWebServer server(80);

bool webSerialStarted = false;
bool wifiEnabled = false;
wl_status_t previousWiFiStatus = WL_IDLE_STATUS;
unsigned long lastWiFiAttemptMs = 0;

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

void writeLogLine(const char* line) {
  Serial.print(line);
  if (webSerialStarted) {
    WebSerial.print(line);
  }
}

void startWiFi() {
  wifiEnabled = WIFI_SSID[0] != '\0';

  if (!wifiEnabled) {
    LOGW("WIFI", "Disabled: copy secrets.example.h to include/secrets.h and fill credentials");
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWiFiAttemptMs = millis();

  LOGI("WIFI", "Connecting to %s...", WIFI_SSID);
}

void serviceWiFi() {
  if (!wifiEnabled) return;

  const wl_status_t status = WiFi.status();

  if (status == WL_CONNECTED) {
    if (previousWiFiStatus != WL_CONNECTED) {
      LOGI("WIFI", "Connected. IP: %s", WiFi.localIP().toString().c_str());

      if (!webSerialStarted) {
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
      lastWiFiAttemptMs = millis();
      WiFi.reconnect();
    }
  }

  previousWiFiStatus = status;

  if (webSerialStarted) {
    WebSerial.loop();
  }
}

void resetFailsafe() {
  failsafeStage = FailsafeStage::Idle;
}

void serviceFailsafe() {
  const unsigned long now = millis();

  switch (failsafeStage) {
    case FailsafeStage::Idle:
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
      if (now - failsafeStageStartedMs >= 800UL) {
        motors.coastA();
        motors.coastB();
        failsafeStage = FailsafeStage::Coasting;
        failsafeStageStartedMs = now;
      }
      break;

    case FailsafeStage::Coasting:
      if (now - failsafeStageStartedMs >= 1000UL) {
        failsafeStage = FailsafeStage::Idle;
      }
      break;
  }
}

void setup() {
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

  PocketRadio.begin("rc-cal");
  PocketRadio.setDeadzoneUs(20);
  PocketRadio.loadCalibration();

  if (PocketRadio.isCallibrated) {
    LOGI("LED", "Calibration found -> Blink 3x");
  }

  steering.begin();
  steering.setInput(0);

  motors.begin();
  motors.setZeroMode(ZeroMode::Brake);
  // Safety: remain braked at boot until CRSF is linked and SA is armed.

  startWiFi();
}

void loop() {
  PocketRadio.update();
  serviceWiFi();

  if (PocketRadio.isLinkUp()) {
    resetFailsafe();

    if (PocketRadio.val(SA) == 1) {
      if (!PocketRadio.isArmed) {
        PocketRadio.isArmed = true;
        LOGI("RC", "SA UP -> system armed");

        if (!PocketRadio.isCallibrated) {
          PocketRadio.loadCalibration();
        }
      }

      debugPrintChannels();
      motors.setSpeedA(PocketRadio.val(THROTTLE));
      motors.setSpeedB(PocketRadio.val(THROTTLE));
      const int steeringInput = ((int)PocketRadio.val(ROLL) - 500) * 2;
      steering.setInput(steeringInput);
    } else if (PocketRadio.val(SA) == 3) {
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
    serviceFailsafe();
  }

  delay(10);
}

void debugPrintChannels() {
#if defined(DEBUG)
  static unsigned long lastPrintMs = 0;
  const unsigned long now = millis();

  // Keep control updates fast, but limit browser/USB telemetry to 10 Hz.
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
