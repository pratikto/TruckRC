#include <Arduino.h>
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include "Logger.h"
#include "PocketMaster.h"
#include "DualTB6612.h"
#include "PFServo.h"

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

// CRSF uses UART0 on GPIO21/GPIO20. Debug logging still uses the native
// USB-CDC peripheral, so it does not share this hardware UART with CRSF.
//
// Why UART0 instead of UART1?
// Some Arduino-ESP32 3.x releases have shown ESP32-C3 UART1 RX regressions.
// The original Core 2.x program worked on UART1, but the isolated Core 3.x
// test received no CRSF frames. Using UART0 tests that specific regression
// without changing the receiver wiring, CRSF baud rate, or control logic.
HardwareSerial crsfSerial(0);
AlfredoCRSF crsf;
PocketMaster PocketRadio(crsf);

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
void resetFailsafe();
void serviceFailsafe();

// Every LOGx() macro routes its formatted line to the local USB monitor.
void writeLogLine(const char* line) {
  Serial.print(line);
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
  LOGI("CRSF", "UART0 @ %d baud (RX=%d TX=%d)",
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

}

void loop() {
  // Call this as frequently as possible to process CRSF frames and link status.
  PocketRadio.update();

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
  // to 10 Hz to keep logging from consuming excessive CPU time.
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
