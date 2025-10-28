#include <Arduino.h>
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include "Logger.h"
#include "PocketMaster.h"
#include "DualTB6612.h"

// 🔹 [ADDED for Bluetooth logging]
#include <BluetoothSerial.h>
BluetoothSerial BT;

// 🔹 define logger sinks (multi-output)
Print* LOG_OUT1 = &Serial;
Print* LOG_OUT2 = nullptr;   // set later to &BT in setup()

// ============================================================
// ⚙️ Hardware setup
// ============================================================
#define PIN_RX 16
#define PIN_TX 17
#define LED_PIN 2   // ✅ ESP32 DevKitC v4 onboard LED

// === Mapping PIN sesuai skematik ===
// STBY_DRIVER_XL -> masukkan nomor GPIO yang kamu pakai
constexpr int PIN_STBY = 19; // contoh: GPIO19 (ubah sesuai board)

// Kanal A (XL1): DIR1_XL_1, DIR2_XL_1, PWM_BUFF_XL_1
constexpr int PIN_AIN1 = 2;   // contoh GPIO2
constexpr int PIN_AIN2 = 3;   // contoh GPIO3
constexpr int PIN_PWMA = 4;   // contoh GPIO4 (masuk 74HCT14 -> PWM_BUFF_XL_1)

// Kanal B (XL2): DIR1_XL_2, DIR2_XL_2, PWM_BUFF_XL_2
constexpr int PIN_BIN1 = 5;   // contoh GPIO5
constexpr int PIN_BIN2 = 6;   // contoh GPIO6
constexpr int PIN_PWMB = 7;   // contoh GPIO7 (masuk 74HCT14 -> PWM_BUFF_XL_2)

// LEDC channel index (0..7 di ESP32-C3)
constexpr int CH_PWMA = 0;
constexpr int CH_PWMB = 1;

DualTB6612::ChannelPins chA{PIN_AIN1, PIN_AIN2, PIN_PWMA, CH_PWMA};
DualTB6612::ChannelPins chB{PIN_BIN1, PIN_BIN2, PIN_PWMB, CH_PWMB};

// 20kHz, 10-bit, zeroMode=Brake
DualTB6612 motors(PIN_STBY, chA, chB, 20000, 10, ZeroMode::Brake);

HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;
PocketMaster PocketRadio(crsf);

// ============================================================
// 🧠 Function Prototypes
// ============================================================
void debugPrintChannels();
void updateLED();

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

  // 🔹 [ADDED for Bluetooth logging]
  BT.begin("ESP32-Logger");    // start Bluetooth SPP
  LOG_OUT2 = &BT;              // send all logs to BT too
  LOGI("BOOT", "Bluetooth SPP ready (ESP32-Logger)");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  crsf.begin(crsfSerial);
  LOGI("CRSF", "UART1 @ %d baud (RX=%d TX=%d)", (int)CRSF_BAUDRATE, PIN_RX, PIN_TX);

  // PocketMaster init
  PocketRadio.begin("rc-cal");     // load calibration from NVS
  PocketRadio.setDeadzoneUs(20);   // tune if needed

  // loadCalibration();
  PocketRadio.loadCalibration();

  if (PocketRadio.isCallibrated) {
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
  // crsf.update();
  PocketRadio.update();
  
  //Pocketradio is connected
  if (PocketRadio.isLinkUp()) {
    //system armed
    if (PocketRadio.val(SA) == 1) {
      if (!PocketRadio.isArmed) {
        PocketRadio.isArmed = true;
        LOGI("RC", "SA UP → system armed");
        if (!PocketRadio.isCallibrated) 
          PocketRadio.loadCalibration();
      }
      debugPrintChannels();
      //Throtle used for controlling motor speed 
      motors.setSpeedA(PocketRadio.val(THROTTLE));
      motors.setSpeedB(PocketRadio.val(THROTTLE));
    }
    //enter Callibration mode when SA is down 
    else if (PocketRadio.val(SA) == 3) {
      if (PocketRadio.isArmed)
        LOGI("RC", "SA DOWN → system disarmed, entering calibration mode");
      PocketRadio.isArmed = false;
      PocketRadio.startCalibration();
      if(PocketRadio.isCalibSaved()){
        LOGI("CAL", "ROLL[max:%4u min:%4u] PITCH[max:%4u min:%4u] YAW[max:%4u min:%4u] THR[max:%4u min:%4u] S1[max:%4u min:%4u]",
            PocketRadio.max(ROLL), PocketRadio.min(ROLL),
            PocketRadio.max(PITCH), PocketRadio.min(PITCH),
            PocketRadio.max(YAW), PocketRadio.min(YAW),
            PocketRadio.max(THROTTLE), PocketRadio.min(THROTTLE),
            PocketRadio.max(S1), PocketRadio.min(S1));    
      }
    }
  } 
  else {
    if (PocketRadio.isArmed) {
      LOGW("FAILSAFE", "Receiver lost → disarmed");
      PocketRadio.isArmed = false;
    }
    else{
      LOGI("CRSF", "Receiver disconnected");
    }
    // PocketRadio.throttle.val = 0;
  }

  // updateLED();
  delay(10);
}

void debugPrintChannels() {
#if defined(DEBUG)
  LOGD("CRSF", "R:%4u P:%4u T:%4u Y:%4u | SA:%d SB:%d SC:%d SD:%d SE:%d | S1:%4u",
       PocketRadio.val(ROLL), 
       PocketRadio.val(PITCH), 
       PocketRadio.val(THROTTLE), 
       PocketRadio.val(YAW),
       PocketRadio.val(SA),
       PocketRadio.val(SB),
       PocketRadio.val(SC),
       PocketRadio.val(SD),
       PocketRadio.val(SE),
       PocketRadio.val(S1)
      );
#endif
}
