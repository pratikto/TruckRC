#include <Arduino.h>
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include "Logger.h"
#include "PocketMaster.h"

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
    }
    //enter Callibration mode when SA is down 
    else if (PocketRadio.val(SA) == 3) {
      if (PocketRadio.isArmed)
        LOGI("RC", "SA DOWN → system disarmed, entering calibration mode");
      PocketRadio.isArmed = false;
      PocketRadio.startCalibration();
      if(PocketRadio.isCalibSaved()){
        LOGI("CAL", "ROLL[min:%4u max:%4u] PITCH[min:%4u max:%4u] YAW[min:%4u max:%4u] THR[min:%4u max:%4u] S1[min:%4u max:%4u]",
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
  // LOGD("CRSF", "R:%4u P:%4u T:%4u Y:%4u | SA:%d SB:%d SC:%d SD:%d SE:%d | S1:%4u",
  //      PocketRadio.raw(ROLL), 
  //      PocketRadio.raw(PITCH), 
  //      PocketRadio.raw(THROTTLE), 
  //      PocketRadio.raw(YAW),
  //      PocketRadio.raw(SA),
  //      PocketRadio.raw(SB),
  //      PocketRadio.raw(SC),
  //      PocketRadio.raw(SD),
  //      PocketRadio.raw(SE),
  //      PocketRadio.raw(S1)
  //     );
#endif
}
