#include <Arduino.h>
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include "Logger.h"
#include "PocketMaster.h"
#include "DualTB6612.h"
#include "PFServo.h"

// 🔹 [ADDED for Bluetooth logging]
// #include <BluetoothSerial.h>
// BluetoothSerial BT;

// 🔹 define logger sinks (multi-output)
Print* LOG_OUT1 = &Serial;
// Print* LOG_OUT2 = nullptr;   // set later to &BT in setup()

// ============================================================
// ⚙️ Hardware setup
// ============================================================
#define PIN_RX 20
#define PIN_TX 21
#define PIN_LED 8   // ✅ ESP32-C3 Super Mini onboard LED

// Standby control for TB6612 driver
constexpr int PIN_STBY = -1;   // STBY_DRIVER_XL -> enable pin for H-bridge

// Channel A (XL1): DIR1_XL_1, DIR2_XL_1, PWM_BUFF_XL_1
constexpr int PIN_AIN1 = 1;   // Direction A1
constexpr int PIN_AIN2 = 0;   // Direction A2
constexpr int PIN_PWMA = 7;   // PWM output for motor A (goes through 74HCT14 buffer)

// Channel B (XL2): DIR1_XL_2, DIR2_XL_2, PWM_BUFF_XL_2
constexpr int PIN_BIN1 = 2;   // Direction B1
constexpr int PIN_BIN2 = 3;   // Direction B2
constexpr int PIN_PWMB = 6;   // PWM output for motor B (goes through 74HCT14 buffer)

// Servo PF output (through 74HCT14 buffer)
constexpr int PIN_SERVO = 10;  // PWM output for LEGO Power Functions servo

// LEDC PWM channel indexes (0–7 on ESP32)
constexpr int CH_PWMA  = 0;    // PWM channel for motor A
constexpr int CH_PWMB  = 1;    // PWM channel for motor B
constexpr int CH_SERVO = 2;    // PWM channel for servo


DualTB6612::ChannelPins chA{PIN_AIN1, PIN_AIN2, PIN_PWMA, CH_PWMA};
DualTB6612::ChannelPins chB{PIN_BIN1, PIN_BIN2, PIN_PWMB, CH_PWMB};

// ============================================================
// Class initialization
// ============================================================
DualTB6612 motors(PIN_STBY, chA, chB, 20000, 10, ZeroMode::Brake); // 20kHz, 10-bit, zeroMode=Brake
PFServo steering(PIN_SERVO, CH_SERVO); // channel 2
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
  
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 1500)) {delay(10);}
  LOGI("BOOT", "Serial ready. Starting CRSF...");

  // 🔹 [ADDED for Bluetooth logging]
  // BT.begin("ESP32-Logger");    // start Bluetooth SPP
  // LOG_OUT2 = &BT;              // send all logs to BT too
  LOGI("BOOT", "Bluetooth SPP ready (ESP32-Logger)");

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

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
      digitalWrite(PIN_LED, HIGH); delay(150);
      digitalWrite(PIN_LED, LOW); delay(150);
    }
  }

  steering.begin();
  steering.setInput(0);

  motors.begin();            // configure pins, LEDC, and leave standby
  motors.setZeroMode(ZeroMode::Brake); // 0-speed = active brake

  // Soft start to reduce inrush
  motors.rampToA(+400, 3);   // A to +40% with 3 ms per step
  motors.rampToB(+400, 3);
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
      // roll used for controlling
      steering.setInput(PocketRadio.val(ROLL));
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
    // Stop (brake)
    motors.setSpeedA(0);
    motors.setSpeedB(0);
    delay(800);

    // Coast, then standby
    motors.coastA();
    motors.coastB();
    delay(500);
    // motors.standby(true);      // low-power mode (H-bridge off)
    delay(500);
    // motors.standby(false);     // wake
  }
  // updateLED();
  while (crsfSerial.available()) {
  int b = crsfSerial.read();
  Serial.printf("[CRSF RX] %02X ", b);
  }
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
