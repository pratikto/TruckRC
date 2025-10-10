#include <Arduino.h>
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include "Logger.h"   // custom logger (LOGE/W/I/D/V macros)

// ============================================================
// ⚙️ Hardware setup
// ============================================================
// XR2 TX → ESP32 RX (GPIO16)
// GND → GND
// Power XR2 with a stable 5V
// TX (GPIO17) is optional, only needed for telemetry back to TX
#define PIN_RX 16
#define PIN_TX 17

HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;

// ============================================================
// 🎮 RC channel mapping (Radiomaster Pocket – Mode 2 default)
// ============================================================
/*
CH1  → Roll (Right stick X-axis)
CH2  → Pitch (Right stick Y-axis)
CH3  → Throttle (Left stick Y-axis)
CH4  → Yaw (Left stick X-axis)
CH5  → SA switch (3-position, top-left)
CH6  → SB switch (3-position, mid-left)
CH7  → SC switch (3-position, mid-right)
CH8  → SD switch (3-position, top-right)
CH9  → SE momentary switch (rear-right)
CH10 → S1 potentiometer (rear-left knob)
CH11–CH16 → Optional / custom-mapped in EdgeTX mixers
*/

// ============================================================
// 📦 Global data structure for RC inputs
// ============================================================

struct analogValue
{
  uint16_t raw = 1500;
  uint16_t max = 1500;
  uint16_t min = 1500;
  uint16_t val = 1500;
};

struct swith3Position
{
  uint16_t raw = 1500;
  bool up;
  bool mid;
  bool down;
};

struct swith2Position
{
  uint16_t raw = 1500;
  bool up;
  bool down;
};

struct RCInput_t {
  uint16_t ch[16];      // all 16 CRSF channel values in µs (1000–2000)
  analogValue roll;     // CH1
  analogValue pitch;    // CH2
  analogValue throttle; // CH3
  analogValue yaw;      // CH4
  swith2Position sa;    // CH5
  swith3Position sb;    // CH6
  swith3Position sc;    // CH7
  swith2Position sd;    // CH8
  swith2Position se;    // CH9 (momentary)
  analogValue s1;       // CH10 (analog knob)
  uint16_t LinkQuality; // Link quality
};

RCInput_t RCInput;
bool armed = false;


// ============================================================
// 🧠 Helper functions declaration
// ============================================================

// Read all CRSF channels and update the global struct
void updateRCInputs();

// Print all channels (only when DEBUG mode is active)
void debugPrintChannels();

// Method to get the link quality from CRSF instance
int getLinkQuality(AlfredoCRSF& crsf);

//get max value for calibration
uint16_t assignMax(uint16_t a, uint16_t b);

//get min value for calibration
uint16_t assignMin(uint16_t a, uint16_t b);

//scalling raw value from max and min value
uint16_t scalling(uint16_t raw, uint16_t max, uint16_t min);

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

  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  crsf.begin(crsfSerial);
  LOGI("CRSF", "UART1 @ %d baud (RX=%d TX=%d)", (int)CRSF_BAUDRATE, PIN_RX, PIN_TX);
}

// ============================================================
// 🔁 Main loop
// ============================================================
void loop() {
  crsf.update();        // process incoming CRSF packets
  updateRCInputs();     // refresh all channel values

  // Check if RX1 is connected
  if (crsf.isLinkUp()) {
    // RX1 is connected
    // SA up (value >1800) → system armed
    if (RCInput.sa.raw > 1800) {
      if(not armed){
        armed = true;
        LOGI("RC", "SA UP → system armed");
      }
      // Print all channels (debug only)
      debugPrintChannels();
    }
    else if (RCInput.sa.raw < 1200){
        LOGI("RC", "SA DOWN → system disarmed, Calibarion Mode");
        armed = false;
        //get max value   
        RCInput.roll.max = assignMax(RCInput.roll.max, RCInput.roll.raw);   
        RCInput.pitch.max = assignMax(RCInput.roll.max, RCInput.roll.raw);
        RCInput.throttle.max = assignMax(RCInput.roll.max, RCInput.roll.raw);     
        RCInput.yaw.max = assignMax(RCInput.roll.max, RCInput.roll.raw);
        RCInput.s1.max = assignMax(RCInput.s1.max, RCInput.s1.raw);     
    
        //get min value   
        RCInput.roll.min = assignMin(RCInput.roll.min, RCInput.roll.raw);   
        RCInput.pitch.min = assignMin(RCInput.roll.min, RCInput.roll.raw);
        RCInput.throttle.min = assignMin(RCInput.roll.min, RCInput.roll.raw);     
        RCInput.yaw.min = assignMin(RCInput.roll.min, RCInput.roll.raw);   
        RCInput.s1.max = assignMax(RCInput.s1.max, RCInput.s1.raw);  
      } 
    } 
  else{
    LOGI("CRSF","Receiver is not connected");
  }

  // delay(50); // throttle loop rate, avoid flooding serial monitor
}

// ============================================================
// 🧠 Helper functions
// ============================================================

// Read all CRSF channels and update the global struct
void updateRCInputs() {
  RCInput.LinkQuality = getLinkQuality(crsf);
  for (int i = 0; i < 16; i++) {
    RCInput.ch[i] = crsf.getChannel(i + 1);
  }

  RCInput.roll.raw     = RCInput.ch[0];
  RCInput.pitch.raw    = RCInput.ch[1];
  RCInput.throttle.raw = RCInput.ch[2];
  RCInput.yaw.raw      = RCInput.ch[3];
  // RCInput.sa.val       = RCInput.ch[4];
  // RCInput.sb.val       = RCInput.ch[5];
  // RCInput.sc.val       = RCInput.ch[6];
  // RCInput.sd.val       = RCInput.ch[7];
  // RCInput.se.val       = RCInput.ch[8];
  RCInput.s1.raw       = RCInput.ch[9];

  RCInput.roll.val = scalling(RCInput.roll.raw, RCInput.roll.max, RCInput.roll.min);
  RCInput.pitch.val = scalling(RCInput.pitch.raw, RCInput.pitch.max, RCInput.pitch.min);
  RCInput.throttle.val = scalling(RCInput.throttle.raw, RCInput.throttle.max, RCInput.throttle.min);
  RCInput.yaw.val = scalling(RCInput.yaw.raw, RCInput.yaw.max, RCInput.yaw.min);
  RCInput.s1.val = scalling(RCInput.s1.raw, RCInput.s1.max, RCInput.s1.min);

}

// Print all channels (only when DEBUG mode is active)
void debugPrintChannels() {
#if defined(DEBUG)
  LOGD("CRSF",
       "roll: %4d pitch: %4d throttle: %4d yaw: %4d sa: %4d sb: %4d sc: %4d sd: %4d se: %4d s1: %4d link: %4d",
       RCInput.roll.val,
       RCInput.pitch.val, 
       RCInput.throttle.val, 
       RCInput.yaw.val,
       RCInput.ch[4], 
       RCInput.ch[5], 
       RCInput.ch[6], 
       RCInput.ch[7],
       RCInput.ch[8], 
       RCInput.s1.val, 
       RCInput.LinkQuality);
#endif
}

// Method to get the link quality from CRSF instance
int getLinkQuality(AlfredoCRSF& crsf) {
  const crsfLinkStatistics_t* stat_ptr = crsf.getLinkStatistics();
  return stat_ptr->uplink_Link_quality;
}

//get max value for calibration
uint16_t assignMax(uint16_t a, uint16_t b){
  if(a >= b)
    return b;
}

//get min value for calibration
uint16_t assignMin(uint16_t a, uint16_t b){
  if(a <= b)
    return b;
}

//scalling raw value from max and min value
uint16_t scalling(uint16_t raw, uint16_t max, uint16_t min){

}