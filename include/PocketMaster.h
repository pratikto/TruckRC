#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <AlfredoCRSF.h>

  // RadioMaster Pocket Mode 2 channel indexes as reported by CRSF.
  static constexpr uint8_t ROLL     = 0;
  static constexpr uint8_t PITCH    = 1;
  static constexpr uint8_t THROTTLE = 2;
  static constexpr uint8_t YAW      = 3;
  static constexpr uint8_t SA       = 4;
  static constexpr uint8_t SD       = 5;
  static constexpr uint8_t SB       = 6;
  static constexpr uint8_t SC       = 7;
  static constexpr uint8_t SE       = 8;
  static constexpr uint8_t S1       = 9; 

class PocketMaster {
public:
  // Open the calibration namespace and load saved channel limits from NVS.
  explicit PocketMaster(AlfredoCRSF& crsf) : crsf_(crsf) {}
  bool begin(const char* nvs_ns = "rc-cal");
  void update();

  bool isLinkUp() const { return link_up_; }
  bool isArmed;
  
  // Channel accessors. raw() returns CRSF pulse values; val() returns the
  // normalized value produced by scaleCentered() or scaleLinear().
  uint16_t raw(uint8_t a) const;
  uint16_t val(uint8_t a) const;
  uint16_t max(uint8_t a) const;
  uint16_t min(uint8_t a) const;

  // Return CRSF uplink link quality as reported by the receiver statistics.
  uint16_t linkQuality() const { return lq_; }

// ============================================================
// Persistent channel calibration stored in ESP32 NVS
// ============================================================
  void startCalibration();
  void saveCalibration(bool save = true);
  bool loadCalibration();
  void resetCalibration();
  
// ============================================================
// Calibration status and configuration
// ============================================================
  bool isCalibSaved() {return calibSaved;}
  bool isCallibrated;
  bool MaxMinChanged = false;

  void setDeadzoneUs(uint16_t dz) { deadzone_us_ = dz; }

private:
// ============================================================
// Internal RadioMaster channel representations
// ============================================================
  // Calibration is saved after inputs stop changing for this duration.
  unsigned long lastCalibChange = 0;
  const unsigned long CALIB_SAVE_DELAY = 3000;
  bool calibSaved = false;

  static inline uint16_t readLinkQuality(AlfredoCRSF& crsf) {
    if (const crsfLinkStatistics_t* st = crsf.getLinkStatistics())
        return (uint16_t)st->uplink_Link_quality;
    return 0;
  }
  
  struct Analog {
    uint16_t raw = 1500, 
    min = 1500, 
    max = 1500, 
    val = 500; // Normalized value: 0..1000, with 500 as center for stick axes.
  };

  struct Button {
    uint16_t raw = 1500;
    uint16_t val = 0; // 0=unknown, 1=up, 2=middle, 3=down.
    bool up = false;
    bool mid = false;
    bool down = false;
    bool unknown = true;
  };

// ============================================================
// Scaling and decoding helpers
// ============================================================
  // Clamp and integer-map helpers avoid floating-point channel conversion.
  static inline uint16_t clampi(uint16_t v, uint16_t lo, uint16_t hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
  }
  static inline uint16_t imap(uint16_t x, uint16_t inMin, uint16_t inMax,
                              uint16_t outMin, uint16_t outMax) {
    if (inMax <= inMin) return outMin;
    return outMin + (uint32_t)(x - inMin) * (outMax - outMin) / (inMax - inMin);
  }
  uint16_t assignMax(uint16_t a, uint16_t b) { return (b > a) ? b : a; }
  uint16_t assignMin(uint16_t a, uint16_t b) { return (b < a) ? b : a; }
  
  void updateMinMax(Analog& axis);

  // Decode a three-position switch from its raw CRSF pulse value.
  void decodeSwitch (Button &v);

  // Centered scaling is used for sticks; linear scaling is used for throttle
  // and the S1 control. Both produce values in the range 0..1000.
  void scaleCentered(Analog &axis) const;
  void scaleLinear(Analog &axis) const;
  AlfredoCRSF& crsf_;
  Preferences  nvs_;
  const char*  nvs_ns_ = "rc-cal";

  Analog roll_, pitch_, throttle_, yaw_, s1_;
  Button sa_, sb_, sc_, sd_, se_;
  uint16_t lq_ = 0;
  bool     link_up_ = false;
  bool     cal_running_ = false;
  uint16_t deadzone_us_ = 20;
};
