#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <AlfredoCRSF.h>

  // Pocket (Mode 2) channel mapping:
  static constexpr uint8_t ROLL     = 0;
  static constexpr uint8_t PITCH    = 1;
  static constexpr uint8_t THROTTLE = 2;
  static constexpr uint8_t YAW      = 3;
  static constexpr uint8_t SA       = 4;
  static constexpr uint8_t SB       = 5;
  static constexpr uint8_t SC       = 6;
  static constexpr uint8_t SD       = 7;
  static constexpr uint8_t SE       = 8;
  static constexpr uint8_t S1       = 9; 

class PocketMaster {
public:
  //initialization function
  explicit PocketMaster(AlfredoCRSF& crsf) : crsf_(crsf) {}
  bool begin(const char* nvs_ns = "rc-cal");
  void update();

  bool isLinkUp() const { return link_up_; }
  bool isArmed;
  
  //data passing function
  uint16_t raw(uint8_t a) const;
  uint16_t val(uint8_t a) const;
  uint16_t max(uint8_t a) const;
  uint16_t min(uint8_t a) const;

  //check link quality
  uint16_t linkQuality() const { return lq_; }

// ============================================================
// 💾 Persistent calibration (NVS)
// ============================================================
  void startCalibration();
  void saveCalibration(bool save = true);
  bool loadCalibration();
  void resetCalibration();
  
// ============================================================
// ⏱️ Auto-save calibration variables
// ============================================================
  bool isCalibSaved() {return calibSaved;}
  bool isCallibrated;
  bool MaxMinChanged = false;

  void setDeadzoneUs(uint16_t dz) { deadzone_us_ = dz; }

private:
// ============================================================
// 📦 RadioMaster Data Structures
// ============================================================
  // ⏱️ Auto-save calibration variables
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
    val = 500; // val: 0..1000
  };

  struct Button {
    uint16_t raw = 1500;
    uint16_t val = 0;   //0 = Unknown, 1 = up, 2 = mid, 3 = down
    bool up = false;
    bool mid = false;
    bool down = false;
    bool unknown = true;
  };

// ============================================================
// 🔧 Utilities & helpers
// ============================================================
  // helpers
  static inline uint16_t clampi(uint16_t v, uint16_t lo, uint16_t hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
  }
  static inline uint16_t imap(uint16_t x, uint16_t inMin, uint16_t inMax,
                              uint16_t outMin, uint16_t outMax) {
    if (inMax <= inMin) return outMin;
    return outMin + (uint32_t)(x - inMin) * (outMax - outMin) / (inMax - inMin);
  }
    // a.max = assignMax(a.max, raw);
    // a.min = assignMin(a.min, raw);
  uint16_t assignMax(uint16_t a, uint16_t b) { return (b > a) ? b : a; }
  uint16_t assignMin(uint16_t a, uint16_t b) { return (b < a) ? b : a; }
  
  void updateMinMax(Analog& axis);

  // 🧩 Decode switch helpers (with string return)
  void decodeSwitch (Button &v);
  // uint16_t scaleCentered(uint16_t raw, uint16_t min, uint16_t max) const;
  // uint16_t scaleLinear  (uint16_t raw, uint16_t min, uint16_t max) const;
  void scaleCentered(Analog &axis) const;
  void scaleLinear(Analog &axis) const;
  // void scaleCentered(Analog &axis, uint16_t raw) const;
  // void scaleLinear(Analog &axis, uint16_t raw) const;
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
