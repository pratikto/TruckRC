#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <AlfredoCRSF.h>

class PocketMaster {
public:
  enum class Axis { Roll, Pitch, Throttle, Yaw, S1 };
  enum class Sw2  { Down, Up, Unknown };
  enum class Sw3  { Down, Mid, Up, Unknown };

  explicit PocketMaster(AlfredoCRSF& crsf) : crsf_(crsf) {}

  bool   begin(const char* nvs_ns = "rc-cal");
  void   update();
  bool   isLinkUp() const { return link_up_; }

  uint16_t raw(Axis a) const;
  uint16_t val(Axis a) const;

  Sw2 sa() const { return sa_; }  // CH5
  Sw3 sb() const { return sb_; }  // CH6
  Sw3 sc() const { return sc_; }  // CH7
  Sw2 sd() const { return sd_; }  // CH8
  Sw2 se() const { return se_; }  // CH9

  uint16_t linkQuality() const { return lq_; }

  void startCalibration();
  void stepCalibration();
  void endCalibration(bool save = true);
  bool loadCalibration();
  void resetCalibration();

  void setDeadzoneUs(uint16_t dz) { deadzone_us_ = dz; }

private:
  // Pocket (Mode 2) channel mapping:
  static constexpr uint8_t CH_ROLL     = 1;
  static constexpr uint8_t CH_PITCH    = 2;
  static constexpr uint8_t CH_THROTTLE = 3;
  static constexpr uint8_t CH_YAW      = 4;
  static constexpr uint8_t CH_SA       = 5;
  static constexpr uint8_t CH_SB       = 6;
  static constexpr uint8_t CH_SC       = 7;
  static constexpr uint8_t CH_SD       = 8;
  static constexpr uint8_t CH_SE       = 9;
  static constexpr uint8_t CH_S1       = 10;

  struct Analog {
    uint16_t raw = 1500, min = 1500, max = 1500, val = 500; // val: 0..1000
  };

  // helpers
  static inline uint16_t clampu(uint16_t v, uint16_t lo, uint16_t hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
  }
  static inline uint16_t imap(uint16_t x, uint16_t inMin, uint16_t inMax,
                              uint16_t outMin, uint16_t outMax) {
    if (inMax <= inMin) return outMin;
    return outMin + (uint32_t)(x - inMin) * (outMax - outMin) / (inMax - inMin);
  }
  uint16_t scaleCentered(uint16_t raw, uint16_t min, uint16_t max) const;
  uint16_t scaleLinear  (uint16_t raw, uint16_t min, uint16_t max) const;

  AlfredoCRSF& crsf_;
  Preferences  nvs_;
  const char*  nvs_ns_ = "rc-cal";

  Analog roll_, pitch_, throttle_, yaw_, s1_;
  Sw2 sa_ = Sw2::Unknown, sd_ = Sw2::Unknown, se_ = Sw2::Unknown;
  Sw3 sb_ = Sw3::Unknown, sc_ = Sw3::Unknown;

  uint16_t lq_ = 0;
  bool     link_up_ = false;
  bool     cal_running_ = false;
  uint16_t deadzone_us_ = 20;
};
