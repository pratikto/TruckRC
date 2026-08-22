#include "PocketMaster.h"
#include "Logger.h"

bool PocketMaster::begin(const char* nvs_ns) {
  nvs_ns_ = nvs_ns;
  nvs_.begin(nvs_ns_, true);
  loadCalibration();
  nvs_.end();
  return true;
}

  void PocketMaster::decodeSwitch(Button &v){
    // RadioMaster three-position switches are decoded from CRSF pulse widths.
    if (v.raw > 1800) {
      v.up = true; v.mid = false; v.down = false; v.unknown = false; v.val = 1;
    } else if (v.raw < 1200) {
      v.up = false; v.mid = false; v.down = true; v.unknown = false; v.val = 3;
    } else {
      v.up = false; v.mid = true; v.down = false; v.unknown = false; v.val = 2;
    }
  };

void PocketMaster::update() {
  crsf_.update();
  link_up_ = crsf_.isLinkUp();
  if (!link_up_) return;

  // Read all 16 CRSF channels once so each control uses the same received frame.
  uint16_t ch[16];
  for (int i = 0; i < 16; i++) ch[i] = crsf_.getChannel(i + 1);

  roll_.raw     = ch[ROLL];
  pitch_.raw    = ch[PITCH];
  throttle_.raw = ch[THROTTLE];
  yaw_.raw      = ch[YAW];
  s1_.raw       = ch[S1];
  sa_.raw       = ch[SA];
  sb_.raw       = ch[SB];
  sc_.raw       = ch[SC];
  sd_.raw       = ch[SD];
  se_.raw       = ch[SE];

  scaleCentered(roll_);
  scaleCentered(pitch_);
  scaleCentered(yaw_);
  scaleLinear(throttle_);
  scaleLinear(s1_);

  decodeSwitch(sa_);
  decodeSwitch(sb_);
  decodeSwitch(sc_);
  decodeSwitch(sd_);
  decodeSwitch(se_);

  lq_ = readLinkQuality(crsf_);
}

uint16_t PocketMaster::raw(uint8_t a) const {
  switch (a) {
    case ROLL: return roll_.raw;
    case PITCH: return pitch_.raw;
    case THROTTLE : return throttle_.raw;
    case YAW : return yaw_.raw;
    case S1 : return s1_.raw;
    case SA : return sa_.raw;
    case SB : return sb_.raw;
    case SC : return sc_.raw;
    case SD : return sd_.raw;
    case SE : return se_.raw;
    default: return 1500;
  }
}

uint16_t PocketMaster::val(uint8_t a) const {
  switch (a) {
    case ROLL: return roll_.val;
    case PITCH: return pitch_.val;
    case THROTTLE : return throttle_.val;
    case YAW : return yaw_.val;
    case S1 : return s1_.val;
    case SA : return sa_.val;
    case SB : return sb_.val;
    case SC : return sc_.val;
    case SD : return sd_.val;
    case SE : return se_.val;
    default: return 0;
  }
}

uint16_t PocketMaster::max(uint8_t a) const {
  switch (a) {
    case ROLL: return roll_.max;
    case PITCH: return pitch_.max;
    case THROTTLE : return throttle_.max;
    case YAW : return yaw_.max;
    case S1 : return s1_.max;
    default: return 0;
  }
}

uint16_t PocketMaster::min(uint8_t a) const {
  switch (a) {
    case ROLL: return roll_.min;
    case PITCH: return pitch_.min;
    case THROTTLE : return throttle_.min;
    case YAW : return yaw_.min;
    case S1 : return s1_.min;
    default: return 0;
  }
}

void PocketMaster::startCalibration() {
  cal_running_ = true;

  // Expand each stored minimum/maximum whenever the user moves a control
  // beyond its previously observed calibration range.
  MaxMinChanged = false;
  updateMinMax(roll_);
  updateMinMax(pitch_);
  updateMinMax(throttle_);
  updateMinMax(yaw_);
  updateMinMax(s1_);

  if (MaxMinChanged) {
    lastCalibChange = millis();
    calibSaved = false;
  }

  if (!calibSaved && millis() - lastCalibChange > CALIB_SAVE_DELAY) {
    saveCalibration(true);
    calibSaved = true;
  }
}

void PocketMaster::saveCalibration(bool save) {
  cal_running_ = false;
  if (!save) return;

  nvs_.begin(nvs_ns_, false);
  nvs_.putUShort("roll_min", roll_.min);
  nvs_.putUShort("roll_max", roll_.max);
  nvs_.putUShort("pitch_min", pitch_.min);
  nvs_.putUShort("pitch_max", pitch_.max);
  nvs_.putUShort("thrt_min", throttle_.min);
  nvs_.putUShort("thrt_max", throttle_.max);
  nvs_.putUShort("yaw_min", yaw_.min);
  nvs_.putUShort("yaw_max", yaw_.max);
  nvs_.putUShort("s1_min", s1_.min);
  nvs_.putUShort("s1_max", s1_.max);
  nvs_.end();
}

bool PocketMaster::loadCalibration() {
  nvs_.begin(nvs_ns_, true);
  roll_.min     = nvs_.getUShort("roll_min", 1500);
  roll_.max     = nvs_.getUShort("roll_max", 1500);
  pitch_.min    = nvs_.getUShort("pitch_min", 1500);
  pitch_.max    = nvs_.getUShort("pitch_max", 1500);
  throttle_.min = nvs_.getUShort("thrt_min", 1500);
  throttle_.max = nvs_.getUShort("thrt_max", 1500);
  yaw_.min      = nvs_.getUShort("yaw_min", 1500);
  yaw_.max      = nvs_.getUShort("yaw_max", 1500);
  s1_.min       = nvs_.getUShort("s1_min", 1500);
  s1_.max       = nvs_.getUShort("s1_max", 1500);
  isCallibrated = nvs_.getBool("isCal", false);
  nvs_.end();
  return true;
}

void PocketMaster::updateMinMax(Analog& axis){
  // Remember the old limits so startCalibration() knows whether the quiet
  // auto-save timer must be restarted.
  uint16_t oldMin = axis.min, oldMax = axis.max;
  axis.max = assignMax(axis.max, axis.raw);
  axis.min = assignMin(axis.min, axis.raw);
  if (axis.min != oldMin || axis.max != oldMax) MaxMinChanged = true;
}    

// Centered axes (roll/pitch/yaw) map [min..mid] to [0..500] and
// [mid..max] to [500..1000], while preserving a neutral dead zone.
void PocketMaster::scaleCentered(Analog &axis) const{  
  // Without a valid range, return the safe neutral value.
  if (axis.max == axis.min) { 
    axis.val = 500; 
    return; 
  }

  // Normalize the limits and remember whether the axis is inverted.
  uint16_t lo = axis.min, hi = axis.max;
  bool inverted = false;
  if (lo > hi) { std::swap(lo, hi); inverted = true; }

  // Clamp the raw input to the learned range and calculate its midpoint.
  uint16_t raw = (uint16_t)clampi((int)axis.raw, (int)lo, (int)hi);
  uint16_t mid = (uint16_t)((lo + hi) / 2);

  // Any input inside the configured dead zone is exactly neutral (500).
  if (raw >= mid - deadzone_us_ && raw <= mid + deadzone_us_) {
    axis.val = 500;
    return;
  }

  // Scale the two sides separately so both endpoints retain full resolution.
  if (raw < mid - deadzone_us_) {
    uint16_t v = (uint16_t)imap((int)raw, (int)lo, (int)(mid - deadzone_us_), 0, 500);
    axis.val = inverted ? (uint16_t)(1000 - v) : v;
  } else { // raw is above the upper edge of the dead zone.
    uint16_t v = (uint16_t)imap((int)raw, (int)(mid + deadzone_us_), (int)hi, 500, 1000);
    axis.val = inverted ? (uint16_t)(1000 - v) : v;
  }
}

// Linear controls (throttle and S1) map [min..max] directly to [0..1000].
void PocketMaster::scaleLinear  (Analog &axis) const{
  // A missing/invalid calibration range produces the safe minimum output.
  if (axis.max <= axis.min) axis.val = 0;
  axis.val = (uint16_t)imap((int)axis.raw, (int)axis.min, (int)axis.max, 0, 1000);
}
