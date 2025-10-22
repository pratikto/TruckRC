#include "PocketMaster.h"

bool PocketMaster::begin(const char* nvs_ns) {
  nvs_ns_ = nvs_ns ? nvs_ns : "rc-cal";
  return loadCalibration();
}

void PocketMaster::update() {
  crsf_.update();
  link_up_ = crsf_.isLinkUp();

  // read raw channels (1-based)
  roll_.raw     = crsf_.getChannel(CH_ROLL);
  pitch_.raw    = crsf_.getChannel(CH_PITCH);
  throttle_.raw = crsf_.getChannel(CH_THROTTLE);
  yaw_.raw      = crsf_.getChannel(CH_YAW);
  s1_.raw       = crsf_.getChannel(CH_S1);

  uint16_t c_sa = crsf_.getChannel(CH_SA);
  uint16_t c_sb = crsf_.getChannel(CH_SB);
  uint16_t c_sc = crsf_.getChannel(CH_SC);
  uint16_t c_sd = crsf_.getChannel(CH_SD);
  uint16_t c_se = crsf_.getChannel(CH_SE);

  // switches
  sa_ = (c_sa > 1800) ? Sw2::Up : (c_sa < 1200 ? Sw2::Down : Sw2::Unknown);
  sd_ = (c_sd > 1800) ? Sw2::Up : (c_sd < 1200 ? Sw2::Down : Sw2::Unknown);
  se_ = (c_se > 1800) ? Sw2::Up : (c_se < 1200 ? Sw2::Down : Sw2::Unknown);

  auto dec3 = [](uint16_t v)->Sw3 {
    if (v > 1800) return Sw3::Up;
    if (v < 1200) return Sw3::Down;
    if (v >= 1400 && v <= 1600) return Sw3::Mid;
    return Sw3::Unknown;
  };
  sb_ = dec3(c_sb);
  sc_ = dec3(c_sc);

  // link quality
  if (const crsfLinkStatistics_t* st = crsf_.getLinkStatistics())
    lq_ = st->uplink_Link_quality;
  else
    lq_ = 0;

  if (cal_running_) stepCalibration();

  // scaling → 0..1000
  roll_.val     = scaleCentered(roll_.raw,     roll_.min,     roll_.max);
  pitch_.val    = scaleCentered(pitch_.raw,    pitch_.min,    pitch_.max);
  yaw_.val      = scaleCentered(yaw_.raw,      yaw_.min,      yaw_.max);
  throttle_.val = scaleLinear  (throttle_.raw, throttle_.min, throttle_.max);
  s1_.val       = scaleLinear  (s1_.raw,       s1_.min,       s1_.max);

  if (!link_up_) throttle_.val = 0; // basic failsafe suggestion
}

uint16_t PocketMaster::scaleCentered(uint16_t raw, uint16_t min, uint16_t max) const {
  if (max <= min) return 500;
  uint16_t mid = (min + max) / 2;
  raw = clampu(raw, min, max);
  if (raw >= mid - deadzone_us_ && raw <= mid + deadzone_us_) return 500;
  return (raw < mid)
    ? imap(raw, min, (uint16_t)(mid - deadzone_us_), 0,   499)
    : imap(raw, (uint16_t)(mid + deadzone_us_), max, 501, 1000);
}

uint16_t PocketMaster::scaleLinear(uint16_t raw, uint16_t min, uint16_t max) const {
  if (max <= min) return 0;
  raw = clampu(raw, min, max);
  return imap(raw, min, max, 0, 1000);
}

uint16_t PocketMaster::raw(Axis a) const {
  switch (a) {
    case Axis::Roll:     return roll_.raw;
    case Axis::Pitch:    return pitch_.raw;
    case Axis::Throttle: return throttle_.raw;
    case Axis::Yaw:      return yaw_.raw;
    case Axis::S1:       return s1_.raw;
  }
  return 1500;
}

uint16_t PocketMaster::val(Axis a) const {
  switch (a) {
    case Axis::Roll:     return roll_.val;
    case Axis::Pitch:    return pitch_.val;
    case Axis::Throttle: return throttle_.val;
    case Axis::Yaw:      return yaw_.val;
    case Axis::S1:       return s1_.val;
  }
  return 500;
}

void PocketMaster::startCalibration() { cal_running_ = true; }

void PocketMaster::stepCalibration() {
  auto upd = [](uint16_t& mn, uint16_t& mx, uint16_t v){
    if (v < mn) mn = v; if (v > mx) mx = v;
  };
  upd(roll_.min,     roll_.max,     roll_.raw);
  upd(pitch_.min,    pitch_.max,    pitch_.raw);
  upd(yaw_.min,      yaw_.max,      yaw_.raw);
  upd(throttle_.min, throttle_.max, throttle_.raw);
  upd(s1_.min,       s1_.max,       s1_.raw);
}

void PocketMaster::endCalibration(bool save) {
  cal_running_ = false;
  if (!save) return;
  nvs_.begin(nvs_ns_, false);
  nvs_.putUShort("roll_min", roll_.min);     nvs_.putUShort("roll_max", roll_.max);
  nvs_.putUShort("pitch_min", pitch_.min);   nvs_.putUShort("pitch_max", pitch_.max);
  nvs_.putUShort("yaw_min", yaw_.min);       nvs_.putUShort("yaw_max", yaw_.max);
  nvs_.putUShort("thr_min", throttle_.min);  nvs_.putUShort("thr_max", throttle_.max);
  nvs_.putUShort("s1_min", s1_.min);         nvs_.putUShort("s1_max", s1_.max);
  nvs_.end();
}

bool PocketMaster::loadCalibration() {
  nvs_.begin(nvs_ns_, true);
  roll_.min     = nvs_.getUShort("roll_min", 1500);
  roll_.max     = nvs_.getUShort("roll_max", 1500);
  pitch_.min    = nvs_.getUShort("pitch_min", 1500);
  pitch_.max    = nvs_.getUShort("pitch_max", 1500);
  yaw_.min      = nvs_.getUShort("yaw_min", 1500);
  yaw_.max      = nvs_.getUShort("yaw_max", 1500);
  throttle_.min = nvs_.getUShort("thr_min", 1500);
  throttle_.max = nvs_.getUShort("thr_max", 1500);
  s1_.min       = nvs_.getUShort("s1_min", 1500);
  s1_.max       = nvs_.getUShort("s1_max", 1500);
  nvs_.end();
  return true;
}

void PocketMaster::resetCalibration() {
  roll_.min = pitch_.min = yaw_.min = throttle_.min = s1_.min = 1500;
  roll_.max = pitch_.max = yaw_.max = throttle_.max = s1_.max = 1500;
  nvs_.begin(nvs_ns_, false); nvs_.clear(); nvs_.end();
}
