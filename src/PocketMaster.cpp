#include "PocketMaster.h"
#include "Logger.h"

bool PocketMaster::begin(const char* nvs_ns) {
  nvs_ns_ = nvs_ns;
  nvs_.begin(nvs_ns_, true);
  loadCalibration();
  nvs_.end();
  return true;
}

void PocketMaster::update() {
  crsf_.update();
  link_up_ = crsf_.isLinkUp();
  if (!link_up_) return;

  // read channels
  uint16_t ch[16];
  for (int i = 0; i < 16; i++) ch[i] = crsf_.getChannel(i + 1);

  roll_.raw     = ch[CH_ROLL];
  pitch_.raw    = ch[CH_PITCH];
  throttle_.raw = ch[CH_THROTTLE];
  yaw_.raw      = ch[CH_YAW];
  s1_.raw       = ch[CH_S1];

  auto dec2 = [](uint16_t v) -> Sw2 {
    if (v > 1800) return Sw2::Up;
    if (v < 1200) return Sw2::Down;
    return Sw2::Unknown;
  };
  auto dec3 = [](uint16_t v) -> Sw3 {
    if (v > 1800) return Sw3::Up;
    if (v < 1200) return Sw3::Down;
    if (v >= 1400 && v <= 1600) return Sw3::Mid;
    return Sw3::Unknown;
  };

  sa_ = dec2(ch[CH_SA]);
  sb_ = dec3(ch[CH_SB]);
  sc_ = dec3(ch[CH_SC]);
  sd_ = dec2(ch[CH_SD]);
  se_ = dec2(ch[CH_SE]);

  roll_.val     = scaleCentered(roll_.raw, roll_.min, roll_.max);
  pitch_.val    = scaleCentered(pitch_.raw, pitch_.min, pitch_.max);
  yaw_.val      = scaleCentered(yaw_.raw, yaw_.min, yaw_.max);
  throttle_.val = scaleLinear(throttle_.raw, throttle_.min, throttle_.max);
  s1_.val       = scaleLinear(s1_.raw, s1_.min, s1_.max);

  lq_ = readLinkQuality(crsf_);
}

uint16_t PocketMaster::raw(PocketMaster::Axis a) const {
  switch (a) {
    case Axis::Roll: return roll_.raw;
    case Axis::Pitch: return pitch_.raw;
    case Axis::Throttle: return throttle_.raw;
    case Axis::Yaw: return yaw_.raw;
    case Axis::S1: return s1_.raw;
    default: return 1500;
  }
}

uint16_t PocketMaster::val(PocketMaster::Axis a) const {
  switch (a) {
    case Axis::Roll: return roll_.val;
    case Axis::Pitch: return pitch_.val;
    case Axis::Throttle: return throttle_.val;
    case Axis::Yaw: return yaw_.val;
    case Axis::S1: return s1_.val;
    default: return 500;
  }
}

void PocketMaster::startCalibration() {
  cal_running_ = true;
  roll_.min = pitch_.min = throttle_.min = yaw_.min = s1_.min = 2000;
  roll_.max = pitch_.max = throttle_.max = yaw_.max = s1_.max = 1000;
}

void PocketMaster::stepCalibration() {
  if (!cal_running_) return;
  roll_.min     = min(roll_.min, roll_.raw);
  roll_.max     = max(roll_.max, roll_.raw);
  pitch_.min    = min(pitch_.min, pitch_.raw);
  pitch_.max    = max(pitch_.max, pitch_.raw);
  throttle_.min = min(throttle_.min, throttle_.raw);
  throttle_.max = max(throttle_.max, throttle_.raw);
  yaw_.min      = min(yaw_.min, yaw_.raw);
  yaw_.max      = max(yaw_.max, yaw_.raw);
  s1_.min       = min(s1_.min, s1_.raw);
  s1_.max       = max(s1_.max, s1_.raw);
}

void PocketMaster::endCalibration(bool save) {
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
  nvs_.end();
  return true;
}

uint16_t PocketMaster::scaleCentered(uint16_t raw, uint16_t min, uint16_t max) const{
  if (max <= min) return 500;                  // not calibrated → neutral
  uint16_t mid = (uint16_t)((min + max) / 2);
  raw = (uint16_t)clampi((int)raw, (int)min, (int)max);

  // Flat deadzone around mid returns exactly 500
  if ((raw >= mid - deadzone_us_) && (raw <= mid + deadzone_us_)) return 500;

  if (raw < mid - deadzone_us_) {
    // Left side: 0..500
    return (uint16_t)imap((int)raw, (int)min, (int)(mid - deadzone_us_), 0, 500);
  } else {
    // Right side: 500..1000
    return (uint16_t)imap((int)raw, (int)(mid + deadzone_us_), (int)max, 500, 1000);
  }
}

uint16_t PocketMaster::scaleLinear  (uint16_t raw, uint16_t min, uint16_t max) const{

}

