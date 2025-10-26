#include "PocketMaster.h"
#include "Logger.h"

bool PocketMaster::begin(const char* nvs_ns) {
  nvs_ns_ = nvs_ns;
  nvs_.begin(nvs_ns_, true);
  loadCalibration();
  nvs_.end();
  return true;
}

  void PocketMaster::decodeSwitch(Button &v){//, uint16_t raw) {
    // v.raw = raw;
    if (v.raw > 1800) {
      v.up = true; v.mid = false; v.down = false; v.unknown = false; v.val = 1;
    } else if (v.raw < 1200) {
      v.up = false; v.mid = false; v.down = true; v.unknown = false; v.val = 3;
    } else {
      v.up = false; v.mid = true; v.down = false; v.unknown = false; v.val = 2;
    }
    // return state;
  };

void PocketMaster::update() {
  crsf_.update();
  link_up_ = crsf_.isLinkUp();
  if (!link_up_) return;

  // read channels
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
  //init max and min value (is it needed??)
  roll_.min = pitch_.min = throttle_.min = yaw_.min = s1_.min = 2000;
  roll_.max = pitch_.max = throttle_.max = yaw_.max = s1_.max = 1000;

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
    // LOGI("CAL", "No changes → auto-saving calibration");
    saveCalibration(true);
    calibSaved = true;
  }
}

// void PocketMaster::stepCalibration() {
//   if (!cal_running_) return;
//   roll_.min     = min(roll_.min, roll_.raw);
//   roll_.max     = max(roll_.max, roll_.raw);
//   pitch_.min    = min(pitch_.min, pitch_.raw);
//   pitch_.max    = max(pitch_.max, pitch_.raw);
//   throttle_.min = min(throttle_.min, throttle_.raw);
//   throttle_.max = max(throttle_.max, throttle_.raw);
//   yaw_.min      = min(yaw_.min, yaw_.raw);
//   yaw_.max      = max(yaw_.max, yaw_.raw);
//   s1_.min       = min(s1_.min, s1_.raw);
//   s1_.max       = max(s1_.max, s1_.raw);
// }

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

void PocketMaster::updateMinMax(Analog& axis){//, uint16_t raw) {
  MaxMinChanged = false;
  uint16_t oldMin = axis.min;
  uint16_t oldMax = axis.max;
  if(axis.max < axis.raw){
    axis.max = axis.raw;
  }
  if(axis.min > axis.raw){
    axis.min = axis.raw;
  }
}    

void PocketMaster::scaleCentered(Analog &axis) const{  
  
  uint16_t tempVal = axis.raw;

  if (axis.max <= axis.min){
     axis.val = 500;                  // not calibrated → neutral
  }
  else{
    uint16_t mid = (uint16_t)((axis.min + axis.max) / 2);
    tempVal = (uint16_t)clampi((int)tempVal, (int)axis.min, (int)axis.max);

    // Flat deadzone around mid returns exactly 500
    if ((tempVal >= mid - deadzone_us_) && (tempVal <= mid + deadzone_us_)) 
      axis.val = 500;
    else{
      uint16_t mid = (uint16_t)((axis.min + axis.max) / 2);
      tempVal = (uint16_t)clampi((int)tempVal, (int)axis.min, (int)axis.max);

      // Flat deadzone around mid returns exactly 500
      if ((tempVal >= mid - deadzone_us_) && (tempVal <= mid + deadzone_us_)) axis.val = 500;

      if (tempVal < mid - deadzone_us_) {
        // Left side: 0..500
        axis.val =  (uint16_t)imap((int)tempVal, (int)axis.min, (int)(mid - deadzone_us_), 0, 500);
      } else {
        // Right side: 500..1000
        axis.val = (uint16_t)imap((int)tempVal, (int)(mid + deadzone_us_), (int)axis.max, 500, 1000);
      }      
    }
  }
};

void PocketMaster::scaleLinear  (Analog &axis) const{
  uint16_t tempVal = axis.raw;
  if (axis.max <= axis.min)
    axis.val = 0; // not calibrated
  else
    axis.val =(uint16_t)imap((int)tempVal, (int)axis.max, (int)axis.min, 0, 1000);
}
