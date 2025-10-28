#include "PFServo.h"

PFServo::PFServo(int pinServo, int ledcChannel,
                 uint32_t freq, uint8_t resolutionBits,
                 int minUs, int maxUs)
: _pin(pinServo), _ch(ledcChannel), _freq(freq),
  _resBits(resolutionBits), _minUs(minUs), _maxUs(maxUs), _angle(90)
{}

void PFServo::begin() {
  ledcSetup(_ch, _freq, _resBits);
  ledcAttachPin(_pin, _ch);
  setAngle(90);  // posisi tengah saat mulai
}

void PFServo::setAngle(int angleDeg) {
  if (angleDeg < 0) angleDeg = 0;
  if (angleDeg > 180) angleDeg = 180;
  _angle = angleDeg;
  int us = map(angleDeg, 0, 180, _minUs, _maxUs);
  _applyUs(us);
}

void PFServo::setInput(int value) {
  if (value > 1000) value = 1000;
  if (value < -1000) value = -1000;
  // map ke 0–180
  int angleDeg = map(value, -1000, 1000, 0, 180);
  setAngle(angleDeg);
}

void PFServo::_applyUs(int us) {
  // period 20 ms → 50 Hz
  float dutyCycle = (float)us / 20000.0f;  // us ke rasio
  uint32_t duty = (uint32_t)(dutyCycle * ((1UL << _resBits) - 1));
  ledcWrite(_ch, duty);
}
