#include "PFServo.h"

PFServo::PFServo(int pinServo, int ledcChannel,
                 uint32_t freq, uint8_t resolutionBits,
                 int minUs, int maxUs)
: _pin(pinServo), _ch(ledcChannel), _freq(freq),
  _resBits(resolutionBits), _minUs(minUs), _maxUs(maxUs), _angle(90)
{}

void PFServo::begin() {
  // Arduino ESP32 3.x: attach the servo GPIO to its explicit channel.
  // The servo needs a separate LEDC channel because it runs at 50 Hz,
  // while the motor PWM channels run at 20 kHz.
  ledcAttachChannel(_pin, _freq, _resBits, _ch);
  setAngle(90);
}

void PFServo::setAngle(int angleDeg) {
  // Clamp the command to the configured mechanical range of 0..180 degrees.
  if (angleDeg < 0) angleDeg = 0;
  if (angleDeg > 180) angleDeg = 180;
  _angle = angleDeg;
  int us = map(angleDeg, 0, 180, _minUs, _maxUs);
  _applyUs(us);
}

void PFServo::setInput(int value) {
  // RC input convention: -1000=full left, 0=center, +1000=full right.
  if (value > 1000) value = 1000;
  if (value < -1000) value = -1000;
  int angleDeg = map(value, -1000, 1000, 0, 180);
  setAngle(angleDeg);
}

void PFServo::_applyUs(int us) {
  // 50 Hz means a 20 ms period.
  // Example: a 1500 us pulse over a 20000 us period equals 7.5% duty.
  float dutyCycle = (float)us / 20000.0f;
  uint32_t duty = (uint32_t)(dutyCycle * ((1UL << _resBits) - 1));
  ledcWriteChannel(_ch, duty);
}
