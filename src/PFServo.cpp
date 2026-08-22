#include "PFServo.h"

PFServo::PFServo(int pinServo, int ledcChannel,
                 uint32_t freq, uint8_t resolutionBits,
                 int minUs, int maxUs)
: _pin(pinServo), _ch(ledcChannel), _freq(freq),
  _resBits(resolutionBits), _minUs(minUs), _maxUs(maxUs), _angle(90)
{}

void PFServo::begin() {
  // Arduino ESP32 3.x: attach the servo GPIO to its explicit channel.
  // Channel servo harus berbeda dari channel PWM motor karena frekuensinya
  // 50 Hz, sedangkan motor menggunakan 20 kHz.
  ledcAttachChannel(_pin, _freq, _resBits, _ch);
  setAngle(90);
}

void PFServo::setAngle(int angleDeg) {
  // Clamp melindungi servo dari command di luar rentang mekanis 0..180 derajat.
  if (angleDeg < 0) angleDeg = 0;
  if (angleDeg > 180) angleDeg = 180;
  _angle = angleDeg;
  int us = map(angleDeg, 0, 180, _minUs, _maxUs);
  _applyUs(us);
}

void PFServo::setInput(int value) {
  // Interface untuk data RC: -1000=kiri, 0=tengah, +1000=kanan.
  if (value > 1000) value = 1000;
  if (value < -1000) value = -1000;
  int angleDeg = map(value, -1000, 1000, 0, 180);
  setAngle(angleDeg);
}

void PFServo::_applyUs(int us) {
  // 50 Hz means a 20 ms period.
  // Contoh: pulse 1500 us / periode 20000 us = duty 7.5%.
  float dutyCycle = (float)us / 20000.0f;
  uint32_t duty = (uint32_t)(dutyCycle * ((1UL << _resBits) - 1));
  ledcWriteChannel(_ch, duty);
}
