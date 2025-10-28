#pragma once
#include <Arduino.h>

/**
 * PFServo class
 * Kontrol LEGO Power Functions Servo melalui output PWM buffered 5V (74HCT14)
 * Kompatibel ESP32/ESP32-C3 (LEDC PWM)
 *
 * - Input sudut 0–180° atau input analog -1000..+1000
 * - Frekuensi default 50 Hz (servo standar)
 * - Resolusi default 16-bit (65535)
 */

class PFServo {
public:
  PFServo(int pinServo, int ledcChannel = 0,
          uint32_t freq = 50, uint8_t resolutionBits = 16,
          int minUs = 1000, int maxUs = 2000);

  void begin();
  
  // Set posisi servo dalam derajat (0–180)
  void setAngle(int angleDeg);

  // Set posisi dari input analog -1000..+1000
  // -1000 = full kiri, 0 = tengah, +1000 = full kanan
  void setInput(int value);

  // Dapatkan posisi terakhir
  int getAngle() const { return _angle; }

private:
  int _pin, _ch;
  uint32_t _freq;
  uint8_t _resBits;
  int _minUs, _maxUs;
  int _angle;

  void _applyUs(int us);
};
