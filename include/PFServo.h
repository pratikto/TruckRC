#pragma once
#include <Arduino.h>

/**
 * PFServo class
 * Drives a LEGO Power Functions servo through a 5 V buffered PWM output
 * provided by the 74HCT14. Compatible with ESP32/ESP32-C3 LEDC PWM.
 *
 * - Accepts either an angle of 0..180 degrees or normalized RC input
 *   in the range -1000..+1000.
 * - Default frequency: 50 Hz (standard servo timing).
 * - Default resolution: 16-bit (duty range 0..65535).
 */

class PFServo {
public:
  PFServo(int pinServo, int ledcChannel = 0,
          uint32_t freq = 50, uint8_t resolutionBits = 16,
          int minUs = 1000, int maxUs = 2000);

  void begin();
  
  // Set the servo position in degrees (0..180).
  void setAngle(int angleDeg);

  // Set the servo position from normalized RC input.
  // -1000=full left, 0=center, +1000=full right.
  void setInput(int value);

  // Return the last commanded angle in degrees.
  int getAngle() const { return _angle; }

private:
  int _pin, _ch;
  uint32_t _freq;
  uint8_t _resBits;
  int _minUs, _maxUs;
  int _angle;

  void _applyUs(int us);
};
