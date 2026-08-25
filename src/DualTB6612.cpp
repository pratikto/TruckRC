#include "DualTB6612.h"
#include <esp_arduino_version.h>

// Arduino ESP32 changed the LEDC API in core 3.x. These small compatibility
// helpers keep the motor behavior identical on both core 2.x and core 3.x.
static inline void writePwmChannel(uint8_t channel, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteChannel(channel, duty);
#else
  ledcWrite(channel, duty);
#endif
}


void DualTB6612::begin() {
  // STBY is optional. A value of -1 means hardware already pulls STBY high,
  // so the ESP32 does not need to control it.
  if (_stby >= 0) {
    pinMode(_stby, OUTPUT);
    digitalWrite(_stby, HIGH);
  }

  pinMode(_chA.in1, OUTPUT);
  pinMode(_chA.in2, OUTPUT);
  pinMode(_chB.in1, OUTPUT);
  pinMode(_chB.in2, OUTPUT);

  cfgPwm(_chA);
  cfgPwm(_chB);

  // A 10-bit resolution produces a maximum duty value of (2^10)-1 = 1023.
  _maxDuty = (1 << _bits) - 1;
  // Start in a safe state with both channels actively braked.
  brakeA();
  brakeB();
}

void DualTB6612::cfgPwm(const ChannelPins& ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  // Core 3.x combines timer setup and pin attachment in one call.
  ledcAttachChannel(ch.pwm, _freq, _bits, ch.pwmChan);
#else
  // Core 2.x uses the original setup-then-attach API.
  ledcSetup(ch.pwmChan, _freq, _bits);
  ledcAttachPin(ch.pwm, ch.pwmChan);
#endif
  writePwmChannel(ch.pwmChan, 0);
}

void DualTB6612::apply(int speed, const ChannelPins& ch) {
  // Map the normalized -1000..+1000 speed command to LEDC duty 0..1023.
  speed = clampSpeed(speed);
  const int duty = map(abs(speed), 0, 1000, 0, _maxDuty);

  if (speed > 0) {
    // Forward polarity: IN1=HIGH and IN2=LOW.
    digitalWrite(ch.in1, HIGH);
    digitalWrite(ch.in2, LOW);
    writePwmChannel(ch.pwmChan, duty);
  } else if (speed < 0) {
    // Reverse the input polarity for backward rotation.
    digitalWrite(ch.in1, LOW);
    digitalWrite(ch.in2, HIGH);
    writePwmChannel(ch.pwmChan, duty);
  } else if (_zeroMode == ZeroMode::Brake) {
    doBrake(ch);
  } else {
    doCoast(ch);
  }
}

void DualTB6612::doBrake(const ChannelPins& ch) {
  // Driving both inputs HIGH with zero PWM applies active/short braking.
  digitalWrite(ch.in1, HIGH);
  digitalWrite(ch.in2, HIGH);
  writePwmChannel(ch.pwmChan, 0);
}

void DualTB6612::doCoast(const ChannelPins& ch) {
  // Driving both inputs LOW releases the H-bridge so the motor can coast.
  digitalWrite(ch.in1, LOW);
  digitalWrite(ch.in2, LOW);
  writePwmChannel(ch.pwmChan, 0);
}

void DualTB6612::setSpeedA(int speed) {
  _spdA = clampSpeed(speed);
  apply(_spdA, _chA);
}

void DualTB6612::setSpeedB(int speed) {
  _spdB = clampSpeed(speed);
  apply(_spdB, _chB);
}

// Convert percentage helpers to the driver's internal 0..1000 scale.
void DualTB6612::forwardA(uint8_t pct) { setSpeedA(constrain(pct, 0, 100) * 10); }
void DualTB6612::reverseA(uint8_t pct) { setSpeedA(-constrain(pct, 0, 100) * 10); }
void DualTB6612::forwardB(uint8_t pct) { setSpeedB(constrain(pct, 0, 100) * 10); }
void DualTB6612::reverseB(uint8_t pct) { setSpeedB(-constrain(pct, 0, 100) * 10); }

void DualTB6612::brakeA() { _spdA = 0; doBrake(_chA); }
void DualTB6612::brakeB() { _spdB = 0; doBrake(_chB); }
void DualTB6612::coastA() { _spdA = 0; doCoast(_chA); }
void DualTB6612::coastB() { _spdB = 0; doCoast(_chB); }

void DualTB6612::standby(bool enable) {
  if (_stby < 0) return;

  digitalWrite(_stby, enable ? LOW : HIGH);
  if (enable) {
    writePwmChannel(_chA.pwmChan, 0);
    writePwmChannel(_chB.pwmChan, 0);
  }
}

void DualTB6612::rampToA(int targetSpeed, uint16_t stepDelayMs) {
  // This blocking helper is suitable for bench tests. Do not call it repeatedly
  // from the main loop because its delays would reduce CRSF responsiveness.
  targetSpeed = clampSpeed(targetSpeed);
  int speed = _spdA;
  const int step = (targetSpeed >= speed) ? 1 : -1;

  for (; speed != targetSpeed; speed += step) {
    setSpeedA(speed);
    delay(stepDelayMs);
  }
  setSpeedA(targetSpeed);
}

void DualTB6612::rampToB(int targetSpeed, uint16_t stepDelayMs) {
  targetSpeed = clampSpeed(targetSpeed);
  int speed = _spdB;
  const int step = (targetSpeed >= speed) ? 1 : -1;

  for (; speed != targetSpeed; speed += step) {
    setSpeedB(speed);
    delay(stepDelayMs);
  }
  setSpeedB(targetSpeed);
}
