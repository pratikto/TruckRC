#include "DualTB6612.h"

static inline int clampPct(int v){ if(v>100) return 100; if(v<-100) return -100; return v; }

void DualTB6612::begin() {
  pinMode(_stby, OUTPUT);
  digitalWrite(_stby, HIGH); // keluar standby

  pinMode(_chA.in1, OUTPUT); pinMode(_chA.in2, OUTPUT);
  pinMode(_chB.in1, OUTPUT); pinMode(_chB.in2, OUTPUT);

  // LEDC config per channel
  cfgPwm(_chA);
  cfgPwm(_chB);

  _maxDuty = (1 << _bits) - 1;
  brakeA(); brakeB();
}

void DualTB6612::cfgPwm(const ChannelPins& ch) {
  // Timer dipilih otomatis berdasarkan channel (ESP32-C3: timer 0..3)
  // Biar sederhana, semua channel pakai timer 0.
  ledcSetup(ch.pwmChan, _freq, _bits);
  ledcAttachPin(ch.pwm, ch.pwmChan);
  ledcWrite(ch.pwmChan, 0);
}

void DualTB6612::apply(int speed, const ChannelPins& ch) {
  speed = clampSpeed(speed);
  int duty = map(abs(speed), 0, 1000, 0, _maxDuty);

  if (speed > 0) {
    digitalWrite(ch.in1, HIGH);
    digitalWrite(ch.in2, LOW);
    ledcWrite(ch.pwmChan, duty);
  } else if (speed < 0) {
    digitalWrite(ch.in1, LOW);
    digitalWrite(ch.in2, HIGH);
    ledcWrite(ch.pwmChan, duty);
  } else {
    if (_zeroMode == ZeroMode::Brake) doBrake(ch);
    else doCoast(ch);
  }
}

void DualTB6612::doBrake(const ChannelPins& ch) {
  // fast decay (AIN1=AIN2=HIGH) + PWM duty 0
  digitalWrite(ch.in1, HIGH);
  digitalWrite(ch.in2, HIGH);
  ledcWrite(ch.pwmChan, 0);
}

void DualTB6612::doCoast(const ChannelPins& ch) {
  // slow decay (AIN1=AIN2=LOW) + PWM duty 0
  digitalWrite(ch.in1, LOW);
  digitalWrite(ch.in2, LOW);
  ledcWrite(ch.pwmChan, 0);
}

void DualTB6612::setSpeedA(int speed) { _spdA = clampSpeed(speed); apply(_spdA, _chA); }
void DualTB6612::setSpeedB(int speed) { _spdB = clampSpeed(speed); apply(_spdB, _chB); }

void DualTB6612::forwardA(uint8_t pct){ setSpeedA((int)pct); }
void DualTB6612::reverseA(uint8_t pct){ setSpeedA(-((int)pct)); }
void DualTB6612::forwardB(uint8_t pct){ setSpeedB((int)pct); }
void DualTB6612::reverseB(uint8_t pct){ setSpeedB(-((int)pct)); }

void DualTB6612::brakeA(){ _spdA = 0; doBrake(_chA); }
void DualTB6612::brakeB(){ _spdB = 0; doBrake(_chB); }
void DualTB6612::coastA(){ _spdA = 0; doCoast(_chA); }
void DualTB6612::coastB(){ _spdB = 0; doCoast(_chB); }

void DualTB6612::standby(bool enable){
  digitalWrite(_stby, enable ? LOW : HIGH); // STBY low = sleep
  if (enable) { ledcWrite(_chA.pwmChan, 0); ledcWrite(_chB.pwmChan, 0); }
}

void DualTB6612::rampToA(int targetPct, uint16_t stepDelayMs){
  targetPct = clampPct(targetPct);
  int s = _spdA;
  int step = (targetPct >= s) ? 1 : -1;
  for (; s != targetPct; s += step) { setSpeedA(s); delay(stepDelayMs); }
  setSpeedA(targetPct);
}

void DualTB6612::rampToB(int targetPct, uint16_t stepDelayMs){
  targetPct = clampPct(targetPct);
  int s = _spdB;
  int step = (targetPct >= s) ? 1 : -1;
  for (; s != targetPct; s += step) { setSpeedB(s); delay(stepDelayMs); }
  setSpeedB(targetPct);
}
