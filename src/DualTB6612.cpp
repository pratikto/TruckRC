#include "DualTB6612.h"

void DualTB6612::begin() {
  // STBY bersifat opsional. Nilai -1 berarti pin STBY sudah ditarik HIGH
  // oleh hardware sehingga tidak perlu dikontrol ESP32.
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

  // Resolusi 10-bit menghasilkan duty maksimum (2^10)-1 = 1023.
  _maxDuty = (1 << _bits) - 1;
  // Kondisi awal wajib aman: kedua channel berhenti dalam mode brake.
  brakeA();
  brakeB();
}

void DualTB6612::cfgPwm(const ChannelPins& ch) {
  // Arduino ESP32 3.x: attach GPIO to an explicit LEDC channel.
  // API lama ledcSetup()/ledcAttachPin() sudah dihapus pada core 3.x.
  ledcAttachChannel(ch.pwm, _freq, _bits, ch.pwmChan);
  ledcWriteChannel(ch.pwmChan, 0);
}

void DualTB6612::apply(int speed, const ChannelPins& ch) {
  // Command ternormalisasi -1000..+1000 dipetakan ke duty LEDC 0..1023.
  speed = clampSpeed(speed);
  const int duty = map(abs(speed), 0, 1000, 0, _maxDuty);

  if (speed > 0) {
    // Polaritas arah maju: IN1=HIGH dan IN2=LOW.
    digitalWrite(ch.in1, HIGH);
    digitalWrite(ch.in2, LOW);
    ledcWriteChannel(ch.pwmChan, duty);
  } else if (speed < 0) {
    // Polaritas dibalik untuk arah mundur.
    digitalWrite(ch.in1, LOW);
    digitalWrite(ch.in2, HIGH);
    ledcWriteChannel(ch.pwmChan, duty);
  } else if (_zeroMode == ZeroMode::Brake) {
    doBrake(ch);
  } else {
    doCoast(ch);
  }
}

void DualTB6612::doBrake(const ChannelPins& ch) {
  // Kedua input HIGH dengan PWM 0 menghasilkan active/short brake.
  digitalWrite(ch.in1, HIGH);
  digitalWrite(ch.in2, HIGH);
  ledcWriteChannel(ch.pwmChan, 0);
}

void DualTB6612::doCoast(const ChannelPins& ch) {
  // Kedua input LOW membuat output H-bridge bebas (motor coasting).
  digitalWrite(ch.in1, LOW);
  digitalWrite(ch.in2, LOW);
  ledcWriteChannel(ch.pwmChan, 0);
}

void DualTB6612::setSpeedA(int speed) {
  _spdA = clampSpeed(speed);
  apply(_spdA, _chA);
}

void DualTB6612::setSpeedB(int speed) {
  _spdB = clampSpeed(speed);
  apply(_spdB, _chB);
}

// Helper persen dikalikan 10 karena API internal memakai skala 0..1000.
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
    ledcWriteChannel(_chA.pwmChan, 0);
    ledcWriteChannel(_chB.pwmChan, 0);
  }
}

void DualTB6612::rampToA(int targetSpeed, uint16_t stepDelayMs) {
  // Utility blocking ini cocok untuk bench test, tetapi jangan dipanggil
  // berulang kali di loop utama karena delay akan memperlambat CRSF.
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
