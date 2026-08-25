#pragma once
#include <Arduino.h>

/** Speed sign: + = forward, - = reverse. Range: -1000..+1000 */
enum class ZeroMode : uint8_t { Brake, Coast };

class DualTB6612 {
public:
    struct ChannelPins {
        int in1;
        int in2;
        int pwm;
        int pwmChan;
    };

    DualTB6612(int pinSTBY,
               ChannelPins chA,
               ChannelPins chB,
               uint32_t pwmFreqHz = 20000,
               uint8_t pwmBits    = 10,
               ZeroMode zeroMode  = ZeroMode::Brake)
    : _stby(pinSTBY), _chA(chA), _chB(chB),
      _freq(pwmFreqHz), _bits(pwmBits), _zeroMode(zeroMode) {}

    void begin();

    DualTB6612(ChannelPins chA,
               ChannelPins chB,
               uint32_t pwmFreqHz = 20000,
               uint8_t pwmBits = 10,
               ZeroMode zeroMode = ZeroMode::Brake)
    : _stby(-1), _chA(chA), _chB(chB),
      _freq(pwmFreqHz), _bits(pwmBits), _zeroMode(zeroMode) {}

    // Normalized speed: -1000..+1000 (0 = stop).
    void setSpeedA(int speed);
    void setSpeedB(int speed);

    // Percentage helpers: 0..100%.
    void forwardA(uint8_t pct);
    void reverseA(uint8_t pct);
    void forwardB(uint8_t pct);
    void reverseB(uint8_t pct);

    void brakeA();
    void brakeB();
    void coastA();
    void coastB();

    void standby(bool enable);
    void setZeroMode(ZeroMode m) { _zeroMode = m; }

    // Blocking utility for bench tests. Target uses -1000..+1000.
    void rampToA(int targetSpeed, uint16_t stepDelayMs = 5);
    void rampToB(int targetSpeed, uint16_t stepDelayMs = 5);

    int speedA() const { return _spdA; }
    int speedB() const { return _spdB; }

private:
    int _stby;
    ChannelPins _chA, _chB;
    uint32_t _freq;
    uint8_t _bits;
    ZeroMode _zeroMode;
    int _maxDuty = 1023;
    int _spdA = 0;
    int _spdB = 0;

    static inline int clampSpeed(int v) {
        if (v > 1000) return 1000;
        if (v < -1000) return -1000;
        return v;
    }

    void apply(int speed, const ChannelPins& ch);
    void doBrake(const ChannelPins& ch);
    void doCoast(const ChannelPins& ch);
    void cfgPwm(const ChannelPins& ch);
};
