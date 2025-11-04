#pragma once
#include <Arduino.h>

/** Speed sign: + = forward, - = reverse. Range: -100..+100 */
enum class ZeroMode : uint8_t { Brake, Coast };

class DualTB6612 {
public:
    struct ChannelPins {
        int in1;      // AIN1/BIN1  -> DIR1_XL_n
        int in2;      // AIN2/BIN2  -> DIR2_XL_n
        int pwm;      // PWMA/PWMB  -> PWM_BUFF_XL_n  (melalui 74HCT14)
        int pwmChan;  // LEDC channel index (0..7 di C3)
    };

    //with standby pin connenected to ESP32
    DualTB6612(int pinSTBY,
                ChannelPins chA,
                ChannelPins chB,
                uint32_t pwmFreqHz = 20000,   // 20 kHz aman buat telinga
                uint8_t pwmBits    = 10,      // 10-bit (0..1023)
                ZeroMode zeroMode  = ZeroMode::Brake)
    : _stby(pinSTBY), _chA(chA), _chB(chB),
        _freq(pwmFreqHz), _bits(pwmBits), _zeroMode(zeroMode) {}

    void begin();

    //standby pin is pullup 
    DualTB6612(ChannelPins chA,
               ChannelPins chB,
               uint32_t pwmFreqHz = 20000,
               uint8_t pwmBits = 10,
               ZeroMode zeroMode = ZeroMode::Brake)
    : _stby(-1), _chA(chA), _chB(chB),
      _freq(pwmFreqHz), _bits(pwmBits), _zeroMode(zeroMode) {}

    // speed: -1000..+1000 (0 = stop)
    void setSpeedA(int speed);  
    void setSpeedB(int speed);

    // helper arah/aksi
    void forwardA(uint8_t pct);    // 0..100
    void reverseA(uint8_t pct);
    void forwardB(uint8_t pct);
    void reverseB(uint8_t pct);
    void brakeA();
    void brakeB();
    void coastA();
    void coastB();

    // kontrol global
    void standby(bool enable);     // true=masuk standby (motor off)
    void setZeroMode(ZeroMode m) { _zeroMode = m; }

    // ramp halus (ms per step, step = 1%)
    void rampToA(int targetPct, uint16_t stepDelayMs = 5);
    void rampToB(int targetPct, uint16_t stepDelayMs = 5);

    // baca status
    int  speedA() const { return _spdA; }
    int  speedB() const { return _spdB; }

private:
    int _stby;
    ChannelPins _chA, _chB;
    uint32_t _freq; uint8_t _bits;
    ZeroMode _zeroMode;
    int _maxDuty = 1023;
    int _spdA = 0, _spdB = 0; // -100..+100

    static inline int clampSpeed(int v){
        if(v > 1000) return 1000;
        if(v < -1000) return -1000;
        return v;
    }

    void apply(int speed, const ChannelPins& ch);
    void doBrake(const ChannelPins& ch);
    void doCoast(const ChannelPins& ch);
    void cfgPwm(const ChannelPins& ch);
};
