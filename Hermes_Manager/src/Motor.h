#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>

#include "Types.h"
#include "Constants.h"

class Motor {
public:

    struct EncoderConfig {
        TwoWire* bus = nullptr;
        uint8_t address = Const::Encoder::AS5600_ADDR;
    };

    struct Tuning {
        float voltage_limit = 0.0f;
        float velocity_limit = 0.0f;
        float encoder_offset = 0.0f;
    };

    struct Constraints {
        float min_angle_rad = 0.0f;
        float max_angle_rad = 0.0f;
        float max_velocity_rad_per_s = 0.0f;
    };

    struct Config {
        MotorConfig motor;
        EncoderConfig encoder;
        Tuning tuning;
        Constraints limits;
    };

    explicit Motor(const Config& cfg);

    bool init();

    bool calibrate();
    
    void enable();
    void disable();

    void execute();

    void setCommand(const MotorCommand& cmd);
    void applyControl();

    void updateState();
    const MotorState& state() const {return _state;}

    float angleRad() const {return _state.angle_rad;}

    void setTuning(const Tuning& t);
    void setConstrains(const Constraints& c);

    bool sensorOk() const {return _state.sensor_ok;}
    bool outputsEnabled() const {return _outputEnabled;}

private:
    MotorCommand _clampCommand(const MotorCommand& cmd) const;
    void _writeEnable(bool enable);
    bool _verifyEncoder();

private:
    Config _config;

    MagneticSensorI2C _encoder;
    BLDCMotor _motor;
    BLDCDriver3PWM _driver;

    MotorCommand _command{};
    MotorState _state{};

    bool _hasInit = false;
    bool _hasCalibrated = false;
    bool _outputEnabled = false;

};