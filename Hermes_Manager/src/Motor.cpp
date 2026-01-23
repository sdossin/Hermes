#include <Arduino.h>
#include <math.h>
#include "Motor.h"

Motor::Motor(const Config& config)
: _config(config)
, _encoder(MagneticSensorI2C::AS5600())
, _motor(config.motor.pole_pairs)
, _driver(config.motor.pins.pwm_u, config.motor.pins.pwm_v, config.motor.pins.pwm_w)
{
    _state.output_enabled = false;
    _state.sensor_ok = false;
};

bool Motor::init() {
    if (_config.encoder.bus == nullptr) return false;
    if(_config.motor.pole_pairs <= 0) return false;

    pinMode(_config.motor.pins.en, OUTPUT);
    disable();

    _encoder.init(_config.encoder.bus);
    _config.encoder.bus->setClock(Const::Encoder::I2C_CLOCK_HZ);

    _state.sensor_ok = _verifyEncoder();
    if (!_state.sensor_ok) {
        return false;
    }

    _driver.voltage_power_supply = _config.motor.supply_voltage;
    _driver.init();

    _motor.linkDriver(&_driver);
    _motor.linkSensor(&_encoder);

    _motor.controller = MotionControlType::angle;

    const float vlim = (_config.tuning.voltage_limit > 0.0f) ? _config.tuning.voltage_limit : Const::Motor::VOLTAGE_LIMIT;

    _motor.voltage_limit = vlim;

    if (_config.tuning.velocity_limit > 0.0f) {
        _motor.velocity_limit = _config.tuning.velocity_limit;
    } else {
        _motor.velocity_limit = Const::Motor::VELOCITY_LIMIT;
    }

    _motor.init();

    _hasInit = true;
    updateState();
    return true;
};

bool Motor::calibrate() {
    if(!_hasInit) return false;

    _state.sensor_ok = _verifyEncoder();
    if(!_state.sensor_ok) {
        return false;
    }

    const float vlim = (_config.tuning.voltage_limit > 0.0f) ? _config.tuning.voltage_limit : Const::Motor::VOLTAGE_LIMIT;
    _motor.voltage_limit = vlim;

    enable();
    _motor.initFOC();
    disable();

    _hasCalibrated = true;
    updateState();
    return true;
}

void Motor::enable() {
    _writeEnable(true);
    _outputEnabled = true;
    _state.output_enabled = true;
}

void Motor::disable() {
    _writeEnable(false);
    _outputEnabled = false;
    _state.output_enabled = false;
}

void Motor::execute() {
    _motor.loopFOC();
}

void Motor::setCommand(const MotorCommand& cmd) {
    _command = _clampCommand(cmd);
}

void Motor::applyControl() {
    if (!_outputEnabled) return;

    switch(_command.mode){
        case ControlMode::Angle:
            _motor.controller = MotionControlType::angle;
            _motor.move(_command.value);
            break;
        case ControlMode::Velocity:
            _motor.controller = MotionControlType::velocity;
            _motor.move(_command.value);
            break;
        case ControlMode::Torque:
            _motor.controller = MotionControlType::torque;
            _motor.move(_command.value);
            break;
    }
}

void Motor::updateState() {
  _state.angle_rad = _motor.shaftAngle();
  _state.velocity_rad_per_sec = _motor.shaftVelocity();
  _state.output_enabled = _outputEnabled;

  if (!_state.sensor_ok) {
    _state.sensor_ok = _verifyEncoder();
  }
}

void Motor::setTuning(const Tuning& t) {
    _config.tuning = t;

    if (_hasInit) {
        if (_config.tuning.voltage_limit > 0.0f) {
        _motor.voltage_limit = _config.tuning.voltage_limit;
        }
        if (_config.tuning.velocity_limit > 0.0f) {
        _motor.velocity_limit = _config.tuning.velocity_limit;
        }
    }
}

void Motor::setConstrains(const Constraints& c) {
    _config.limits = c;
}

MotorCommand Motor::_clampCommand(const MotorCommand& cmd) const {
    MotorCommand out = cmd;

    if (out.mode == ControlMode::Angle) {
        if (out.value < _config.limits.min_angle_rad) out.value = _config.limits.min_angle_rad;
        if (out.value > _config.limits.max_angle_rad) out.value = _config.limits.max_angle_rad;
    }

    if (out.mode == ControlMode::Velocity && _config.limits.max_velocity_rad_per_s > 0.0f) {
        const float v = _config.limits.max_velocity_rad_per_s;
        if (out.value < -v) out.value = -v;
        if (out.value >  v) out.value =  v;
    }

    return out;
}

void Motor::_writeEnable(bool enable) {
    const bool active_high = _config.motor.enable_active_high;
    const uint8_t pin = _config.motor.pins.en;

    if (active_high) {
        digitalWrite(pin, enable ? HIGH : LOW);
    } else {
        digitalWrite(pin, enable ? LOW : HIGH);
    }
}

bool Motor::_verifyEncoder() {
    _encoder.update();
    const float a = _encoder.getAngle();
    return isfinite(a);
}