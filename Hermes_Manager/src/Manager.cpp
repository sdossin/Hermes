#include "Manager.h"

Manager::Manager(const Config& config)
: _config(config) {
  _status.mode = Mode::Idle;
  _status.fault = FaultCode::None;

  // Default commands = Angle 0 for all slots
  for (uint8_t i = 0; i < MAX_MOTORS; i++) {
    _cmd[i].mode = ControlMode::Angle;
    _cmd[i].value = 0.0f;
  }
}

bool Manager::init() {
  _mode = Mode::Idle;
  _armed = false;
  _fault = FaultCode::None;

  bool ok_all = true;

  for (uint8_t i = 0; i < _config.motor_count; i++) {
    if (_config.motors[i] == nullptr) { ok_all = false; continue; }
    ok_all &= _config.motors[i]->init();
  }

  disable();        // enforces Idle policy (outputs disabled)
  refreshStatus();

  return ok_all;
}

void Manager::execute() {
  _executeMotors();
  _supervisorStep();
  _applyOutputs();

  const uint32_t now = millis();
  if (now - _last_status_ms >= 20) {
    _last_status_ms = now;
    refreshStatus();
  }
}

void Manager::_executeMotors() {
  for (uint8_t i = 0; i < _config.motor_count; i++) {
    if (auto* m = _config.motors[i]) m->execute();
  }
}

void Manager::setMode(Mode m) {
  if (_mode == Mode::Independent && m != Mode::Independent) {
    disable();
  }

  if (m == Mode::Calibration) {
    _calibrationState = CalibrationState::Entry;
  }

  _mode = m;
}

void Manager::disable() {
  _armed = false;
  for (uint8_t i = 0; i < _config.motor_count; i++) {
    if (auto* m = _config.motors[i]) m->disable();
  }
}

void Manager::arm(bool on) {
  if (_mode != Mode::Independent) return;
  _armed = on;
  if (!_armed) disable();
}

void Manager::setCommand(uint8_t idx, float rad) {
  if (idx >= _config.motor_count) return;
  _cmd[idx].mode = ControlMode::Angle;
  _cmd[idx].value = rad;
}

void Manager::calibrate() {
  disable();
  setMode(Mode::Calibration);
}

void Manager::triggerFault(FaultCode code) {
  _fault = code;
  _mode = Mode::Fault;
  disable();
}

void Manager::clearFault() {
  if (_mode != Mode::Fault) return;
  _fault = FaultCode::None;
  _mode = Mode::Idle;
  disable();
}

void Manager::_supervisorStep() {
  if (_mode == Mode::Fault) return;

  if (_mode == Mode::Idle) {
    disable();
    return;
  }

  if (_mode == Mode::Calibration) {
    switch (_calibrationState) {

      case CalibrationState::Entry: {
        disable();
        _calibrationState = CalibrationState::SensorVerification;
      } break;

      case CalibrationState::SensorVerification: {
        refreshStatus();

        bool ok = true;
        for (uint8_t i = 0; i < _config.motor_count; i++) {
          auto* m = _config.motors[i];
          if (!m) { ok = false; continue; }
          ok &= m->sensorOk();
        }

        _calibrationState = ok ? CalibrationState::AlignFOC
                               : CalibrationState::Fail;
      } break;

      case CalibrationState::AlignFOC: {
        bool ok = true;
        for (uint8_t i = 0; i < _config.motor_count; i++) {
          auto* m = _config.motors[i];
          if (!m) { ok = false; continue; }
          ok &= m->calibrate();
        }

        _calibrationState = ok ? CalibrationState::Validate
                               : CalibrationState::Fail;
      } break;

      case CalibrationState::Validate:
        _calibrationState = CalibrationState::Done;
        break;

      case CalibrationState::Done:
        setMode(Mode::Idle);
        break;

      case CalibrationState::Fail:
        triggerFault(FaultCode::CalibrationFailed);
        break;
    }
  }
}

void Manager::_applyOutputs() {
  if (_mode == Mode::Independent) {
    if (!_armed) {
      disable();
      return;
    }

    for (uint8_t i = 0; i < _config.motor_count; i++) {
      auto* m = _config.motors[i];
      if (!m) continue;

      m->enable();
      m->setCommand(_cmd[i]);
      m->applyControl();
    }
    return;
  }

  if (_mode == Mode::Idle || _mode == Mode::Fault) {
    disable();
  }
}

void Manager::refreshStatus() {
  _status.mode = _mode;
  _status.fault = _fault;
  _status.armed = _armed;

  // If your ManagerStatus struct only has m0/m1, you have two choices:
  //   A) change it to an array (recommended)
  //   B) only fill the first two for now
  //
  // Recommended: update ManagerStatus to:
  //   MotorState motors[Manager::MAX_MOTORS];
  //
  // For now, here's the "fill first two" fallback:

    _status.motor_count = _config.motor_count;
    for (uint8_t i = 0; i < _config.motor_count; i++) {
        auto* m = _config.motors[i];
        if (!m) continue;
        m->updateState();
        _status.motors[i] = m->state();
    }
}
