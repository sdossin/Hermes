#pragma once
#include <Arduino.h>
#include "Types.h"
#include "Motor.h"   // whatever your motor class header is

class Manager {
public:
  static constexpr uint8_t MAX_MOTORS = 1;   // pick your upper bound

  struct Config {
    Motor* motors[MAX_MOTORS] = {nullptr};
    uint8_t motor_count = 0;
  };

  explicit Manager(const Config& config);

  bool init();
  void execute();

  void setMode(Mode m);
  void disable();
  void arm(bool on);

  // Generic command setter (replaces setCommand0/1)
  void setCommand(uint8_t idx, float rad);

  void calibrate();
  void triggerFault(FaultCode code);
  void clearFault();

  void refreshStatus();

private:
  void _executeMotors();
  void _supervisorStep();
  void _applyOutputs();

private:
  Config _config;

  Mode _mode = Mode::Idle;
  bool _armed = false;
  FaultCode _fault = FaultCode::None;

  CalibrationState _calibrationState = CalibrationState::Entry;

  // Per-motor commands (same ordering as _config.motors)
  MotorCommand _cmd[MAX_MOTORS];

  // Status
  ManagerStatus _status{};
  uint32_t _last_status_ms = 0;
};
