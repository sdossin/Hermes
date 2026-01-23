#pragma once
#include <Arduino.h>

enum class Mode : uint8_t {
    Idle, 
    Independent, 
    Calibration, 
    Fault
};

enum class ControlMode: uint8_t {
    Angle, 
    Velocity,
    Torque
};

enum class FaultCode : uint8_t {
    None,
    EncoderNotDetected,
    EncoderInvalid,
    DriverFault,
    OverVoltage,
    UnderVoltage,
    CalibrationFailed,
    CommandTimeout,
    Unknown
};

enum class CalibrationState : uint8_t {
    Entry,
    SensorVerification,
    AlignFOC,
    Validate,
    Done,
    Fail
};

struct MotorPins {
    uint8_t pwm_u;
    uint8_t pwm_v;
    uint8_t pwm_w;
    uint8_t en;
};

struct MotorConfig {
    int pole_pairs;
    float supply_voltage;
    bool enable_active_high;

    MotorPins pins;
};

// struct MotorTuning {
//     float voltage_limit;
//     float encoder_offset;
// };

// struct JointConstraints {
//     float min_angle_rad;
//     float max_angle_rad;
//     float max_velocity_rad_per_sec;
// };

struct MotorCommand {
    ControlMode mode = ControlMode::Angle;
    float value = 0.0f;
    float feedforward = 0.0f;
};

struct MotorState {
    float angle_rad = 0.0f;
    float velocity_rad_per_sec = 0.0f;
    bool output_enabled = false;
    bool sensor_ok = false;
};

struct ManagerStatus {
  Mode mode = Mode::Idle;
  FaultCode fault = FaultCode::None;
  bool armed = false;
  uint32_t last_cmd_age_ms = 0;

  MotorState motors[Manager::MAX_MOTORS];
  uint8_t motor_count = 0;
};
