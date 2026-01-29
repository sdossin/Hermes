// MotorController.hpp
// Motor controller with integrated SimpleFOC closed-loop control
// Supports both low-level PWM control and high-level FOC control modes
#pragma once

#include <cstdint>
#include <SimpleFOC.h>

// Forward declarations
class AS5600Encoder;

class MotorController {
public:
  struct Pins {
    int ch1 = -1;
    int ch2 = -1;
    int ch3 = -1;
    int en  = -1;
  };

  struct Config {
    // PWM settings
    uint32_t pwm_hz = 20000;      // 20 kHz (quiet)
    uint8_t pwm_bits = 12;        // 0..4095

    // ESP32 LEDC channels
    uint8_t ledc_ch1 = 0;
    uint8_t ledc_ch2 = 1;
    uint8_t ledc_ch3 = 2;

    // Enable pin polarity
    bool en_active_high = true;
    
    // SimpleFOC configuration
    bool use_foc = false;              // Enable FOC mode
    float pole_pairs = 7.0f;           // Motor pole pairs
    float phase_resistance = 0.0f;     // Phase resistance (Ohms), 0 = auto
    float voltage_limit = 12.0f;       // Voltage limit
    float velocity_limit = 2000.0f;    // Velocity limit (deg/s)
    
    // PID tuning - Velocity
    float vel_p = 0.2f;
    float vel_i = 20.0f;
    float vel_d = 0.001f;
    float vel_ramp = 1000.0f;          // deg/s^2
    
    // PID tuning - Position
    float pos_p = 20.0f;
    float pos_i = 0.0f;
    float pos_d = 0.0f;
    float pos_vel_limit = 1000.0f;     // Max vel for position control (deg/s)
  };

  struct State {
    bool configured = false;
    bool enabled = false;
    bool foc_mode = false;
    float u1 = 0.0f;
    float u2 = 0.0f;
    float u3 = 0.0f;
    
    // FOC state
    float target = 0.0f;
    float current_angle = 0.0f;
    float current_velocity = 0.0f;
    const char* control_mode = "none";
  };

  MotorController();
  ~MotorController();

  // Basic initialization (low-level PWM control only)
  void begin(const Pins& pins, const Config& cfg);
  
  // FOC initialization (requires encoder)
  void begin_foc(const Pins& pins, const Config& cfg, AS5600Encoder* encoder);

  // Low-level control (works in both modes)
  void enable(bool on);
  void set_outputs(float u1, float u2, float u3);
  void stop();
  void safe();

  // FOC control (only works in FOC mode)
  void foc_enable();
  void foc_disable();
  void foc_set_mode_torque();
  void foc_set_mode_velocity();
  void foc_set_mode_angle();
  void foc_set_target(float target);
  void foc_loop();  // Call in main loop as fast as possible
  
  // PID tuning
  void foc_tune_velocity(float p, float i, float d);
  void foc_tune_position(float p, float i, float d);
  void foc_set_velocity_limit(float limit_dps);
  void foc_set_voltage_limit(float limit_v);

  // Getters
  const Pins& pins() const { return pins_; }
  const Config& cfg() const { return cfg_; }
  const State& state() const { return st_; }
  BLDCMotor* foc_motor() { return motor_; }

private:
  // Low-level PWM control
  static float clamp_(float x, float lo, float hi);
  uint32_t duty_from_u_(float u) const;
  void apply_pwm_();
  
  // SimpleFOC integration
  class AS5600EncoderSensor : public Sensor {
  public:
    AS5600EncoderSensor(AS5600Encoder* enc) : enc_(enc) {}
    void init() override {}
    float getSensorAngle() override;
    float getVelocity() override;
    int needsSearch() override { return 0; }
  private:
    AS5600Encoder* enc_;
  };
  
  void init_foc_();
  void update_foc_state_();

  // Member variables
  Pins pins_{};
  Config cfg_{};
  State st_{};
  
  // SimpleFOC objects (only allocated if FOC mode enabled)
  BLDCMotor* motor_ = nullptr;
  BLDCDriver3PWM* driver_ = nullptr;
  AS5600EncoderSensor* sensor_ = nullptr;
  AS5600Encoder* encoder_ = nullptr;
};