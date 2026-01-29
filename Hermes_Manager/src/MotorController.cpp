// MotorController.cpp
// Motor controller implementation with SimpleFOC integration

#include "MotorController.hpp"
#include "Encoder.hpp"

#if defined(ARDUINO)
  #include <Arduino.h>
#endif

// ============================================================================
// AS5600EncoderSensor Implementation (SimpleFOC sensor wrapper)
// ============================================================================

float MotorController::AS5600EncoderSensor::getSensorAngle() {
  if (!enc_) return 0;
  enc_->read(millis());
  // SimpleFOC expects radians
  return enc_->last().deg * DEG_TO_RAD;
}

float MotorController::AS5600EncoderSensor::getVelocity() {
  if (!enc_) return 0;
  // SimpleFOC expects rad/s
  return enc_->last().vel_dps * DEG_TO_RAD;
}

// ============================================================================
// MotorController Implementation
// ============================================================================

MotorController::MotorController() {}

MotorController::~MotorController() {
  // Clean up SimpleFOC objects if allocated
  if (motor_) delete motor_;
  if (driver_) delete driver_;
  if (sensor_) delete sensor_;
}

void MotorController::begin(const Pins& pins, const Config& cfg) {
  pins_ = pins;
  cfg_ = cfg;

  st_.configured = false;
  st_.enabled = false;
  st_.foc_mode = false;
  st_.u1 = 0.0f;
  st_.u2 = 0.0f;
  st_.u3 = 0.0f;

#if defined(ARDUINO)
  // Setup PWM channel 1
  if (pins_.ch1 >= 0) {
    ledcSetup(cfg_.ledc_ch1, cfg_.pwm_hz, cfg_.pwm_bits);
    ledcAttachPin(pins_.ch1, cfg_.ledc_ch1);
    ledcWrite(cfg_.ledc_ch1, 0);
  }

  // Setup PWM channel 2
  if (pins_.ch2 >= 0) {
    ledcSetup(cfg_.ledc_ch2, cfg_.pwm_hz, cfg_.pwm_bits);
    ledcAttachPin(pins_.ch2, cfg_.ledc_ch2);
    ledcWrite(cfg_.ledc_ch2, 0);
  }

  // Setup PWM channel 3
  if (pins_.ch3 >= 0) {
    ledcSetup(cfg_.ledc_ch3, cfg_.pwm_hz, cfg_.pwm_bits);
    ledcAttachPin(pins_.ch3, cfg_.ledc_ch3);
    ledcWrite(cfg_.ledc_ch3, 0);
  }

  // Setup enable pin (start disabled)
  if (pins_.en >= 0) {
    pinMode(pins_.en, OUTPUT);
    digitalWrite(pins_.en, cfg_.en_active_high ? LOW : HIGH);
  }
#endif

  st_.configured = true;
}

void MotorController::begin_foc(const Pins& pins, const Config& cfg, AS5600Encoder* encoder) {
  // First do basic initialization
  begin(pins, cfg);
  
  if (!encoder) {
    Serial.println("MotorController: ERROR - null encoder for FOC mode");
    return;
  }
  
  encoder_ = encoder;
  
  // Create SimpleFOC objects
  motor_ = new BLDCMotor(cfg_.pole_pairs);
  driver_ = new BLDCDriver3PWM(pins_.ch1, pins_.ch2, pins_.ch3, pins_.en);
  sensor_ = new AS5600EncoderSensor(encoder_);
  
  // Initialize FOC
  init_foc_();
  
  st_.foc_mode = true;
  Serial.println("MotorController: FOC mode enabled");
}

void MotorController::init_foc_() {
  if (!motor_ || !driver_ || !sensor_) return;
  
  Serial.println("MotorController: Initializing SimpleFOC...");
  
  // Configure driver
  driver_->voltage_power_supply = cfg_.voltage_limit;
  driver_->pwm_frequency = cfg_.pwm_hz;
  driver_->init();
  driver_->enable();
  
  // Configure motor
  motor_->linkDriver(driver_);
  motor_->linkSensor(sensor_);
  
  // Set limits
  motor_->voltage_limit = cfg_.voltage_limit;
  motor_->velocity_limit = cfg_.velocity_limit * DEG_TO_RAD;
  
  // Set phase resistance if known
  if (cfg_.phase_resistance > 0) {
    motor_->phase_resistance = cfg_.phase_resistance;
  }
  
  // Configure velocity PID
  motor_->PID_velocity.P = cfg_.vel_p;
  motor_->PID_velocity.I = cfg_.vel_i;
  motor_->PID_velocity.D = cfg_.vel_d;
  motor_->PID_velocity.output_ramp = cfg_.vel_ramp * DEG_TO_RAD;
  motor_->PID_velocity.limit = cfg_.voltage_limit;
  
  // Configure position PID
  motor_->P_angle.P = cfg_.pos_p;
  motor_->P_angle.I = cfg_.pos_i;
  motor_->P_angle.D = cfg_.pos_d;
  motor_->P_angle.limit = cfg_.pos_vel_limit * DEG_TO_RAD;
  
  // Set control mode (default to angle control)
  motor_->controller = MotionControlType::angle;
  st_.control_mode = "angle";
  
  // Initialize motor and FOC
  motor_->init();
  motor_->initFOC();
  
  Serial.println("MotorController: SimpleFOC initialized successfully");
}

// ============================================================================
// Low-level Control (works in both modes)
// ============================================================================

void MotorController::enable(bool on) {
  if (!st_.configured) return;

  st_.enabled = on;

#if defined(ARDUINO)
  if (pins_.en >= 0) {
    bool pin_state = (on == cfg_.en_active_high);
    digitalWrite(pins_.en, pin_state ? HIGH : LOW);
  }
#endif

  if (!on) {
    apply_pwm_();
  }
}

void MotorController::set_outputs(float u1, float u2, float u3) {
  if (!st_.configured) return;
  
  // Don't allow manual PWM control in FOC mode
  if (st_.foc_mode) {
    Serial.println("MotorController: Cannot set outputs in FOC mode. Use foc_set_target()");
    return;
  }

  st_.u1 = clamp_(u1, -1.0f, 1.0f);
  st_.u2 = clamp_(u2, -1.0f, 1.0f);
  st_.u3 = clamp_(u3, -1.0f, 1.0f);

  apply_pwm_();
}

void MotorController::stop() {
  if (st_.foc_mode && motor_) {
    // Only set target to 0 in velocity and torque modes
    // In angle mode, "stop" means stay at current position
    if (motor_->controller == MotionControlType::velocity ||
        motor_->controller == MotionControlType::velocity_openloop ||
        motor_->controller == MotionControlType::torque) {
      motor_->target = 0;
      Serial.println("Motor stopped (target = 0)");
    } else if (motor_->controller == MotionControlType::angle ||
               motor_->controller == MotionControlType::angle_openloop) {
      // In angle mode, set target to current position
      motor_->target = motor_->shaft_angle;
      Serial.printf("Motor stopped (holding at %.1f deg)\n", 
                    motor_->shaft_angle * RAD_TO_DEG);
    }
  } else {
    st_.u1 = 0.0f;
    st_.u2 = 0.0f;
    st_.u3 = 0.0f;
    apply_pwm_();
  }
}

void MotorController::safe() {
  stop();
  enable(false);
}

// ============================================================================
// FOC Control
// ============================================================================

void MotorController::foc_enable() {
  if (!st_.foc_mode) {
    Serial.println("MotorController: Not in FOC mode");
    return;
  }
  enable(true);
  Serial.println("MotorController: FOC enabled");
}

void MotorController::foc_disable() {
  if (!st_.foc_mode) {
    Serial.println("MotorController: Not in FOC mode");
    return;
  }
  
  // Ramp down smoothly
  if (motor_) {
    motor_->target = 0;
    for (int i = 0; i < 100; i++) {
      motor_->loopFOC();
      motor_->move();
      delay(1);
    }
  }
  
  enable(false);
  Serial.println("MotorController: FOC disabled");
}

void MotorController::foc_set_mode_torque() {
  if (!st_.foc_mode || !motor_) return;
  motor_->controller = MotionControlType::torque;
  st_.control_mode = "torque";
  Serial.println("MotorController: FOC mode = torque");
}

void MotorController::foc_set_mode_velocity() {
  if (!st_.foc_mode || !motor_) return;
  motor_->controller = MotionControlType::velocity;
  st_.control_mode = "velocity";
  Serial.println("MotorController: FOC mode = velocity");
}

void MotorController::foc_set_mode_angle() {
  if (!st_.foc_mode || !motor_) return;
  motor_->controller = MotionControlType::angle;
  st_.control_mode = "angle";
  Serial.println("MotorController: FOC mode = angle");
}

void MotorController::foc_set_target(float target) {
  if (!st_.foc_mode || !motor_) return;
  
  // Convert degrees to radians for angle and velocity modes
  if (motor_->controller == MotionControlType::angle ||
      motor_->controller == MotionControlType::angle_openloop) {
    motor_->target = target * DEG_TO_RAD;
  } else if (motor_->controller == MotionControlType::velocity ||
             motor_->controller == MotionControlType::velocity_openloop) {
    motor_->target = target * DEG_TO_RAD;
  } else {
    // Torque mode - target is voltage
    motor_->target = target;
  }
  
  st_.target = target;
}

void MotorController::foc_loop() {
  if (!st_.foc_mode || !motor_ || !st_.enabled) return;
  
  // Real-time FOC loop - must run as fast as possible!
  motor_->loopFOC();
  motor_->move();
  
  // Update state
  update_foc_state_();
}

void MotorController::foc_tune_velocity(float p, float i, float d) {
  if (!st_.foc_mode || !motor_) return;
  
  motor_->PID_velocity.P = p;
  motor_->PID_velocity.I = i;
  motor_->PID_velocity.D = d;
  
  Serial.printf("MotorController: Velocity PID = P:%.3f I:%.3f D:%.3f\n", p, i, d);
}

void MotorController::foc_tune_position(float p, float i, float d) {
  if (!st_.foc_mode || !motor_) return;
  
  motor_->P_angle.P = p;
  motor_->P_angle.I = i;
  motor_->P_angle.D = d;
  
  Serial.printf("MotorController: Position PID = P:%.3f I:%.3f D:%.3f\n", p, i, d);
}

void MotorController::foc_set_velocity_limit(float limit_dps) {
  if (!st_.foc_mode || !motor_) return;
  motor_->velocity_limit = limit_dps * DEG_TO_RAD;
  Serial.printf("MotorController: Velocity limit = %.1f deg/s\n", limit_dps);
}

void MotorController::foc_set_voltage_limit(float limit_v) {
  if (!st_.foc_mode || !motor_) return;
  motor_->voltage_limit = limit_v;
  Serial.printf("MotorController: Voltage limit = %.1f V\n", limit_v);
}

void MotorController::update_foc_state_() {
  if (!motor_ || !encoder_) return;
  
  st_.current_angle = motor_->shaft_angle * RAD_TO_DEG;
  st_.current_velocity = motor_->shaft_velocity * RAD_TO_DEG;
}

// ============================================================================
// Private Helper Functions
// ============================================================================

float MotorController::clamp_(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

uint32_t MotorController::duty_from_u_(float u) const {
  const uint32_t max_duty = (1u << cfg_.pwm_bits) - 1;
  const float normalized = (u + 1.0f) * 0.5f;
  const uint32_t duty = static_cast<uint32_t>(normalized * static_cast<float>(max_duty) + 0.5f);
  if (duty > max_duty) return max_duty;
  return duty;
}

void MotorController::apply_pwm_() {
  if (!st_.configured) return;
  
  // Don't override PWM in FOC mode
  if (st_.foc_mode) return;

#if defined(ARDUINO)
  if (!st_.enabled) {
    if (pins_.ch1 >= 0) ledcWrite(cfg_.ledc_ch1, 0);
    if (pins_.ch2 >= 0) ledcWrite(cfg_.ledc_ch2, 0);
    if (pins_.ch3 >= 0) ledcWrite(cfg_.ledc_ch3, 0);
    return;
  }

  if (pins_.ch1 >= 0) {
    const uint32_t duty1 = duty_from_u_(st_.u1);
    ledcWrite(cfg_.ledc_ch1, duty1);
  }

  if (pins_.ch2 >= 0) {
    const uint32_t duty2 = duty_from_u_(st_.u2);
    ledcWrite(cfg_.ledc_ch2, duty2);
  }

  if (pins_.ch3 >= 0) {
    const uint32_t duty3 = duty_from_u_(st_.u3);
    ledcWrite(cfg_.ledc_ch3, duty3);
  }
#endif
}