// MotorController.hpp
// Minimal 3-channel motor controller + enable for ESP32 (Arduino).
// Designed for bring-up and console control (no current sensing).
#pragma once

#include <cstdint>

class MotorController {
public:
  struct Pins {
    int ch1 = -1;
    int ch2 = -1;
    int ch3 = -1;
    int en  = -1;
  };

  struct Config {
    // PWM
    uint32_t pwm_hz = 20000;      // 20 kHz (quiet)
    uint8_t pwm_bits = 12;        // 0..4095

    // ESP32 LEDC channels used for the 3 outputs.
    // Choose any 0..15 that don't collide with other PWM usage.
    uint8_t ledc_ch1 = 0;
    uint8_t ledc_ch2 = 1;
    uint8_t ledc_ch3 = 2;

    // Enable polarity
    bool en_active_high = true;
  };

  struct State {
    bool configured = false;
    bool enabled = false;
    float u1 = 0.0f; // command in [-1, 1]
    float u2 = 0.0f;
    float u3 = 0.0f;
  };

  MotorController();

  // Call once in setup().
  void begin(const Pins& pins, const Config& cfg = Config{});

  // Enable output stage.
  void enable(bool on);

  // Set normalized outputs (each in [-1, 1]). If disabled, commands are stored
  // but PWM outputs remain 0.
  void set_outputs(float u1, float u2, float u3);

  // Immediately force outputs to 0 (does not change enable state).
  void stop();

  // Convenience: disable + stop.
  void safe();

  const Pins& pins() const { return pins_; }
  const Config& cfg() const { return cfg_; }
  const State& state() const { return st_; }

private:
  static float clamp_(float x, float lo, float hi);
  uint32_t duty_from_u_(float u) const;
  void apply_pwm_();

  Pins pins_{};
  Config cfg_{};
  State st_{};
};
