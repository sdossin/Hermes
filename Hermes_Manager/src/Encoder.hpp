// AS5600Encoder.hpp
// Single-encoder support for ESP32 + AS5600 over I2C (Arduino Wire).
// All sensor communication + calibration math is contained in this object.
#pragma once

#include <cstdint>

// Forward-declare TwoWire to avoid pulling Arduino headers into everyone.
class TwoWire;

class AS5600Encoder {
public:
  struct Config {
    uint8_t i2c_addr = 0x36;      // AS5600 default
    uint32_t i2c_hz = 400000;     // 400 kHz
    int sda_gpio = 21;
    int scl_gpio = 22;

    // Output units and wrapping.
    // If wrap_deg > 0, output is wrapped into [0, wrap_deg).
    float wrap_deg = 360.0f;
  };

  struct Reading {
    bool ok = false;
    uint32_t t_ms = 0;

    uint16_t raw12 = 0;       // 0..4095
    float deg_raw = 0.0f;     // raw converted to degrees (0..360)

    // Calibrated output
    float deg = 0.0f;         // after invert+offset+wrap
    float vel_dps = 0.0f;     // degrees per second

    // Diagnostics
    uint8_t i2c_error = 0;    // 0 if OK; otherwise Wire error codes
  };

  AS5600Encoder();

  // Provide the Wire instance you want to use (typically &Wire).
  // Call begin() once in setup().
  void begin(TwoWire* wire, const Config& cfg);

  // Returns true if device ACKs at configured address.
  bool probe();

  // Read sensor and update internal state. Returns ok.
  bool read(uint32_t now_ms);

  // Latest reading.
  const Reading& last() const { return last_; }

  // ---- Calibration controls ----

  // Invert direction (true = invert).
  void set_invert(bool inv) { invert_ = inv; }
  bool invert() const { return invert_; }

  // Set an additive offset in degrees.
  void set_offset_deg(float off) { offset_deg_ = off; }
  float offset_deg() const { return offset_deg_; }

  // Set current physical position to 0 degrees.
  // Requires a recent read() before calling.
  void zero_here();

  // Set current physical position to a specified angle.
  // Requires a recent read() before calling.
  void set_here(float desired_deg);

  // Access config.
  const Config& cfg() const { return cfg_; }

private:
  // AS5600 RAW ANGLE register MSB/LSB: 0x0E, 0x0F
  static constexpr uint8_t kRegRawAngleMSB = 0x0E;

  bool read_raw12_(uint16_t& out_raw12);
  float apply_cal_(float raw_deg) const;
  static float wrap_(float x, float wrap_deg);

  TwoWire* wire_ = nullptr;
  Config cfg_{};

  Reading last_{};
  bool invert_ = true;
  float offset_deg_ = 0.0f;

  // For velocity estimation
  bool have_prev_ = false;
  float prev_deg_ = 0.0f;
  uint32_t prev_t_ms_ = 0;
};
