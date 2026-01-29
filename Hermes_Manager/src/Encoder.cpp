// AS5600Encoder.cpp
// Single-encoder support for ESP32 + AS5600 over I2C (Arduino Wire).

#include "Encoder.hpp"

#if defined(ARDUINO)
  #include <Arduino.h>
  #include <Wire.h>
#endif

AS5600Encoder::AS5600Encoder() = default;

void AS5600Encoder::begin(TwoWire* wire, const Config& cfg) {
  wire_ = wire;
  cfg_ = cfg;

#if defined(ARDUINO)
  // Arduino-ESP32: begin can be called with explicit pins.
  if (wire_) {
    // Some cores allow begin(sda, scl). For ESP32 Arduino this is supported.
    wire_->begin(cfg_.sda_gpio, cfg_.scl_gpio);
    wire_->setClock(cfg_.i2c_hz);
  }
#endif

  last_ = Reading{};
  have_prev_ = false;
  prev_deg_ = 0.0f;
  prev_t_ms_ = 0;
}

bool AS5600Encoder::probe() {
#if defined(ARDUINO)
  if (!wire_) return false;
  wire_->beginTransmission(cfg_.i2c_addr);
  const uint8_t err = wire_->endTransmission();
  return (err == 0);
#else
  return false;
#endif
}

bool AS5600Encoder::read(uint32_t now_ms) {
  last_.t_ms = now_ms;
  last_.i2c_error = 0;

  uint16_t raw12 = 0;
  if (!read_raw12_(raw12)) {
    last_.ok = false;
    return false;
  }

  last_.ok = true;
  last_.raw12 = raw12;

  // Convert to degrees (0..360)
  const float raw_deg = (static_cast<float>(raw12) * 360.0f) / 4096.0f;
  last_.deg_raw = raw_deg;

  // Apply calibration
  const float deg = apply_cal_(raw_deg);
  last_.deg = deg;

  // Velocity estimate
  if (have_prev_ && now_ms > prev_t_ms_) {
    const float dt = static_cast<float>(now_ms - prev_t_ms_) * 0.001f;
    // Use shortest signed delta on circle for velocity stability.
    float d = deg - prev_deg_;
    const float w = (cfg_.wrap_deg > 0.0f) ? cfg_.wrap_deg : 360.0f;
    // Wrap delta to (-w/2, w/2]
    while (d > 0.5f * w) d -= w;
    while (d <= -0.5f * w) d += w;
    last_.vel_dps = d / dt;
  } else {
    last_.vel_dps = 0.0f;
  }

  have_prev_ = true;
  prev_deg_ = deg;
  prev_t_ms_ = now_ms;
  return true;
}

void AS5600Encoder::zero_here() {
  // Set offset so that current calibrated output becomes 0.
  // We compute offset based on last raw reading.
  const float raw_deg = last_.deg_raw;
  const float signed_raw = invert_ ? -raw_deg : raw_deg;
  offset_deg_ = wrap_(0.0f - signed_raw, cfg_.wrap_deg);
}

void AS5600Encoder::set_here(float desired_deg) {
  const float raw_deg = last_.deg_raw;
  const float signed_raw = invert_ ? -raw_deg : raw_deg;
  offset_deg_ = wrap_(desired_deg - signed_raw, cfg_.wrap_deg);
}

bool AS5600Encoder::read_raw12_(uint16_t& out_raw12) {
#if !defined(ARDUINO)
  (void)out_raw12;
  return false;
#else
  if (!wire_) return false;

  // Point to RAW ANGLE MSB register (0x0E)
  wire_->beginTransmission(cfg_.i2c_addr);
  wire_->write(kRegRawAngleMSB);
  uint8_t err = wire_->endTransmission(false); // repeated start
  if (err != 0) {
    last_.i2c_error = err;
    return false;
  }

  const uint8_t n = wire_->requestFrom((int)cfg_.i2c_addr, 2);
  if (n != 2) {
    last_.i2c_error = 0xFF; // pseudo-code: short read
    return false;
  }

  const int hi = wire_->read();
  const int lo = wire_->read();
  if (hi < 0 || lo < 0) {
    last_.i2c_error = 0xFE; // pseudo-code: empty buffer
    return false;
  }

  const uint16_t v = (static_cast<uint16_t>(hi) << 8) | static_cast<uint16_t>(lo);
  out_raw12 = v & 0x0FFF;
  return true;
#endif
}

float AS5600Encoder::apply_cal_(float raw_deg) const {
  const float signed_raw = invert_ ? -raw_deg : raw_deg;
  const float x = signed_raw + offset_deg_;
  return wrap_(x, cfg_.wrap_deg);
}

float AS5600Encoder::wrap_(float x, float wrap_deg) {
  if (wrap_deg <= 0.0f) return x;
  // Bring into [0, wrap_deg)
  while (x >= wrap_deg) x -= wrap_deg;
  while (x < 0.0f) x += wrap_deg;
  return x;
}
