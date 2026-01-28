// main.cpp
// Hermes minimal bring-up:
// - Single AS5600 encoder on I2C
// - Motor controller (3 PWM channels + enable)
// - Serial console for reading + calibration + motor commands (next step)

#include <Arduino.h>
#include <Wire.h>

#include "Encoder.hpp"
#include "MotorController.hpp"
#include "SerialConsole.hpp"

static Encoder g_enc;
static MotorController g_motor;
static SerialConsole g_console;

// ---- Encoder defaults ----
static Encoder::Config make_encoder_cfg() {
  Encoder::Config cfg;
  cfg.i2c_addr = 0x36;   // AS5600 default
  cfg.i2c_hz = 400000;
  cfg.sda_gpio = 21;
  cfg.scl_gpio = 22;
  cfg.wrap_deg = 360.0f;
  return cfg;
}

// ---- Motor defaults ----
// IMPORTANT: Replace these pins with the ones you actually wired.
static MotorController::Pins make_motor_pins() {
  MotorController::Pins p;
  p.ch1 = 25;  // PWM output 1
  p.ch2 = 26;  // PWM output 2
  p.ch3 = 27;  // PWM output 3
  p.en  = 33;  // enable pin
  return p;
}

static MotorController::Config make_motor_cfg() {
  MotorController::Config c;
  c.pwm_hz = 20000;     // 20kHz
  c.pwm_bits = 12;      // 0..4095
  c.ledc_ch1 = 0;
  c.ledc_ch2 = 1;
  c.ledc_ch3 = 2;
  c.en_active_high = true;
  return c;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait */ }

  Serial.println("Hermes bring-up (encoder + motor)");

  // I2C
  Wire.begin(21, 22);
  Wire.setClock(400000);

  // Encoder
  const Encoder::Config enc_cfg = make_encoder_cfg();
  g_enc.begin(&Wire, enc_cfg);

  if (!g_enc.probe()) {
    Serial.println("WARNING: encoder did not ACK at 0x36. Check wiring/address.");
  } else {
    Serial.println("Encoder probe OK");
  }

  // Motor controller
  const MotorController::Pins mp = make_motor_pins();
  const MotorController::Config mc = make_motor_cfg();
  g_motor.begin(mp, mc);
  g_motor.safe(); // ensure disabled at boot

  // Console
  SerialConsole::Config con_cfg;
  con_cfg.baud = 115200;
  con_cfg.poll_ms = 1;
  con_cfg.watch_period_ms = 200;

  // CURRENT (what you have now):
  // g_console.begin(&g_enc, con_cfg);

  // NEXT STEP (after we update SerialConsole to accept motor pointer):
  // g_console.begin(&g_enc, &g_motor, con_cfg);

  g_console.begin(&g_enc, con_cfg);
}

void loop() {
  const uint32_t now_ms = millis();
  g_console.poll(now_ms);
}
