// main.cpp
// Hermes with integrated SimpleFOC closed-loop control
// - AS5600 encoder on I2C
// - Motor controller with SimpleFOC support
// - Serial console for control and monitoring

#include <Arduino.h>
#include <Wire.h>

#include "Encoder.hpp"
#include "MotorController.hpp"
#include "SerialConsole.hpp"

static AS5600Encoder g_enc;
static MotorController g_motor;
static SerialConsole g_console;

// ---- AS5600Encoder Configuration ----
static AS5600Encoder::Config make_encoder_cfg() {
  AS5600Encoder::Config cfg;
  cfg.i2c_addr = 0x36;   // AS5600 default
  cfg.i2c_hz = 400000;
  cfg.sda_gpio = 21;
  cfg.scl_gpio = 22;
  cfg.wrap_deg = 360.0f;
  return cfg;
}

// ---- Motor Pin Configuration ----
// IMPORTANT: Update these pins to match your wiring!
static MotorController::Pins make_motor_pins() {
  MotorController::Pins p;
  p.ch1 = 25;  // PWM output 1
  p.ch2 = 26;  // PWM output 2
  p.ch3 = 27;  // PWM output 3
  p.en  = 33;  // enable pin
  return p;
}

// ---- Motor Configuration with FOC ----
static MotorController::Config make_motor_cfg() {
  MotorController::Config c;
  
  // PWM settings
  c.pwm_hz = 20000;     // 20kHz
  c.pwm_bits = 12;      // 0..4095
  c.ledc_ch1 = 0;
  c.ledc_ch2 = 1;
  c.ledc_ch3 = 2;
  c.en_active_high = true;
  
  // SimpleFOC settings
  c.use_foc = true;                // ENABLE FOC MODE
  
  // Motor parameters - ADJUST FOR YOUR MOTOR!
  c.pole_pairs = 7.0f;             // Count magnets, divide by 2
  c.phase_resistance = 2.3f;       // Ohms (0 = auto-detect)
  c.voltage_limit = 2.0f;         // Max voltage
  c.velocity_limit = 90.0f;      // Max velocity (deg/s)
  
  // Velocity PID tuning
  c.vel_p = 0.2f;                  // Proportional gain
  c.vel_i = 20.0f;                 // Integral gain
  c.vel_d = 0.001f;                // Derivative gain
  c.vel_ramp = 1000.0f;            // Acceleration limit (deg/s^2)
  
  // Position PID tuning
  c.pos_p = 20.0f;                 // Proportional gain (higher = stiffer)
  c.pos_i = 0.0f;                  // Usually not needed
  c.pos_d = 0.0f;                  // Can reduce oscillations
  c.pos_vel_limit = 1000.0f;       // Max velocity when seeking (deg/s)
  
  return c;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait */ }

  Serial.println("\n\n╔════════════════════════════════════════╗");
  Serial.println("║  Hermes - SimpleFOC Motor Control     ║");
  Serial.println("╚════════════════════════════════════════╝\n");

  // Initialize I2C
  Wire.begin(21, 22);
  Wire.setClock(400000);

  // Initialize encoder
  const AS5600Encoder::Config enc_cfg = make_encoder_cfg();
  g_enc.begin(&Wire, enc_cfg);

  if (!g_enc.probe()) {
    Serial.println("⚠  WARNING: AS5600Encoder not detected at 0x36");
    Serial.println("   Check wiring and I2C address");
  } else {
    Serial.println("✓  AS5600Encoder initialized");
  }

  // Initialize motor controller with FOC
  const MotorController::Pins mp = make_motor_pins();
  const MotorController::Config mc = make_motor_cfg();
  
  if (mc.use_foc) {
    Serial.println("\nInitializing SimpleFOC...");
    g_motor.begin_foc(mp, mc, &g_enc);
    Serial.println("✓  FOC mode enabled");
  } else {
    Serial.println("\nInitializing PWM mode...");
    g_motor.begin(mp, mc);
    Serial.println("✓  PWM mode enabled");
  }
  
  g_motor.safe(); // Ensure disabled at boot

  // Initialize console
  SerialConsole::Config con_cfg;
  con_cfg.baud = 115200;
  con_cfg.poll_ms = 1;
  con_cfg.watch_period_ms = 200;
  g_console.begin(&g_enc, &g_motor, con_cfg);

  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║  Setup Complete!                      ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println("\nQuick Start Commands:");
  Serial.println("  help              - Show all commands");
  Serial.println("  m foc enable      - Enable FOC control");
  Serial.println("  m foc mode angle  - Position control");
  Serial.println("  m foc target 90   - Move to 90 degrees");
  Serial.println("  watch on          - Monitor in real-time");
  Serial.println();
}

void loop() {
  const uint32_t now_ms = millis();
  
  // Run FOC control loop (CRITICAL - runs as fast as possible!)
  g_motor.foc_loop();
  
  // Handle serial console (less time-critical)
  g_console.poll(now_ms);
}