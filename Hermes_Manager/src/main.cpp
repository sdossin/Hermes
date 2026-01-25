// main.cpp
// Hermes minimal bring-up:
// - Single AS5600 encoder on I2C
// - Serial console for reading + calibration

#include <Arduino.h>
#include <Wire.h>

#include "Encoder.hpp"
#include "SerialConsole.hpp"

static Encoder g_enc;
static SerialConsole g_console;

// You can tweak these defaults here.
static Encoder::Config make_encoder_cfg() {
  Encoder::Config cfg;
  cfg.i2c_addr = 0x36;   // AS5600 default
  cfg.i2c_hz = 400000;
  cfg.sda_gpio = 21;
  cfg.scl_gpio = 22;
  cfg.wrap_deg = 360.0f;
  return cfg;
}

void setup() {
  // SerialConsole also calls Serial.begin(), but we start Serial early
  // so we can print boot messages even if console isn't up yet.
  Serial.begin(115200);
  while (!Serial) { /* wait */ }

  Serial.println("Hermes encoder bring-up");

  // Initialize I2C at known pins/speed.
  // Encoder::begin() also calls Wire.begin(pins) + setClock, but doing it
  // here makes failures easier to diagnose during bring-up.
  Wire.begin(21, 22);
  Wire.setClock(400000);

  // Start encoder
  const Encoder::Config enc_cfg = make_encoder_cfg();
  g_enc.begin(&Wire, enc_cfg);

  // Optional probe
  if (!g_enc.probe()) {
    Serial.println("WARNING: encoder did not ACK at 0x36. Check wiring/address.");
  } else {
    Serial.println("Encoder probe OK");
  }

  // Start console
  SerialConsole::Config con_cfg;
  con_cfg.baud = 115200;
  con_cfg.poll_ms = 1;
  con_cfg.watch_period_ms = 200;
  g_console.begin(&g_enc, con_cfg);
}

void loop() {
  const uint32_t now_ms = millis();

  // Console drives reading on demand / watch mode.
  g_console.poll(now_ms);
}
