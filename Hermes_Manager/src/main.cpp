// main.cpp
// Bare-bones Hermes bring-up (ALL-IN-ONE FILE)
// - AS5600 over I2C
// - SimpleFOC angle control
// - Motor ENABLED by default
// - Target = 0 deg by default (set to current angle to avoid kick recommended)
// - ONLY serial output: encoder angle + motor angle (in-place with \r)

#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>

// ------------------------ USER WIRING ------------------------
static constexpr int I2C_SDA = 21;
static constexpr int I2C_SCL = 22;
static constexpr uint32_t I2C_HZ = 400000;

static constexpr int PIN_U  = 25;
static constexpr int PIN_V  = 26;
static constexpr int PIN_W  = 27;
static constexpr int PIN_EN = 33;   // set -1 if your driver has no enable

static constexpr int POLE_PAIRS = 7;
// ------------------------------------------------------------

// -------------------- CONTROL / TUNING -----------------------
// Target (deg) - default requested
static volatile float g_target_deg = 90.0f;

// Print rate (Hz)
static constexpr uint32_t PRINT_PERIOD_MS = 10; // 100 Hz in-place printing

// Increase responsiveness/stiffness (start here; adjust carefully)
static constexpr float VOLTAGE_LIMIT_V      = 6.0f;   // higher = more torque authority
static constexpr float VELOCITY_LIMIT_DPS   = 360.0f; // faster position corrections
static constexpr float P_ANGLE_P            = 8.0f;   // stiffer position loop

// Optional damping (helps reduce oscillations when you raise stiffness)
// If your SimpleFOC version exposes P_angle.D, you can use it.
// Start small.
static constexpr float P_ANGLE_D            = 0.05f;  // damping (set 0 if not supported)

// Optional velocity loop tuning (can help smoothness)
// Leave default unless needed.
static constexpr float PID_VEL_P            = 0.2f;
static constexpr float PID_VEL_I            = 2.0f;
static constexpr float PID_VEL_D            = 0.0f;
// ------------------------------------------------------------

// SimpleFOC objects
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_U, PIN_V, PIN_W, PIN_EN);

static uint32_t g_next_print_ms = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_HZ);

  // --- Sensor ---
  sensor.init(&Wire);

  // --- Driver ---
  driver.voltage_power_supply = 12.0f;
  driver.pwm_frequency = 20000;

  // If your driver enable is ACTIVE-LOW, uncomment the next line:
  // driver.enable_active_high = false;

  driver.init();

  // IMPORTANT: enable outputs BEFORE initFOC() so alignment can drive phases
  driver.enable();

  // --- Motor ---
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);

  motor.controller = MotionControlType::angle;

  // Increased authority (responsiveness)
  motor.voltage_limit  = VOLTAGE_LIMIT_V;
  motor.velocity_limit = VELOCITY_LIMIT_DPS * DEG_TO_RAD;

  // Stiffer position control
  motor.P_angle.P = P_ANGLE_P;
  // Many builds expose these members; if your build errors, delete this line.
  motor.P_angle.D = P_ANGLE_D;

  // Mild velocity loop settings (optional)
  motor.PID_velocity.P = PID_VEL_P;
  motor.PID_velocity.I = PID_VEL_I;
  motor.PID_velocity.D = PID_VEL_D;
  motor.PID_velocity.limit = motor.voltage_limit;

  motor.init();

  // Run FOC alignment/calibration (now that driver is enabled)
  motor.initFOC();

  // Default target:
  // If you truly want "0 deg by default", keep the next line.
  // If you want "no kick at boot", comment it out and use the line after it.
  g_target_deg = 90.0f;

  // Recommended "no kick" option:
  // g_target_deg = sensor.getAngle() * RAD_TO_DEG;

  g_next_print_ms = millis() + PRINT_PERIOD_MS;
}

void loop() {
  motor.loopFOC();
  motor.move(g_target_deg * DEG_TO_RAD);

  // Continuous in-place output using '\r'
  const uint32_t now = millis();
  if ((int32_t)(now - g_next_print_ms) >= 0) {
    g_next_print_ms = now + PRINT_PERIOD_MS;

    const float enc_deg   = sensor.getAngle() * RAD_TO_DEG;
    const float motor_deg = motor.shaft_angle * RAD_TO_DEG;

    // Print in-place (no extra text besides the two angles)
    Serial.print("enc_deg=");
    Serial.print(enc_deg, 3);
    Serial.print(" motor_deg=");
    Serial.print(motor_deg, 3);
    Serial.print("\r");
  }
}
