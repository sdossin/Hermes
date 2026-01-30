// main.cpp
// Bare-bones Hermes bring-up (ALL-IN-ONE FILE)
// - AS5600 over I2C
// - SimpleFOC angle control
// - Motor ENABLED by default
// - Target angle set by user input over Serial console
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
static volatile float g_target_deg = 0.0f;

static constexpr uint32_t PRINT_PERIOD_MS = 10; // 100 Hz printing

static constexpr float VOLTAGE_LIMIT_V      = 6.0f;
static constexpr float VELOCITY_LIMIT_DPS   = 360.0f;
static constexpr float P_ANGLE_P            = 8.0f;
static constexpr float P_ANGLE_D            = 0.05f;

static constexpr float PID_VEL_P            = 0.2f;
static constexpr float PID_VEL_I            = 2.0f;
static constexpr float PID_VEL_D            = 0.0f;
// ------------------------------------------------------------

// SimpleFOC objects
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_U, PIN_V, PIN_W, PIN_EN);

static uint32_t g_next_print_ms = 0;

// Buffer for serial input
String inputString = "";

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("\n=== SimpleFOC Angle Control Ready ===");
  Serial.println("Type a target angle in degrees and press ENTER:");
  Serial.println("Example: 90");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_HZ);

  // --- Sensor ---
  sensor.init(&Wire);

  // --- Driver ---
  driver.voltage_power_supply = 12.0f;
  driver.pwm_frequency = 20000;

  // If your driver enable is ACTIVE-LOW, uncomment:
  // driver.enable_active_high = false;

  driver.init();
  driver.enable();

  // --- Motor ---
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);

  motor.controller = MotionControlType::angle;

  motor.voltage_limit  = VOLTAGE_LIMIT_V;
  motor.velocity_limit = VELOCITY_LIMIT_DPS * DEG_TO_RAD;

  motor.P_angle.P = P_ANGLE_P;
  motor.P_angle.D = P_ANGLE_D;

  motor.PID_velocity.P = PID_VEL_P;
  motor.PID_velocity.I = PID_VEL_I;
  motor.PID_velocity.D = PID_VEL_D;
  motor.PID_velocity.limit = motor.voltage_limit;

  motor.init();
  motor.initFOC();

  // Start at current angle to avoid sudden kick
  g_target_deg = sensor.getAngle() * RAD_TO_DEG;

  Serial.print("Initial target set to current angle: ");
  Serial.println(g_target_deg);

  g_next_print_ms = millis() + PRINT_PERIOD_MS;
}

void loop() {
  // --- Handle user serial input ---
  while (Serial.available() > 0) {
    char c = Serial.read();

    // If newline received, parse number
    if (c == '\n' || c == '\r') {
      if (inputString.length() > 0) {
        float newTarget = inputString.toFloat();
        g_target_deg = newTarget;

        Serial.print("\nNew target angle set to: ");
        Serial.print(g_target_deg);
        Serial.println(" deg");

        inputString = "";
      }
    }
    else {
      inputString += c;
    }
  }

  // --- Run motor control loop ---
  motor.loopFOC();
  motor.move(g_target_deg * DEG_TO_RAD);

  // --- Print angles continuously ---
  const uint32_t now = millis();
  if ((int32_t)(now - g_next_print_ms) >= 0) {
    g_next_print_ms = now + PRINT_PERIOD_MS;

    const float enc_deg   = sensor.getAngle() * RAD_TO_DEG;
    const float motor_deg = motor.shaft_angle * RAD_TO_DEG;

    Serial.print("enc_deg=");
    Serial.print(enc_deg, 2);
    Serial.print(" motor_deg=");
    Serial.print(motor_deg, 2);
    Serial.print(" target=");
    Serial.print(g_target_deg, 2);
    Serial.print("   \r");
  }
}
