#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <math.h>

// ------------------------- Hardware pins -------------------------
static constexpr int I2C_SDA = 21;
static constexpr int I2C_SCL = 22;
static constexpr uint32_t I2C_HZ = 400000;

static constexpr int PIN_U = 25;
static constexpr int PIN_V = 26;
static constexpr int PIN_W = 27;
static constexpr int PIN_EN = 33;   // set -1 if no enable pin

// ------------------------- Motor params -------------------------
// IMPORTANT: motor pole pairs (rotor magnets / 2)
static constexpr int POLE_PAIRS = 7;

// If you want your "mechanical zero" to be sensor zero shifted by 47°:
static constexpr float SENSOR_ZERO_OFFSET_DEG = 37.0f;  // set 0 if you don't want this
static constexpr float SENSOR_ZERO_OFFSET_RAD = SENSOR_ZERO_OFFSET_DEG * DEG_TO_RAD;

// ------------------------- Limits / tuning -------------------------
static constexpr float VOLTAGE_SUPPLY_V      = 12.0f;
static constexpr float VOLTAGE_LIMIT_V      = 6.0f;   // keep conservative for position hold
static constexpr float VELOCITY_LIMIT_RAD_S = 6.0f;   // slow and safe (~1 rev/s = 6.28 rad/s)

// Bare-bones angle loop gains (start low)
static constexpr float P_ANGLE_P = 5.0f;

// Optional velocity PID (used internally by angle mode)
// Keep simple and stable
static constexpr float PID_VEL_P = 0.25f;
static constexpr float PID_VEL_I = 1.0f;
static constexpr float PID_VEL_D = 0.0f;

// Filtering (helps I2C sensors)
static constexpr float LPF_VEL_TF = 0.02f;  // 20ms

// ------------------------- Devices -------------------------
MagneticSensorI2C sensor(AS5600_I2C);
BLDCMotor motor(POLE_PAIRS);
BLDCDriver3PWM driver(PIN_U, PIN_V, PIN_W, PIN_EN);

// ------------------------- Target -------------------------
static volatile float g_target_deg = 0.0f;

// ------------------------- Helpers -------------------------
static inline float wrap_0_360(float deg) {
  deg = fmodf(deg, 360.0f);
  if (deg < 0.0f) deg += 360.0f;
  return deg;
}

static inline float wrap_0_2pi(float rad) {
  rad = fmodf(rad, 2.0f * PI);
  if (rad < 0.0f) rad += 2.0f * PI;
  return rad;
}

static void serialTargetLoop() {
  static String s;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      float v = s.toFloat();
      g_target_deg = v;
      Serial.print("New target (deg): ");
      Serial.println(g_target_deg, 2);
      s = "";
    } else {
      s += c;
    }
  }
}

// ------------------------- Setup -------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Bare-bones position control (SimpleFOC + AS5600 I2C)");
  Serial.println("Type a target angle in degrees and press Enter (e.g., 90).");

  // I2C + sensor
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_HZ);
  sensor.init(&Wire);

  // Driver
  driver.voltage_power_supply = VOLTAGE_SUPPLY_V;
  driver.pwm_frequency = 20000;
  driver.init();
  driver.disable();

  // Motor
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);

  // Your sensor direction is correct (as you stated), so we don't override it.
  // If needed you could set:
  // motor.sensor_direction = Direction::CW; or Direction::CCW;

  // Apply your chosen mechanical zero offset (optional)
  motor.sensor_offset = SENSOR_ZERO_OFFSET_RAD;

  // Bare-bones position control
  motor.controller = MotionControlType::angle;

  motor.voltage_limit  = VOLTAGE_LIMIT_V;
  motor.velocity_limit = VELOCITY_LIMIT_RAD_S;

  motor.P_angle.P = P_ANGLE_P;
  motor.P_angle.I = 0.0f;
  motor.P_angle.D = 0.0f;

  motor.PID_velocity.P = PID_VEL_P;
  motor.PID_velocity.I = PID_VEL_I;
  motor.PID_velocity.D = PID_VEL_D;

  motor.LPF_velocity.Tf = LPF_VEL_TF;

  motor.init();

  // Enable + FOC init
  driver.enable();
  motor.initFOC();

  // Start by holding current position
  sensor.update();
  float cur_rad = wrap_0_2pi(sensor.getAngle() - motor.sensor_offset);
  g_target_deg = cur_rad * RAD_TO_DEG;

  Serial.print("FOC ready. Holding current position at (deg): ");
  Serial.println(g_target_deg, 2);
}

// ------------------------- Main loop -------------------------
void loop() {
  serialTargetLoop();

  // FOC update
  motor.loopFOC();

  // Convert target degrees -> radians in shaft domain
  float target_rad = wrap_0_2pi(wrap_0_360(g_target_deg) * DEG_TO_RAD);

  // In SimpleFOC angle mode, motor.move() expects shaft angle (rad).
  motor.move(target_rad);

  // Periodic status print
  static unsigned long last_ms = 0;
  if (millis() - last_ms > 100) {
    last_ms = millis();

    sensor.update();
    float sensor_rad = wrap_0_2pi(sensor.getAngle());
    float shaft_rad  = wrap_0_2pi(sensor_rad - motor.sensor_offset);

    Serial.print("Target(deg): ");
    Serial.print(wrap_0_360(g_target_deg), 2);
    Serial.print(" | Shaft(deg): ");
    Serial.print(shaft_rad * RAD_TO_DEG, 2);
    Serial.print(" | Sensor(deg): ");
    Serial.print(sensor_rad * RAD_TO_DEG, 2);
    Serial.println();
  }
}
