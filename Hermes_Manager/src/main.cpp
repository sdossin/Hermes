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
static constexpr uint32_t PRINT_PERIOD_MS = 10;

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

// State
static uint32_t g_next_print_ms = 0;
static bool g_enabled   = false;
static bool g_foc_ready = false;

// Target in ABSOLUTE degrees (AS5600 frame): 0..360
static volatile float g_target_abs_deg = 0.0f;

// Serial input
String inputString = "";

// ---------- Helpers ----------
static float norm360(float deg) {
  float x = fmodf(deg, 360.0f);
  if (x < 0.0f) x += 360.0f;
  return x;
}

static bool isAngleCommand(const String& s) {
  char *endptr = nullptr;
  const String t = String(s);
  const char* c = t.c_str();
  (void)strtod(c, &endptr);
  if (c == endptr) return false;
  while (*endptr == ' ' || *endptr == '\t') endptr++;
  return *endptr == '\0';
}

static float encAbsDegFresh() {
  sensor.update();
  return norm360(sensor.getAngle() * RAD_TO_DEG);
}

static void printHelp() {
  Serial.println("\nCommands:");
  Serial.println("  e            -> enable motor (runs initFOC once per boot/session)");
  Serial.println("  c            -> run initFOC now (calibrate this session)");
  Serial.println("  d            -> disable motor");
  Serial.println("  p            -> print current abs angle + FOC offset");
  Serial.println("  <number>     -> set target ABS angle in degrees (0..360)");
  Serial.println();
}

static void disableMotor() {
  driver.disable();
  g_enabled = false;
  Serial.println("\nMotor DISABLED");
}

// Runs initFOC() using your library (no overload available).
// This may move the motor slightly.
static void runFOCOnceThisSession() {
  if (g_foc_ready) {
    Serial.println("\nFOC already initialized this session.");
    return;
  }

  driver.enable();
  g_enabled = true;

  motor.initFOC();      // <-- only API available in your SimpleFOC build
  g_foc_ready = true;

  Serial.print("\nFOC initialized (this session). sensor_offset_deg=");
  Serial.println(motor.sensor_offset * RAD_TO_DEG, 2);
}

static void enableMotor() {
  // Run FOC init once per boot/session
  if (!g_foc_ready) runFOCOnceThisSession();

  Serial.print("\nMotor ENABLED. Target(abs)=");
  Serial.print(g_target_abs_deg, 2);
  Serial.println(" deg");
}

static void printStatus() {
  const float abs_deg = encAbsDegFresh();
  Serial.println("\n--- Status ---");
  Serial.print("enc_abs_deg: "); Serial.println(abs_deg, 2);
  Serial.print("enabled: "); Serial.println(g_enabled ? "true" : "false");
  Serial.print("foc_ready: "); Serial.println(g_foc_ready ? "true" : "false");
  if (g_foc_ready) {
    Serial.print("sensor_offset_deg: "); Serial.println(motor.sensor_offset * RAD_TO_DEG, 2);
  }
  Serial.println("--------------");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_HZ);

  sensor.init(&Wire);

  driver.voltage_power_supply = 12.0f;
  driver.pwm_frequency = 20000;

  // If your driver enable is ACTIVE-LOW, uncomment:
  // driver.enable_active_high = false;

  driver.init();
  driver.disable(); // keep off during boot/upload/reset

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

  // Start target at current absolute angle to avoid a jump when enabling
  g_target_abs_deg = encAbsDegFresh();

  Serial.println("\n=== Hermes Angle Control (ABS 0..360) ===");
  Serial.println("Motor starts DISABLED (reduces motion during upload/reset).");
  Serial.println("Display + targets are ABSOLUTE AS5600 degrees (0..360).");
  Serial.println("We compensate motor.sensor_offset when commanding targets.");
  printHelp();
  printStatus();

  g_next_print_ms = millis() + PRINT_PERIOD_MS;
}

void loop() {
  // Always update sensor so angle updates even when disabled
  sensor.update();

  // -------- Serial input --------
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputString.length() > 0) {
        String cmd = inputString;
        cmd.trim();
        inputString = "";

        if (cmd.equalsIgnoreCase("e")) {
          enableMotor();
        } else if (cmd.equalsIgnoreCase("c")) {
          runFOCOnceThisSession();
        } else if (cmd.equalsIgnoreCase("d")) {
          disableMotor();
        } else if (cmd.equalsIgnoreCase("p")) {
          printStatus();
        } else if (isAngleCommand(cmd)) {
          float requested = cmd.toFloat();
          g_target_abs_deg = norm360(requested);

          Serial.print("\nRequested(abs)=");
          Serial.print(requested, 2);
          Serial.print(" -> Target(abs, wrapped)=");
          Serial.print(g_target_abs_deg, 2);
          Serial.println(" deg");
        } else if (cmd.equalsIgnoreCase("h") || cmd.equalsIgnoreCase("help")) {
          printHelp();
        } else {
          Serial.println("\nUnknown command. Type 'h' for help.");
        }
      }
    } else {
      inputString += c;
    }
  }

  // -------- Motor control --------
  if (g_enabled && g_foc_ready) {
    const float target_abs_rad   = g_target_abs_deg * DEG_TO_RAD;
    const float target_shaft_rad = target_abs_rad - motor.sensor_offset; // critical frame fix
    motor.loopFOC();
    motor.move(target_shaft_rad);
  }

  // -------- Print angles (ABS frame, parity) --------
  const uint32_t now = millis();
  if ((int32_t)(now - g_next_print_ms) >= 0) {
    g_next_print_ms = now + PRINT_PERIOD_MS;

    const float abs_deg = norm360(sensor.getAngle() * RAD_TO_DEG);

    // True parity: both are the same absolute angle source
    const float enc_deg   = abs_deg;
    const float motor_deg = abs_deg;

    Serial.print("enc_deg=");
    Serial.print(enc_deg, 2);
    Serial.print(" motor_deg=");
    Serial.print(motor_deg, 2);
    Serial.print(" target=");
    Serial.print(g_target_abs_deg, 2);
    Serial.print(" en=");
    Serial.print(g_enabled ? 1 : 0);
    Serial.print("   \r");
  }
}
